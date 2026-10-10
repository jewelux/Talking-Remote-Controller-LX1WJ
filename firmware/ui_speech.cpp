#include "ui_speech.h"

// The I2S driver below (driver/i2s_std.h) exists from ESP-IDF 5, i.e. core 3.x.
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "HamTRC needs the ESP32 Arduino core 3.x or newer (Boards Manager: esp32 by Espressif Systems)"
#endif

#include "driver/i2s_std.h"
#include "audio_stretch.h"
#include "radio_catalog.h"
#include "voice_fallback.h"
#include "voice_pack.h"

static float g_speechVolume = 0.45f;
uint8_t g_volumeLevel = 1;

// Input samples per output sample for each SpeechSpeed, set by ear on the device.
static constexpr float kSpeechSpeedFactors[SPEECH_SPEED_COUNT] = {0.8f, 1.0f, 1.3f};
SpeechSpeed g_speechSpeed = SpeechSpeed::Normal;
// Read by the audio task at the start of each item.
static volatile float g_speechSpeedFactor = 1.0f;
static const byte KP_ROWS = 4;
static const byte KP_COLS = 4;
#if USE_BUTTONS_KEYPAD
static char kpKeys[KP_ROWS][KP_COLS] = {
  {'1', '4', '7', '*'},
  {'2', '5', '8', '0'},
  {'3', '6', '9', '#'},
  {'A', 'B', 'C', 'D'}
};
#else
static char kpKeys[KP_ROWS][KP_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
#endif
static byte kpRowPins[KP_ROWS] = {KP_ROW_PINS[0], KP_ROW_PINS[1], KP_ROW_PINS[2], KP_ROW_PINS[3]};
static byte kpColPins[KP_COLS] = {KP_COL_PINS[0], KP_COL_PINS[1], KP_COL_PINS[2], KP_COL_PINS[3]};
Keypad keypad = Keypad(makeKeymap(kpKeys), kpRowPins, kpColPins, KP_ROWS, KP_COLS);

// The DMA ring holds I2S_DMA_BUF_COUNT * I2S_DMA_BUF_LEN samples (96 ms at 8 kHz).
// A key press stops the output and refills the ring with silence, so its length
// is the delay before the answer to the key starts.
static constexpr int I2S_DMA_BUF_COUNT = 6;
static constexpr int I2S_DMA_BUF_LEN = 128;
static constexpr size_t I2S_DMA_BUF_BYTES = I2S_DMA_BUF_LEN * sizeof(int16_t);
static i2s_chan_handle_t s_i2sTx = nullptr;
// Bytes written into the DMA buffer being filled, 0 at a buffer boundary.
static size_t s_dmaBufFill = 0;

// The clips carry no silence around the word; this gap follows each one.
static constexpr int CLIP_GAP_MS = 60;

// AUDIO_CLIP is a spoken word, followed by CLIP_GAP_MS; AUDIO_TONE (the beep) is not.
enum AudioItemType : uint8_t { AUDIO_CLIP = 0, AUDIO_SILENCE = 1, AUDIO_TONE = 2 };

struct AudioItem {
  AudioItemType type;
  const uint8_t* data;
  size_t len;
  uint16_t silenceMs;
  uint32_t tuningId;  // 0 for ordinary speech, otherwise the tuning announcement it belongs to
};

static const int AUDIO_QUEUE_LEN = 32;
static volatile int g_aqHead = 0;
static volatile int g_aqTail = 0;
static AudioItem g_audioQ[AUDIO_QUEUE_LEN];
static volatile bool g_audioAbortReq = false;
static volatile bool g_audioAbortEnabled = true;
volatile bool g_audioPlaying = false;

// Tuning announcements are numbered so a stale one can be dropped without
// touching other speech. Ids are issued by the main loop and increase
// monotonically; the audio task skips any item whose id is cancelled.
static uint32_t g_tuningSeq = 0;
static uint32_t g_enqueueTuningId = 0;
static bool g_tuningItemQueued = false;
static volatile uint32_t g_tuningCancelUpTo = 0;
static volatile uint32_t g_tuningDoneId = 0;
static volatile uint32_t g_tuningEndMs = 0;
static volatile uint32_t g_playingTuningId = 0;

static inline bool tuningIdCancelled(uint32_t id) { return id != 0 && id <= g_tuningCancelUpTo; }
static inline bool audioStopRequested() { return g_audioAbortReq || tuningIdCancelled(g_playingTuningId); }

static inline bool audioQueueIsEmpty() { return g_aqHead == g_aqTail; }
static void audioQueueClear() {
  g_aqHead = 0;
  g_aqTail = 0;
}
static bool audioEnqueueClip(AudioItemType type, const uint8_t* data, size_t len) {
  if (!data || !len) return false;
  int next = (g_aqTail + 1) % AUDIO_QUEUE_LEN;
  if (next == g_aqHead) return false;
  g_audioQ[g_aqTail] = {type, data, len, 0, g_enqueueTuningId};
  g_aqTail = next;
  if (g_enqueueTuningId) g_tuningItemQueued = true;
  return true;
}
static bool audioEnqueueSilence(uint16_t ms) {
  int next = (g_aqTail + 1) % AUDIO_QUEUE_LEN;
  if (next == g_aqHead) return false;
  g_audioQ[g_aqTail] = {AUDIO_SILENCE, nullptr, 0, ms, g_enqueueTuningId};
  g_aqTail = next;
  if (g_enqueueTuningId) g_tuningItemQueued = true;
  return true;
}

static inline float volumeLevelToGain(uint8_t lvl) {
  switch (lvl) {
    case 1: return 0.08f;
    case 2: return 0.12f;
    case 3: return 0.18f;
    case 4: return 0.27f;
    case 5: return 0.40f;
    case 6: return 0.55f;
    case 7: return 0.70f;
    case 8: return 0.85f;
    case 9: return 1.00f;
    default: return volumeLevelToGain(DEFAULT_VOLUME_LEVEL);
  }
}

struct VoiceAlias {
  const char* token;
  const char* parts[5];
  uint8_t count;
};

// Only consulted when no clip matches, so a token here must not also be a clip.
static const VoiceAlias kVoiceAliases[] = {
  {"rtty_r", {"rttyr"}, 1},
  {"gt", {"g", "t"}, 2},
  {"id", {"i", "d"}, 2},
  {"if", {"i", "f"}, 2},
  {"pa", {"p", "a"}, 2},
  {"ps", {"p", "s"}, 2},
  {"notchfilter", {"notch", "filter"}, 2},
  {"vfoa", {"vfo", "a"}, 2},
  {"vfob", {"vfo", "b"}, 2},
  {"pbt1", {"pbt", "one"}, 2},
  {"pbt2", {"pbt", "two"}, 2},
  {"filshape", {"filtershape"}, 1},
  {"filwidth", {"filterwidth"}, 1},
};

static bool playClip(const uint8_t* data, size_t length, AudioItemType type = AUDIO_CLIP) {
  if (!g_speechEnabled) return false;
  return audioEnqueueClip(type, data, length);
}

// token must already be normalized (trimmed, lowercase) by speakToken().
static bool speakClipToken(const char* token) {
  VoiceClip c;
  if (!voicePackFindClip(token, &c)) return false;
  return playClip(c.data, c.len);
}

bool speakTokens(const char* const* tokens, size_t count, uint16_t gapMs) {
  if (!g_speechEnabled) return false;
  bool ok = true;
  for (size_t i = 0; i < count; ++i) {
    ok = speakToken(tokens[i]) && ok;
    if (i + 1 < count) playSilenceMs((int)gapMs);
  }
  return ok;
}

void audioAmpOn() { if (AMP_SD_PIN >= 0) digitalWrite(AMP_SD_PIN, HIGH); }
void audioAmpOff() { if (AMP_SD_PIN >= 0) digitalWrite(AMP_SD_PIN, LOW); }

void audioAbortNow() {
  if (!g_audioAbortEnabled) return;
  g_audioAbortReq = true;
  audioQueueClear();
  g_tuningCancelUpTo = g_tuningSeq;
}

void beginTuningSpeech() {
  cancelTuningSpeech();
  g_enqueueTuningId = ++g_tuningSeq;
  g_tuningItemQueued = false;
}

void endTuningSpeech() {
  // Nothing reached the queue (full, or speech off): the audio task will never finish it.
  if (!g_tuningItemQueued) g_tuningDoneId = g_enqueueTuningId;
  g_enqueueTuningId = 0;
}

void cancelTuningSpeech() { g_tuningCancelUpTo = g_tuningSeq; }

bool tuningSpeechActive() {
  const uint32_t id = g_tuningSeq;
  return id != 0 && id > g_tuningDoneId && !tuningIdCancelled(id);
}

uint32_t tuningSpeechEndedMs() { return g_tuningEndMs; }

// Short tone for key presses that do nothing. Generated once into RAM and played
// through the clip queue, so it gets the volume, speech gate and key-press abort.
static constexpr int BEEP_FREQ_HZ = 660;
static constexpr int BEEP_MS = 70;
static constexpr int BEEP_FADE_MS = 5;
static constexpr float BEEP_AMPLITUDE = 0.112f;
static constexpr int BEEP_SAMPLES = I2S_SAMPLE_RATE * BEEP_MS / 1000;
static int16_t s_beepPcm[BEEP_SAMPLES];

static void initBeep() {
  const int n = BEEP_SAMPLES;
  const int fade = I2S_SAMPLE_RATE * BEEP_FADE_MS / 1000;
  for (int i = 0; i < n; ++i) {
    float gain = BEEP_AMPLITUDE * 32767.0f;
    if (i < fade) gain *= (float)i / fade;
    else if (n - 1 - i < fade) gain *= (float)(n - 1 - i) / fade;
    s_beepPcm[i] = (int16_t)(gain * sinf(2.0f * (float)M_PI * BEEP_FREQ_HZ * i / I2S_SAMPLE_RATE));
  }
}

// Every write goes through here, so s_dmaBufFill follows the driver's position in
// its DMA buffer. A timeout only means no DMA buffer came free yet; *written says
// how much went in.
static esp_err_t i2sWrite(const void* data, size_t n, size_t* written) {
  *written = 0;
  esp_err_t err = i2s_channel_write(s_i2sTx, data, n, written, 20);
  s_dmaBufFill = (s_dmaBufFill + *written) % I2S_DMA_BUF_BYTES;
  return err;
}

// False when stopped or on a write error.
static bool writeAll(const void* data, size_t bytes) {
  const uint8_t* p = (const uint8_t*)data;
  while (bytes > 0) {
    if (audioStopRequested()) return false;
    size_t written = 0;
    esp_err_t err = i2sWrite(p, bytes, &written);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) return false;
    p += written;
    bytes -= written;
  }
  return !audioStopRequested();
}

// data is PCM16 (clips in the pack are 4-byte aligned); speed 1 plays it as it is.
static bool playClipBlocking(const uint8_t* data, size_t length, float speed) {
  static AudioStretch stretch;
  static int16_t block[I2S_DMA_BUF_LEN];
  stretchBegin(stretch, (const int16_t*)data, length / sizeof(int16_t), speed);
  for (size_t n; (n = stretchNext(stretch, block, I2S_DMA_BUF_LEN)) > 0;) {
    for (size_t i = 0; i < n; ++i) {
      int32_t v = (int32_t)(block[i] * g_speechVolume);
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      block[i] = (int16_t)v;
    }
    if (!writeAll(block, n * sizeof(int16_t))) return false;
  }
  return true;
}

static void writeSilenceBytes(size_t bytes) {
  static const int16_t zeros[I2S_DMA_BUF_LEN] = {};
  while (bytes > 0 && !audioStopRequested()) {
    size_t written = 0;
    esp_err_t err = i2sWrite(zeros, min(bytes, sizeof(zeros)), &written);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) break;
    bytes -= written;
  }
}

static void playSilenceMsBlocking(int ms) {
  writeSilenceBytes((size_t)ms * I2S_SAMPLE_RATE / 1000 * sizeof(int16_t));
}

// The driver keeps a half-filled DMA buffer between writes, and the next write
// continues in it while it is still queued for playing. A playback starting just
// as the DMA reaches that buffer would be written into a buffer being played.
// Ending every playback on a buffer boundary starts the next in a free buffer.
static void padToDmaBufferBoundary() {
  if (s_dmaBufFill) writeSilenceBytes(I2S_DMA_BUF_BYTES - s_dmaBufFill);
}

// Stopping the channel cuts the sound at once but leaves the unplayed buffers as
// they are, and enabling it plays the ring from the first buffer. Preloading
// silence overwrites all of it, so the cut-off speech cannot come back.
static void i2sDropBufferedAudio() {
  static const int16_t silence[I2S_DMA_BUF_COUNT * I2S_DMA_BUF_LEN] = {};
  size_t loaded = 0;
  i2s_channel_disable(s_i2sTx);
  i2s_channel_preload_data(s_i2sTx, silence, sizeof(silence), &loaded);
  i2s_channel_enable(s_i2sTx);
  // The preload filled every buffer, so the next write takes a fresh one.
  s_dmaBufFill = 0;
}

static void audioTask(void* pv) {
  (void)pv;
  for (;;) {
    if (g_audioAbortReq) {
      g_audioAbortReq = false;
      i2sDropBufferedAudio();
    }

    if (audioQueueIsEmpty()) {
      if (g_audioPlaying) {
        padToDmaBufferBoundary();
        g_audioPlaying = false;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (!g_audioPlaying) {
      audioAmpOn();
      vTaskDelay(pdMS_TO_TICKS(2));
      g_audioPlaying = true;
    }

    AudioItem it = g_audioQ[g_aqHead];
    g_aqHead = (g_aqHead + 1) % AUDIO_QUEUE_LEN;
    if (!tuningIdCancelled(it.tuningId)) {
      g_playingTuningId = it.tuningId;
      const float speed = g_speechSpeedFactor;
      if (it.type == AUDIO_SILENCE) {
        playSilenceMsBlocking((int)lrintf(it.silenceMs / speed));
      } else if (it.type == AUDIO_TONE) {
        (void)playClipBlocking(it.data, it.len, 1.0f);
      } else if (playClipBlocking(it.data, it.len, speed)) {
        playSilenceMsBlocking((int)lrintf(CLIP_GAP_MS / speed));
      }
      g_playingTuningId = 0;
    }
    // A tuning announcement ends with its last queued item.
    if (it.tuningId != 0 && (audioQueueIsEmpty() || g_audioQ[g_aqHead].tuningId != it.tuningId)) {
      g_tuningEndMs = millis();
      g_tuningDoneId = it.tuningId;
    }
  }
}

void initSpeech() {
  if (AMP_SD_PIN >= 0) pinMode(AMP_SD_PIN, OUTPUT);
  voicePackInit();

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = I2S_DMA_BUF_COUNT;
  chanCfg.dma_frame_num = I2S_DMA_BUF_LEN;
  chanCfg.auto_clear = true;
  i2s_new_channel(&chanCfg, &s_i2sTx, nullptr);

  // 16-bit MSB-aligned frames, sound on the left slot only.
  i2s_std_config_t stdCfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK_PIN,
      .ws = (gpio_num_t)I2S_LRCLK_PIN,
      .dout = (gpio_num_t)I2S_DOUT_PIN,
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {},
    },
  };
  stdCfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  i2s_channel_init_std_mode(s_i2sTx, &stdCfg);
  i2s_channel_enable(s_i2sTx);

  xTaskCreatePinnedToCore(audioTask, "audioTask", 4096, nullptr, 2, nullptr, 1);
  applyVolumeLevel(DEFAULT_VOLUME_LEVEL);
  initBeep();
}

void playSilenceMs(int ms) {
  if (ms <= 0) return;
  (void)audioEnqueueSilence((uint16_t)ms);
}

void playDigit(int d) {
  static const char* const kDigits[] = {"zero", "one", "two", "three", "four",
                                        "five", "six", "seven", "eight", "nine"};
  if (d >= 0 && d <= 9) speakClipToken(kDigits[d]);
}

void speakNumber(const String& s) {
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c >= '0' && c <= '9') playDigit(c - '0');
    else if (c == '.' || c == ',') speakClipToken("point");
    else if (c == ' ') playSilenceMs(60);
  }
}

void speakDigitsAndPoint(const String& s) {
  speakNumber(s);
  playSilenceMs(250);
}

// Plays "ten".."sixty"; false if tens is out of range or the clip is missing.
static bool playTens(int tens) {
  static const char* const kTens[] = {"ten", "twenty", "thirty", "forty", "fifty", "sixty"};
  if (tens < 1 || tens > 6) return false;
  return speakClipToken(kTens[tens - 1]);
}

void speakSValue(const SMeterReading& reading) {
  if (!g_speechEnabled) return;
  // From the keypad, the key already said "s meter" (speakKeypadCommandWord).
  if (!g_keypadExecuting) speakLabel("s_meter");
  playDigit((int)min<uint8_t>(reading.sUnits, 9));
  if (reading.dbOverS9) {
    playSilenceMs(60);
    speakToken("plus");
    playSilenceMs(60);
    if (reading.dbOverS9 % 10 == 0 && playTens(reading.dbOverS9 / 10)) {
      playSilenceMs(250);
    } else {
      speakDigitsAndPoint(String(reading.dbOverS9));  // ends with the trailing pause
    }
  } else {
    playSilenceMs(250);
  }
}

bool speakToken(const String& token) {
  if (!g_speechEnabled) return false;

  String normalized = token;
  normalized.trim();
  normalized.toLowerCase();
  normalized.replace("-", "_");
  if (!normalized.length()) return false;

  int spacePos = normalized.indexOf(' ');
  if (spacePos >= 0) {
    bool ok = true;
    int start = 0;
    while (start < normalized.length()) {
      while (start < normalized.length() && normalized[start] == ' ') start++;
      if (start >= normalized.length()) break;
      int end = normalized.indexOf(' ', start);
      if (end < 0) end = normalized.length();
      ok = speakToken(normalized.substring(start, end)) && ok;
      start = end;
      while (start < normalized.length() && normalized[start] == ' ') start++;
      if (start < normalized.length()) playSilenceMs(60);
    }
    return ok;
  }

  if (speakClipToken(normalized.c_str())) return true;

  for (size_t i = 0; i < sizeof(kVoiceAliases) / sizeof(kVoiceAliases[0]); ++i) {
    if (normalized.equals(kVoiceAliases[i].token)) {
      return speakTokens(kVoiceAliases[i].parts, kVoiceAliases[i].count, 60);
    }
  }

  speakClipToken("error");
  return false;
}

bool speakLabel(const String& token) {
  if (!g_speechEnabled || !g_verboseSpeech) return true;
  bool ok = speakToken(token);
  playSilenceMs(60);
  return ok;
}

bool speakTokenState(const String& token, bool on) {
  if (!g_speechEnabled) return false;
  bool ok = speakLabel(token);
  return speakToken(on ? "on" : "off") && ok;
}

bool speakTokenPercent(const String& token, uint8_t percent) {
  if (!g_speechEnabled) return false;
  bool ok = speakLabel(token);
  speakNumber(String((int)percent));
  playSilenceMs(60);
  return speakToken("percent") && ok;
}

void speakValueOk() {
  if (!g_speechEnabled || !g_verboseSpeech) return;
  playSilenceMs(60);
  speakOk();
}

void speakOk() { speakToken("ok"); }
void speakError() { speakToken("error"); }
void speakTimeout() { speakToken("timeout"); }
void speakNotAvailable() { speakToken("notavailable"); }
void playBeep() { (void)playClip((const uint8_t*)s_beepPcm, sizeof(s_beepPcm), AUDIO_TONE); }

void applyVolumeLevel(uint8_t lvl) {
  if (lvl < 1) lvl = 1;
  if (lvl > 9) lvl = 9;
  g_volumeLevel = lvl;
  g_speechVolume = volumeLevelToGain(lvl);
  Serial.print("[VOL] Applied level ");
  Serial.print((int)lvl);
  Serial.print(" (gain=");
  Serial.print(g_speechVolume, 2);
  Serial.println(")");
}

void speakVolumeLevel(uint8_t lvl) {
  if (lvl < 1) lvl = 1;
  if (lvl > 9) lvl = 9;
  playDigit(lvl);
}

static const char* const kSpeechSpeedNames[SPEECH_SPEED_COUNT] = {"SLOW", "NORMAL", "FAST"};

void applySpeechSpeed(SpeechSpeed speed) {
  if ((uint8_t)speed >= SPEECH_SPEED_COUNT) speed = SpeechSpeed::Normal;
  g_speechSpeed = speed;
  g_speechSpeedFactor = kSpeechSpeedFactors[(uint8_t)speed];
}

const char* speechSpeedName(SpeechSpeed speed) {
  return (uint8_t)speed < SPEECH_SPEED_COUNT ? kSpeechSpeedNames[(uint8_t)speed] : "NORMAL";
}

bool parseSpeechSpeed(const String& word, SpeechSpeed& out) {
  String w = word;
  w.trim();
  for (uint8_t i = 0; i < SPEECH_SPEED_COUNT; ++i) {
    if (w.equalsIgnoreCase(kSpeechSpeedNames[i])) {
      out = (SpeechSpeed)i;
      return true;
    }
  }
  return false;
}

void speakSpeechSpeed() {
  if (!g_speechEnabled) return;
  speakLabel("speed");
  speakToken(speechSpeedName(g_speechSpeed));
}

static void playDigitsFromCString(const char* s) {
  if (!s) return;
  for (const char* p = s; *p; ++p) {
    if (*p >= '0' && *p <= '9') playDigit((int)(*p - '0'));
  }
}

void speakProfileIdentityFromSlot(uint8_t id, bool withOk) {
  const RadioProfile* sp = profileForSlot(id);
  if (!sp || !g_speechEnabled) return;

  switch (sp->vendor) {
    case VoiceVendor::Icom: speakClipToken("icom"); break;
    case VoiceVendor::Yaesu: speakClipToken("yaesu"); break;
    case VoiceVendor::Kenwood: speakClipToken("kenwood"); break;
    case VoiceVendor::Elecraft: speakClipToken("elecraft"); break;
    case VoiceVendor::Xiegu: speakClipToken("xiegu"); break;
  }

  playSilenceMs(80);
  playDigitsFromCString(sp->voiceDigits);
  if (withOk) {
    playSilenceMs(60);
    speakClipToken("ok");
  }
}

void speakBootProfile() {
  if (!voicePackReady()) {
    (void)playClip((const uint8_t*)kVoicePackMissingPcm, sizeof(kVoicePackMissingPcm));
    return;
  }
  speakProfileIdentityFromSlot(g_profileId, true);
}

void listVoices() {
  VoiceClip c;
  for (size_t i = 0; voicePackClipAt(i, &c); ++i) Serial.println(c.name);
}

bool playNamedVoice(const String& token) {
  return speakToken(token);
}

// Plays the clips from the calling task, so the audio task must be done first:
// both would write to I2S and share playClipBlocking's buffers. Only the main
// loop queues speech, and it is busy here, so nothing new starts meanwhile.
void voiceTest() {
  while (!audioQueueIsEmpty() || g_audioPlaying) delay(5);
  Serial.println("Voice TEST...");
  VoiceClip c;
  for (size_t i = 0; voicePackClipAt(i, &c); ++i) {
    Serial.print("  ");
    Serial.println(c.name);
    playClipBlocking(c.data, c.len, g_speechSpeedFactor);
    playSilenceMsBlocking(CLIP_GAP_MS + 120);
  }
  padToDmaBufferBoundary();
  Serial.println("Voice TEST done.");
}
