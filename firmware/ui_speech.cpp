#include "ui_speech.h"

#include "radio_catalog.h"

static float g_speechVolume = 0.45f;
uint8_t g_volumeLevel = 1;
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
// A key press zeroes it but cannot drop it, so its length is the delay before
// the answer to the key starts.
static constexpr int I2S_DMA_BUF_COUNT = 6;
static constexpr int I2S_DMA_BUF_LEN = 128;

enum AudioItemType : uint8_t { AUDIO_CLIP = 0, AUDIO_SILENCE = 1 };

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
static void audioQueueClear() { g_aqHead = g_aqTail = 0; }
static bool audioEnqueueClip(const uint8_t* data, size_t len) {
  if (!data || !len) return false;
  int next = (g_aqTail + 1) % AUDIO_QUEUE_LEN;
  if (next == g_aqHead) return false;
  g_audioQ[g_aqTail] = {AUDIO_CLIP, data, len, 0, g_enqueueTuningId};
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

#define VOICE_CLIP(n) {#n, voice_##n, voice_##n##_len}

// Sorted by name; every clip here must exist in voice_data.h.
static const VoiceClip kVoiceClips[] = {
  VOICE_CLIP(a),
  VOICE_CLIP(am),
  VOICE_CLIP(auto),
  VOICE_CLIP(b),
  VOICE_CLIP(bank),
  VOICE_CLIP(c),
  VOICE_CLIP(cancel),
  VOICE_CLIP(choose),
  VOICE_CLIP(clarifier),
  VOICE_CLIP(ctcss),
  VOICE_CLIP(cw),
  VOICE_CLIP(cwr),
  VOICE_CLIP(d),
  VOICE_CLIP(db),
  VOICE_CLIP(dcs),
  VOICE_CLIP(digi),
  VOICE_CLIP(e),
  VOICE_CLIP(eight),
  VOICE_CLIP(elecraft),
  VOICE_CLIP(equals),
  VOICE_CLIP(error),
  VOICE_CLIP(f),
  VOICE_CLIP(fast),
  VOICE_CLIP(fifty),
  VOICE_CLIP(filter),
  VOICE_CLIP(filtershape),
  VOICE_CLIP(filterwidth),
  VOICE_CLIP(five),
  VOICE_CLIP(fm),
  VOICE_CLIP(forty),
  VOICE_CLIP(four),
  VOICE_CLIP(frequency),
  VOICE_CLIP(g),
  VOICE_CLIP(h),
  VOICE_CLIP(hertz),
  VOICE_CLIP(high),
  VOICE_CLIP(i),
  VOICE_CLIP(icom),
  VOICE_CLIP(j),
  VOICE_CLIP(k),
  VOICE_CLIP(kenwood),
  VOICE_CLIP(kilohertz),
  VOICE_CLIP(l),
  VOICE_CLIP(level),
  VOICE_CLIP(lock),
  VOICE_CLIP(lsb),
  VOICE_CLIP(m),
  VOICE_CLIP(megahertz),
  VOICE_CLIP(menu),
  VOICE_CLIP(minus),
  VOICE_CLIP(mode),
  VOICE_CLIP(monitor),
  VOICE_CLIP(n),
  VOICE_CLIP(nine),
  VOICE_CLIP(noiseblanker),
  VOICE_CLIP(noisereduction),
  VOICE_CLIP(notavailable),
  VOICE_CLIP(notch),
  VOICE_CLIP(o),
  VOICE_CLIP(off),
  VOICE_CLIP(ok),
  VOICE_CLIP(on),
  VOICE_CLIP(one),
  VOICE_CLIP(p),
  VOICE_CLIP(pbt),
  VOICE_CLIP(percent),
  VOICE_CLIP(please),
  VOICE_CLIP(plus),
  VOICE_CLIP(point),
  VOICE_CLIP(power),
  VOICE_CLIP(profile),
  VOICE_CLIP(ptt),
  VOICE_CLIP(q),
  VOICE_CLIP(r),
  VOICE_CLIP(repeater),
  VOICE_CLIP(rit),
  VOICE_CLIP(row),
  VOICE_CLIP(rtty),
  VOICE_CLIP(rttyr),
  VOICE_CLIP(rx),
  VOICE_CLIP(s),
  VOICE_CLIP(s_meter),
  VOICE_CLIP(seven),
  VOICE_CLIP(sharp),
  VOICE_CLIP(six),
  VOICE_CLIP(sixty),
  VOICE_CLIP(slow),
  VOICE_CLIP(soft),
  VOICE_CLIP(split),
  VOICE_CLIP(stack),
  VOICE_CLIP(step),
  VOICE_CLIP(swr),
  VOICE_CLIP(sync),
  VOICE_CLIP(t),
  VOICE_CLIP(ten),
  VOICE_CLIP(thankyou),
  VOICE_CLIP(thirty),
  VOICE_CLIP(three),
  VOICE_CLIP(timeout),
  VOICE_CLIP(tone),
  VOICE_CLIP(transceive),
  VOICE_CLIP(transceiver),
  VOICE_CLIP(tune),
  VOICE_CLIP(tuner),
  VOICE_CLIP(twenty),
  VOICE_CLIP(two),
  VOICE_CLIP(tx),
  VOICE_CLIP(u),
  VOICE_CLIP(usb),
  VOICE_CLIP(v),
  VOICE_CLIP(vfo),
  VOICE_CLIP(volume),
  VOICE_CLIP(w),
  VOICE_CLIP(watts),
  VOICE_CLIP(wfm),
  VOICE_CLIP(x),
  VOICE_CLIP(xiegu),
  VOICE_CLIP(y),
  VOICE_CLIP(yaesu),
  VOICE_CLIP(z),
  VOICE_CLIP(zero),
};

#undef VOICE_CLIP
static const size_t kVoiceClipsCount = sizeof(kVoiceClips) / sizeof(kVoiceClips[0]);

// token must already be normalized (trimmed, lowercase) by speakToken().
static const VoiceClip* findVoiceClip(const String& token) {
  for (size_t i = 0; i < kVoiceClipsCount; ++i) {
    if (strcmp(token.c_str(), kVoiceClips[i].name) == 0) return &kVoiceClips[i];
  }
  return nullptr;
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

static bool speakClipToken(const String& token) {
  const VoiceClip* c = findVoiceClip(token);
  if (!c) return false;
  return playClipProgmem(c->data, c->len);
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
static constexpr float BEEP_AMPLITUDE = 0.2f;
// The I2S driver resumes writing into the DMA buffer the previous playback left
// half full, and that buffer plays whenever the DMA ring reaches it, out of order
// with the rest. One DMA buffer of leading silence guarantees only silence lands
// there and the tone starts in fresh buffers.
static constexpr int BEEP_LEAD_SAMPLES = I2S_DMA_BUF_LEN;
static constexpr int BEEP_TONE_SAMPLES = I2S_SAMPLE_RATE * BEEP_MS / 1000;
static int16_t s_beepPcm[BEEP_LEAD_SAMPLES + BEEP_TONE_SAMPLES];

static void initBeep() {
  const int n = BEEP_TONE_SAMPLES;
  const int fade = I2S_SAMPLE_RATE * BEEP_FADE_MS / 1000;
  for (int i = 0; i < n; ++i) {
    float gain = BEEP_AMPLITUDE * 32767.0f;
    if (i < fade) gain *= (float)i / fade;
    else if (n - 1 - i < fade) gain *= (float)(n - 1 - i) / fade;
    s_beepPcm[BEEP_LEAD_SAMPLES + i] = (int16_t)(gain * sinf(2.0f * (float)M_PI * BEEP_FREQ_HZ * i / I2S_SAMPLE_RATE));
  }
}

static bool playClipProgmemBlocking(const uint8_t* data, size_t length) {
  const size_t CHUNK = 512;
  static uint8_t buffer[CHUNK];
  size_t offset = 0;
  while (offset < length) {
    if (audioStopRequested()) return false;
    size_t n = length - offset;
    if (n > CHUNK) n = CHUNK;
    memcpy_P(buffer, data + offset, n);
    int16_t* samples = (int16_t*)buffer;
    size_t sampleCount = n / 2;
    for (size_t i = 0; i < sampleCount; ++i) {
      int32_t v = samples[i];
      v = (int32_t)(v * g_speechVolume);
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }
    size_t written = 0;
    esp_err_t err = i2s_write(I2S_NUM_0, buffer, n, &written, pdMS_TO_TICKS(20));
    if (audioStopRequested()) return false;
    if (err != ESP_OK) return false;
    if (written == 0) continue;
    offset += written;
  }
  return true;
}

static void playSilenceMsBlocking(int ms) {
  int16_t z[80];
  memset(z, 0, sizeof(z));
  size_t written = 0;
  int loops = max(1, ms / 10);
  for (int i = 0; i < loops; ++i) {
    if (audioStopRequested()) break;
    i2s_write(I2S_NUM_0, z, sizeof(z), &written, pdMS_TO_TICKS(20));
  }
}

static void audioTask(void* pv) {
  (void)pv;
  for (;;) {
    if (g_audioAbortReq) {
      g_audioAbortReq = false;
      i2s_zero_dma_buffer(I2S_NUM_0);
    }

    if (audioQueueIsEmpty()) {
      if (g_audioPlaying) {
        playSilenceMsBlocking(40);
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
      if (it.type == AUDIO_CLIP) (void)playClipProgmemBlocking(it.data, it.len);
      else playSilenceMsBlocking((int)it.silenceMs);
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

  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = I2S_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB;
  cfg.dma_buf_count = I2S_DMA_BUF_COUNT;
  cfg.dma_buf_len = I2S_DMA_BUF_LEN;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;

  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
  pins.bck_io_num = I2S_BCLK_PIN;
  pins.ws_io_num = I2S_LRCLK_PIN;
  pins.data_out_num = I2S_DOUT_PIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);

  xTaskCreatePinnedToCore(audioTask, "audioTask", 4096, nullptr, 2, nullptr, 1);
  applyVolumeLevel(DEFAULT_VOLUME_LEVEL);
  initBeep();
}

bool playClipProgmem(const uint8_t* data, size_t length) {
  if (!g_speechEnabled) return false;
  return audioEnqueueClip(data, length);
}

void playSilenceMs(int ms) {
  if (ms <= 0) return;
  (void)audioEnqueueSilence((uint16_t)ms);
}

void playDigit(int d) {
  switch (d) {
    case 0: playClipProgmem(voice_zero, voice_zero_len); break;
    case 1: playClipProgmem(voice_one, voice_one_len); break;
    case 2: playClipProgmem(voice_two, voice_two_len); break;
    case 3: playClipProgmem(voice_three, voice_three_len); break;
    case 4: playClipProgmem(voice_four, voice_four_len); break;
    case 5: playClipProgmem(voice_five, voice_five_len); break;
    case 6: playClipProgmem(voice_six, voice_six_len); break;
    case 7: playClipProgmem(voice_seven, voice_seven_len); break;
    case 8: playClipProgmem(voice_eight, voice_eight_len); break;
    case 9: playClipProgmem(voice_nine, voice_nine_len); break;
  }
}

void speakDigitsAndPoint(const String& s) {
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c >= '0' && c <= '9') playDigit(c - '0');
    else if (c == '.' || c == ',') playClipProgmem(voice_point, voice_point_len);
    else if (c == ' ') playSilenceMs(60);
  }
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
  if (!g_keypadExecuting) speakToken("s_meter");
  playSilenceMs(60);
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

  if (speakClipToken(normalized)) return true;

  for (size_t i = 0; i < sizeof(kVoiceAliases) / sizeof(kVoiceAliases[0]); ++i) {
    if (normalized.equals(kVoiceAliases[i].token)) {
      return speakTokens(kVoiceAliases[i].parts, kVoiceAliases[i].count, 60);
    }
  }

  playClipProgmem(voice_error, voice_error_len);
  return false;
}

bool speakTokenState(const String& token, bool on) {
  if (!g_speechEnabled) return false;
  bool ok = speakToken(token);
  playSilenceMs(60);
  return speakToken(on ? "on" : "off") && ok;
}

bool speakTokenPercent(const String& token, uint8_t percent) {
  if (!g_speechEnabled) return false;
  bool ok = speakToken(token);
  playSilenceMs(60);
  speakDigitsAndPoint(String((int)percent));
  playSilenceMs(60);
  return speakToken("percent") && ok;
}

void speakOk() { speakToken("ok"); }
void speakError() { speakToken("error"); }
void speakTimeout() { speakToken("timeout"); }
void speakNotAvailable() { speakToken("notavailable"); }
void playBeep() { (void)playClipProgmem((const uint8_t*)s_beepPcm, sizeof(s_beepPcm)); }

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

static void playDigitsFromCString(const char* s) {
  if (!s) return;
  for (const char* p = s; *p; ++p) {
    if (*p >= '0' && *p <= '9') playDigit((int)(*p - '0'));
  }
}

void speakProfileIdentityFromSlot(uint8_t id, bool withOk) {
  const StoredProfile* sp = storedProfileForId(id);
  if (!sp || !g_speechEnabled) return;

  String vendor = sp->voiceVendor;
  vendor.toLowerCase();
  if (vendor == "xiegu") {
    playClipProgmem(voice_xiegu, voice_xiegu_len);
  } else if (vendor == "icom") {
    playClipProgmem(voice_icom, voice_icom_len);
  } else if (vendor == "kenwood") {
    playClipProgmem(voice_kenwood, voice_kenwood_len);
  } else if (vendor == "yaesu") {
    playClipProgmem(voice_yaesu, voice_yaesu_len);
  } else if (vendor == "elecraft") {
    playClipProgmem(voice_elecraft, voice_elecraft_len);
  } else {
    playClipProgmem(voice_icom, voice_icom_len);
  }

  playSilenceMs(80);
  playDigitsFromCString(sp->voiceDigits);
  if (withOk) {
    playSilenceMs(60);
    playClipProgmem(voice_ok, voice_ok_len);
  }
}

void speakBootProfile() {
  speakProfileIdentityFromSlot(g_profileId, true);
}

void listVoices() {
  for (size_t i = 0; i < kVoiceClipsCount; ++i) Serial.println(kVoiceClips[i].name);
}

bool playNamedVoice(const String& token) {
  return speakToken(token);
}

void voiceTest() {
  Serial.println("Voice TEST...");
  for (size_t i = 0; i < kVoiceClipsCount; ++i) {
    Serial.print("  ");
    Serial.println(kVoiceClips[i].name);
    playClipProgmemBlocking(kVoiceClips[i].data, kVoiceClips[i].len);
    playSilenceMsBlocking(120);
  }
  Serial.println("Voice TEST done.");
}
