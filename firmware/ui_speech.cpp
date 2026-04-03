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

enum AudioItemType : uint8_t { AUDIO_CLIP = 0, AUDIO_SILENCE = 1 };

struct AudioItem {
  AudioItemType type;
  const uint8_t* data;
  size_t len;
  uint16_t silenceMs;
};

static const int AUDIO_QUEUE_LEN = 32;
static volatile int g_aqHead = 0;
static volatile int g_aqTail = 0;
static AudioItem g_audioQ[AUDIO_QUEUE_LEN];
static volatile bool g_audioAbortReq = false;
static volatile bool g_audioAbortEnabled = true;
volatile bool g_audioPlaying = false;

static inline bool audioQueueIsEmpty() { return g_aqHead == g_aqTail; }
static void audioQueueClear() { g_aqHead = g_aqTail = 0; }
static bool audioEnqueueClip(const uint8_t* data, size_t len) {
  if (!data || !len) return false;
  int next = (g_aqTail + 1) % AUDIO_QUEUE_LEN;
  if (next == g_aqHead) return false;
  g_audioQ[g_aqTail] = {AUDIO_CLIP, data, len, 0};
  g_aqTail = next;
  return true;
}
static bool audioEnqueueSilence(uint16_t ms) {
  int next = (g_aqTail + 1) % AUDIO_QUEUE_LEN;
  if (next == g_aqHead) return false;
  g_audioQ[g_aqTail] = {AUDIO_SILENCE, nullptr, 0, ms};
  g_aqTail = next;
  return true;
}

static inline float volumeLevelToGain(uint8_t lvl) {
  switch (lvl) {
    case 0: return 0.25f;
    case 1: return 0.45f;
    case 2: return 0.70f;
    case 3: return 1.00f;
    default: return 0.70f;
  }
}

static const VoiceClip kVoiceClips[] = {
  {"voice_a", voice_a, voice_a_len},
  {"voice_am", voice_am, voice_am_len},
  {"voice_bank", voice_bank, voice_bank_len},
  {"voice_b", voice_b, voice_b_len},
  {"voice_c", voice_c, voice_c_len},
  {"voice_choose", voice_choose, voice_choose_len},
#if defined(HAS_VOICE_voice_clarifier)
  {"voice_clarifier", voice_clarifier, voice_clarifier_len},
#endif
#if defined(HAS_VOICE_voice_ctcss)
  {"voice_ctcss", voice_ctcss, voice_ctcss_len},
#endif
  {"voice_cw", voice_cw, voice_cw_len},
  {"voice_cwr", voice_cwr, voice_cwr_len},
  {"voice_d", voice_d, voice_d_len},
#if defined(HAS_VOICE_voice_dcs)
  {"voice_dcs", voice_dcs, voice_dcs_len},
#endif
  {"voice_db", voice_db, voice_db_len},
  {"voice_digi", voice_digi, voice_digi_len},
  {"voice_eight", voice_eight, voice_eight_len},
  {"voice_elecraft", voice_elecraft, voice_elecraft_len},
#if defined(HAS_VOICE_voice_equals)
  {"voice_equals", voice_equals, voice_equals_len},
#endif
  {"voice_error", voice_error, voice_error_len},
  {"voice_f", voice_f, voice_f_len},
  {"voice_five", voice_five, voice_five_len},
  {"voice_filter", voice_filter, voice_filter_len},
  {"voice_filtershape", voice_filtershape, voice_filtershape_len},
  {"voice_filterwidth", voice_filterwidth, voice_filterwidth_len},
  {"voice_fm", voice_fm, voice_fm_len},
  {"voice_four", voice_four, voice_four_len},
  {"voice_frequency", voice_frequency, voice_frequency_len},
#if defined(HAS_VOICE_voice_g)
  {"voice_g", voice_g, voice_g_len},
#endif
  {"voice_hertz", voice_hertz, voice_hertz_len},
#if defined(HAS_VOICE_voice_mode)
  {"voice_mode", voice_mode, voice_mode_len},
#endif
  {"voice_i", voice_i, voice_i_len},
  {"voice_icom", voice_icom, voice_icom_len},
  {"voice_kenwood", voice_kenwood, voice_kenwood_len},
  {"voice_kilohertz", voice_kilohertz, voice_kilohertz_len},
  {"voice_l", voice_l, voice_l_len},
  {"voice_level", voice_level, voice_level_len},
  {"voice_lsb", voice_lsb, voice_lsb_len},
  {"voice_lock", voice_lock, voice_lock_len},
  {"voice_m", voice_m, voice_m_len},
  {"voice_megahertz", voice_megahertz, voice_megahertz_len},
  {"voice_minus", voice_minus, voice_minus_len},
  {"voice_monitor", voice_monitor, voice_monitor_len},
  {"voice_nine", voice_nine, voice_nine_len},
#if defined(HAS_VOICE_voice_noiseblanker)
  {"voice_noiseblanker", voice_noiseblanker, voice_noiseblanker_len},
#endif
#if defined(HAS_VOICE_voice_noisereduction)
  {"voice_noisereduction", voice_noisereduction, voice_noisereduction_len},
#endif
  {"voice_notch", voice_notch, voice_notch_len},
#if defined(HAS_VOICE_voice_notchfilter)
  {"voice_notchfilter", voice_notchfilter, voice_notchfilter_len},
#endif
  {"voice_off", voice_off, voice_off_len},
  {"voice_ok", voice_ok, voice_ok_len},
  {"voice_on", voice_on, voice_on_len},
  {"voice_one", voice_one, voice_one_len},
  {"voice_pbt", voice_pbt, voice_pbt_len},
  {"voice_percent", voice_percent, voice_percent_len},
  {"voice_please", voice_please, voice_please_len},
  {"voice_plus", voice_plus, voice_plus_len},
  {"voice_point", voice_point, voice_point_len},
  {"voice_power", voice_power, voice_power_len},
#if defined(HAS_VOICE_voice_ptt)
  {"voice_ptt", voice_ptt, voice_ptt_len},
#endif
#if defined(HAS_VOICE_voice_r)
  {"voice_r", voice_r, voice_r_len},
#endif
#if defined(HAS_VOICE_voice_repeater)
  {"voice_repeater", voice_repeater, voice_repeater_len},
#endif
#if defined(HAS_VOICE_voice_rit)
  {"voice_rit", voice_rit, voice_rit_len},
#endif
  {"voice_rtty", voice_rtty, voice_rtty_len},
  {"voice_rttyr", voice_rttyr, voice_rttyr_len},
  {"voice_s", voice_s, voice_s_len},
#if defined(HAS_VOICE_voice_s_meter)
  {"voice_s_meter", voice_s_meter, voice_s_meter_len},
#endif
  {"voice_seven", voice_seven, voice_seven_len},
  {"voice_sharp", voice_sharp, voice_sharp_len},
  {"voice_six", voice_six, voice_six_len},
  {"voice_soft", voice_soft, voice_soft_len},
  {"voice_split", voice_split, voice_split_len},
  {"voice_stack", voice_stack, voice_stack_len},
  {"voice_step", voice_step, voice_step_len},
  {"voice_swr", voice_swr, voice_swr_len},
#if defined(HAS_VOICE_voice_sync)
  {"voice_sync", voice_sync, voice_sync_len},
#endif
  {"voice_t", voice_t, voice_t_len},
  {"voice_thankyou", voice_thankyou, voice_thankyou_len},
  {"voice_three", voice_three, voice_three_len},
  {"voice_transceiver", voice_transceiver, voice_transceiver_len},
#if defined(HAS_VOICE_voice_tone)
  {"voice_tone", voice_tone, voice_tone_len},
#endif
  {"voice_tune", voice_tune, voice_tune_len},
  {"voice_tuner", voice_tuner, voice_tuner_len},
  {"voice_two", voice_two, voice_two_len},
  {"voice_u", voice_u, voice_u_len},
  {"voice_usb", voice_usb, voice_usb_len},
  {"voice_vfo", voice_vfo, voice_vfo_len},
  {"voice_w", voice_w, voice_w_len},
  {"voice_watts", voice_watts, voice_watts_len},
  {"voice_y", voice_y, voice_y_len},
  {"voice_yaesu", voice_yaesu, voice_yaesu_len},
#if defined(HAS_VOICE_voice_xiegu)
  {"voice_xiegu", voice_xiegu, voice_xiegu_len},
#endif
  {"voice_zero", voice_zero, voice_zero_len},
};
static const size_t kVoiceClipsCount = sizeof(kVoiceClips) / sizeof(kVoiceClips[0]);

static const VoiceClip* findVoiceClip(String token) {
  token.trim();
  if (!token.length()) return nullptr;
  if (!token.startsWith("voice_")) token = String("voice_") + token;
  for (size_t i = 0; i < kVoiceClipsCount; ++i) {
    if (token.equalsIgnoreCase(kVoiceClips[i].name)) return &kVoiceClips[i];
  }
  return nullptr;
}

struct VoiceAlias {
  const char* token;
  const char* parts[5];
  uint8_t count;
};

static const VoiceAlias kVoiceAliases[] = {
  {"usb", {"u", "s", "b"}, 3},
  {"lsb", {"l", "s", "b"}, 3},
  {"cw", {"c", "w"}, 2},
  {"cwr", {"c", "w", "r"}, 3},
  {"fm", {"f", "m"}, 2},
  {"wfm", {"w", "f", "m"}, 3},
  {"db", {"d", "b"}, 2},
  {"swr", {"s", "w", "r"}, 3},
  {"rtty", {"r", "t", "t", "y"}, 4},
  {"rttyr", {"r", "t", "t", "y", "r"}, 5},
  {"rtty_r", {"r", "t", "t", "y", "r"}, 5},
  {"notchfilter", {"notch", "filter"}, 2},
  {"notch filter", {"notch", "filter"}, 2},
  {"vfoa", {"vfo", "a"}, 2},
  {"vfob", {"vfo", "b"}, 2},
  {"pbt1", {"pbt", "one"}, 2},
  {"pbt2", {"pbt", "two"}, 2},
  {"filshape", {"filtershape"}, 1},
  {"filwidth", {"filterwidth"}, 1},
  {"transceive", {"transceiver"}, 1},
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
}

static bool playClipProgmemBlocking(const uint8_t* data, size_t length) {
  const size_t CHUNK = 512;
  static uint8_t buffer[CHUNK];
  size_t offset = 0;
  while (offset < length) {
    if (g_audioAbortReq) return false;
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
    if (g_audioAbortReq) return false;
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
    if (g_audioAbortReq) break;
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
    if (it.type == AUDIO_CLIP) (void)playClipProgmemBlocking(it.data, it.len);
    else playSilenceMsBlocking((int)it.silenceMs);
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
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
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

void speakSValue(uint8_t sVal) {
  if (!g_speechEnabled) return;
  if (!g_keypadExecuting) speakToken("s_meter");
  playSilenceMs(60);
  playDigit((int)min<uint8_t>(sVal, 9));
  playSilenceMs(250);
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

void applyVolumeLevel(uint8_t lvl) {
  if (lvl > 3) lvl = 2;
  g_volumeLevel = lvl;
  g_speechVolume = volumeLevelToGain(lvl);
  Serial.print("[VOL] Applied level ");
  Serial.print((int)lvl);
  Serial.print(" (gain=");
  Serial.print(g_speechVolume, 2);
  Serial.println(")");
}

void speakVolumeLevel(uint8_t lvl) {
  uint8_t spoken = (lvl <= 3) ? (uint8_t)(lvl + 1) : 3;
  playDigit(spoken);
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
#ifdef HAS_VOICE_voice_xiegu
    playClipProgmem(voice_xiegu, voice_xiegu_len);
#else
    playClipProgmem(voice_icom, voice_icom_len);
#endif
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
