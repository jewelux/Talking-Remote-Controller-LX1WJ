/*
  ESP32 Talking Remote Controller — ESP32-S3 + Keypad + Speech + Multi-Radio Profiles
  File: ESP32S3_TalkingRemote_V1.0.ino
  Version: V1.0  (stable core; ongoing development)

  Purpose
  - Accessible “talking” remote controller for amateur radio transceivers.
  - Provides spoken prompts and spoken radio status (frequency, mode, S-meter, SWR, power, etc.).
  - Supports multiple radio profiles and multiple physical interfaces (CI-V, TTL-CAT, RS-232).

  High-level architecture (summary)
  1) Keypad / UI
     - 4x4 matrix keypad with banks and short/long press semantics.
     - Input can be “immediate query” (short press) or “staged entry” (long press + ENTER).
  2) Radio I/O
     - CI-V (Icom) over UART (shared/open-collector wiring as configured).
     - Optional RS-232 via MAX3232.
     - Optional TTL-CAT (e.g., Xiegu) via UART.
     - Incoming frames update the current “radio state” (latest freq/mode/etc.).
  3) Speech / Audio
     - Pre-recorded voice tokens compiled into flash via voice_data.h.
     - Audio plays from a dedicated FreeRTOS task (queue-based).
     - Any NEW key press aborts current speech immediately (and clears queued speech).

  Files
  - This sketch: the complete firmware (configuration + profiles + UI + audio).
  - voice_data.h: generated/embedded voice clips (digits, words, prompts, radio names).

  Configuration
  - Edit the parameters in the “USER CONFIG” section (pins, baud rates, feature flags).
  - Select/extend radio profiles in the profiles section (CI-V/RS232/TTL-CAT mapping).
  - Add new voice tokens by updating voice_data.h (and matching filenames/tokens).

  Notes for maintainers
  - Keep “project-wide” explanations in this header.
  - Keep “section-local” comments next to the code that implements that section.

*/


#include <Arduino.h>
#include <Preferences.h>
#include "driver/uart.h"
#include <Keypad.h>

extern "C" {
  #include "driver/i2s.h"
}

#include "voice_data.h"

// ---- Serial / CI-V configuration (must be defined before profiles) ----
// PC Serial
static const uint32_t PC_BAUD = 115200;

// CI-V UART (Open-Collector wiring shared by IC-7300 and IC-706 direct CI-V)
static const uint32_t CIV_BAUD = 9600;   // must match radio CI-V baud
static const int CIV_RX_PIN = 18;        // adjust to your wiring
static const int CIV_TX_PIN = 17;        // adjust to your wiring

// Optional RS232 UART (MAX3232 / CT-17) for profile 3 testing
static const int RS232_RX_PIN = 9;       // adjust to your wiring
static const int RS232_TX_PIN = 10;      // adjust to your wiring


// TTL-CAT UART for Xiegu G106 (3.3V TTL, 19200 8N1, non-inverted)
static const uint32_t CAT_BAUD = 19200;
static const int CAT_TX_PIN = 11;   // ESP32 -> G106 (Ring = RXD)
static const int CAT_RX_PIN = 12;   // G106 (Tip = TXD) -> ESP32
static const bool CAT_TX_INVERT = false;
static const bool CAT_RX_INVERT = false;


// ------------------------------------------------------------
// Types used in auto-generated Arduino function prototypes
// (Must be defined BEFORE any function that returns/uses them.)
// ------------------------------------------------------------
struct VoiceClip {
  const char* name;
  const uint8_t* data;
  size_t len;
};


// ============================================================
// USER CONFIG (compile-time)
// ============================================================

// Query keys: if true, Bank 1 query keys speak immediately on short press (no ENTER needed)
static constexpr bool AUTO_SEND_BANK1_QUERIES = true;

// ============================================================
//  ICOM RADIO SELECTION (compile-time)
// ============================================================

enum IcomModel : uint8_t {
  ICOM_IC_7300,
  ICOM_IC_706MKIIG
};

// <<< CHANGE THIS ONE LINE >>>
static constexpr IcomModel ICOM_MODEL = ICOM_IC_7300;
// static constexpr IcomModel ICOM_MODEL = ICOM_IC_706MKIIG;

struct CivProfile {
  uint8_t civAddr;      // radio CI-V address (1 byte)
  const char* name;

  // Link/physical layer configuration (per profile)
  uint32_t baud;
  int8_t uartNum;       // 1 or 2 (ESP32 UART)
  int8_t rxPin;
  int8_t txPin;
  bool txInvert;        // ESP32 UART TX invert (depends on your CI-V/level hardware)
  bool rxInvert;
};

// Profile 1: IC-7300 via CI-V Open-Collector (same wiring you already use)
static constexpr CivProfile PROFILE_7300_CIV = { 0x94, "IC-7300", CIV_BAUD, 1, CIV_RX_PIN, CIV_TX_PIN, true, false };
// Profile 2: IC-706 via CI-V Open-Collector (direct CI-V, NOT CT-17)
static constexpr CivProfile PROFILE_706_CIV  = { 0x58, "IC-706MkIIG", CIV_BAUD, 1, CIV_RX_PIN, CIV_TX_PIN, true, false };
// Profile 3: IC-706 command set over RS232 (CT-17 / MAX3232) for further testing
static constexpr CivProfile PROFILE_706_RS232 = { 0x58, "IC-706MkIIG RS232", CIV_BAUD, 2, RS232_RX_PIN, RS232_TX_PIN, false, false };
// Profile 4: Xiegu G106 via TTL-CAT (TRS jack, Tip=TXD, Ring=RXD, GND=Sleeve)
// CI-V-like framing: FE FE 76 E0 ... FD  (radio addr = 0x76, controller addr = 0xE0)
static constexpr CivProfile PROFILE_G106_CAT = { 0x76, "Xiegu G106", CAT_BAUD, 2, CAT_RX_PIN, CAT_TX_PIN, CAT_TX_INVERT, CAT_RX_INVERT };


static constexpr CivProfile getProfile(IcomModel m) {
  return (m == ICOM_IC_7300) ? PROFILE_7300_CIV : PROFILE_706_CIV;
}

static constexpr CivProfile CIV = getProfile(ICOM_MODEL);
// ---- Runtime profile selection (Bank 3) ----
enum ProfileId : uint8_t {
  PROFILE_ID_7300 = 1,
  PROFILE_ID_706_CIV = 2,
  PROFILE_ID_706_RS232 = 3,
  PROFILE_ID_G106_CAT = 4
};

// ============================================================
// Persistent settings (NVS / Preferences)
// ============================================================
static uint8_t loadProfileFromNvs(uint8_t fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  uint8_t v = prefs.getUChar("profile", fallback);
  prefs.end();
  if (v == PROFILE_ID_7300 || v == PROFILE_ID_706_CIV || v == PROFILE_ID_706_RS232 || v == PROFILE_ID_G106_CAT) return v;
  return fallback;
  }

static void saveProfileToNvs(uint8_t id) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putUChar("profile", id);
  prefs.end();
}


static bool loadTuningSpeakFromNvs(bool fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  bool v = prefs.getBool("tuningspk", fallback);
  prefs.end();
  return v;
}

static void saveTuningSpeakToNvs(bool v) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putBool("tuningspk", v);
  prefs.end();
}


static uint8_t g_profileId = (ICOM_MODEL == ICOM_IC_706MKIIG) ? PROFILE_ID_706_CIV : PROFILE_ID_7300;
static uint8_t g_lastSavedProfile = 0xFF; // to reduce NVS writes

static inline const CivProfile& currentProfile() {
  if (g_profileId == PROFILE_ID_G106_CAT)  return PROFILE_G106_CAT;
  if (g_profileId == PROFILE_ID_706_RS232) return PROFILE_706_RS232;
  if (g_profileId == PROFILE_ID_706_CIV)   return PROFILE_706_CIV;
  return PROFILE_7300_CIV;
}

static void applyProfile(uint8_t profileId);
static void speakCurrentProfile();


// Controller (ESP32) CI-V address (typical for controllers/PC)
static constexpr uint8_t CIV_MY_ADDR = 0xE0;

// [Legacy note] Older V7 reader documentation was consolidated into the header at the top of this file.


// ============================================================
// USER TUNABLE PARAMETERS (all here)
// ============================================================

// ---- 4x4 Matrix Keypad ----
// Choose 8 free GPIOs (4 rows + 4 columns). Adjust to your wiring.
// NOTE (V8 JTAG/LED-free pinmap):
// - Avoids JTAG pins GPIO39-42 and onboard RGB LED GPIO38.
// - Also avoids FSPI flash signal pins GPIO9-14.
// - Keypad pins are now: ROWS={GPIO4,GPIO8,GPIO15,GPIO16} COLS={GPIO1,GPIO2,GPIO3,GPIO21}
// NOTE: Avoid 5,6,7 (I2S) and 17,18 (CI-V) and strap pins like 0/45/46.

// ============================================================
// USER CONFIG (compile-time)
// ============================================================
// Default speech volume at boot (0..3):
// 0 = very quiet, 1 = quiet, 2 = medium, 3 = loud
#define DEFAULT_VOLUME_LEVEL 1

// Select keypad type before compiling/flashing:
// 0 = Folien-Tastatur (standard mapping)
// 1 = Tasten-Tastatur (rotated mapping, from your measurements)
#define USE_BUTTONS_KEYPAD 0


// Keypad timing (milliseconds)
#define KEYPAD_DEBOUNCE_MS 30
#define KEYPAD_HOLD_MS     700

static const byte KP_ROW_PINS[4] = {4, 8, 15, 16};
static const byte KP_COL_PINS[4] = {1, 2, 3, 21};


// Use a second UART so you can keep TTL CI-V and RS232 wired at the same time.
static uint8_t g_civRadioAddr = CIV.civAddr;   // active radio CI-V address (profile-selectable)
static const uint8_t CIV_CTRL_ADDR  = CIV_MY_ADDR;   // controller (ESP32) address

// ---- CI-V handling ----
static const uint32_t CIV_PUMP_BUDGET_MS = 3; // per loop slice

// ---- Speech behaviour for frequency while tuning ----
// If true: speak the first "neighbor" frequency when tuning starts, then wait and speak end frequency.
// If false: only speak end frequency after you stop tuning.
static const bool     FREQ_SPEAK_START_IMMEDIATELY = true;

// "Idle time" until we assume tuning stopped and speak the final frequency
static const uint32_t FREQ_SPEAK_IDLE_MS = 2000;

// Minimum change to consider (helps when the rig sends tiny updates)
// For 1 kHz step use 1000; for 10 Hz step use 50..100
static const uint32_t FREQ_SPEAK_MIN_STEP_HZ = 1000;

// Don't speak more often than this (safety)
static const uint32_t FREQ_SPEAK_MIN_INTERVAL_MS = 5000;

// ---- S-meter polling + speaking ----
// IC-7300 does not broadcast S-meter continuously; we poll it with CI-V cmd 0x15 subcmd 0x02.
static const bool     SMETER_POLL_ENABLE = false; // default: OFF (only on explicit SM? command)
static const uint32_t SMETER_POLL_MS     = 350;   // poll rate (250..500ms is typical)

// Speaking policy
static const bool     SMETER_SPEAK_ENABLE          = false; // default: OFF (we speak only on explicit SM? command)
static const uint8_t  SMETER_SPEAK_MIN_DELTA_S     = 1;    // speak only if S changes >= 1
static const uint32_t SMETER_SPEAK_MIN_INTERVAL_MS = 2500; // safety (speech rate limit)

// Calibration (raw -> S estimate). You can tune these later.
// You provided approx: S0 ~ 0x15 (21), S9 ~ 0x129 (297).
static const int32_t  SMETER_RAW_AT_S0 = 21;
static const int32_t  SMETER_RAW_AT_S9 = 297;

// ---- MAX98357 / I2S ----
static const int I2S_BCLK_PIN  = 5;
static const int I2S_LRCLK_PIN = 6;
static const int I2S_DOUT_PIN  = 7;

// If you wired SD/EN to a GPIO, set it here. If SD/EN is tied to 3V3, keep -1.
static const int AMP_SD_PIN = -1;

// Sample format (must match your voice_data.h samples; your SWR meter used 16k mono 16-bit)
static const int   I2S_SAMPLE_RATE = 11025;  // must match generated voice_data.h (PCM16 mono @ 11025 Hz)
static float g_speechVolume   = 0.45f;  // runtime-adjustable (default=low)
static uint8_t g_volumeLevel   = 1;     // 0=very low,1=low,2=medium,3=loud
static bool    g_volStageActive = false;
static uint8_t g_volStageLevel  = 2;

static inline float volumeLevelToGain(uint8_t lvl) {
  switch (lvl) {
    case 0: return 0.25f; // very low
    case 1: return 0.45f; // low
    case 2: return 0.70f; // medium
    case 3: return 1.00f; // loud
    default: return 0.70f;
  }
}

static inline void applyVolumeLevel(uint8_t lvl) {
  if (lvl > 3) lvl = 2;
  g_volumeLevel = lvl;
  g_speechVolume = volumeLevelToGain(lvl);
  Serial.print("[VOL] Applied level ");
  Serial.print((int)lvl);
  Serial.print(" (gain=");
  Serial.print(g_speechVolume, 2);
  Serial.println(")");
}

void playDigit(int d);  // forward decl
static inline void speakVolumeLevel(uint8_t lvl) {
  // speak 1..4 as level indicator (1=very low ... 4=loud)
  uint8_t spoken = (lvl <= 3) ? (uint8_t)(lvl + 1) : 3;
  playDigit(spoken);
}


// ---- Optional mode speech ----
// Your current voice_data.h does NOT contain USB/LSB/CW samples yet.
// When you add them (voice_usb, voice_lsb, voice_cw), set this to 1.
#define HAVE_MODE_VOICE 1

// ============================================================
// Types (must be before any function prototypes)
// ============================================================

struct CivDecoded {
  bool ok = false;
  uint8_t to = 0;
  uint8_t from = 0;
  uint8_t cmd = 0;
  const uint8_t* payload = nullptr;
  size_t payloadLen = 0;
};

struct LiveState {
  // frequency
  bool freqValid = false;
  uint64_t freqHz = 0;
  uint32_t lastFreqMs = 0;

  // tuning debounce
  bool tuning = false;
  uint64_t pendingHz = 0;
  uint64_t tuningStartSpokenHz = 0;
  uint32_t lastChangeMs = 0;

  // speech rate limiting
  uint64_t lastSpokenHz = 0;
  uint32_t lastSpokenMs = 0;

  // mode
  bool modeValid = false;
  uint8_t mode = 0xFF;
  uint32_t lastModeMs = 0;

  // s-meter
  bool smValid = false;
  int32_t smRaw = 0;
  uint8_t smS = 0;              // estimated S (0..9)
  uint8_t lastSpokenS = 0xFF;
  uint32_t lastSmPollMs = 0;
  uint32_t lastSmSpokenMs = 0;
} live;

// ============================================================
// 4x4 Keypad
// ============================================================
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
static byte kpRowPins[KP_ROWS] = { KP_ROW_PINS[0], KP_ROW_PINS[1], KP_ROW_PINS[2], KP_ROW_PINS[3] };
static byte kpColPins[KP_COLS] = { KP_COL_PINS[0], KP_COL_PINS[1], KP_COL_PINS[2], KP_COL_PINS[3] };
Keypad keypad = Keypad(makeKeymap(kpKeys), kpRowPins, kpColPins, KP_ROWS, KP_COLS);

// ============================================================
// Asynchronous audio engine (non-blocking speech + abort on keypress)
// Speech enqueues clips/silence into a small queue. Main loop remains responsive.
// Any key press while audio is playing aborts the current speech immediately.
// ============================================================

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
static bool g_bootSpeakPending = false;
static uint32_t g_bootSpeakAtMs = 0;

static volatile bool g_audioPlaying  = false;

static inline bool audioQueueIsEmpty() { return g_aqHead == g_aqTail; }
static inline bool audioQueueIsFull()  { return ((g_aqTail + 1) % AUDIO_QUEUE_LEN) == g_aqHead; }

static void audioQueueClear() { g_aqHead = g_aqTail = 0; }

static void audioAbortEnable(bool en) { g_audioAbortEnabled = en; }

void audioAbortNow() {
  if (!g_audioAbortEnabled) return;
  g_audioAbortReq = true;
  audioQueueClear();
}

static bool audioEnqueueClip(const uint8_t* data, size_t len) {
  if (!data || !len) return false;
  int next = (g_aqTail + 1) % AUDIO_QUEUE_LEN;
  if (next == g_aqHead) return false;
  g_audioQ[g_aqTail] = { AUDIO_CLIP, data, len, 0 };
  g_aqTail = next;
  return true;
}

static bool audioEnqueueSilence(uint16_t ms) {
  int next = (g_aqTail + 1) % AUDIO_QUEUE_LEN;
  if (next == g_aqHead) return false;
  g_audioQ[g_aqTail] = { AUDIO_SILENCE, nullptr, 0, ms };
  g_aqTail = next;
  return true;
}

// ---- audio helpers (forward declarations) ----
static inline void audioAmpOn();
static inline void audioAmpOff();
bool playClipProgmemBlocking(const uint8_t* data, size_t length);
void playSilenceMsBlocking(int ms);

void audioTask(void* pv) {
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

    if (it.type == AUDIO_CLIP) {
      (void)playClipProgmemBlocking(it.data, it.len);
    } else {
      playSilenceMsBlocking((int)it.silenceMs);
    }
  }
}


// ============================================================
// Forward declarations (avoid Arduino auto-prototype pitfalls)
// ============================================================

// Audio
static inline void audioAmpOn();
static inline void audioAmpOff();
void initI2S();
bool playClipProgmemBlocking(const uint8_t* data, size_t length); // async enqueue
bool playClipProgmemBlocking(const uint8_t* data, size_t length); // internal
void playSilenceMs(int ms); // async enqueue
void playSilenceMsBlocking(int ms); // internal
void playDigit(int d);
void speakDigitsAndPoint(const String& s);
void speakSValue(uint8_t sVal);

// CI-V core
uint64_t decodeBcdFrequencyHz(const uint8_t* bcd, size_t len);
String hzToMHzString3(uint64_t hz);
size_t civReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs);
CivDecoded civDecode(const uint8_t* buf, size_t n);
void civFlushInput();
void civSend(uint8_t cmd, const uint8_t* data, size_t dataLen);
bool waitReply(uint8_t expectCmd, CivDecoded &out, uint32_t timeoutMs);
bool queryFrequency(uint64_t &hzOut);
bool setFrequency(uint64_t hz);
bool queryMode(uint8_t &modeOut);
bool setMode(uint8_t mode, uint8_t filter = 1);
bool querySMeterRaw(int32_t &rawOut);

// CI-V handlers
void handleIncomingFrame(const CivDecoded& d);
void pumpIncoming(uint32_t maxMs);

// Speech debounce
void updateFreqSpeechDebounce(uint64_t newHz);
void speakPendingFreqIfIdle();

// Mode helpers
const char* modeToString(uint8_t mode);
void speakMode(uint8_t mode);

// S-meter helpers
int32_t bcdDigitsToInt(const uint8_t* b, size_t n);
uint8_t smRawToS(int32_t raw);
void pollSMeterIfDue();
void handleSMeterRaw(int32_t raw);

void keypadEvent(KeypadEvent k);
static void processCommand(String line);

// CLI
String readLine();
String upperCopy(String s);
void printHelp();
void voiceTest();

// ============================================================
// Globals
// ============================================================

HardwareSerial civUart1(1);
HardwareSerial civUart2(2);
static HardwareSerial* g_civSerial = &civUart1;
#define CIVSER (*g_civSerial)
bool g_quiet = false;
bool g_speechEnabled = true;


// VFO / tuning speech enable (toggleable)
static bool g_tuningSpeakEnabled = true;

// Suppress automatic frequency speech for a short time after manual SET FREQ (to avoid double speak)
static uint32_t g_suppressFreqSpeakUntilMs = 0;

// ============================================================
// Voice registry (maps all clips from voice_data.h by name)
// - You can address clips as "voice_ok" or just "ok".
// ============================================================


static const VoiceClip kVoiceClips[] = {
  {"voice_am", voice_am, voice_am_len},
  {"voice_bank", voice_bank, voice_bank_len},
  {"voice_c", voice_c, voice_c_len},
  {"voice_choose", voice_choose, voice_choose_len},
  {"voice_cw", voice_cw, voice_cw_len},
  {"voice_cwr", voice_cwr, voice_cwr_len},
  {"voice_db", voice_db, voice_db_len},
  {"voice_digi", voice_digi, voice_digi_len},
  {"voice_eight", voice_eight, voice_eight_len},
  {"voice_elecraft", voice_elecraft, voice_elecraft_len},
  {"voice_error", voice_error, voice_error_len},
  {"voice_five", voice_five, voice_five_len},
  {"voice_fm", voice_fm, voice_fm_len},
  {"voice_four", voice_four, voice_four_len},
  {"voice_frequency", voice_frequency, voice_frequency_len},
#if defined(HAS_VOICE_voice_mode)
  {"voice_mode", voice_mode, voice_mode_len},
#endif
  {"voice_i", voice_i, voice_i_len},
  {"voice_icom", voice_icom, voice_icom_len},
  {"voice_kenwood", voice_kenwood, voice_kenwood_len},
  {"voice_kilohertz", voice_kilohertz, voice_kilohertz_len},
  {"voice_lsb", voice_lsb, voice_lsb_len},
  {"voice_megahertz", voice_megahertz, voice_megahertz_len},
  {"voice_nine", voice_nine, voice_nine_len},
  {"voice_off", voice_off, voice_off_len},
  {"voice_ok", voice_ok, voice_ok_len},
  {"voice_on", voice_on, voice_on_len},
  {"voice_one", voice_one, voice_one_len},
  {"voice_please", voice_please, voice_please_len},
  {"voice_plus", voice_plus, voice_plus_len},
  {"voice_point", voice_point, voice_point_len},
  {"voice_power", voice_power, voice_power_len},
  {"voice_rtty", voice_rtty, voice_rtty_len},
  {"voice_rttyr", voice_rttyr, voice_rttyr_len},
  {"voice_s", voice_s, voice_s_len},
  {"voice_seven", voice_seven, voice_seven_len},
  {"voice_six", voice_six, voice_six_len},
  {"voice_swr", voice_swr, voice_swr_len},
  {"voice_thankyou", voice_thankyou, voice_thankyou_len},
  {"voice_three", voice_three, voice_three_len},
  {"voice_transceiver", voice_transceiver, voice_transceiver_len},
  {"voice_two", voice_two, voice_two_len},
  {"voice_u", voice_u, voice_u_len},
  {"voice_usb", voice_usb, voice_usb_len},
  {"voice_watts", voice_watts, voice_watts_len},
  {"voice_yaesu", voice_yaesu, voice_yaesu_len},
  {"voice_zero", voice_zero, voice_zero_len},
};

static const size_t kVoiceClipsCount = sizeof(kVoiceClips) / sizeof(kVoiceClips[0]);

static const VoiceClip* findVoiceClip(String token) {
  token.trim();
  if (!token.length()) return nullptr;

  // allow "ok" as shorthand for "voice_ok"
  if (!token.startsWith("voice_")) token = String("voice_") + token;

  for (size_t i = 0; i < kVoiceClipsCount; i++) {
    if (token.equalsIgnoreCase(kVoiceClips[i].name)) return &kVoiceClips[i];
  }
  return nullptr;
}

// Async enqueue wrapper: returns immediately (speech plays in background)
bool playClipProgmem(const uint8_t* data, size_t length) {
  if (!g_speechEnabled) return false;
  return audioEnqueueClip(data, length);
}

static bool speakToken(const String& token) {
  if (!g_speechEnabled) return false;
  const VoiceClip* c = findVoiceClip(token);
  if (!c) {
    playClipProgmem(voice_error, voice_error_len);
    return false;
  }
  return playClipProgmem(c->data, c->len);
}

static inline void speakOk()    { speakToken("ok"); }
static inline void speakError() { speakToken("error"); }
bool g_keypadExecuting = false; // true while executing via keypad ENTER (suppresses repeating command word)
bool g_suppressModePrefixOnce = false; // set when MODE? is staged as 'mode' already, so speak only the mode value on reply

// --- Keypad banks (A/B/C) ---
static uint8_t g_bank = 1; // 1..3
static bool g_modeSetActive = false;
static bool g_modeStageActive = false; // digit chosen in MODE-SET, waiting for Enter
static uint8_t g_modeStageMode = 0xFF;
static char g_modeStageKey = 0; // original digit for speaking

static bool g_profileSelectActive = false;
static bool g_nineHoldConsumed = false; // suppress RELEASE after HOLD on '9'

static String  g_kpStagedCmd;
static bool    g_kpHasStagedCmd = false;

static bool g_starHoldConsumed = false;
static bool g_zeroHoldConsumed = false;
static bool g_aHoldConsumed = false; // suppress RELEASE after HOLD on 'A' (Bank 1 volume up)
// Frequency entry mode (Bank B / key 0)
static bool    g_freqEntryActive = false;
static String  g_freqEntryDigits; // digits in kHz (e.g. 7074 -> 7.074 MHz)
static bool    g_freqEntryIsMHz = false; // false=kHz entry (bank2), true=MHz integer entry (bank1 long-0)


// ============================================================
// AUDIO IMPLEMENTATION (sample-based like your SWR meter)
// ============================================================

static inline void audioAmpOn()  { if (AMP_SD_PIN >= 0) digitalWrite(AMP_SD_PIN, HIGH); }

bool setFrequency(uint64_t hz) {
  // Set operating frequency: cmd 0x05, 5 BCD bytes (LS digit first).
  // After sending, we verify by reading the frequency back.
  uint8_t bcd[5] = {0};
  uint64_t v = hz;
  for (int i = 0; i < 5; i++) {
    uint8_t d1 = v % 10; v /= 10;
    uint8_t d2 = v % 10; v /= 10;
    bcd[i] = (d2 << 4) | d1;
  }

  civFlushInput();
  civSend(0x05, bcd, 5);
  delay(40);          // give the rig a moment
  pumpIncoming(40);   // process any echo/transceive frames

  uint64_t readHz = 0;
  if (!queryFrequency(readHz)) return false;
  return (readHz == hz);
}


static inline void audioAmpOff() { if (AMP_SD_PIN >= 0) digitalWrite(AMP_SD_PIN, LOW); }

void initI2S() {
  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = I2S_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len   = 256;
  cfg.use_apll      = false;
  cfg.tx_desc_auto_clear = true;

  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
  pins.bck_io_num   = I2S_BCLK_PIN;
  pins.ws_io_num    = I2S_LRCLK_PIN;
  pins.data_out_num = I2S_DOUT_PIN;
  pins.data_in_num  = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

bool playClipProgmemBlocking(const uint8_t* data, size_t length) {
  const size_t CHUNK = 512;
  static uint8_t buffer[CHUNK];

  size_t offset = 0;
  while (offset < length) {
    if (g_audioAbortReq) return false;

    size_t n = length - offset;
    if (n > CHUNK) n = CHUNK;

    memcpy_P(buffer, data + offset, n);

    // apply speech volume (16-bit signed PCM)
    int16_t* samples = (int16_t*)buffer;
    size_t sampleCount = n / 2;
    for (size_t i = 0; i < sampleCount; i++) {
      int32_t v = samples[i];
      v = (int32_t)(v * g_speechVolume);
      if (v >  32767) v =  32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }

    // write with short timeout so abort can interrupt quickly
    size_t written = 0;
    esp_err_t err = i2s_write(I2S_NUM_0, buffer, n, &written, pdMS_TO_TICKS(20));
    if (g_audioAbortReq) return false;
    if (err != ESP_OK) return false;

    // In rare cases written can be 0 on timeout; just retry same chunk.
    if (written == 0) continue;

    offset += written;
  }
  return true;
}

void playSilenceMsBlocking(int ms) {
  int16_t z[80];
  memset(z, 0, sizeof(z));
  size_t written = 0;
  int loops = max(1, ms / 10);
  for (int i = 0; i < loops; i++) {
    if (g_audioAbortReq) break;
    // short timeout so we can react quickly to abort
    i2s_write(I2S_NUM_0, z, sizeof(z), &written, pdMS_TO_TICKS(20));
  }
}
// Async silence enqueue
void playSilenceMs(int ms) {
  if (ms <= 0) return;
  (void)audioEnqueueSilence((uint16_t)ms);
}


void playDigit(int d) {
  switch (d) {
    case 0: playClipProgmem(voice_zero,  voice_zero_len);  break;
    case 1: playClipProgmem(voice_one,   voice_one_len);   break;
    case 2: playClipProgmem(voice_two,   voice_two_len);   break;
    case 3: playClipProgmem(voice_three, voice_three_len); break;
    case 4: playClipProgmem(voice_four,  voice_four_len);  break;
    case 5: playClipProgmem(voice_five,  voice_five_len);  break;
    case 6: playClipProgmem(voice_six,   voice_six_len);   break;
    case 7: playClipProgmem(voice_seven, voice_seven_len); break;
    case 8: playClipProgmem(voice_eight, voice_eight_len); break;
    case 9: playClipProgmem(voice_nine,  voice_nine_len);  break;
  }
}

void speakDigitsAndPoint(const String& s) {
for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') playDigit(c - '0');
    else if (c == '.' || c == ',') playClipProgmem(voice_point, voice_point_len);
    else if (c == ' ') playSilenceMs(60);
  }

  playSilenceMs(250);
}

void speakSValue(uint8_t sVal) {
  // Needs: voice_s + digits (already present in your voice_data.h)
  if (!g_speechEnabled) return;
if (!g_keypadExecuting) {
    playClipProgmem(voice_s, voice_s_len);
    }
  playSilenceMs(60);
  playDigit((int)min<uint8_t>(sVal, 9));
  playSilenceMs(250);
}

// ============================================================
// CI-V IMPLEMENTATION
// ============================================================

uint64_t decodeBcdFrequencyHz(const uint8_t* bcd, size_t len) {
  uint64_t hz = 0, place = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t lo = bcd[i] & 0x0F;
    uint8_t hi = (bcd[i] >> 4) & 0x0F;
    hz += (uint64_t)lo * place; place *= 10;
    hz += (uint64_t)hi * place; place *= 10;
  }
  return hz;
}

String hzToMHzString3(uint64_t hz) {
  uint64_t mhz = hz / 1000000ULL;
  uint64_t khz = (hz / 1000ULL) % 1000ULL;

  String s = String((uint32_t)mhz);
  s += ".";
  if (khz < 100) s += "0";
  if (khz < 10)  s += "0";
  s += String((uint32_t)khz);
  return s;
}

size_t civReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t n = 0;
  uint8_t feCount = 0;

  while (millis() - start < timeoutMs) {
    while (CIVSER.available()) {
      uint8_t b = (uint8_t)CIVSER.read();

      if (n == 0) {
        if (b == 0xFE) { buf[n++] = b; feCount = 1; }
        continue;
      }
      if (n == 1) {
        if (b == 0xFE && feCount == 1) buf[n++] = b;
        else { n = 0; feCount = 0; }
        continue;
      }

      if (n < bufMax) buf[n++] = b;
      if (b == 0xFD) return n;
    }
    delay(1);
  }
  return 0;
}

CivDecoded civDecode(const uint8_t* buf, size_t n) {
  CivDecoded d;
  if (n < 6) return d;
  if (buf[0] != 0xFE || buf[1] != 0xFE) return d;
  if (buf[n-1] != 0xFD) return d;

  d.to   = buf[2];
  d.from = buf[3];
  d.cmd  = buf[4];
  d.payload = &buf[5];
  d.payloadLen = (n - 1) - 5;
  d.ok = true;
  return d;
}

void civFlushInput() {
  while (CIVSER.available()) (void)CIVSER.read();
}

void civSend(uint8_t cmd, const uint8_t* data, size_t dataLen) {
  const uint8_t pre[4] = {0xFE, 0xFE, g_civRadioAddr, CIV_CTRL_ADDR};
  CIVSER.write(pre, sizeof(pre));
  CIVSER.write(cmd);
  if (data && dataLen) CIVSER.write(data, dataLen);
  CIVSER.write((uint8_t)0xFD);
  CIVSER.flush();
}

bool waitReply(uint8_t expectCmd, CivDecoded &out, uint32_t timeoutMs) {
  uint32_t start = millis();
  uint8_t buf[96];

  while (millis() - start < timeoutMs) {
    size_t n = civReadFrame(buf, sizeof(buf), 60);
    if (!n) continue;

    CivDecoded d = civDecode(buf, n);
    if (!d.ok) continue;

    if (d.from != g_civRadioAddr) continue;
    if (!(d.to == CIV_CTRL_ADDR || d.to == 0x00)) continue;
    if (d.cmd != expectCmd) continue;

    out = d;
    return true;
  }
  return false;
}

bool queryFrequency(uint64_t &hzOut) {
  civFlushInput();
  civSend(0x03, nullptr, 0);

  CivDecoded d;
  if (!waitReply(0x03, d, 800)) return false;
  if (d.payloadLen < 5) return false;

  hzOut = decodeBcdFrequencyHz(d.payload, 5);
  return true;
}

bool queryMode(uint8_t &modeOut) {
  // Read operating mode: cmd 0x04 (common CI-V); response payload: <mode> <filter>
  civFlushInput();
  civSend(0x04, nullptr, 0);

  CivDecoded d;
  if (!waitReply(0x04, d, 800)) return false;
  if (d.payloadLen < 1) return false;

  modeOut = d.payload[0];
  return true;
}

bool setMode(uint8_t mode, uint8_t filter) {
  // Set operating mode: cmd 0x06, payload: <mode> <filter>
  uint8_t pl[2] = { mode, filter };
  civFlushInput();
  civSend(0x06, pl, 2);
  return true; // many rigs don't ACK; confirm via MODE? if needed
}


int32_t bcdDigitsToInt(const uint8_t* b, size_t n) {
  // Each nibble is a decimal digit; build integer ignoring leading zeros.
  int32_t v = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t hi = (b[i] >> 4) & 0x0F;
    uint8_t lo = b[i] & 0x0F;
    if (hi <= 9) v = v * 10 + hi;
    if (lo <= 9) v = v * 10 + lo;
  }
  return v;
}

bool querySMeterRaw(int32_t &rawOut) {
  // Read S-meter: cmd 0x15, subcmd 0x02 (common CI-V convention)
  // Example response includes payload: 0x02 <BCD...>
  civFlushInput();
  const uint8_t sub[] = {0x02};
  civSend(0x15, sub, 1);

  CivDecoded d;
  if (!waitReply(0x15, d, 800)) return false;
  if (d.payloadLen < 2) return false;
  if (d.payload[0] != 0x02) return false;

  rawOut = bcdDigitsToInt(d.payload + 1, d.payloadLen - 1);
  return true;
}

bool queryPoMeterRaw(int32_t &rawOut) {
  // Read Po meter: cmd 0x15, subcmd 0x11 (documented on modern rigs; works on IC-7300)
  civFlushInput();
  const uint8_t sub[] = {0x11};
  civSend(0x15, sub, 1);

  CivDecoded d;
  if (!waitReply(0x15, d, 800)) return false;
  if (d.payloadLen < 2) return false;
  if (d.payload[0] != 0x11) return false;

  rawOut = bcdDigitsToInt(d.payload + 1, d.payloadLen - 1);
  return true;
}

bool querySWRRaw(int32_t &rawOut) {
  // Read SWR meter: cmd 0x15, subcmd 0x12 (documented for modern rigs)
  civFlushInput();
  const uint8_t sub[] = {0x12};
  civSend(0x15, sub, 1);

  CivDecoded d;
  if (!waitReply(0x15, d, 800)) return false;
  if (d.payloadLen < 2) return false;
  if (d.payload[0] != 0x12) return false;

  rawOut = bcdDigitsToInt(d.payload + 1, d.payloadLen - 1);
  return true;
}

// Convert SWR "raw" (0..255-ish) to an approximate SWR value.
// Reference points (from Icom CI-V guides):
//   raw=0   -> SWR 1.0
//   raw=48  -> SWR 1.5
//   raw=80  -> SWR 2.0
//   raw=120 -> SWR 3.0
static float swrRawToValue(int32_t raw) {
  if (raw <= 0) return 1.0f;
  if (raw <= 48)  return 1.0f + (raw / 48.0f) * 0.5f;                 // 1.0..1.5
  if (raw <= 80)  return 1.5f + ((raw - 48) / 32.0f) * 0.5f;          // 1.5..2.0
  if (raw <= 120) return 2.0f + ((raw - 80) / 40.0f) * 1.0f;          // 2.0..3.0
  return 3.0f + ((raw - 120) / 135.0f) * 3.0f;                        // rough tail
}


// ============================================================
// Speech debounce for frequency
// ============================================================

void updateFreqSpeechDebounce(uint64_t newHz) {
  const uint32_t now = millis();
  if (!g_tuningSpeakEnabled) return;
  if ((int32_t)(now - g_suppressFreqSpeakUntilMs) < 0) return;

  if (live.pendingHz != 0) {
    uint64_t diff = (newHz > live.pendingHz) ? (newHz - live.pendingHz) : (live.pendingHz - newHz);
    if (diff < FREQ_SPEAK_MIN_STEP_HZ) return;
  }

  if (!live.tuning) {
    live.tuning = true;
    live.tuningStartSpokenHz = 0;
  }

  live.pendingHz = newHz;
  live.lastChangeMs = now;

  if (g_speechEnabled && FREQ_SPEAK_START_IMMEDIATELY && live.tuningStartSpokenHz == 0) {
    if (now - live.lastSpokenMs >= FREQ_SPEAK_MIN_INTERVAL_MS) {
      live.tuningStartSpokenHz = newHz;
      live.lastSpokenHz = newHz;
      live.lastSpokenMs = now;
      speakDigitsAndPoint(hzToMHzString3(newHz));
    }
  }
}

void speakPendingFreqIfIdle() {
  if (!g_speechEnabled) return;
  if (!g_tuningSpeakEnabled) return;
  const uint32_t now0 = millis();
  if ((int32_t)(now0 - g_suppressFreqSpeakUntilMs) < 0) return;
  if (!live.tuning) return;
  if (live.pendingHz == 0) return;

  const uint32_t now = millis();
  if (now - live.lastChangeMs < FREQ_SPEAK_IDLE_MS) return;
  if (now - live.lastSpokenMs < FREQ_SPEAK_MIN_INTERVAL_MS) return;

  if (live.pendingHz != live.lastSpokenHz) {
    live.lastSpokenHz = live.pendingHz;
    live.lastSpokenMs = now;
    speakDigitsAndPoint(hzToMHzString3(live.pendingHz));
  }

  live.tuning = false;
}

// ============================================================
// MODE (detect + announce on change)
// ============================================================

const char* modeToString(uint8_t mode) {
  // Common CI-V table: 00 LSB, 01 USB, 02 AM, 03 CW, 04 RTTY, 05 FM, 06 WFM
  switch (mode) {
    case 0x00: return "LSB";
    case 0x01: return "USB";
    case 0x02: return "AM";
    case 0x03: return "CW";
    case 0x04: return "RTTY";
    case 0x05: return "FM";
    case 0x06: return "WFM";
    default:   return "UNK";
  }
}

void speakMode(uint8_t mode) {
#if HAVE_MODE_VOICE
  // Only enable when your voice_data.h contains voice_usb/voice_lsb/voice_cw
// Say the word "mode" first if sampled (optional).
#if defined(HAS_VOICE_voice_mode)
  if (!g_suppressModePrefixOnce) {
    playClipProgmem(voice_mode, voice_mode_len);
    playSilenceMs(120);
  } else {
    // consume the one-shot suppression
    g_suppressModePrefixOnce = false;
  }
#endif
  switch (mode) {
    case 0x00: playClipProgmem(voice_lsb,  voice_lsb_len);  break;
    case 0x01: playClipProgmem(voice_usb,  voice_usb_len);  break;
    case 0x02: playClipProgmem(voice_am,   voice_am_len);   break;
    case 0x03: playClipProgmem(voice_cw,   voice_cw_len);   break;
    case 0x04: playClipProgmem(voice_rtty, voice_rtty_len); break;
    case 0x05: playClipProgmem(voice_fm,   voice_fm_len);   break;
    case 0x07: playClipProgmem(voice_cwr,  voice_cwr_len);  break;
    case 0x08: playClipProgmem(voice_rttyr, voice_rttyr_len); break;
    case 0x11: playClipProgmem(voice_digi, voice_digi_len); break;
    default:
      // unknown / not sampled
      break;
  }
  playSilenceMs(200);
#else
  (void)mode;
#endif
}

// ============================================================
// S-METER (poll + speak only strong changes)
// ============================================================

uint8_t smRawToS(int32_t raw) {
  if (SMETER_RAW_AT_S9 <= SMETER_RAW_AT_S0) return 0;

  if (raw <= SMETER_RAW_AT_S0) return 0;
  if (raw >= SMETER_RAW_AT_S9) return 9;

  float t = float(raw - SMETER_RAW_AT_S0) / float(SMETER_RAW_AT_S9 - SMETER_RAW_AT_S0);
  int s = (int)(t * 9.0f + 0.5f);
  if (s < 0) s = 0;
  if (s > 9) s = 9;
  return (uint8_t)s;
}

void handleSMeterRaw(int32_t raw) {
  live.smRaw = raw;
  live.smValid = true;
  live.smS = smRawToS(raw);

  if (!g_quiet) {
    Serial.print("SM: raw=");
    Serial.print(raw);
    Serial.print("  est=S");
    Serial.println((int)live.smS);
  }

  if (!g_speechEnabled || !SMETER_SPEAK_ENABLE) return;

  const uint32_t now = millis();
  if (now - live.lastSmSpokenMs < SMETER_SPEAK_MIN_INTERVAL_MS) return;

  if (live.lastSpokenS == 0xFF) {
    live.lastSpokenS = live.smS;
    live.lastSmSpokenMs = now;
    speakSValue(live.smS);
    return;
  }

  uint8_t a = live.smS;
  uint8_t b = live.lastSpokenS;
  uint8_t diff = (a > b) ? (a - b) : (b - a);

  if (diff >= SMETER_SPEAK_MIN_DELTA_S) {
    live.lastSpokenS = a;
    live.lastSmSpokenMs = now;
    speakSValue(a);
  }
}

void pollSMeterIfDue() {
  if (!SMETER_POLL_ENABLE) return;
  const uint32_t now = millis();
  if (now - live.lastSmPollMs < SMETER_POLL_MS) return;
  live.lastSmPollMs = now;

  int32_t raw = 0;
  if (querySMeterRaw(raw)) {
    handleSMeterRaw(raw);
  } else {
    // no reply is OK; don't spam
  }
}

// ============================================================
// CI-V frame handler
// ============================================================

void handleIncomingFrame(const CivDecoded& d) {
  if (d.from != g_civRadioAddr) return;

  // Broadcast frequency: FE FE 00 94 00 <5 BCD> FD
  if (d.cmd == 0x00 && d.payloadLen >= 5) {
    uint64_t hz = decodeBcdFrequencyHz(d.payload, 5);

    live.freqHz = hz;
    live.freqValid = true;
    live.lastFreqMs = millis();

    if (!g_quiet) {
      Serial.print("FREQ: ");
      Serial.print(hzToMHzString3(hz));
      Serial.print(" MHz (");
      Serial.print(hz);
      Serial.println(" Hz)");
    }

    updateFreqSpeechDebounce(hz);
    return;
  }

  // Mode can be broadcast by some rigs when CI-V Transceive is ON.
  // Depending on model/firmware this may arrive as cmd 0x01 or 0x04.
  if ((d.cmd == 0x01 || d.cmd == 0x04) && d.payloadLen >= 1) {
    uint8_t m = d.payload[0];
    if (!live.modeValid || m != live.mode) {
      live.mode = m;
      live.modeValid = true;
      live.lastModeMs = millis();

      Serial.print("MODE: ");
      Serial.println(modeToString(m));

      speakMode(m);
    }
    return;
  }
}

void pumpIncoming(uint32_t maxMs) {
  uint32_t start = millis();
  uint8_t buf[96];
  while (millis() - start < maxMs) {
    if (!CIVSER.available()) break;
    size_t n = civReadFrame(buf, sizeof(buf), 20);
    if (!n) break;
    CivDecoded d = civDecode(buf, n);
    if (d.ok) handleIncomingFrame(d);
  }
}

// ============================================================
// CLI (case-insensitive)
// ============================================================

String readLine() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { String r = line; line = ""; r.trim(); return r; }
    line += c;
  }
  return "";
}

String upperCopy(String s) { s.toUpperCase(); return s; }

void printHelp() {
  Serial.println();
  Serial.println("Commands (case-insensitive):");
  Serial.println("  HELP");
  Serial.println("  QUIET ON | QUIET OFF");
  Serial.println("  SPEECH ON | SPEECH OFF");
  Serial.println("  LFREQ        -> show last live frequency");
  Serial.println("  FREQ?        -> query frequency (0x03)");
  Serial.println("  MODE?        -> query mode (0x04)");
  Serial.println("  SM?          -> query S-meter once");
  Serial.println("  SAY <digits> -> speak digits/point (e.g. SAY 13.207)");
  Serial.println("  TEST         -> voice samples test");
  Serial.println("  BANK 1|2|3   -> set keypad bank (A/B/C)");
  Serial.println("  LISTVOICES   -> list all available voice clips");
  Serial.println("  VOICE <name> -> play a clip (e.g. VOICE ok, VOICE voice_megahertz)");
  Serial.println();
}

void voiceTest() {
  Serial.println("Voice TEST...");
for (size_t i = 0; i < kVoiceClipsCount; i++) {
    Serial.print("  ");
    Serial.println(kVoiceClips[i].name);
    playClipProgmemBlocking(kVoiceClips[i].data, kVoiceClips[i].len);
    playSilenceMsBlocking(120);
  }
Serial.println("Voice TEST done.");
}


// ============================================================
// Arduino entry points
// ============================================================

static void playDigitsFromCString(const char* s) {
  if (!s) return;
  for (const char* p = s; *p; ++p) {
    if (*p >= '0' && *p <= '9') {
      playDigit((int)(*p - '0'));
    }
  }
}


static void speakBootProfile() {
  // Speak: manufacturer + digits + "ok"
  if (g_profileId == PROFILE_ID_G106_CAT) {
#ifdef HAS_VOICE_voice_xiegu
    playClipProgmem(voice_xiegu, voice_xiegu_len);
#else
    playClipProgmem(voice_icom, voice_icom_len);
#endif
  } else {
    playClipProgmem(voice_icom, voice_icom_len);
  }
  if (g_profileId == PROFILE_ID_7300) {
    playDigitsFromCString("7300");
  } else if (g_profileId == PROFILE_ID_G106_CAT) {
    playDigitsFromCString("106");
  } else if (g_profileId == PROFILE_ID_706_CIV || g_profileId == PROFILE_ID_706_RS232) {
    playDigitsFromCString("706");
  } else {
    playDigitsFromCString("7300");
  }

  playClipProgmem(voice_ok, voice_ok_len);
}


void setup() {
  Serial.begin(PC_BAUD);
  delay(200);

  if (AMP_SD_PIN >= 0) {
    pinMode(AMP_SD_PIN, OUTPUT);
}
  initI2S();

  // Start background audio task (keeps keypad responsive during speech)
  xTaskCreatePinnedToCore(audioTask, "audioTask", 4096, nullptr, 2, nullptr, 1);

  applyVolumeLevel(DEFAULT_VOLUME_LEVEL);

  // CI-V UART is configured via applyProfile() so profiles can select TTL vs RS232.
  // Restore last used profile (1..3) from NVS
  g_profileId = loadProfileFromNvs(g_profileId);

  
  g_tuningSpeakEnabled = loadTuningSpeakFromNvs(true);
applyProfile(g_profileId);

  Serial.print("IC-7300/706 Talking Remote Controller (V9.1.1) - Selected radio: ");
  Serial.print(currentProfile().name);
  Serial.print("  CI-V addr=0x");
  Serial.println(g_civRadioAddr, HEX);


// Schedule boot confirmation voice (MAX/I2S may not be ready immediately)
g_bootSpeakPending = true;
g_bootSpeakAtMs = millis() + 900;


  // Keypad timing (long press = bank change)
  keypad.setDebounceTime(KEYPAD_DEBOUNCE_MS);
  keypad.setHoldTime(KEYPAD_HOLD_MS);
  keypad.addEventListener(keypadEvent);
  printHelp();

  // initial state queries
  uint64_t hz;
  if (queryFrequency(hz)) {
    live.freqHz = hz;
    live.freqValid = true;
    live.lastFreqMs = millis();
    Serial.print("INIT FREQ: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
  } else {
    Serial.println("INIT FREQ: no reply (CI-V Transceive ON recommended)");
  }

  uint8_t m;
  if (queryMode(m)) {
    live.mode = m;
    live.modeValid = true;
    live.lastModeMs = millis();
    Serial.print("INIT MODE: ");
    Serial.println(modeToString(m));
  } else {
    Serial.println("INIT MODE: no reply");
  }
}


// ============================================================
// Keypad bank logic
// ============================================================

static void applyProfile(uint8_t profileId) {
  // ---- HARD SAFETY (universal front-end): make ALL ports passive first ----
  // Stop both UARTs and set TX pins to INPUT (hi-Z) so inactive interfaces cannot drive lines.
  civUart1.end();
  civUart2.end();
  pinMode(CIV_TX_PIN, INPUT);
  pinMode(RS232_TX_PIN, INPUT);
  pinMode(CAT_TX_PIN, INPUT);
  pinMode(CAT_RX_PIN, INPUT);

// Accept only known IDs (1..4), otherwise fall back to IC-7300
if (profileId == PROFILE_ID_G106_CAT)       g_profileId = PROFILE_ID_G106_CAT;
else if (profileId == PROFILE_ID_706_RS232) g_profileId = PROFILE_ID_706_RS232;
else if (profileId == PROFILE_ID_706_CIV)   g_profileId = PROFILE_ID_706_CIV;
else                                        g_profileId = PROFILE_ID_7300;

  const CivProfile& p = currentProfile();
  g_civRadioAddr = p.civAddr;

  // Select UART instance
  g_civSerial = (p.uartNum == 2) ? &civUart2 : &civUart1;

  // (Re)start UART with profile settings
  CIVSER.end();
  CIVSER.begin(p.baud, SERIAL_8N1, p.rxPin, p.txPin);

  // Apply UART line inversion if requested (useful with certain open-collector stages)
  uart_port_t up = (p.uartNum == 2) ? UART_NUM_2 : UART_NUM_1;
  uint32_t invMask = 0;
  if (p.txInvert) invMask |= UART_SIGNAL_TXD_INV;
  if (p.rxInvert) invMask |= UART_SIGNAL_RXD_INV;

  uart_set_line_inverse(up, UART_SIGNAL_INV_DISABLE);
  if (invMask) uart_set_line_inverse(up, invMask);

  // Persist selection (avoid frequent writes)
  if (g_profileId != g_lastSavedProfile) {
    saveProfileToNvs(g_profileId);
    g_lastSavedProfile = g_profileId;
  }

  Serial.print("[PROFILE] Active: ");
  Serial.print(p.name);
  Serial.print("  CI-V addr=0x");
  Serial.print(g_civRadioAddr, HEX);
  Serial.print("  UART");
  Serial.print(p.uartNum);
  Serial.print("  baud=");
  Serial.print(p.baud);
  Serial.print("  RX=");
  Serial.print(p.rxPin);
  Serial.print("  TX=");
  Serial.println(p.txPin);
}

static void speakCurrentProfile() {
  if (!g_speechEnabled) return;
// Say: "icom" + ("seven three zero zero" OR "seven zero six") + "ok"
  // If G106 profile is active and "voice_xiegu" exists, say "xiegu", else say "icom".
  if (g_profileId == PROFILE_ID_G106_CAT) {
#ifdef HAS_VOICE_voice_xiegu
    playClipProgmem(voice_xiegu, voice_xiegu_len);
    playSilenceMs(80);
#elif defined(HAS_VOICE_voice_icom)
    playClipProgmem(voice_icom, voice_icom_len);
    playSilenceMs(80);
#endif
  } else {
#ifdef HAS_VOICE_voice_icom
    playClipProgmem(voice_icom, voice_icom_len);
    playSilenceMs(80);
#endif
  }

  if (g_profileId == PROFILE_ID_7300) {
#ifdef HAS_VOICE_voice_seven
    playClipProgmem(voice_seven, voice_seven_len); playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_three
    playClipProgmem(voice_three, voice_three_len); playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_zero
    playClipProgmem(voice_zero, voice_zero_len); playSilenceMs(40);
    playClipProgmem(voice_zero, voice_zero_len); playSilenceMs(60);
#endif
  } else if (g_profileId == PROFILE_ID_G106_CAT) {
    // Speak "106"
#ifdef HAS_VOICE_voice_one
    playClipProgmem(voice_one, voice_one_len); playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_zero
    playClipProgmem(voice_zero, voice_zero_len); playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_six
    playClipProgmem(voice_six, voice_six_len); playSilenceMs(60);
#endif
  } else  { 
    // Both profile 2 and 3 are IC-706 (different physical link), so speak "706"
#ifdef HAS_VOICE_voice_seven
    playClipProgmem(voice_seven, voice_seven_len); playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_zero
    playClipProgmem(voice_zero, voice_zero_len); playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_six
    playClipProgmem(voice_six, voice_six_len); playSilenceMs(60);
#endif
  }

#ifdef HAS_VOICE_voice_ok
  playClipProgmem(voice_ok, voice_ok_len);
#endif
}

static void speakBankNumber() {
  if (!g_speechEnabled) return;
  speakToken("bank");
  playSilenceMs(60);
if (g_bank == 1) playClipProgmem(voice_one, voice_one_len);
  else if (g_bank == 2) playClipProgmem(voice_two, voice_two_len);
  else playClipProgmem(voice_three, voice_three_len);
}

static void keypadClearAll() {
  g_kpStagedCmd = "";
  g_kpHasStagedCmd = false;

  // cancel any modal flows
  g_freqEntryActive = false;
  g_freqEntryDigits = "";
  g_freqEntryIsMHz = false;

  g_modeSetActive = false;
  g_modeStageActive = false;
  g_modeStageMode = 0xFF;
  g_modeStageKey = 0;

  g_profileSelectActive = false;
  g_volStageActive = false;

  Serial.println("[KP] CLEAR");
  if (g_speechEnabled) playClipProgmem(voice_ok, voice_ok_len);
}


static void speakStagedCommand(const String& cmd) {
  if (!g_speechEnabled) return;

  String u = cmd;
  u.toUpperCase();

  // Speak a short confirmation of what was staged (no execution).
  // Use existing clips from voice_data.h; fall back to "OK".
  if (u == "FREQ?") {
    // We have a dedicated "frequency" word.
    playClipProgmem(voice_frequency, voice_frequency_len);
    return;
  }
  if (u == "SM?") {
    playClipProgmem(voice_s, voice_s_len);
    return;
  }
  if (u == "SWR?") {
    playClipProgmem(voice_swr, voice_swr_len);
    return;
  }
  if (u == "MODE?") {
#if defined(HAS_VOICE_voice_mode)
    playClipProgmem(voice_mode, voice_mode_len);
#else
    playClipProgmem(voice_ok, voice_ok_len);
#endif
    return;
  }
  if (u == "PO?" || u == "PWR?" || u == "POWER?") {
    playClipProgmem(voice_power, voice_power_len);
    return;
  }

  // For commands without a dedicated recorded word, confirm with OK
  playClipProgmem(voice_ok, voice_ok_len);
}

// Helper used by the keypad auto-send shortcuts (Bank 1 short presses).
// Thin wrapper so call sites can explicitly state what they request.
static inline void speakKeypadCommandWord(const char* cmd) {
  speakStagedCommand(String(cmd));
}


static void keypadStageCommand(const String& cmd) {
  g_kpStagedCmd = cmd;
  g_kpHasStagedCmd = true;
  Serial.print("[KP STAGE] ");
  Serial.println(cmd);
  speakStagedCommand(cmd);
}


static void keypadSendNow(const String& cmd) {
  Serial.print("[KP SEND] ");
  Serial.println(cmd);
  g_keypadExecuting = true;
  processCommand(cmd);
  g_keypadExecuting = false;
}

static void keypadEnter() {
  if (g_freqEntryActive) {
    if (!g_freqEntryDigits.length()) {
      Serial.println("[KP ENTER] freq entry empty");
      if (g_speechEnabled) playClipProgmem(voice_error, voice_error_len);
      return;
    }
    // interpret digits as MHz or kHz depending on entry mode
    uint64_t hz = 0;
    if (g_freqEntryIsMHz) {
      uint32_t mhz = (uint32_t) g_freqEntryDigits.toInt();
      hz = (uint64_t)mhz * 1000000ULL;
    } else {
      uint32_t khz = (uint32_t) g_freqEntryDigits.toInt();
      hz = (uint64_t)khz * 1000ULL;
    }

    Serial.print("[KP ENTER] SET FREQ ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");

    g_keypadExecuting = true;
    bool ok = setFrequency(hz);
    g_keypadExecuting = false;

    if (ok) {
      // Speak confirmation ONCE. Suppress the upcoming CI-V transceive broadcast from speaking again.
      g_suppressFreqSpeakUntilMs = millis() + 1500;
      if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    } else {
      Serial.println("SET FREQ -> no reply");
      if (g_speechEnabled) playClipProgmem(voice_error, voice_error_len);
    }

    // exit mode
    g_freqEntryActive = false;
    g_freqEntryDigits = "";
    g_freqEntryIsMHz = false;
    return;
  }


  // Apply staged MODE (Bank1: 9 HOLD, then digit 1..9, then Enter to apply)
  if (g_modeStageActive) {
    Serial.print("[KP ENTER] APPLY MODE key ");
    Serial.println(g_modeStageKey ? String(g_modeStageKey) : String('?'));
    setMode(g_modeStageMode, 1);

    if (g_speechEnabled) {
// confirm with spoken mode name if available
      if (g_modeStageKey == '1') {
#ifdef HAS_VOICE_voice_lsb
        playClipProgmem(voice_lsb, voice_lsb_len);
#endif
      } else if (g_modeStageKey == '2') {
#ifdef HAS_VOICE_voice_usb
        playClipProgmem(voice_usb, voice_usb_len);
#endif
      } else if (g_modeStageKey == '3') {
#ifdef HAS_VOICE_voice_cw
        playClipProgmem(voice_cw, voice_cw_len);
#endif
      } else if (g_modeStageKey == '4') {
#ifdef HAS_VOICE_voice_am
        playClipProgmem(voice_am, voice_am_len);
#endif
      } else if (g_modeStageKey == '5') {
#ifdef HAS_VOICE_voice_fm
        playClipProgmem(voice_fm, voice_fm_len);
#endif
      } else if (g_modeStageKey == '6') {
#ifdef HAS_VOICE_voice_digi
        playClipProgmem(voice_digi, voice_digi_len);
#else
#ifdef HAS_VOICE_voice_usb
        playClipProgmem(voice_usb, voice_usb_len);
#endif
#endif
      } else if (g_modeStageKey == '7') {
#ifdef HAS_VOICE_voice_rtty
        playClipProgmem(voice_rtty, voice_rtty_len);
#endif
      } else if (g_modeStageKey == '8') {
#ifdef HAS_VOICE_voice_cwr
        playClipProgmem(voice_cwr, voice_cwr_len);
#endif
      } else if (g_modeStageKey == '9') {
#ifdef HAS_VOICE_voice_rttyr
        playClipProgmem(voice_rttyr, voice_rttyr_len);
#endif
      }
#ifdef HAS_VOICE_voice_ok
      playSilenceMs(60);
      playClipProgmem(voice_ok, voice_ok_len);
#endif
}

    g_modeStageActive = false;
    g_modeStageMode = 0xFF;
    g_modeStageKey = 0;
    return;
  }

  // Apply staged volume (Bank3: 0/7/8/9 then Enter)
  if (g_volStageActive) {
    Serial.print("[KP ENTER] APPLY VOLUME level ");
    Serial.println((int)g_volStageLevel);
    applyVolumeLevel(g_volStageLevel);
    if (g_speechEnabled) {
      // confirm with spoken level (1..4)
speakVolumeLevel(g_volStageLevel);
}
    g_volStageActive = false;
    return;
  }

  if (!g_kpHasStagedCmd) {
    Serial.println("[KP ENTER] (no staged cmd)");
    if (g_speechEnabled) playClipProgmem(voice_error, voice_error_len);
    return;
  }
  Serial.print("[KP ENTER] ");
  Serial.println(g_kpStagedCmd);
  g_keypadExecuting = true;
  processCommand(g_kpStagedCmd);
  g_keypadExecuting = false;
  g_kpStagedCmd = "";
  g_kpHasStagedCmd = false;
}

static void keypadHandleReleased(char k) {
  // '#' = clear, 'D' = enter handled elsewhere
  if (k == '#') { keypadClearAll(); return; }

  // MODE SET flow: after Bank1 '9' HOLD, next digit 1..9 stages the mode (apply with Enter)
  if (g_modeSetActive) {
    if (k == '#') { g_modeSetActive = false; return; }

    uint8_t mode = 0xFF;
    switch (k) {
      case '1': mode = 0x00; break; // LSB
      case '2': mode = 0x01; break; // USB
      case '3': mode = 0x03; break; // CW
      case '4': mode = 0x02; break; // AM
      case '5': mode = 0x05; break; // FM
      case '6': mode = 0x01; break; // DIGI (maps to USB for now)
      case '7': mode = 0x04; break; // RTTY
      case '8': mode = 0x07; break; // CWR
      case '9': mode = 0x08; break; // RTTYR
      default: break;
    }

    if (mode != 0xFF) {
      g_modeStageActive = true;
      g_modeStageMode = mode;
      g_modeStageKey  = k;

      Serial.print("[KP] MODE STAGED via key ");
      Serial.print(k);
      Serial.println(" — press D/Enter to apply.");

      if (g_speechEnabled) {
// say the selected mode immediately (staged)
        if (k=='1') {
#ifdef HAS_VOICE_voice_lsb
          playClipProgmem(voice_lsb, voice_lsb_len);
#endif
        } else if (k=='2') {
#ifdef HAS_VOICE_voice_usb
          playClipProgmem(voice_usb, voice_usb_len);
#endif
        } else if (k=='3') {
#ifdef HAS_VOICE_voice_cw
          playClipProgmem(voice_cw, voice_cw_len);
#endif
        } else if (k=='4') {
#ifdef HAS_VOICE_voice_am
          playClipProgmem(voice_am, voice_am_len);
#endif
        } else if (k=='5') {
#ifdef HAS_VOICE_voice_fm
          playClipProgmem(voice_fm, voice_fm_len);
#endif
        } else if (k=='6') {
#ifdef HAS_VOICE_voice_digi
          playClipProgmem(voice_digi, voice_digi_len);
#else
#ifdef HAS_VOICE_voice_usb
          playClipProgmem(voice_usb, voice_usb_len);
#endif
#endif
        } else if (k=='7') {
#ifdef HAS_VOICE_voice_rtty
          playClipProgmem(voice_rtty, voice_rtty_len);
#endif
        } else if (k=='8') {
#ifdef HAS_VOICE_voice_cwr
          playClipProgmem(voice_cwr, voice_cwr_len);
#endif
        } else if (k=='9') {
#ifdef HAS_VOICE_voice_rttyr
          playClipProgmem(voice_rttyr, voice_rttyr_len);
#endif
        }
#ifdef HAS_VOICE_voice_enter
        playSilenceMs(60);
        playClipProgmem(voice_enter, voice_enter_len);
#endif
}
    } else {
      if (g_speechEnabled) playClipProgmem(voice_error, voice_error_len);
    }

    g_modeSetActive = false;
    return;
  }

  // PROFILE SELECT flow: after Bank3 'A' HOLD, next key 1/2/3 selects profile
  if (g_profileSelectActive) {
    if (k == '#') { g_profileSelectActive = false; return; }
    if (k == '1') { applyProfile(PROFILE_ID_7300);        speakCurrentProfile(); g_profileSelectActive = false; return; }
    if (k == '2') { applyProfile(PROFILE_ID_706_CIV);   speakCurrentProfile(); g_profileSelectActive = false; return; }
    if (k == '3') { applyProfile(PROFILE_ID_706_RS232); speakCurrentProfile(); g_profileSelectActive = false; return; }
    if (k == '4') { applyProfile(PROFILE_ID_G106_CAT);    speakCurrentProfile(); g_profileSelectActive = false; return; }
    if (g_speechEnabled) playClipProgmem(voice_error, voice_error_len);
    return;
  }


  // digits during freq entry
  if (g_freqEntryActive) {
    if (k >= '0' && k <= '9') {
      if (g_freqEntryDigits.length() < 9) {
        g_freqEntryDigits += k;
        Serial.print("[KP FREQ] ");
        Serial.println(g_freqEntryDigits);
        // Immediate audible confirmation for blind operation
        if (g_speechEnabled) speakDigitsAndPoint(String(k));
      }
      return;
    }
    // ignore A/B/C in freq entry for now
    return;
  }

  // normal staging by bank
  if (g_bank == 1) {
    switch (k) {
      case '0':
        if (AUTO_SEND_BANK1_QUERIES) {
          if (g_speechEnabled) { speakKeypadCommandWord("FREQ?"); playSilenceMs(60); }
          keypadSendNow("FREQ?");
          return;
        }
        keypadStageCommand("FREQ?"); return;
      case '7':
        if (AUTO_SEND_BANK1_QUERIES) {
          if (g_speechEnabled) { speakKeypadCommandWord("SM?"); playSilenceMs(60); }
          keypadSendNow("SM?");
          return;
        }
        keypadStageCommand("SM?");   return;
      case '8':
        if (AUTO_SEND_BANK1_QUERIES) {
          if (g_speechEnabled) { speakKeypadCommandWord("SWR?"); playSilenceMs(60); }
          keypadSendNow("SWR?");
          return;
        }
        keypadStageCommand("SWR?");  return;
      case '9':
        // Short press = query mode, HOLD is reserved for MODE SET flow
        if (AUTO_SEND_BANK1_QUERIES) {
          if (g_speechEnabled) { speakKeypadCommandWord("MODE?"); playSilenceMs(60); }
          g_suppressModePrefixOnce = true;
          keypadSendNow("MODE?");
          return;
        }
        keypadStageCommand("MODE?"); return;
      case '4':
        if (AUTO_SEND_BANK1_QUERIES) {
          if (g_speechEnabled) { speakKeypadCommandWord("PO?"); playSilenceMs(60); }
          keypadSendNow("PO?");
          return;
        }
        keypadStageCommand("PO?");   return;
      default: break;
    }
  } else if (g_bank == 2) {
    // Bank 2: reserved (frequency entry moved to Bank 1: hold '0')
  } else if (g_bank == 3) {
    // Bank C: Volume staging (0 very low, 7 low, 8 medium, 9 loud) then Enter(D) to apply
    uint8_t lvl;
    bool mapped = true;
    switch (k) {
      case '0': lvl = 0; break;
      case '7': lvl = 1; break;
      case '8': lvl = 2; break;
      case '9': lvl = 3; break;
      default: mapped = false; break;
    }
    if (mapped) {
      g_volStageActive = true;
      g_volStageLevel  = lvl;
      Serial.print("[KP] VOLUME STAGED level ");
      Serial.print((int)lvl);
      Serial.println(" (0=very low,1=low,2=medium,3=loud). Press D/Enter to apply.");
      if (g_speechEnabled) {
        // say the level (1..4) immediately, like other staged commands
speakVolumeLevel(lvl);
}
      return;
    }
  }

  // If unmapped, ignore silently
}

void keypadEvent(KeypadEvent k) {
  KeyState s = keypad.getState();
  // Abort speech ONLY when a NEW key is pressed (not on RELEASE/HOLD events).
  // Otherwise the release of the same key would immediately stop the prompt.
  if (s == PRESSED && g_audioPlaying) { audioAbortNow(); }

  if (k == 'D' && s == RELEASED) {
    keypadEnter();
    return;
  }

  if (k == '*') {
    if (s == HOLD) {
      // cycle bank (long press)
      g_bank++;
      if (g_bank > 3) g_bank = 1;
      g_starHoldConsumed = true; // suppress the following RELEASED event speech
      Serial.print("[KP] BANK -> ");
      Serial.println(g_bank);
      speakBankNumber();
    } else if (s == RELEASED) {
      if (g_starHoldConsumed) {
        g_starHoldConsumed = false;
        return;
      }
      // short press: speak current bank
      Serial.print("[KP] BANK = ");
      Serial.println(g_bank);
      speakBankNumber();
    }
    return;
  }


  
  // Bank 1: 'A' short -> volume down, 'A' hold -> volume up
  if (g_bank == 1 && k == 'A') {
    if (s == HOLD) {
      // louder
      if (g_volumeLevel < 3) applyVolumeLevel(g_volumeLevel + 1);
      else applyVolumeLevel(3);
      g_aHoldConsumed = true; // suppress following RELEASED
      if (g_speechEnabled) { audioAmpOn(); delay(5); speakVolumeLevel(g_volumeLevel); audioAmpOff(); }
      return;
    } else if (s == RELEASED) {
      if (g_aHoldConsumed) { g_aHoldConsumed = false; return; }
      // quieter
      if (g_volumeLevel > 0) applyVolumeLevel(g_volumeLevel - 1);
      else applyVolumeLevel(0);
      if (g_speechEnabled) { audioAmpOn(); delay(5); speakVolumeLevel(g_volumeLevel); audioAmpOff(); }
      return;
    }
  }

// Bank 1: HOLD '9' -> enter MODE SET flow (digit 1..9 sets mode)
  if (g_bank == 1 && k == '9') {
    if (s == HOLD) {
      g_modeSetActive = true;
      g_nineHoldConsumed = true;
      Serial.println("[KP] MODE SET: awaiting digit 1..9");
      if (g_speechEnabled) {
#ifdef HAS_VOICE_voice_mode
        playClipProgmem(voice_mode, voice_mode_len);
        playSilenceMs(80);
#endif
#ifdef HAS_VOICE_voice_please
        playClipProgmem(voice_please, voice_please_len);
#endif
}
      return;
    }
    if (s == RELEASED && g_nineHoldConsumed) {
      g_nineHoldConsumed = false;
      return;
    }
  }

  // Bank 3: CI-V profile (moved to 'A' to keep '0' for volume)
  if (g_bank == 3 && k == 'A') {
    if (s == HOLD) {
      g_profileSelectActive = true;
      Serial.println("[KP] PROFILE SELECT: press 1=IC-7300, 2=IC-706 (CI-V), 3=IC-706 (RS232), 4=Xiegu G106");
      if (g_speechEnabled) {
#ifdef HAS_VOICE_voice_choose
        playClipProgmem(voice_choose, voice_choose_len);
        playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_please
        playClipProgmem(voice_please, voice_please_len);
#endif
}
      return;
    }
    if (s == RELEASED) {
      speakCurrentProfile();
      return;
    }
  }


  // Bank 3: Toggle VFO/tuning frequency announcements (so turning the knob doesn't always talk)
  // Short press 'B' toggles ON/OFF, HOLD speaks the current state.
  if (g_bank == 3 && k == 'B') {
    if (s == HOLD) {
      if (g_speechEnabled) {
#ifdef HAS_VOICE_voice_frequency
        playClipProgmem(voice_frequency, voice_frequency_len);
        playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_on
#ifdef HAS_VOICE_voice_off
        if (g_tuningSpeakEnabled) playClipProgmem(voice_on, voice_on_len);
        else playClipProgmem(voice_off, voice_off_len);
#endif
#endif
      }
      return;
    }
    if (s == RELEASED) {
      g_tuningSpeakEnabled = !g_tuningSpeakEnabled;
      saveTuningSpeakToNvs(g_tuningSpeakEnabled);
      Serial.print("[KP] TUNING SPEAK -> ");
      Serial.println(g_tuningSpeakEnabled ? "ON" : "OFF");
      if (g_speechEnabled) {
#ifdef HAS_VOICE_voice_frequency
        playClipProgmem(voice_frequency, voice_frequency_len);
        playSilenceMs(60);
#endif
#ifdef HAS_VOICE_voice_on
#ifdef HAS_VOICE_voice_off
        if (g_tuningSpeakEnabled) playClipProgmem(voice_on, voice_on_len);
        else playClipProgmem(voice_off, voice_off_len);
#endif
#endif
      }
      return;
    }
  }

  // Long-press '0' in Bank 1: start kHz frequency entry
  if (k == '0') {
    if (s == HOLD && g_bank == 1) {
      if (!g_freqEntryActive) {
        g_freqEntryActive = true;
        g_freqEntryIsMHz = false;
        g_freqEntryDigits = "";
        g_zeroHoldConsumed = true; // suppress RELEASED staging of '0'
        Serial.println("[KP] FREQ ENTRY (kHz). Type digits (e.g. 14070 for 14.070MHz), D=Enter, #=Clear.");
        if (g_speechEnabled) {
playClipProgmem(voice_frequency, voice_frequency_len);
          playSilenceMs(80);
          playClipProgmem(voice_please, voice_please_len);
}
      }
      return;
    }
    if (s == RELEASED && g_zeroHoldConsumed) {
      g_zeroHoldConsumed = false;
      return;
    }
  }

  if (s == RELEASED) {
    keypadHandleReleased((char)k);
  }
}

void loop() {// Boot confirmation voice (delayed)
if (g_bootSpeakPending && (int32_t)(millis() - g_bootSpeakAtMs) >= 0) {
  g_bootSpeakPending = false;
  speakBootProfile();
}


  pumpIncoming(CIV_PUMP_BUDGET_MS);

  // speak end-frequency once tuning is idle
  speakPendingFreqIfIdle();

  // poll S-meter (non-blocking)
  pollSMeterIfDue();

  // --- Keypad (event-driven) ---
  // NOTE: We call getKey() to drive the library's state machine.
  // Actual actions are handled in keypadEvent().
  (void)keypad.getKey();

  // --- Serial Monitor line input ---

  String line = readLine();
  if (!line.length()) return;
  processCommand(line);
}

static void processCommand(String line) {
  line.trim();
  if (!line.length()) return;
  if (g_audioPlaying) { audioAbortNow(); }
  String upper = upperCopy(line);
if (upper == "HELP") { printHelp(); return; }

  

  if (upper == "LISTVOICES") {
    for (size_t i = 0; i < kVoiceClipsCount; i++) {
      Serial.println(kVoiceClips[i].name);
    }
    return;
  }

  if (upper.startsWith("VOICE ")) {
    String tok = line.substring(6);
    tok.trim();
    Serial.print("VOICE: "); Serial.println(tok);
speakToken(tok);
return;
  }

  if (upper.startsWith("BANK ")) {
    int b = line.substring(5).toInt();
    if (b < 1 || b > 3) { Serial.println("ERR BANK (use 1..3)"); speakError(); return; }
    g_bank = (uint8_t)b;
    Serial.print("OK BANK "); Serial.println(g_bank);
    speakBankNumber();
    return;
  }
if (upper == "QUIET ON")  { g_quiet = true;  Serial.println("OK QUIET ON"); return; }
  if (upper == "QUIET OFF") { g_quiet = false; Serial.println("OK QUIET OFF"); return; }

  if (upper == "SPEECH ON")  { g_speechEnabled = true;  Serial.println("OK SPEECH ON"); return; }
  if (upper == "SPEECH OFF") { g_speechEnabled = false; Serial.println("OK SPEECH OFF"); return; }

  if (upper == "LFREQ") {
    if (!live.freqValid) { Serial.println("No live frequency yet."); return; }
    Serial.print("Last live: ");
    Serial.print(hzToMHzString3(live.freqHz));
    Serial.print(" MHz (age ");
    Serial.print((millis() - live.lastFreqMs) / 1000);
    Serial.println(" s)");
    return;
  }

  if (upper == "FREQ?") {
    uint64_t hz2 = 0;
    if (!queryFrequency(hz2)) { Serial.println("FREQ? -> no reply"); return; }
    Serial.print("Query FREQ: ");
    Serial.print(hzToMHzString3(hz2));
    Serial.println(" MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz2));
    return;
  }

  if (upper == "MODE?") {
    uint8_t m = 0xFF;
    if (!queryMode(m)) { Serial.println("MODE? -> no reply"); return; }
    Serial.print("Query MODE: ");
    Serial.println(modeToString(m));
    // If MODE? was staged, we already spoke the word "mode" on key-press.
    if (g_keypadExecuting) g_suppressModePrefixOnce = true;
    // speaks only if you enable HAVE_MODE_VOICE and add samples
    speakMode(m);
    return;
  }

  if (upper == "SM?") {
    int32_t raw = 0;
    if (!querySMeterRaw(raw)) { Serial.println("SM? -> no reply"); return; }
    Serial.print("SM: raw=");
    Serial.print(raw);
    Serial.print("  est=S");
    Serial.println((int)smRawToS(raw));
    // Speak ONLY when explicitly requested with SM?
    if (g_speechEnabled) {
      uint8_t s = smRawToS(raw);
      speakSValue(s);
    }
    return;
  }

  
  if (upper == "SWR?") {
    int32_t raw = 0;
    if (!querySWRRaw(raw)) { Serial.println("SWR? -> no reply"); return; }
    float swr = swrRawToValue(raw);
    Serial.print("SWR: raw=");
    Serial.print(raw);
    Serial.print("  est=");
    Serial.println(swr, 2);
    if (g_speechEnabled) {
      if (!g_keypadExecuting) playClipProgmem(voice_swr, voice_swr_len);
      // speak like: "two point zero" (very rough)
      speakDigitsAndPoint(String(swr, 2));
    }
    return;
  }

  if (upper == "PO?") {
    int32_t raw = 0;
    if (!queryPoMeterRaw(raw)) { Serial.println("PO? -> no reply"); return; }
    Serial.print("PO: raw=");
    Serial.println(raw);
    if (g_speechEnabled) {
      if (!g_keypadExecuting) playClipProgmem(voice_power, voice_power_len);
      speakDigitsAndPoint(String(raw));
    }
    return;
  }

if (upper == "TEST") { voiceTest(); return; }

  if (upper.startsWith("SAY ")) {
    String t = line.substring(4);
    t.trim();
    Serial.print("SAY: ");
    Serial.println(t);
    if (g_speechEnabled) speakDigitsAndPoint(t);
    return;
  }

  Serial.println("Unknown. Type HELP");
}