#include "ui_keypad_common.h"
#include "ui_features.h"

#include "radio_catalog.h"
#include "radio_monitor.h"
#include "radio_protocol.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "sd_slots.h"
#include "ui_speech.h"

static constexpr uint32_t KEYPAD_POLL_SUSPEND_MS = 900;
static constexpr uint32_t KEYPAD_WRITE_POLL_SUSPEND_MS = 1400;
static constexpr uint32_t KEYPAD_ANSWER_SPEECH_QUIET_MS = 2000;

bool keypadReportIfTimedOut(const char* label) {
  if (!g_radioReplyTimedOut) return false;
  printKeypadStatus("{} -> timeout", label);
  if (g_speechEnabled) speakTimeout();
  return true;
}

void queryKeypadFt8x7Setting(Ft8x7Setting setting) {
  const char* label = ft8x7SettingLabel(setting);
  printKeypadAction("{}", label);
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  if (keypadReportFeatureFailure(ft8x7SettingQuery(setting, state), label)) return;
  printKeypadStatus("{}", ft8x7SettingText(state).c_str());
  speakFt8x7Setting(state);
}

bool keypadReportFeatureFailure(FeatureStatus status, const char* label) {
  switch (status) {
    case FeatureStatus::Ok: return false;
    case FeatureStatus::Unsupported: return keypadReportIfUnsupported(false, label);
    case FeatureStatus::Timeout:
      printKeypadStatus("{} -> timeout", label);
      if (g_speechEnabled) speakTimeout();
      return true;
    default:
      printKeypadStatus("{} -> {}", label, featureStatusText(status));
      if (g_speechEnabled) speakError();
      return true;
  }
}

void keypadReportUnassigned(const char* label) {
  printKeypadStatus("{} -> unassigned", label);
  playBeep();
}

bool keypadReportIfUnsupported(bool supported, const char* label) {
  if (supported) return false;
  printKeypadStatus("{} -> unsupported", label);
  playBeep();
  return true;
}

void reportFtdx10HiddenKey() {
  const char* key = keypadActiveKey();
  printKeypadStatus("{} hidden on FTDX10", key ? key : "KEY");
  if (g_speechEnabled) speakNotAvailable();
}

bool isFtdx10KeypadProfile() {
  const StoredProfile& sp = currentStoredProfile();
  return sp.protocolType == PROTO_YAESU_FTDX_ASCII &&
         strcmp(sp.voiceVendor, "yaesu") == 0 &&
         strcmp(sp.voiceDigits, "10") == 0;
}

bool isFt8x7Ft817Keypad() {
  return currentIsFt817Family();
}

bool isFt8x7Ft857FamilyKeypad() {
  return currentIsFt857Family();
}

static bool tracksFt8x7Vfo() {
  return isFt8x7Ft817Keypad() || isFt8x7Ft857FamilyKeypad();
}

// Without a reply (or on the FT-817, which has no readback) the tracked VFO stays, starting at A.
void refreshFt8x7ActiveVfo() {
  if (!tracksFt8x7Vfo()) return;
  bool vfoA = true;
  if (queryActiveVfo(vfoA, 300)) return;
  if (!live.activeVfoKnown) rememberActiveVfo(true);
}

char ft8x7CurrentVfoLabel() {
  if (!tracksFt8x7Vfo()) return '?';
  if (!live.activeVfoKnown) rememberActiveVfo(true);
  return live.activeVfoA ? 'A' : 'B';
}

char ft8x7OtherVfoLabel() {
  if (!tracksFt8x7Vfo()) return '?';
  if (!live.activeVfoKnown) rememberActiveVfo(true);
  return live.activeVfoA ? 'B' : 'A';
}

void formatHexByte(uint8_t value, char* out, size_t outSize) {
  if (!out || outSize < 3) return;
  snprintf(out, outSize, "%02X", (unsigned)value);
}

void speakHexNibble(char c) {
  if (c >= '0' && c <= '9') {
    playDigit(c - '0');
    return;
  }
  switch (c) {
    case 'A': speakToken("a"); break;
    case 'B': speakToken("b"); break;
    case 'C': speakToken("c"); break;
    case 'D': speakToken("d"); break;
    case 'E': speakError(); break;
    case 'F': speakToken("f"); break;
    default: speakError(); break;
  }
}

void speakCivAddressValue(uint8_t addr, bool ok) {
  if (!g_speechEnabled) return;
  char hex[3] = "";
  formatHexByte(addr, hex, sizeof(hex));
  if (g_verboseSpeech) {
    speakToken("c");
    playSilenceMs(50);
    speakToken("i");
    playSilenceMs(80);
  }
  speakHexNibble(hex[0]);
  playSilenceMs(50);
  speakHexNibble(hex[1]);
  if (ok) speakValueOk();
}

void formatCtcssTenthsLabel(uint16_t toneTenths, char* out, size_t outSize) {
  if (!out || outSize < 2) return;
  snprintf(out, outSize, "%u.%u", (unsigned)(toneTenths / 10), (unsigned)(toneTenths % 10));
}

void speakFrequencyWord() {
  if (!g_speechEnabled) return;
  speakToken("frequency");
}

void speakPlease() {
  if (!g_speechEnabled) return;
  playSilenceMs(20);
  speakToken("please");
}

void speakPrompt(const char* token) {
  if (!g_speechEnabled) return;
  speakToken(token);
  speakPlease();
}

static void speakVfoLetter(char which) {
  if (which == 'A') speakToken("a");
  else if (which == 'B') speakToken("b");
}

// The letter is the value, so verbose off keeps it.
void speakVfoLabel(char which) {
  if (!g_speechEnabled) return;
  speakLabel("vfo");
  speakVfoLetter(which);
}

// A prompt: the same words with verbose off.
void speakVfoFrequencyLabel(char which) {
  if (!g_speechEnabled) return;
  speakToken("vfo");
  playSilenceMs(60);
  speakVfoLetter(which);
  playSilenceMs(60);
  speakFrequencyWord();
}

// Verbose off: "a 7.1".
void speakVfoFrequency(char which, uint64_t hz) {
  if (!g_speechEnabled) return;
  speakVfoLabel(which);
  playSilenceMs(60);
  speakLabel("frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

void holdKeypadPolling() {
  g_suspendPollingUntilMs = millis() + KEYPAD_POLL_SUSPEND_MS;
}

void prepareKeypadSpeechResponse() {
  holdKeypadPolling();
  g_suppressFreqSpeakUntilMs = millis() + KEYPAD_ANSWER_SPEECH_QUIET_MS;
  cancelPendingFreqAnnouncement();
}

void prepareKeypadRadioWrite() {
  g_suspendPollingUntilMs = millis() + KEYPAD_WRITE_POLL_SUSPEND_MS;
  g_suppressFreqSpeakUntilMs = millis() + KEYPAD_ANSWER_SPEECH_QUIET_MS;
  cancelPendingFreqAnnouncement();
}

bool guardFt8x7VfoToggleLock() {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return true;
  // Reads the radio's lock, so one set on the front panel counts too. No reply: let the toggle
  // try.
  bool on = false;
  if (!queryDialLock(on, 300) || !on) return true;
  printKeypadStatus("LOCK ON");
  if (g_speechEnabled) speakTokenState("lock", true);
  return false;
}

void speakSimpleBinaryState(bool on) {
  if (!g_speechEnabled) return;
  playClipProgmem(on ? voice_on : voice_off, on ? voice_on_len : voice_off_len);
}

void speakRxTxState(bool tx) {
  if (!g_speechEnabled) return;
  speakLabel("transceiver");
  speakToken(tx ? "tx" : "rx");
}

void speakQueriedFrequencyHz(uint64_t hz) {
  if (!g_speechEnabled) return;
  speakLabel("frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

uint8_t levelRawToPercent(uint16_t raw) {
  if (raw >= 255) return 100;
  return (uint8_t)((raw * 100U + 127U) / 255U);
}

uint16_t levelPercentToRaw(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return (uint16_t)((percent * 255 + 50) / 100);
}

void speakFeatureValue(const uint8_t* featureData, size_t featureLen, uint8_t value) {
  if (!g_speechEnabled) return;
  speakLabelClip(featureData, featureLen);
  speakDigitsAndPoint(String((int)value));
}

bool lightIcomFallbackActive() {
  return getLastSdLoadStatus() != SD_LOAD_OK;
}

void speakKeypadCommandWord(const String& cmd) {
  if (!g_speechEnabled) return;
  if (cmd == "FREQ?") speakLabel("frequency");
  else if (cmd == "MODE?") speakLabel("mode");
  else if (cmd == "SM?") speakLabel("s_meter");
  else if (cmd == "SWR?") speakLabel("swr");
  else if (cmd == "RFPOWER?") speakLabel("power");
  else if (cmd == "NOTCH?") speakLabel("notch filter");
}

void sendKeypadCommand(const char* cmd) {
  printKeypadAction("{}", cmd);
  keypadSendNow(cmd);
}
