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

void printKeypadStatus(const String& line) {
  if ((bool)Serial) Serial.println(line);
}

void printKeypadCommand(const String& line) {
  if ((bool)Serial) {
    Serial.print("CMD ");
    Serial.println(line);
  }
}

void printKeypadAction(const String& what) {
  const char* key = keypadActiveKey();
  printKeypadCommand(key ? String(key) + " -> " + what : what);
}

bool keypadReportIfTimedOut(const char* label) {
  if (!g_radioReplyTimedOut) return false;
  printKeypadStatus(String(label) + " -> timeout");
  if (g_speechEnabled) speakTimeout();
  return true;
}

bool keypadReportFeatureFailure(FeatureStatus status, const char* label) {
  switch (status) {
    case FeatureStatus::Ok: return false;
    case FeatureStatus::Unsupported: return keypadReportIfUnsupported(false, label);
    case FeatureStatus::Timeout:
      printKeypadStatus(String(label) + " -> timeout");
      if (g_speechEnabled) speakTimeout();
      return true;
    default:
      printKeypadStatus(String(label) + " -> " + featureStatusText(status));
      if (g_speechEnabled) speakError();
      return true;
  }
}

void keypadReportUnassigned(const String& label) {
  printKeypadStatus(label + " -> unassigned");
  playBeep();
}

bool keypadReportIfUnsupported(bool supported, const char* label) {
  if (supported) return false;
  printKeypadStatus(String(label) + " -> unsupported");
  playBeep();
  return true;
}

void reportFtdx10HiddenKey() {
  const char* key = keypadActiveKey();
  printKeypadStatus(String(key ? key : "KEY") + " hidden on FTDX10");
  if (g_speechEnabled) speakNotAvailable();
}

bool isFtdx10KeypadProfile() {
  const StoredProfile& sp = currentStoredProfile();
  return sp.protocolType == PROTO_YAESU_FTDX_ASCII &&
         strcmp(sp.voiceVendor, "yaesu") == 0 &&
         strcmp(sp.voiceDigits, "10") == 0;
}

bool isFt8x7Ft817Keypad() {
  return currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817");
}

bool isFt8x7Ft857FamilyKeypad() {
  return currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897");
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
  speakToken("c");
  playSilenceMs(50);
  speakToken("i");
  playSilenceMs(80);
  speakHexNibble(hex[0]);
  playSilenceMs(50);
  speakHexNibble(hex[1]);
  if (ok) {
    playSilenceMs(80);
    speakOk();
  }
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

void speakVfoLabel(char which) {
  if (!g_speechEnabled) return;
  speakToken("vfo");
  playSilenceMs(60);
  if (which == 'A') speakToken("a");
  else if (which == 'B') speakToken("b");
}

void speakVfoFrequencyLabel(char which) {
  if (!g_speechEnabled) return;
  speakVfoLabel(which);
  playSilenceMs(60);
  speakFrequencyWord();
}

void speakVfoFrequency(char which, uint64_t hz) {
  if (!g_speechEnabled) return;
  speakVfoFrequencyLabel(which);
  playSilenceMs(60);
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
  // Reads the FT-857/897 lock, so one set on the front panel counts too. No reply: let the
  // toggle try.
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

void speakQueriedFrequencyHz(uint64_t hz) {
  if (!g_speechEnabled) return;
  speakFrequencyWord();
  playSilenceMs(60);
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
  playClipProgmem(featureData, featureLen);
  playSilenceMs(60);
  speakDigitsAndPoint(String((int)value));
}

bool lightIcomFallbackActive() {
  return getLastSdLoadStatus() != SD_LOAD_OK;
}

void speakKeypadCommandWord(const String& cmd) {
  if (!g_speechEnabled) return;
  if (cmd == "FREQ?") speakFrequencyWord();
  else if (cmd == "MODE?") speakToken("mode");
  else if (cmd == "SM?") speakToken("s_meter");
  else if (cmd == "SWR?") speakToken("swr");
  else if (cmd == "RFPOWER?") speakToken("power");
  else if (cmd == "NOTCH?") speakToken("notch filter");
}

void sendKeypadCommand(const char* cmd) {
  printKeypadAction(cmd);
  keypadSendNow(cmd);
}
