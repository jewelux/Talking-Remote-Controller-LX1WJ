#include "ui_keypad_common.h"

#include "radio_catalog.h"
#include "radio_state.h"
#include "ui_speech.h"

void printKeypadStatus(const String& line) {
  if ((bool)Serial) Serial.println(line);
}

void printKeypadCommand(const String& line) {
  if ((bool)Serial) {
    Serial.print("CMD ");
    Serial.println(line);
  }
}

bool keypadReportIfTimedOut(const char* label) {
  if (!g_radioReplyTimedOut) return false;
  printKeypadStatus(String(label) + " -> timeout");
  if (g_speechEnabled) speakTimeout();
  return true;
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

void reportFtdx10HiddenKey(const char* label) {
  printKeypadStatus(String(label) + " hidden on FTDX10");
  if (g_speechEnabled) speakNotAvailable();
}

bool isFtdx10KeypadProfile() {
  const StoredProfile& sp = currentStoredProfile();
  return sp.protocolType == PROTO_YAESU_FTDX_ASCII &&
         strcmp(sp.voiceVendor, "yaesu") == 0 &&
         strcmp(sp.voiceDigits, "10") == 0;
}

bool isFt8x7Keypad() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
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

void ensureFt8x7VfoTrackingInitialized() {
  if (tracksFt8x7Vfo() && !live.activeVfoKnown) rememberActiveVfo(true);
}

char ft8x7CurrentVfoLabel() {
  ensureFt8x7VfoTrackingInitialized();
  if (!tracksFt8x7Vfo()) return '?';
  return live.activeVfoA ? 'A' : 'B';
}

char ft8x7OtherVfoLabel() {
  ensureFt8x7VfoTrackingInitialized();
  if (!tracksFt8x7Vfo()) return '?';
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

bool encodeCtcssTenths(uint16_t toneTenths, uint8_t& b0, uint8_t& b1) {
  if (toneTenths > 9999) return false;
  uint16_t value = toneTenths;
  uint8_t d1 = (uint8_t)(value % 10); value /= 10;
  uint8_t d10 = (uint8_t)(value % 10); value /= 10;
  uint8_t d100 = (uint8_t)(value % 10); value /= 10;
  uint8_t d1000 = (uint8_t)(value % 10);
  b0 = (uint8_t)((d1000 << 4) | d100);
  b1 = (uint8_t)((d10 << 4) | d1);
  return true;
}

bool encodeDcsCode(uint16_t dcsCode, uint8_t& b0, uint8_t& b1) {
  if (dcsCode > 999) return false;
  uint16_t value = dcsCode;
  uint8_t d1 = (uint8_t)(value % 10); value /= 10;
  uint8_t d10 = (uint8_t)(value % 10); value /= 10;
  uint8_t d100 = (uint8_t)(value % 10);
  b0 = d100;
  b1 = (uint8_t)((d10 << 4) | d1);
  return true;
}

void speakFrequencyWord() {
  if (!g_speechEnabled) return;
  speakToken("frequency");
}

void speakBinaryFeatureState(const uint8_t* featureData, size_t featureLen, bool on) {
  if (!g_speechEnabled) return;
  playClipProgmem(featureData, featureLen);
  playSilenceMs(60);
  playClipProgmem(on ? voice_on : voice_off, on ? voice_on_len : voice_off_len);
}

void speakNotchCycleState(bool on, NotchWidth width) {
  if (!g_speechEnabled) return;
  speakToken("notch filter");
  playSilenceMs(60);
  if (!on) {
    speakToken("off");
    return;
  }
  switch (width) {
    case NOTCH_WIDTH_NAR: playDigit(1); break;
    case NOTCH_WIDTH_MID: playDigit(2); break;
    case NOTCH_WIDTH_WIDE: playDigit(3); break;
    default: speakToken("on"); break;
  }
}
