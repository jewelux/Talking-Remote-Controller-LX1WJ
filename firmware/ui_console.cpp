#include "ui_console.h"

#include "radio_catalog.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_civ.h"
#include "ui_console_ft847.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_yaesu.h"
#include "protocol_yaesu_cat.h"
#include "radio_profile.h"
#include "radio_prefs.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "transport_serial.h"
#include "ui_features.h"
#include "ui_speech.h"
#include "ui_console_support.h"
#include "ui_keypad.h"
#include "ui_keypad_common.h"
#include "firmware_version.h"

static uint16_t rfPowerRawToWatts(uint16_t raw) {
  const uint16_t maxWatts = currentProfile().rfPowerMaxWatts ? currentProfile().rfPowerMaxWatts : 100;
  if (raw >= 255) return maxWatts;
  return (uint16_t)((raw * (uint32_t)maxWatts + 127U) / 255U);
}

static uint16_t rfPowerWattsToRaw(int watts) {
  const uint16_t maxWatts = currentProfile().rfPowerMaxWatts ? currentProfile().rfPowerMaxWatts : 100;
  if (watts < 0) watts = 0;
  if (watts > maxWatts) watts = maxWatts;
  return (uint16_t)((watts * 255UL + (maxWatts / 2U)) / maxWatts);
}

static int pbtRawToOffset(uint16_t raw) {
  if (raw > 255) raw = 255;
  return (int)raw - 128;
}

static void printYaesuProbeByte(const char* label, bool ok, uint8_t raw) {
  Serial.print(label);
  if (!ok) {
    Serial.println("no reply");
    return;
  }
  Serial.print("0x");
  if (raw < 0x10) Serial.print('0');
  Serial.println(raw, HEX);
}

static void probeYaesuFt817ModeTxRx(const char* phaseLabel) {
  uint8_t raw = 0;
  Serial.println(phaseLabel);
  yaesuCatFlushInput();
  delay(90);
  printYaesuProbeByte("  MODE: ", yaesuCatQueryModeRawByte(raw, 800), raw);
  yaesuCatFlushInput();
  delay(90);
  printYaesuProbeByte("  TX:   ", yaesuCatQueryTxStatusRaw(raw, 800), raw);
  yaesuCatFlushInput();
  delay(90);
  printYaesuProbeByte("  RX:   ", yaesuCatQueryRxStatusRaw(raw, 800), raw);
}

static void printLiveToneStateSummary() {
  Serial.print("  CTCSS cache: ");
  if (!live.ctcssValid) {
    Serial.println("unknown");
  } else {
    Serial.print((double)live.ctcssTenths / 10.0, 1);
    Serial.println(" Hz");
  }
  Serial.print("  DCS cache:   ");
  if (!live.dcsValid) {
    Serial.println("unknown");
  } else {
    char label[6];
    snprintf(label, sizeof(label), "%03u", (unsigned)live.dcsCode);
    Serial.println(label);
  }
}

static constexpr uint32_t FT817_DEBUG_POLL_HOLD_MS = 2500;

// Holds polling for a multi-step FT-817 debug exchange, then restores the
// previous hold. Does nothing when active is false.
class Ft817DebugPollingHold {
 public:
  explicit Ft817DebugPollingHold(bool active = true)
      : active_(active), savedUntilMs_(g_suspendPollingUntilMs) {
    if (active_) g_suspendPollingUntilMs = millis() + FT817_DEBUG_POLL_HOLD_MS;
  }
  ~Ft817DebugPollingHold() {
    if (active_) g_suspendPollingUntilMs = savedUntilMs_;
  }
  Ft817DebugPollingHold(const Ft817DebugPollingHold&) = delete;
  Ft817DebugPollingHold& operator=(const Ft817DebugPollingHold&) = delete;

 private:
  const bool active_;
  const uint32_t savedUntilMs_;
};

static void printYaesuFt817FmContext() {
  const Ft817DebugPollingHold pollingHold;
  Serial.println("[FT-817 FM CONTEXT]");

  uint64_t hz = 0;
  if (queryFrequency(hz, 800)) {
    Serial.print("  FREQ:        ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
  } else {
    Serial.println("  FREQ:        no reply");
  }

  uint8_t mode = 0xFF;
  if (queryMode(mode, 800)) {
    Serial.print("  MODE:        ");
    Serial.println(modeToString(mode));
  } else {
    Serial.println("  MODE:        no reply");
  }

  uint8_t raw = 0;
  if (yaesuCatQueryModeRawByte(raw, 800)) {
    Serial.print("  MODE RAW:    0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
  } else {
    Serial.println("  MODE RAW:    no reply");
  }

  if (yaesuCatQueryTxStatusRaw(raw, 800)) {
    Serial.print("  TX RAW:      0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
  } else {
    Serial.println("  TX RAW:      no reply");
  }

  if (yaesuCatQueryRxStatusRaw(raw, 800)) {
    Serial.print("  RX RAW:      0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
  } else {
    Serial.println("  RX RAW:      no reply");
  }

  bool splitOn = false;
  if (querySplit(splitOn, 800)) {
    Serial.print("  SPLIT:       ");
    Serial.println(splitOn ? "on" : "off");
  } else {
    Serial.println("  SPLIT:       no reply");
  }

  printLiveToneStateSummary();
  Serial.print("  TRACE:       ");
  Serial.println(g_yaesuCatTrace ? "on" : "off");
}

static uint16_t pbtOffsetToRaw(int offset) {
  if (offset < -128) offset = -128;
  if (offset > 127) offset = 127;
  return (uint16_t)(offset + 128);
}

static bool queryCurrentModeValue(uint8_t& modeOut) {
  if (live.modeValid) {
    modeOut = live.mode;
    return true;
  }
  return queryMode(modeOut, 800);
}

static bool queryCurrentFilterSlot(uint8_t& filterOut) {
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  const bool hadKnown = live.activeVfoKnown;
  if (!hadKnown) {
    if (!queryVfoMode(true, mode, filter, 800)) return false;
  }
  if (live.activeVfoKnown) {
    return queryVfoMode(live.activeVfoA, mode, filterOut, 800);
  }
  filterOut = filter;
  return true;
}

static bool queryCurrentFrequencyValue(uint64_t& hzOut) {
  if (live.freqValid) {
    hzOut = live.freqHz;
    return true;
  }
  return queryFrequency(hzOut, 800);
}

static bool bandCodeFromFrequency(uint64_t hz, uint8_t& bandCodeOut) {
  if (hz >= 1800000ULL && hz <= 1999999ULL) { bandCodeOut = 0x01; return true; }
  if (hz >= 3400000ULL && hz <= 4099999ULL) { bandCodeOut = 0x02; return true; }
  if (hz >= 6900000ULL && hz <= 7499999ULL) { bandCodeOut = 0x03; return true; }
  if (hz >= 9900000ULL && hz <= 10499999ULL) { bandCodeOut = 0x04; return true; }
  if (hz >= 13900000ULL && hz <= 14499999ULL) { bandCodeOut = 0x05; return true; }
  if (hz >= 17900000ULL && hz <= 18499999ULL) { bandCodeOut = 0x06; return true; }
  if (hz >= 20900000ULL && hz <= 21499999ULL) { bandCodeOut = 0x07; return true; }
  if (hz >= 24400000ULL && hz <= 25099999ULL) { bandCodeOut = 0x08; return true; }
  if (hz >= 28000000ULL && hz <= 29999999ULL) { bandCodeOut = 0x09; return true; }
  if (hz >= 50000000ULL && hz <= 54000000ULL) { bandCodeOut = 0x0A; return true; }
  bandCodeOut = 0x0B;
  return true;
}

static const char* bandLabelForCode(uint8_t bandCode) {
  switch (bandCode) {
    case 0x01: return "1.8";
    case 0x02: return "3.5";
    case 0x03: return "7";
    case 0x04: return "10";
    case 0x05: return "14";
    case 0x06: return "18";
    case 0x07: return "21";
    case 0x08: return "24";
    case 0x09: return "28";
    case 0x0A: return "50";
    case 0x0B: return "GENE";
    default: return "?";
  }
}

static bool usbConsoleReady() {
  return (bool)Serial;
}

// Report a failed radio command. If the radio never answered, say "timeout";
// other failures (unsupported, bad argument) stay silent as before.
static void reportCommandFailure(const char* label, const char* reason) {
  Serial.print(label);
  if (g_radioReplyTimedOut) {
    Serial.println(" -> timeout");
    if (g_speechEnabled) speakTimeout();
    return;
  }
  Serial.print(" -> ");
  Serial.println(reason);
}

// Report a command the radio or profile cannot do, and say "not available".
static void reportNotAvailable(const char* message) {
  Serial.println(message);
  if (g_speechEnabled) speakNotAvailable();
}

// A shared feature operation (radio_features.h) that did not succeed: print
// why, with the speech the other console commands give. Ok: false.
static bool reportFeatureFailure(FeatureStatus status, const char* label) {
  switch (status) {
    case FeatureStatus::Ok: return false;
    case FeatureStatus::Unsupported:
      reportNotAvailable((String(label) + " -> unsupported").c_str());
      return true;
    case FeatureStatus::Timeout:
      Serial.print(label);
      Serial.println(" -> timeout");
      if (g_speechEnabled) speakTimeout();
      return true;
    default:
      Serial.print(label);
      Serial.print(" -> ");
      Serial.println(featureStatusText(status));
      return true;
  }
}

static void printNrState(const NrState& state) {
  Serial.println(nrStateText(state));
  speakNrState(state);
}

static void printNbState(bool on) {
  Serial.println(nbStateText(on));
  speakNbState(on);
}

static void printNotchState(const NotchState& state) {
  Serial.println(notchStateText(state));
  speakNotchState(state);
}

// NR, NB and NOTCH: the same operations as the Bank 2 keys. NR <n> sets a
// level on radios that have them (TS-480: NR 0, NR 1, NR 2).
static bool handleConsoleFeatureCommand(const String& upper) {
  const char* label = upper.c_str();
  if (upper == "NR?" || upper == "NR TOGGLE") {
    NrState state;
    if (!reportFeatureFailure(upper == "NR?" ? nrQuery(state) : nrToggle(state), label)) printNrState(state);
    return true;
  }
  if (upper == "NR ON" || upper == "NR OFF") {
    NrState state;
    state.on = upper == "NR ON";
    if (!reportFeatureFailure(nrSet(state.on), label)) printNrState(state);
    return true;
  }
  if (upper.startsWith("NR ") && upper.length() == 4 && isDigit(upper[3])) {
    const uint8_t level = (uint8_t)(upper[3] - '0');
    if (nrLevelCount() > 0 && level > nrLevelCount()) {
      Serial.print("NR -> invalid (use 0..");
      Serial.print((int)nrLevelCount());
      Serial.println(")");
      return true;
    }
    NrState state;
    if (!reportFeatureFailure(nrSetLevel(level, state), label)) printNrState(state);
    return true;
  }
  if (upper == "NB?" || upper == "NB TOGGLE") {
    bool on = false;
    if (!reportFeatureFailure(upper == "NB?" ? nbQuery(on) : nbToggle(on), label)) printNbState(on);
    return true;
  }
  if (upper == "NB ON" || upper == "NB OFF") {
    const bool on = upper == "NB ON";
    if (!reportFeatureFailure(nbSet(on), label)) printNbState(on);
    return true;
  }
  if (upper == "NOTCH?" || upper == "NOTCH TOGGLE") {
    NotchState state;
    if (!reportFeatureFailure(upper == "NOTCH?" ? notchQuery(state) : notchToggle(state), label)) printNotchState(state);
    return true;
  }
  if (upper == "NOTCH ON" || upper == "NOTCH OFF") {
    NotchState state;
    state.on = upper == "NOTCH ON";
    if (!reportFeatureFailure(notchSet(state.on), label)) printNotchState(state);
    return true;
  }
  if (upper == "NOTCH NAR" || upper == "NOTCH MID" || upper == "NOTCH WIDE") {
    NotchState state;
    state.on = true;
    state.width = upper == "NOTCH NAR" ? NOTCH_WIDTH_NAR : upper == "NOTCH MID" ? NOTCH_WIDTH_MID : NOTCH_WIDTH_WIDE;
    if (!reportFeatureFailure(notchSetWidth(state.width), label)) printNotchState(state);
    return true;
  }
  return false;
}

static void speakConsoleTokenOrGap(const char* token) {
  if (!g_speechEnabled) return;
  if (!speakToken(token)) speakError();
}

static void speakRitStateAndOffset(bool on, int32_t offset) {
  if (!g_speechEnabled) return;
  speakTokenState("rit", on);
  if (offset == 0) return;
  playSilenceMs(60);
  if (offset > 0) {
    speakToken("plus");
    playSilenceMs(60);
  } else {
    speakToken("minus");
    playSilenceMs(60);
    offset = -offset;
  }
  speakDigitsAndPoint(String(offset));
  playSilenceMs(60);
  speakToken("hertz");
}

static void speakSignedStepValue(const String& label, int value) {
  if (!g_speechEnabled) return;
  speakLabel(label);
  if (value < 0) {
    speakToken("minus");
    playSilenceMs(60);
    value = -value;
  }
  speakDigitsAndPoint(String(value));
  playSilenceMs(60);
  speakToken("step");
}

// "<prefix><n>" with n an optional sign and digits, e.g. "RIT STEP -100".
static bool parseStepArg(const String& line, const String& upper, const char* prefix, int32_t& stepOut) {
  if (!upper.startsWith(prefix)) return false;
  String arg = line.substring(strlen(prefix));
  arg.trim();
  const int first = (arg.startsWith("+") || arg.startsWith("-")) ? 1 : 0;
  if ((int)arg.length() <= first) return false;
  for (int i = first; i < (int)arg.length(); ++i) {
    if (!isDigit(arg[i])) return false;
  }
  stepOut = arg.toInt();
  return true;
}

static int32_t clampInt32(int32_t value, int32_t lo, int32_t hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static void speakBandStackLabel(uint8_t reg) {
  if (!g_speechEnabled) return;
  speakLabel("b stack");
  playDigit((int)reg);
}

// A key already spoke its own label, so its reply is just the mode name.
static void speakModeReply(uint8_t mode) {
  if (g_keypadExecuting) speakModeName(mode);
  else speakMode(mode);
}

static bool isCurrentYaesuFt8x7() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
}

static void printAsciiReplyPayload(const String& line, const char* prefix) {
  int start = prefix ? (int)strlen(prefix) : 0;
  int semi = line.indexOf(';', start);
  if (semi < 0) semi = line.length();
  String value = line.substring(start, semi);
  value.trim();
  Serial.println(value);
}

static bool parseHexNybbleString(String s, uint8_t* out, size_t count) {
  s.replace(" ", "");
  s.replace("-", "");
  s.replace(":", "");
  s.trim();
  if (s.length() != (int)(count * 2)) return false;
  for (size_t i = 0; i < count; ++i) {
    if (!parseHexByteString(s.substring((int)(i * 2), (int)(i * 2 + 2)), out[i])) return false;
  }
  return true;
}

static bool parseTwoHexByteArgs(const String& input, uint8_t& firstOut, uint8_t& secondOut) {
  String s = input;
  s.trim();
  int sep = s.indexOf(' ');
  if (sep < 0) sep = s.indexOf(',');
  if (sep < 0) sep = s.indexOf('-');
  if (sep < 0) return false;
  String a = s.substring(0, sep);
  String b = s.substring(sep + 1);
  a.trim();
  b.trim();
  return parseHexByteString(a, firstOut) && parseHexByteString(b, secondOut);
}

static void printHexByte2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

static bool parseCivRawArgs(String args, uint8_t& cmdOut, uint8_t* payloadOut, size_t& payloadLenOut, size_t payloadMax) {
  args.trim();
  if (!args.length()) return false;
  payloadLenOut = 0;

  int sep = args.indexOf(' ');
  String cmdToken = sep < 0 ? args : args.substring(0, sep);
  cmdToken.trim();
  if (!parseHexByteString(cmdToken, cmdOut)) return false;
  if (sep < 0) return true;

  String rest = args.substring(sep + 1);
  rest.trim();
  while (rest.length()) {
    if (payloadLenOut >= payloadMax) return false;
    int next = rest.indexOf(' ');
    String token = next < 0 ? rest : rest.substring(0, next);
    token.trim();
    if (token.length()) {
      if (!parseHexByteString(token, payloadOut[payloadLenOut])) return false;
      ++payloadLenOut;
    }
    if (next < 0) break;
    rest = rest.substring(next + 1);
    rest.trim();
  }
  return true;
}

static bool printCivRawTransaction(uint8_t cmd, const uint8_t* payload, size_t payloadLen, bool query) {
  civFlushInput();
  civSend(cmd, payload, payloadLen);
  Serial.print(query ? "CIVRAW? TX: " : "CIVRAW TX: ");
  printHexByte2(cmd);
  for (size_t i = 0; i < payloadLen; ++i) {
    Serial.print(' ');
    printHexByte2(payload[i]);
  }
  Serial.println();

  const uint32_t start = millis();
  bool any = false;
  uint8_t buf[kCivMaxFrame];
  while (millis() - start < 1000) {
    size_t n = civReadFrame(buf, sizeof(buf), 80);
    if (!n) continue;
    const std::optional<CivFrame> d = civDecode(buf, n);
    if (!d) continue;
    if (d->from != currentConnectionProfile().civAddr) continue;
    any = true;
    Serial.print("CIVRAW RX: to=");
    printHexByte2(d->to);
    Serial.print(" from=");
    printHexByte2(d->from);
    Serial.print(" cmd=");
    printHexByte2(d->cmd);
    Serial.print(" payload=");
    if (!d->payloadLen) {
      Serial.print("(none)");
    } else {
      for (size_t i = 0; i < d->payloadLen; ++i) {
        if (i) Serial.print(' ');
        printHexByte2(d->payload[i]);
      }
    }
    Serial.println();
  }
  if (!any) Serial.println("CIVRAW RX: no reply");
  return true;
}

String readLineFrom(Stream& input, String& lineBuffer) {
  while (input.available()) {
    char c = (char)input.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String r = lineBuffer;
      lineBuffer = "";
      r.trim();
      return r;
    }
    if (lineBuffer.length() < 96) {
      lineBuffer += c;
    } else {
      lineBuffer = "";
    }
  }
  return "";
}

String readLine() {
  static String line;
  return readLineFrom(Serial, line);
}

String upperCopy(String s) {
  s.toUpperCase();
  return s;
}

static bool isFtdx10ConsoleProfile() {
  return currentRadioModel() == RadioModel::Ftdx10;
}

static bool handleFtdx10BlockedConsoleCommand(const String& upper) {
  if (!isFtdx10ConsoleProfile()) return false;
  if (upper == "MONITOR?" || upper == "MONITOR ON" || upper == "MONITOR OFF" || upper == "MONITOR TOGGLE") {
    reportNotAvailable("MONITOR -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "MONLEVEL?" || upper.startsWith("MONLEVEL ")) {
    reportNotAvailable("MONLEVEL -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "TRANSCEIVE?" || upper == "TRANSCEIVE ON" || upper == "TRANSCEIVE OFF" ||
      upper == "TRANSCEIVE TOGGLE") {
    reportNotAvailable("TRANSCEIVE -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "PBT1?" || upper.startsWith("PBT1 ")) {
    reportNotAvailable("PBT1 -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "PBT2?" || upper.startsWith("PBT2 ")) {
    reportNotAvailable("PBT2 -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "FILSHAPE?" || upper == "FILSHAPE SHARP" || upper == "FILSHAPE SOFT" ||
      upper == "FILSHAPE TOGGLE") {
    reportNotAvailable("FILSHAPE -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "FILWIDTH?" || upper.startsWith("FILWIDTH ")) {
    reportNotAvailable("FILWIDTH -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "RIT?" || upper == "RIT ON" || upper == "RIT OFF" || upper.startsWith("RIT ")) {
    reportNotAvailable("RIT -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "NBLEVEL?" || upper.startsWith("NBLEVEL ")) {
    reportNotAvailable("NBLEVEL -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "NRLEVEL?" || upper.startsWith("NRLEVEL ")) {
    reportNotAvailable("NRLEVEL -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "NOTCH NAR" || upper == "NOTCH MID" || upper == "NOTCH WIDE") {
    reportNotAvailable("NOTCH width -> hidden on FTDX10 (only clean ON/OFF path is exposed)");
    return true;
  }
  return false;
}

static bool parseConsoleModeToken(String token, uint8_t& modeOut) {
  token.trim();
  if (!token.length()) return false;

  if (token.length() == 1 && modeFromDigit(token[0], modeOut)) return true;

  String upper = upperCopy(token);
  if (upper == "LSB") { modeOut = 0x00; return true; }
  if (upper == "USB") { modeOut = 0x01; return true; }
  if (upper == "AM") { modeOut = 0x02; return true; }
  if (upper == "CW") { modeOut = 0x03; return true; }
  if (upper == "RTTY") { modeOut = 0x04; return true; }
  if (upper == "FM") { modeOut = 0x05; return true; }
  if (upper == "WFM") { modeOut = 0x06; return true; }
  if (upper == "CWR") { modeOut = 0x07; return true; }
  if (upper == "RTTY-R" || upper == "RTTYR") { modeOut = 0x08; return true; }
  if (upper == "DIGI" || upper == "DIG" || upper == "PKT") { modeOut = 0x11; return true; }
  return false;
}

void printHelp() {
  const bool ftdx10 = isFtdx10ConsoleProfile();
  const bool ft8x7 = currentProtocolType() == PROTO_YAESU_FT8X7;
  const bool ft817 = ft8x7 && currentIsFt817Family();
  const bool ft857Family = ft8x7 && currentIsFt857Family();
  const bool ft847 = currentProtocolType() == PROTO_YAESU_FT847;
  Serial.println();
  Serial.println("Commands (case-insensitive):");
  Serial.println("  General:");
  Serial.println("    AK?");
  Serial.println("    BANK?");
  Serial.println("    BANK <1..9>");
  Serial.println("    BANK NEXT | PREV");
  Serial.println("    BAUD <rate> | BAUD?");
  if (!ftdx10) {
    Serial.println("    BSTACK <1..3>  (hamTRC internal)");
    Serial.println("    BSTACK? <1..3>  (hamTRC internal)");
  }
  Serial.println("    EXPERIMENTAL ON | OFF  (all caps on for testing, not saved)");
  Serial.println("    EXPERIMENTAL?");
  Serial.println("    FB? | FB <kHz> | FBMHZ <MHz>");
  Serial.println("    FREQ <kHz>");
  Serial.println("    FREQ?");
  Serial.println("    FREQMHZ <MHz>");
  Serial.println("    FR? | FR0");
  Serial.println("    FT? | FT A | FT B");
  Serial.println("    HELP");
  Serial.println("    ID? | IF? | OM?");
  Serial.println("    LFREQ");
  Serial.println("    LISTVOICES");
  Serial.println("    MODE <n|name>");
  Serial.println("    MODE LIST");
  Serial.println("    MODE?");
  Serial.println("    PROFILE <slot>  (SLOTS? lists them)");
  Serial.println("    PROFILE NEXT | PREV");
  Serial.println("    PROFILE RESET [ALL]  (saved baud and CI-V address back to the defaults)");
  Serial.println("    PROFILE?");
  Serial.println("    QUIET OFF | QUIET ON");
  Serial.println("    QUIET?");
  Serial.println("    ROUND [<Hz>]  (round the frequency, default 500 Hz)");
  Serial.println("    RX | TX");
  Serial.println("    SAY <digits>");
  Serial.println("    SLOTS?");
  Serial.println("    SPEECH OFF | SPEECH ON");
  Serial.println("    SPEECH?");
  Serial.println("    STATUS?");
  Serial.println("    SWT <nn> | SWH <nn>");
  Serial.println("    TEST");
  Serial.println("    TUNINGSPEECH OFF | ON | TOGGLE");
  Serial.println("    TUNINGSPEECH?");
  Serial.println("    VERBOSE OFF | ON | TOGGLE");
  Serial.println("    VERBOSE?");
  Serial.println("    VOICE <name>");
  Serial.println("    VOLUME <1..9> | VOLUME STEP <+-n>");
  Serial.println("    VOLUME?");
  Serial.println();
  if (!ftdx10 && !ft8x7 && !ft847) {
    Serial.println("  IC-7300 / CI-V Extensions:");
    Serial.println("    CIVADDR <hex> | CIVADDR?");
    Serial.println("    NBLEVEL <0..100> | NBLEVEL STEP <+-n>");
    Serial.println("    NBLEVEL?");
    Serial.println("    NB OFF | ON | TOGGLE");
    Serial.println("    NB?");
    Serial.println("    FILSHAPE SHARP | SOFT | TOGGLE");
    Serial.println("    FILSHAPE?");
    Serial.println("    FILWIDTH <1..3> | NEXT | PREV");
    Serial.println("    FILWIDTH?");
    Serial.println("    LOCK OFF | ON | TOGGLE");
    Serial.println("    LOCK?");
    Serial.println("    MONITOR OFF | ON | TOGGLE");
    Serial.println("    MONITOR?");
    Serial.println("    MONLEVEL <0..100> | MONLEVEL STEP <+-n>");
    Serial.println("    MONLEVEL?");
    Serial.println("    NOTCH MID | NAR | WIDE");
    Serial.println("    NOTCH OFF | ON | TOGGLE");
    Serial.println("    NOTCH?");
    Serial.println("    NRLEVEL <0..100> | NRLEVEL STEP <+-n>");
    Serial.println("    NRLEVEL?");
    Serial.println("    NR OFF | ON | TOGGLE");
    Serial.println("    NR?");
    Serial.println("    PBT1 <-128..127> | CENTER | STEP <+-n>");
    Serial.println("    PBT1?");
    Serial.println("    PBT2 <-128..127> | CENTER | STEP <+-n>");
    Serial.println("    PBT2?");
    Serial.print("    RFPOWER <0..");
    Serial.print((int)(currentProfile().rfPowerMaxWatts ? currentProfile().rfPowerMaxWatts : 100));
    Serial.println(" W>");
    Serial.println("    RFPOWER?");
    Serial.println("    RIT <Hz> | RIT STEP <+-Hz>");
    Serial.println("    RIT OFF | ON | TOGGLE");
    Serial.println("    RIT?");
    Serial.println("    RXTX?");
    Serial.println("    SPLIT OFF | ON | TOGGLE");
    Serial.println("    SPLIT?");
    Serial.println("    TUNE");
    Serial.println("    TRANSCEIVE OFF | ON | TOGGLE");
    Serial.println("    TRANSCEIVE?");
    Serial.println("    TUNER OFF | ON");
    Serial.println("    TUNER? | TUNER OFF | ON | TOGGLE");
    Serial.println("    TXFREQ?");
    Serial.println("    VFO A | B");
    Serial.println("    MAIN <kHz> | MAIN?");
    Serial.println("    MAIN MODE <n> | MAIN MODE?");
    Serial.println("    SUB <kHz> | SUB?");
    Serial.println("    SUB MODE <n> | SUB MODE?");
    Serial.println("    VFOA <kHz> | VFOA?");
    Serial.println("    VFOA MODE <n> | VFOA MODE?");
    Serial.println("    VFOB <kHz> | VFOB?");
    Serial.println("    VFOB MODE <n> | VFOB MODE?");
    Serial.println();
    Serial.println("  ASCII / Yaesu Extensions:");
  } else if (ft847) {
    printFt847ConsoleHelp();
  } else if (ft8x7) {
    Serial.println("  Yaesu FT8x7:");
    Serial.println("    RXTX?");
    Serial.println("    SPLIT OFF | ON | TOGGLE");
    Serial.println("    SPLIT?");
    Serial.println("    PTT OFF | ON");
    Serial.println("    SM?");
    Serial.println("    SWR?");
    Serial.println("    CLAR OFF | ON  (the CAT clarifier commands, which switch RIT)");
    Serial.println("    CLAR OFFSET <8 hex digits>");
    if (ft817 || ft857Family) Serial.println("    IFSHIFT?  (IF shift on or off, which CAT cannot switch; not spoken)");
    if (ft817 || ft857Family) Serial.println("    RIT? | RIT OFF | ON | TOGGLE");
    Serial.println("    CTCSS <Hz> | CTCSS?");
    Serial.println("    DCS <code> | DCS?");
    Serial.println("    GT? | GT FAST | SLOW | OFF");
    Serial.println("    PA? | PA OFF | ON | TOGGLE");
    Serial.println("    PS? | PS OFF | ON");
    if (ft817 || ft857Family) Serial.println("    VFO SYNC A | B   (set the tracked VFO)");
    if (ft817) {
      Serial.println("    VOL? | SQL?");
      Serial.println("    VFO TOGGLE | A | B");
      Serial.println("    VFO A=B  (copy the active VFO to the other)");
      Serial.println("    YPOWER OFF | ON");
    } else if (ft857Family) {
      Serial.println("    VFO TOGGLE  (raw)");
    }
    Serial.println();
    Serial.println("  Yaesu FT8x7 Diagnostics:");
  } else {
    Serial.println("  FTDX10 / hamTRC:");
    Serial.println("    LOCK OFF | ON | TOGGLE");
    Serial.println("    LOCK?");
    Serial.println("    NB OFF | ON | TOGGLE");
    Serial.println("    NB?");
    Serial.println("    NOTCH OFF | ON | TOGGLE");
    Serial.println("    NOTCH?");
    Serial.println("    NR OFF | ON | TOGGLE");
    Serial.println("    NR?");
    Serial.println("    RXTX?");
    Serial.println("    SPLIT OFF | ON | TOGGLE");
    Serial.println("    SPLIT?");
    Serial.println("    TUNE");
    Serial.println("    TUNER? | TUNER OFF | ON | TOGGLE");
    Serial.println("    TXFREQ?");
    Serial.println("    VFO A | B");
    Serial.println("    VFOA <kHz> | VFOA?");
    Serial.println("    VFOA MODE <n> | VFOA MODE?");
    Serial.println("    VFOB <kHz> | VFOB?");
    Serial.println("    VFOB MODE <n> | VFOB MODE?");
    Serial.println();
    Serial.println("  FTDX10 ASCII / Yaesu:");
  }
  if (ft847) {
    // Listed above.
  } else if (!ftdx10 && !ft8x7) {
    Serial.println("    ALC? | VOL? | SQL?");
    Serial.println("    AGC <hex byte>");
    Serial.println("    CIVRAW? <cmd hex> [payload hex bytes]");
    Serial.println("    CIVRAW <cmd hex> [payload hex bytes]");
    Serial.println("    CLAR OFFSET <8 hex digits>");
    Serial.println("    CLAR OFF | ON");
    Serial.println("    GT? | GT FAST | SLOW | OFF");
    Serial.println("    LOCKDOC OFF | ON");
    Serial.println("    PA? | PA OFF | ON | TOGGLE");
    Serial.println("    PS? | PS OFF | ON");
    Serial.println("    PTT OFF | ON");
    Serial.println("    SM?");
    Serial.println("    SWR?");
    Serial.println("    VFO TOGGLE | A | B");
    Serial.println("    YALL?");
    Serial.println("    YDCS <hex>");
    Serial.println("    YPOWER OFF | ON");
    Serial.println("    YRPT MINUS | PLUS | SIMPLEX");
    Serial.println("    YRPTSHIFT <MHz>");
    Serial.println("    YLOCKRAW OFF | ON");
    Serial.println("    YMODEBYTE?");
    Serial.println("    YFMCTX?");
    Serial.println("    YSETMODEQ <hex byte>  (quiet write, then wait)");
    Serial.println("    YRXSTATUS?");
    Serial.println("    YSETMODE <hex byte>");
    Serial.println("    YTRACE OFF | ON");
    Serial.println("    YTMODE <name|hex>");
    Serial.println("    YTONE <hex>");
    Serial.println("    YTXSTATUS?");
    Serial.println("    YVAR?");
    Serial.println("    YCAT <10 hex digits>");
    Serial.println("    YCAT? <10 hex digits>");
    Serial.println("    YCAT1? <hex byte>");
    Serial.println("    YSCAN1 <start hex> <end hex>");
    Serial.println("    YSNIFF <ms>");
    Serial.println("    YSTATUS?");
  } else if (ft8x7) {
    Serial.println("    ALC?");
    Serial.println(ft817 ? "    AGC? | BK? | KYR?  (radio settings, read only)"
                         : "    AGC? | IPO? | ATT? | NAR? | DBF? | BK? | KYR?  (radio settings, read only)");
    if (!ft817) Serial.println("    NRLEVEL? | NBLEVEL? | HPF? | LPF? | MICEQ?  (DSP menu settings, read only)");
    Serial.println(ft817 ? "    RFPOWER?  (the TX power setting)" : "    RFPOWER?  (menu 75 power of the current band)");
    Serial.println("    MENU? | ROW?  (menu item and soft key row, saved when the radio's menu is exited)");
    Serial.println("    AGC <hex byte>");
    Serial.println("    CIVRAW? <cmd hex> [payload hex bytes]");
    Serial.println("    CIVRAW <cmd hex> [payload hex bytes]");
    Serial.println("    YALL?");
    Serial.println("    YDCS <hex>");
    Serial.println("    YRPT MINUS | PLUS | SIMPLEX");
    Serial.println("    YRPTSHIFT <MHz>");
    Serial.println("    YLOCKRAW OFF | ON");
    Serial.println("    YMODEBYTE?");
    if (ft817) Serial.println("    YFMCTX?");
    Serial.println("    YSETMODEQ <hex byte>  (quiet write, then wait)");
    Serial.println("    YRXSTATUS?");
    Serial.println("    YSETMODE <hex byte>");
    Serial.println("    YTRACE OFF | ON");
    Serial.println("    YTMODE <name|hex>");
    Serial.println("    YTONE <hex>");
    Serial.println("    YTXSTATUS?");
    Serial.println("    YVAR?");
    Serial.println("    YCAT <10 hex digits>");
    Serial.println("    YCAT? <10 hex digits>");
    Serial.println("    YCAT1? <hex byte>");
    Serial.println("    YEEPROM? <start hex> [count 1..32]  (read only; not while operating the radio)");
    Serial.println("    YEEPROM! <addr hex> <byte hex> <byte hex>  (CAUTION: writes the EEPROM at addr and addr+1)");
    Serial.println("    YSETTINGS?  (settings read from the EEPROM)");
    Serial.println("    YSCAN1 <start hex> <end hex>");
    Serial.println("    YSNIFF <ms>");
    Serial.println("    YSTATUS?");
  } else {
    Serial.println("    GT? | GT FAST | SLOW | OFF");
    Serial.println("    ID?");
    Serial.println("    IF?");
    Serial.println("    PA? | PA OFF | ON | TOGGLE");
    Serial.println("    PS? | PS OFF | ON");
    Serial.println("    SM?");
    Serial.println("    SWR?");
  }
  Serial.println();
}

static bool handleConsoleInfoCommands(const String& upper) {
  if (upper == "HELP") { printHelp(); return true; }
  if (upper == "STATUS?") { printStatusSummary(); return true; }
  if (upper == "MODE LIST") { printModeList(); return true; }
  if (upper.startsWith("CIVRAW? ") || upper.startsWith("CIVRAW ")) {
    uint8_t cmd = 0;
    uint8_t payload[32] = {0};
    size_t payloadLen = 0;
    const bool query = upper.startsWith("CIVRAW? ");
    const int prefixLen = query ? 8 : 7;
    if (currentProtocolType() != PROTO_CIV) {
      Serial.println("CIVRAW -> CI-V profile required");
      return true;
    }
    if (!parseCivRawArgs(upper.substring(prefixLen), cmd, payload, payloadLen, sizeof(payload))) {
      Serial.println(query ? "CIVRAW? -> use: CIVRAW? 14 0A" : "CIVRAW -> use: CIVRAW 14 0A 00 26");
      return true;
    }
    printCivRawTransaction(cmd, payload, payloadLen, query);
    return true;
  }
  if (upper == "EXPERIMENTAL?") {
    Serial.print("EXPERIMENTAL ");
    Serial.println(g_experimentalCaps ? "ON" : "OFF");
    return true;
  }
  if (upper == "QUIET?") {
    Serial.print("QUIET ");
    Serial.println(g_quiet ? "ON" : "OFF");
    return true;
  }
  if (upper == "SPEECH?") {
    Serial.print("SPEECH ");
    Serial.println(g_speechEnabled ? "ON" : "OFF");
    return true;
  }
  if (upper == "BANK?") {
    Serial.print("BANK ");
    Serial.println((int)uiGetBank());
    if (g_speechEnabled) speakBankNumber();
    return true;
  }
  if (upper == "PROFILE?") { printActiveProfileDetails(); return true; }
  if (upper == "SLOTS?") { printProfileSlots(); return true; }
  if (upper == "LISTVOICES") { listVoices(); return true; }
  if (upper == "TUNINGSPEECH?") {
    Serial.print("TUNINGSPEECH ");
    Serial.println(g_tuningSpeakEnabled ? "ON" : "OFF");
    speakTuningSpeechState();
    return true;
  }
  if (upper == "VERBOSE?") {
    Serial.print("VERBOSE ");
    Serial.println(g_verboseSpeech ? "ON" : "OFF");
    speakVerboseState();
    return true;
  }
  if (upper == "VOLUME?") {
    Serial.print("VOLUME ");
    Serial.println((int)g_volumeLevel);
    if (g_speechEnabled) speakVolumeLevel(g_volumeLevel);
    return true;
  }
  return false;
}

static void printConnectionSettings(const char* prefix) {
  const ConnectionProfile& link = currentConnectionProfile();
  Serial.print(prefix);
  Serial.print("BAUD ");
  Serial.print((unsigned long)link.baud);
  if (currentProtocolType() == PROTO_CIV) {
    char hex[3] = "";
    formatHexByte(link.civAddr, hex, sizeof(hex));
    Serial.print(" CIVADDR ");
    Serial.print(hex);
  }
  Serial.println();
}

static bool handleConsoleProfileCommands(const String& line, const String& upper) {
  if (upper == "PROFILE RESET") {
    resetCurrentConnection();
    printConnectionSettings("OK PROFILE RESET  ");
    speakProfileReset();
    return true;
  }
  if (upper == "PROFILE RESET ALL") {
    resetAllConnections();
    printConnectionSettings("OK PROFILE RESET ALL  active: ");
    speakProfileReset();
    return true;
  }
  if (upper == "PROFILE NEXT") {
    uint8_t next = adjacentProfileSlot(g_profileId, 1);
    applyProfile(next);
    Serial.print("OK PROFILE ");
    Serial.println((int)next);
    speakCurrentProfile();
    return true;
  }
  if (upper == "PROFILE PREV") {
    uint8_t prev = adjacentProfileSlot(g_profileId, -1);
    applyProfile(prev);
    Serial.print("OK PROFILE ");
    Serial.println((int)prev);
    speakCurrentProfile();
    return true;
  }
  if (upper.startsWith("PROFILE ")) {
    int slot = line.substring(8).toInt();
    if (slot < 1 || slot > 255 || !profileForSlot((uint8_t)slot)) {
      Serial.println("PROFILE -> invalid or empty slot");
      speakError();
      return true;
    }
    applyProfile((uint8_t)slot);
    Serial.print("OK PROFILE ");
    Serial.println(slot);
    speakCurrentProfile();
    return true;
  }
  if (upper.startsWith("BSTACK? ")) {
    if (isFtdx10ConsoleProfile()) {
      reportNotAvailable("BSTACK? -> hidden on FTDX10 (hamTRC internal, not Yaesu CAT)");
      return true;
    }
    int reg = line.substring(8).toInt();
    uint64_t hz = 0;
    uint8_t bandCode = 0;
    BandStackEntry entry;
    if (reg < 1 || reg > 3) { Serial.println("BSTACK? -> invalid register (use 1..3)"); return true; }
    if (!queryCurrentFrequencyValue(hz) || !bandCodeFromFrequency(hz, bandCode)) { Serial.println("BSTACK? -> no current band"); return true; }
    if (!queryBandStackEntry(bandCode, (uint8_t)reg, entry, 800)) { reportCommandFailure("BSTACK?", "no reply"); return true; }
    Serial.print("BSTACK ");
    Serial.print(bandLabelForCode(entry.bandCode));
    Serial.print("M REG");
    Serial.print((int)entry.registerCode);
    Serial.print(": ");
    Serial.print(hzToMHzString3(entry.freqHz));
    Serial.print(" MHz ");
    Serial.print(modeToString(entry.mode));
    Serial.print(" FIL");
    Serial.println((int)entry.filter);
    if (g_speechEnabled) {
      speakBandStackLabel((uint8_t)reg);
      playSilenceMs(60);
      speakDigitsAndPoint(hzToMHzString3(entry.freqHz));
      speakModeName(entry.mode);
    }
    return true;
  }
  if (upper.startsWith("BSTACK ")) {
    if (isFtdx10ConsoleProfile()) {
      reportNotAvailable("BSTACK -> hidden on FTDX10 (hamTRC internal, not Yaesu CAT)");
      return true;
    }
    int reg = line.substring(7).toInt();
    uint64_t hz = 0;
    uint8_t bandCode = 0;
    BandStackEntry entry;
    if (reg < 1 || reg > 3) { Serial.println("BSTACK -> invalid register (use 1..3)"); return true; }
    if (!queryCurrentFrequencyValue(hz) || !bandCodeFromFrequency(hz, bandCode)) { Serial.println("BSTACK -> no current band"); return true; }
    if (!queryBandStackEntry(bandCode, (uint8_t)reg, entry, 800)) { reportCommandFailure("BSTACK", "no reply"); return true; }
    if (!setFrequency(entry.freqHz) || !setMode(entry.mode, entry.filter)) { reportCommandFailure("BSTACK", "failed"); return true; }
    Serial.print("BSTACK ");
    Serial.print(bandLabelForCode(entry.bandCode));
    Serial.print("M REG");
    Serial.print((int)entry.registerCode);
    Serial.print(": ");
    Serial.print(hzToMHzString3(entry.freqHz));
    Serial.print(" MHz ");
    Serial.print(modeToString(entry.mode));
    Serial.print(" FIL");
    Serial.println((int)entry.filter);
    if (g_speechEnabled) {
      speakBandStackLabel((uint8_t)reg);
      playSilenceMs(60);
      speakDigitsAndPoint(hzToMHzString3(entry.freqHz));
      speakModeName(entry.mode);
    }
    return true;
  }
  return false;
}

static void printCivAddress(const char* prefix, uint8_t addr) {
  char hex[3] = "";
  formatHexByte(addr, hex, sizeof(hex));
  Serial.print(prefix);
  Serial.println(hex);
}

// Baud and CI-V address of the current profile, saved for its slot.
static bool handleConsoleConnectionCommands(const String& line, const String& upper) {
  const bool civAddrCmd = upper == "CIVADDR?" || upper.startsWith("CIVADDR ");
  const bool baudCmd = upper == "BAUD?" || upper.startsWith("BAUD ");
  if (!civAddrCmd && !baudCmd) return false;
  if (civAddrCmd && currentProtocolType() != PROTO_CIV) {
    reportNotAvailable("CIVADDR -> CI-V profile required");
    return true;
  }
  const ConnectionProfile& p = currentConnectionProfile();
  if (upper == "CIVADDR?") {
    printCivAddress("CIVADDR ", p.civAddr);
    speakCivAddressValue(p.civAddr, false);
    return true;
  }
  if (civAddrCmd) {
    String arg = line.substring(8);
    arg.trim();
    uint8_t addr = 0;
    if (!parseHexByteString(arg, addr)) {
      Serial.println("CIVADDR -> use a hex byte, e.g. CIVADDR 94");
      return true;
    }
    setCurrentCivAddress(addr);
    printCivAddress("OK CIVADDR ", addr);
    speakCivAddressValue(addr, true);
    return true;
  }
  if (upper == "BAUD?") {
    Serial.print("BAUD ");
    Serial.println((unsigned long)p.baud);
    speakBaudValue(p.baud, false);
    return true;
  }
  const uint32_t baud = (uint32_t)line.substring(5).toInt();
  if (!setCurrentBaud(baud)) {
    const BaudRates& bauds = currentProfile().link.bauds;
    Serial.print("BAUD -> invalid (use");
    for (uint8_t i = 0; i < bauds.count; ++i) {
      Serial.print(i ? ", " : " ");
      Serial.print((unsigned long)bauds.rates[i]);
    }
    Serial.println(")");
    return true;
  }
  Serial.print("OK BAUD ");
  Serial.println((unsigned long)baud);
  speakBaudValue(baud, true);
  return true;
}

static bool handleConsoleToggleCommands(const String& line, const String& upper) {
  if (upper == "EXPERIMENTAL ON" || upper == "EXPERIMENTAL OFF") {
    setExperimentalCaps(upper == "EXPERIMENTAL ON");
    Serial.println(g_experimentalCaps ? "OK EXPERIMENTAL ON  (all caps on until OFF or restart)" : "OK EXPERIMENTAL OFF");
    return true;
  }
  if (upper == "QUIET ON") { g_quiet = true; Serial.println("OK QUIET ON"); return true; }
  if (upper == "QUIET OFF") { g_quiet = false; Serial.println("OK QUIET OFF"); return true; }
  if (upper == "SPEECH ON") { g_speechEnabled = true; Serial.println("OK SPEECH ON"); return true; }
  if (upper == "SPEECH OFF") { g_speechEnabled = false; Serial.println("OK SPEECH OFF"); return true; }
  if (upper == "TUNINGSPEECH ON") {
    setTuningSpeechEnabled(true);
    Serial.println("OK TUNINGSPEECH ON");
    speakTuningSpeechState();
    return true;
  }
  if (upper == "TUNINGSPEECH OFF") {
    setTuningSpeechEnabled(false);
    Serial.println("OK TUNINGSPEECH OFF");
    speakTuningSpeechState();
    return true;
  }
  if (upper == "TUNINGSPEECH TOGGLE") {
    setTuningSpeechEnabled(!g_tuningSpeakEnabled);
    Serial.println(g_tuningSpeakEnabled ? "OK TUNINGSPEECH ON" : "OK TUNINGSPEECH OFF");
    speakTuningSpeechState();
    return true;
  }
  if (upper == "VERBOSE ON" || upper == "VERBOSE OFF" || upper == "VERBOSE TOGGLE") {
    if (upper == "VERBOSE TOGGLE") setVerboseSpeech(!g_verboseSpeech);
    else setVerboseSpeech(upper == "VERBOSE ON");
    Serial.println(g_verboseSpeech ? "OK VERBOSE ON" : "OK VERBOSE OFF");
    speakVerboseState();
    return true;
  }
  if (upper.startsWith("VOLUME STEP")) {
    int32_t step = 0;
    if (!parseStepArg(line, upper, "VOLUME STEP ", step)) {
      Serial.println("VOLUME STEP -> use a signed step, e.g. VOLUME STEP -1");
      return true;
    }
    const uint8_t lvl = (uint8_t)clampInt32((int32_t)g_volumeLevel + step, 1, 9);
    applyVolumeLevel(lvl);
    saveVolumeToNvs(lvl);
    Serial.print("OK VOLUME ");
    Serial.println((int)lvl);
    if (g_speechEnabled) {
      speakVolumeLevel(lvl);
      speakValueOk();
    }
    return true;
  }
  if (upper.startsWith("VOLUME ")) {
    int lvl = line.substring(7).toInt();
    if (lvl < 1 || lvl > 9) {
      Serial.println("VOLUME -> invalid (use 1..9)");
      speakError();
      return true;
    }
    applyVolumeLevel((uint8_t)lvl);
    saveVolumeToNvs((uint8_t)lvl);
    Serial.print("OK VOLUME ");
    Serial.println(lvl);
    if (g_speechEnabled) {
      speakVolumeLevel((uint8_t)lvl);
      speakValueOk();
    }
    return true;
  }
  if (upper.startsWith("VOICE ")) { playNamedVoice(line.substring(6)); return true; }
  if (upper == "TEST") { voiceTest(); return true; }
  if (upper.startsWith("SAY ")) { if (g_speechEnabled) speakDigitsAndPoint(line.substring(4)); return true; }
  return false;
}

// FT-8x7 meters exist only while transmitting: in receive, say "<meter> rx".
static void reportFt8x7MeterInReceive(const char* label, const char* meterToken) {
  Serial.print(label);
  Serial.println(": RX (not transmitting)");
  if (!g_speechEnabled) return;
  speakLabel(meterToken);
  speakToken("rx");
}

static void speakFt8x7HighSwr() {
  speakToken("swr");
  playSilenceMs(60);
  speakToken("high");
}

static bool handleConsoleFt8x7Meters(const String& upper) {
  if (upper == "PO?") {
    if (!currentProfile().caps.getPower) { reportNotAvailable("PO? -> not enabled in this profile"); return true; }
    YaesuTxMeters meters;
    if (!yaesuCatQueryTxMeters(meters, false, 800)) { reportCommandFailure("PO?", "no reply"); return true; }
    if (!meters.transmitting) { reportFt8x7MeterInReceive("PO", "power"); return true; }
    rememberLivePower(meters.po, millis());
    Serial.print("PO: ");
    Serial.print(meters.po);
    Serial.println(meters.highSwr ? " of 15  HIGH SWR" : " of 15");
    if (g_speechEnabled) {
      speakLabel("power");
      speakDigitsAndPoint(String(meters.po));
      if (meters.highSwr) {
        playSilenceMs(120);
        speakFt8x7HighSwr();
      }
    }
    return true;
  }
  if (upper == "SWR?") {
    if (!currentProfile().caps.getSwr) { reportNotAvailable("SWR? -> not enabled in this profile"); return true; }
    YaesuTxMeters meters;
    if (!yaesuCatQueryTxMeters(meters, true, 800)) { reportCommandFailure("SWR?", "no reply"); return true; }
    if (!meters.transmitting) { reportFt8x7MeterInReceive("SWR", "swr"); return true; }
    rememberLiveSwr(meters.swr, millis());
    const float swr = yaesuSwrFromMeter(meters.swr);
    Serial.print("SWR: ");
    Serial.print(swr, 1);
    Serial.print("  (meter ");
    Serial.print(meters.swr);
    Serial.println(meters.highSwr ? " of 15, HIGH SWR)" : " of 15)");
    if (g_speechEnabled) {
      // The warning stays with verbose off.
      if (meters.highSwr) {
        speakFt8x7HighSwr();
        playSilenceMs(60);
      } else {
        speakLabel("swr");
      }
      speakDigitsAndPoint(String(swr, 1));
    }
    return true;
  }
  if (upper == "ALC?") {
    YaesuTxMeters meters;
    if (!yaesuCatQueryTxMeters(meters, true, 800)) { reportCommandFailure("ALC?", "no reply"); return true; }
    if (!meters.transmitting) {
      Serial.println("ALC: RX (not transmitting)");
      return true;
    }
    Serial.print("ALC: ");
    Serial.print(meters.alc);
    Serial.println(" of 15");
    return true;
  }
  return false;
}

// FT-8x7 settings read from the EEPROM, like the Bank 2 and Bank 8 keys.
static bool handleConsoleFt8x7Settings(const String& upper) {
  static constexpr Ft8x7Setting kSettings[] = {
    Ft8x7Setting::Agc, Ft8x7Setting::Ipo, Ft8x7Setting::Att, Ft8x7Setting::Nar,
    Ft8x7Setting::Dbf, Ft8x7Setting::BreakIn, Ft8x7Setting::Keyer, Ft8x7Setting::RfPower,
    Ft8x7Setting::Menu, Ft8x7Setting::Row, Ft8x7Setting::IfShift, Ft8x7Setting::NrLevel,
    Ft8x7Setting::NbLevel, Ft8x7Setting::LowCut, Ft8x7Setting::HighCut, Ft8x7Setting::MicEq,
    Ft8x7Setting::Antenna,
  };
  for (Ft8x7Setting setting : kSettings) {
    const char* label = ft8x7SettingLabel(setting);
    if (upper != label) continue;
    Ft8x7SettingState state;
    if (reportFeatureFailure(ft8x7SettingQuery(setting, state), label)) return true;
    Serial.println(ft8x7SettingText(state));
    speakFt8x7Setting(state);
    return true;
  }
  return false;
}

static bool handleConsoleYaesuFt8x7Commands(const String& line, const String& upper) {
  if (!isCurrentYaesuFt8x7()) return false;
  if (handleConsoleFt8x7Meters(upper)) return true;
  if (handleConsoleFt8x7Settings(upper)) return true;

  if (upper == "YALL?") {
    Serial.println("[YAESU FT8X7]");
    Serial.print("  MODEL: ");
    Serial.println(radioModelName(currentRadioModel()));
    const bool isFt817 = currentIsFt817Family();
    const bool isFt857Family = currentIsFt857Family();

    uint64_t hz = 0;
    if (queryFrequency(hz, 800)) {
      Serial.print("  FREQ: ");
      Serial.print(hzToMHzString3(hz));
      Serial.println(" MHz");
    } else {
      Serial.println("  FREQ: no reply");
    }

    uint8_t mode = 0xFF;
    if (queryMode(mode, 800)) {
      Serial.print("  MODE: ");
      Serial.println(modeToString(mode));
    } else {
      Serial.println("  MODE: no reply");
    }

    int32_t raw = 0;
    if (yaesuCatQuerySMeterRaw(currentProfile(), raw, 800)) {
      Serial.print("  SM: ");
      Serial.println(raw);
    } else {
      Serial.println("  SM: no reply");
    }
    if (yaesuCatQueryPoMeterRaw(currentProfile(), raw, 800)) {
      Serial.print("  PO: ");
      Serial.println(raw);
    } else {
      Serial.println("  PO: no reply");
    }
    if (yaesuCatQuerySWRRaw(currentProfile(), raw, 800)) {
      Serial.print("  SWR: ");
      Serial.println(raw);
    } else {
      Serial.println("  SWR: no reply");
    }
    if (yaesuCatQueryAlcRaw(raw, 800)) {
      Serial.print("  ALC: ");
      Serial.println(raw);
    } else {
      Serial.println("  ALC: no reply");
    }
    if (isFt817) {
      if (yaesuCatQueryVolumeRaw(raw, 800)) {
        Serial.print("  VOL: ");
        Serial.println(raw);
      } else {
        Serial.println("  VOL: no reply");
      }
    } else if (isFt857Family) {
      Serial.println("  VOL: unsupported on verified FT-857/897 path");
    } else {
      Serial.println("  VOL: model unknown");
    }
    if (isFt817) {
      if (yaesuCatQuerySquelchRaw(raw, 800)) {
        Serial.print("  SQL: ");
        Serial.println(raw);
      } else {
        Serial.println("  SQL: no reply");
      }
    } else if (isFt857Family) {
      Serial.println("  SQL: unsupported on verified FT-857/897 path");
    } else {
      Serial.println("  SQL: model unknown");
    }

    if (isFt817 || isFt857Family) {
      uint8_t status = 0;
      if (yaesuCatQueryTxStatusRaw(status, 800)) {
        Serial.print("  STATUS: 0x");
        if (status < 0x10) Serial.print('0');
        Serial.println(status, HEX);
      } else {
        Serial.println("  STATUS: no reply");
      }
    } else {
      Serial.println("  STATUS: model unknown");
    }
    return true;
  }

  if (upper == "YVAR?") {
    Serial.print("YVAR: ");
    Serial.println(radioModelName(currentRadioModel()));
    if (currentIsFt817Family()) {
      Serial.println("  Tone/DCS family: FT-817 style (simple Tone/DCS layout)");
    } else if (currentIsFt857Family()) {
      Serial.println("  Tone/DCS family: FT-857/897 style (separate TX/RX Tone/DCS layout)");
    } else {
      Serial.println("  Tone/DCS family: unknown model");
    }
    return true;
  }

  if (upper.startsWith("YTMODE ")) {
    String arg = line.substring(7);
    arg.trim();
    String modeName = arg;
    modeName.toUpperCase();
    uint8_t modeByte = 0;
    bool known = true;
    if (modeName == "DCS") modeByte = 0x0A;
    else if (modeName == "DCSDEC" || modeName == "DCS_DEC" || modeName == "DCS-DEC") modeByte = 0x0B;
    else if (modeName == "DCSENC" || modeName == "DCS_ENC" || modeName == "DCS-ENC") modeByte = 0x0C;
    else if (modeName == "CTCSS") modeByte = 0x2A;
    else if (modeName == "CTCSSDEC" || modeName == "CTCSS_DEC" || modeName == "CTCSS-DEC") modeByte = 0x3A;
    else if (modeName == "CTCSSENC" || modeName == "CTCSS_ENC" || modeName == "CTCSS-ENC" || modeName == "ENCODER") modeByte = 0x4A;
    else if (modeName == "OFF") modeByte = 0x8A;
    else known = parseHexByteString(arg, modeByte);
    if (!known) {
      Serial.println("YTMODE -> use DCS, DCSDEC, DCSENC, CTCSS, CTCSSDEC, CTCSSENC, OFF, or hex byte");
      return true;
    }
    if (currentIsFt817Family()) {
      if (!(modeByte == 0x0A || modeByte == 0x2A || modeByte == 0x4A || modeByte == 0x8A)) {
        Serial.println("YTMODE -> mode not documented for the FT-817");
        return true;
      }
    } else if (currentIsFt857Family()) {
      if (!(modeByte == 0x0A || modeByte == 0x0B || modeByte == 0x0C || modeByte == 0x2A || modeByte == 0x3A || modeByte == 0x4A || modeByte == 0x8A)) {
        Serial.println("YTMODE -> mode not documented for the FT-857/897");
        return true;
      }
    } else {
      Serial.println("YTMODE -> unknown FT-8x7 model");
      return true;
    }
    yaesuCatSetToneDcsModeRaw(modeByte);
    Serial.print("YTMODE 0x");
    if (modeByte < 0x10) Serial.print('0');
    Serial.println(modeByte, HEX);
    return true;
  }

  if (upper.startsWith("YTONE ")) {
    uint8_t data[4] = {0};
    String arg = line.substring(6);
    bool ok = false;
    if (currentIsFt817Family()) {
      uint8_t pair[2] = {0};
      ok = parseHexNybbleString(arg, pair, 2);
      data[0] = pair[0];
      data[1] = pair[1];
    } else if (currentIsFt857Family()) {
      ok = parseHexNybbleString(arg, data, 4);
    }
    if (!ok) {
      if (currentIsFt817Family()) Serial.println("YTONE -> FT-817 expects 4 hex digits, e.g. 0885");
      else Serial.println("YTONE -> FT-857/897 expects 8 hex digits, e.g. 08851000");
      return true;
    }
    yaesuCatSetCtcssToneRaw(data);
    Serial.print("YTONE RAW: ");
    const uint8_t frame[5] = {data[0], data[1], data[2], data[3], 0x0B};
    yaesuCatPrintFrame(frame);
    Serial.println();
    return true;
  }

  if (upper == "CTCSS?") {
    const uint16_t toneTenths =
        live.ctcssValid ? live.ctcssTenths : currentProfile().ft8x7Bank6.ctcssDefaultTenths;
    char label[12] = "";
    formatCtcssTenthsLabel(toneTenths, label, sizeof(label));
    Serial.print("CTCSS ");
    Serial.print(label);
    Serial.println(live.ctcssValid ? " Hz (last set)" : " Hz (profile default)");
    if (g_speechEnabled) {
      speakLabel("ctcss");
      speakDigitsAndPoint(label);
    }
    return true;
  }
  if (upper.startsWith("CTCSS ")) {
    const float hz = line.substring(6).toFloat();
    const uint16_t toneTenths = (uint16_t)(hz * 10.0f + 0.5f);
    if (!yaesuCtcssTenthsValid(toneTenths)) {
      Serial.println("CTCSS -> invalid (use a standard tone in Hz, e.g. 88.5)");
      return true;
    }
    if (!yaesuCatSetCtcssTenths(toneTenths)) { reportCommandFailure("CTCSS", "failed"); return true; }
    char label[12] = "";
    formatCtcssTenthsLabel(toneTenths, label, sizeof(label));
    Serial.print("CTCSS ");
    Serial.print(label);
    Serial.println(" Hz");
    if (g_speechEnabled) {
      speakLabel("ctcss");
      speakDigitsAndPoint(label);
    }
    return true;
  }
  if (upper == "DCS?") {
    const uint16_t dcsCode = live.dcsValid ? live.dcsCode : currentProfile().ft8x7Bank6.dcsDefaultCode;
    char label[8] = "";
    snprintf(label, sizeof(label), "%03u", (unsigned)dcsCode);
    Serial.print("DCS ");
    Serial.print(label);
    Serial.println(live.dcsValid ? " (last set)" : " (profile default)");
    if (g_speechEnabled) {
      speakLabel("dcs");
      speakDigitsAndPoint(label);
    }
    return true;
  }
  if (upper.startsWith("DCS ")) {
    const uint16_t dcsCode = (uint16_t)line.substring(4).toInt();
    if (!yaesuDcsCodeValid(dcsCode)) {
      Serial.println("DCS -> invalid (use a standard code, e.g. 023)");
      return true;
    }
    if (!yaesuCatSetDcsCode(dcsCode)) { reportCommandFailure("DCS", "failed"); return true; }
    char label[8] = "";
    snprintf(label, sizeof(label), "%03u", (unsigned)dcsCode);
    Serial.print("DCS ");
    Serial.println(label);
    if (g_speechEnabled) {
      speakLabel("dcs");
      speakDigitsAndPoint(label);
    }
    return true;
  }
  // The FT-817 cannot report the active VFO; this corrects the local tracking.
  if (upper == "VFO SYNC A" || upper == "VFO SYNC B") {
    const bool vfoA = upper == "VFO SYNC A";
    rememberActiveVfo(vfoA);
    Serial.println(vfoA ? "SYNC VFOA" : "SYNC VFOB");
    if (g_speechEnabled) {
      speakToken("sync");
      playSilenceMs(60);
      speakVfoLabel(vfoA ? 'A' : 'B');
    }
    return true;
  }

  if (upper.startsWith("YDCS ")) {
    uint8_t data[4] = {0};
    String arg = line.substring(5);
    bool ok = false;
    if (currentIsFt817Family()) {
      uint8_t pair[2] = {0};
      ok = parseHexNybbleString(arg, pair, 2);
      data[0] = pair[0];
      data[1] = pair[1];
    } else if (currentIsFt857Family()) {
      ok = parseHexNybbleString(arg, data, 4);
    }
    if (!ok) {
      if (currentIsFt817Family()) Serial.println("YDCS -> FT-817 expects 4 hex digits, e.g. 0023");
      else Serial.println("YDCS -> FT-857/897 expects 8 hex digits, e.g. 00230371");
      return true;
    }
    yaesuCatSetDcsCodeRaw(data);
    Serial.print("YDCS RAW: ");
    const uint8_t frame[5] = {data[0], data[1], data[2], data[3], 0x0C};
    yaesuCatPrintFrame(frame);
    Serial.println();
    return true;
  }

  if (upper == "YPOWER ON" || upper == "YPOWER OFF") {
    if (!currentIsFt817Family()) {
      Serial.println("YPOWER -> documented only for the FT-817/818");
      return true;
    }
    const bool on = (upper == "YPOWER ON");
    yaesuCatSetPowerDocumentedRaw(on);
    Serial.println(on ? "YPOWER ON" : "YPOWER OFF");
    return true;
  }

  if (upper == "YRPT MINUS" || upper == "YRPT PLUS" || upper == "YRPT SIMPLEX") {
    uint8_t shiftByte = 0x89;
    if (upper == "YRPT MINUS") shiftByte = 0x09;
    else if (upper == "YRPT PLUS") shiftByte = 0x49;
    yaesuCatSetRepeaterShiftRaw(shiftByte);
    Serial.print("YRPT RAW: ");
    const uint8_t frame[5] = {shiftByte, 0x00, 0x00, 0x00, 0x09};
    yaesuCatPrintFrame(frame);
    Serial.println();
    return true;
  }

  if (upper.startsWith("YRPTSHIFT ")) {
    String arg = line.substring(10);
    arg.trim();
    char* endPtr = nullptr;
    const double mhz = strtod(arg.c_str(), &endPtr);
    if (!arg.length() || *endPtr != '\0' || mhz < 0.0) {
      Serial.println("YRPTSHIFT -> use MHz, e.g. 0, 0.600 or 5.000");
      return true;
    }
    uint64_t hz = (uint64_t)(mhz * 1000000.0 + 0.5);
    yaesuCatSetRepeaterOffsetHzRaw(hz);
    uint8_t frame[5] = {0x00, 0x00, 0x00, 0x00, 0xF9};
    yaesuCatEncodeFreqHz(hz, frame);
    Serial.print("YRPTSHIFT RAW: ");
    yaesuCatPrintFrame(frame);
    Serial.print("  (");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz)");
    return true;
  }

  if (upper == "YLOCKRAW ON" || upper == "YLOCKRAW OFF") {
    const bool on = (upper == "YLOCKRAW ON");
    yaesuCatSetLockDocumentedRaw(on);
    rememberDialLockState(on);
    Serial.println(on ? "YLOCKRAW ON -> sent documented FT8x7 raw bytes 00 00 00 00 00"
                      : "YLOCKRAW OFF -> sent documented FT8x7 raw bytes 00 00 00 00 80");
    return true;
  }

  if (upper == "YMODEBYTE?") {
    uint8_t modeByte = 0;
    if (!yaesuCatQueryModeRawByte(modeByte, 800)) {
      reportCommandFailure("YMODEBYTE?", "no reply");
      return true;
    }
    Serial.print("YMODEBYTE: 0x");
    if (modeByte < 0x10) Serial.print('0');
    Serial.println(modeByte, HEX);
    return true;
  }

  if (upper.startsWith("YSETMODE ")) {
    uint8_t modeByte = 0;
    if (!parseHexByteString(line.substring(9), modeByte)) {
      Serial.println("YSETMODE -> use hex byte, e.g. 08, 88, 0A, 0C");
      return true;
    }
    const bool ft817Debug = currentIsFt817Family();
    const Ft817DebugPollingHold pollingHold(ft817Debug);
    if (ft817Debug) {
      const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x07};
      Serial.print("YSETMODE CMD: ");
      yaesuCatPrintFrame(cmd);
      Serial.println();
      probeYaesuFt817ModeTxRx("YSETMODE BEFORE");
    }
    yaesuCatSetModeRawByte(modeByte);
    Serial.print("YSETMODE 0x");
    if (modeByte < 0x10) Serial.print('0');
    Serial.println(modeByte, HEX);
    if (ft817Debug) {
      delay(320);
      probeYaesuFt817ModeTxRx("YSETMODE AFTER");
    }
    return true;
  }

  if (upper.startsWith("YSETMODEQ ")) {
    uint8_t modeByte = 0;
    if (!parseHexByteString(line.substring(10), modeByte)) {
      Serial.println("YSETMODEQ -> use hex byte, e.g. 08, 04, 02");
      return true;
    }
    const bool ft817Quiet = currentIsFt817Family();
    const Ft817DebugPollingHold pollingHold(ft817Quiet);
    yaesuCatFlushInput();
    delay(120);
    yaesuCatSetModeRawByte(modeByte);
    Serial.print("YSETMODEQ 0x");
    if (modeByte < 0x10) Serial.print('0');
    Serial.println(modeByte, HEX);
    Serial.println("  quiet wait...");
    delay(ft817Quiet ? 1400 : 600);
    return true;
  }

  if (upper == "YFMCTX?") {
    if (!currentIsFt817Family()) {
      Serial.println("YFMCTX? -> FT-817 only");
      return true;
    }
    printYaesuFt817FmContext();
    return true;
  }

  if (upper == "YTRACE ON" || upper == "YTRACE OFF") {
    g_yaesuCatTrace = (upper == "YTRACE ON");
    Serial.println(g_yaesuCatTrace ? "YTRACE ON" : "YTRACE OFF");
    return true;
  }
  if (upper.startsWith("YSNIFF")) {
    uint32_t windowMs = 1000;
    String arg = line.substring(6);
    arg.trim();
    if (arg.length()) windowMs = (uint32_t)constrain(arg.toInt(), 1, 10000);
    bool savedTrace = g_yaesuCatTrace;
    g_yaesuCatTrace = true;
    Serial.print("YSNIFF ");
    Serial.print(windowMs);
    Serial.println(" ms");
    yaesuCatSniff(windowMs);
    g_yaesuCatTrace = savedTrace;
    return true;
  }

  if (upper == "YRXSTATUS?") {
    uint8_t raw = 0;
    if (!yaesuCatQueryRxStatusRaw(raw, 800)) { reportCommandFailure("YRXSTATUS?", "no reply"); return true; }
    Serial.print("YRXSTATUS: 0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
    return true;
  }
  if (upper == "YTXSTATUS?" || upper == "YSTATUS?") {
    uint8_t raw = 0;
    if (!yaesuCatQueryTxStatusRaw(raw, 800)) {
      reportCommandFailure((upper == "YSTATUS?") ? "YSTATUS?" : "YTXSTATUS?", "no reply");
      return true;
    }
    Serial.print((upper == "YSTATUS?") ? "YSTATUS: 0x" : "YTXSTATUS: 0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
    return true;
  }
  if (upper == "RXTX?" && currentIsFt857Family()) {
    bool tx = false;
    if (!queryRxTxStatus(tx, 800)) { reportCommandFailure("RXTX?", "no reply"); return true; }
    Serial.println(tx ? "TX" : "RX");
    return true;
  }
  if (upper == "VOL?") {
    if (currentIsFt857Family()) {
      reportNotAvailable("VOL? -> unsupported on verified FT-857/897 path");
      return true;
    }
    int32_t raw = 0;
    if (!yaesuCatQueryVolumeRaw(raw, 800)) { reportCommandFailure("VOL?", "no reply"); return true; }
    Serial.print("VOL: ");
    Serial.println(raw);
    return true;
  }
  if (upper == "SQL?") {
    if (currentIsFt857Family()) {
      reportNotAvailable("SQL? -> unsupported on verified FT-857/897 path");
      return true;
    }
    int32_t raw = 0;
    if (!yaesuCatQuerySquelchRaw(raw, 800)) { reportCommandFailure("SQL?", "no reply"); return true; }
    Serial.print("SQL: ");
    Serial.println(raw);
    return true;
  }
  if (upper == "VFO TOGGLE") {
    yaesuCatToggleVfo();
    Serial.println("VFO TOGGLE");
    return true;
  }
  if (upper == "VFO A=B") {
    if (!currentIsFt817Family()) {
      Serial.println("VFO A=B -> enabled only for FT-817");
      return true;
    }
    if (!ft8x7CopyActiveVfoToOther()) { reportCommandFailure("VFO A=B", "failed"); return true; }
    Serial.println("A=B");
    if (g_speechEnabled) {
      speakToken("a");
      playSilenceMs(60);
      speakToken("equals");
      playSilenceMs(60);
      speakToken("b");
    }
    return true;
  }
  if (upper == "VFO A") {
    if (!currentIsFt817Family()) {
      Serial.println("VFO A -> raw FT8x7 VFO select is currently enabled only for FT-817");
      return true;
    }
    yaesuCatSelectVfoA();
    Serial.println("VFO A");
    return true;
  }
  if (upper == "VFO B") {
    if (!currentIsFt817Family()) {
      Serial.println("VFO B -> raw FT8x7 VFO select is currently enabled only for FT-817");
      return true;
    }
    yaesuCatSelectVfoB();
    Serial.println("VFO B");
    return true;
  }
  if (upper == "PTT ON") {
    yaesuCatSetPtt(true);
    Serial.println("PTT ON");
    return true;
  }
  if (upper == "PTT OFF") {
    yaesuCatSetPtt(false);
    Serial.println("PTT OFF");
    return true;
  }
  if (upper == "SPLIT ON") {
    yaesuCatSetSplit(true);
    Serial.println("SPLIT ON");
    return true;
  }
  if (upper == "SPLIT OFF") {
    yaesuCatSetSplit(false);
    Serial.println("SPLIT OFF");
    return true;
  }
  // The clarifier commands switch RIT; the radio answers whether it switched.
  if (upper == "CLAR ON" || upper == "CLAR OFF") {
    if (!yaesuFt8x7SetRit(upper == "CLAR ON", YAESU_CAT_REPLY_TIMEOUT_MS)) {
      reportCommandFailure(upper.c_str(), "failed");
      return true;
    }
    Serial.println(upper);
    return true;
  }
  if (upper.startsWith("CLAR OFFSET ")) {
    uint8_t data[4] = {0};
    if (!parseHexNybbleString(line.substring(12), data, 4)) {
      Serial.println("CLAR OFFSET -> use 8 hex digits, e.g. 00000123");
      return true;
    }
    yaesuCatSetClarifierOffsetRaw(data);
    Serial.print("CLAR OFFSET RAW: ");
    const uint8_t frame[5] = {data[0], data[1], data[2], data[3], 0xF5};
    yaesuCatPrintFrame(frame);
    Serial.println();
    return true;
  }
  if (upper == "LOCKDOC ON") {
    yaesuCatSetLockDocumentedRaw(true);
    rememberDialLockState(true);
    Serial.println("LOCKDOC ON -> sent documented raw bytes 00 00 00 00 00");
    return true;
  }
  if (upper == "LOCKDOC OFF") {
    yaesuCatSetLockDocumentedRaw(false);
    rememberDialLockState(false);
    Serial.println("LOCKDOC OFF -> sent documented raw bytes 00 00 00 00 80");
    return true;
  }
  if (upper == "MEM WRITE") {
    reportNotAvailable("MEM WRITE -> unsupported on FT8x7 CAT; command disabled to protect radio settings");
    return true;
  }
  if (upper == "MEM READ RAW") {
    reportNotAvailable("MEM READ RAW -> unsupported on FT8x7 CAT; opcode is write-only tone data, not memory read");
    return true;
  }
  if (upper.startsWith("AGC ")) {
    uint8_t modeByte = 0;
    if (!parseHexByteString(line.substring(4), modeByte)) {
      Serial.println("AGC -> use hex byte, e.g. 00 or 02");
      return true;
    }
    yaesuCatSetAgcMode(modeByte);
    Serial.print("AGC 0x");
    if (modeByte < 0x10) Serial.print('0');
    Serial.println(modeByte, HEX);
    return true;
  }
  if (upper.startsWith("YCAT ")) {
    uint8_t cmd[5] = {0};
    if (!parseHexNybbleString(line.substring(5), cmd, 5)) {
      Serial.println("YCAT -> use 10 hex digits, e.g. 0000000003");
      return true;
    }
    yaesuCatFlushInput();
    yaesuCatSend5(cmd);
    Serial.print("YCAT SENT: ");
    yaesuCatPrintFrame(cmd);
    Serial.println();
    return true;
  }
  if (upper.startsWith("YCAT? ")) {
    uint8_t cmd[5] = {0};
    uint8_t rsp[5] = {0};
    if (!parseHexNybbleString(line.substring(6), cmd, 5)) {
      Serial.println("YCAT? -> use 10 hex digits, e.g. 0000000003");
      return true;
    }
    if (!yaesuCatTransact5(cmd, rsp, 800)) {
      reportCommandFailure("YCAT?", "no reply");
      return true;
    }
    Serial.print("YCAT RX: ");
    yaesuCatPrintFrame(rsp);
    Serial.println();
    return true;
  }
  if (upper.startsWith("YCAT1? ")) {
    uint8_t opcode = 0;
    if (!parseHexByteString(line.substring(7), opcode)) {
      Serial.println("YCAT1? -> use hex byte, e.g. E7 or F7");
      return true;
    }
    const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, opcode};
    uint8_t rsp = 0;
    if (!yaesuCatTransact1(cmd, rsp, 800)) {
      reportCommandFailure("YCAT1?", "no reply");
      return true;
    }
    Serial.print("YCAT1 RX 0x");
    if (opcode < 0x10) Serial.print('0');
    Serial.print(opcode, HEX);
    Serial.print(": 0x");
    if (rsp < 0x10) Serial.print('0');
    Serial.println(rsp, HEX);
    return true;
  }
  if (upper == "YSETTINGS?") {
    // The on/off settings the model keeps.
    struct NamedFlag { const char* name; Ft8x7Flag flag; };
    static const NamedFlag flags[] = {
      {"VOX", Ft8x7Flag::Vox},       {"PROC", Ft8x7Flag::Proc},   {"LOCK", Ft8x7Flag::Lock},
      {"FAST", Ft8x7Flag::FastTuning}, {"NB", Ft8x7Flag::Nb},      {"BK", Ft8x7Flag::BreakIn},
      {"KYR", Ft8x7Flag::Keyer},     {"DSPROW", Ft8x7Flag::DspRow}, {"IFSHIFT", Ft8x7Flag::IfShift},
    };
    for (const NamedFlag& f : flags) {
      if (!yaesuFt8x7HasFlag(f.flag)) continue;
      bool on = false;
      Serial.print(f.name);
      Serial.println(yaesuFt8x7QueryFlag(f.flag, on, 300) ? (on ? " ON" : " OFF") : " --");
    }
    if (!currentIsFt857Family()) return true;
    bool on = false;
    Serial.print("FILTER ");
    Serial.println(yaesuFt8x7QueryFlag(Ft8x7Flag::Filter2, on, 300) ? (on ? "2" : "BUILTIN") : "--");
    Serial.print("KNOB ");
    Serial.println(yaesuFt8x7QueryFlag(Ft8x7Flag::KnobIsSquelch, on, 300) ? (on ? "SQL" : "RFGAIN") : "--");
    static const char* const kMicEqNames[] = {"OFF", "LPF", "HPF", "BOTH"};
    YaesuFt857MicEq micEq = YaesuFt857MicEq::Off;
    Serial.print("MICEQ ");
    Serial.println(yaesuFt857QueryMicEq(micEq, 300) ? kMicEqNames[(uint8_t)micEq] : "--");
    struct NamedLevel { const char* name; YaesuFt857Level level; };
    static const NamedLevel levels[] = {
      {"CWSPEED", YaesuFt857Level::CwSpeed},     {"AMMIC", YaesuFt857Level::AmMicGain},
      {"DIGGAIN", YaesuFt857Level::DigGain},     {"DIGVOX", YaesuFt857Level::DigVox},
      {"BPF", YaesuFt857Level::BpfWidth},        {"HPF", YaesuFt857Level::HpfCutoff},
      {"LPF", YaesuFt857Level::LpfCutoff},       {"NRLEVEL", YaesuFt857Level::NrLevel},
      {"FMMIC", YaesuFt857Level::FmMicGain},     {"NBLEVEL", YaesuFt857Level::NbLevel},
      {"PKT1200", YaesuFt857Level::Pkt1200},     {"PKT9600", YaesuFt857Level::Pkt9600},
      {"PROCLEVEL", YaesuFt857Level::ProcLevel}, {"SSBMIC", YaesuFt857Level::SsbMicGain},
      {"VOXDELAY", YaesuFt857Level::VoxDelay},   {"VOXGAIN", YaesuFt857Level::VoxGain},
    };
    for (const NamedLevel& l : levels) {
      uint16_t value = 0;
      Serial.print(l.name);
      Serial.print(' ');
      if (yaesuFt857QueryLevel(l.level, value, 300)) Serial.println(value);
      else Serial.println("--");
    }
    uint64_t hz = 0;
    int32_t offset = 0;
    Serial.print("RITOFFSET ");
    if (queryFrequency(hz, 800) && yaesuFt857QueryRitOffsetHz(hz, offset, 300)) Serial.println(offset);
    else Serial.println("--");
    return true;
  }
  // Few bytes at a time: an FT-897 hung when a logger read 192 bytes every 5 s while the dial
  // was being turned (the radio writes its EEPROM as it tunes).
  if (upper.startsWith("YEEPROM? ")) {
    String args = line.substring(9);
    args.trim();
    const int space = args.indexOf(' ');
    const String addrArg = space < 0 ? args : args.substring(0, space);
    char* endPtr = nullptr;
    const long start = strtol(addrArg.c_str(), &endPtr, 16);
    const long count = space < 0 ? 16 : args.substring(space + 1).toInt();
    if (!addrArg.length() || *endPtr != '\0' || start < 0 || start > 0xFFFF || count < 1 || count > 32) {
      Serial.println("YEEPROM? -> use <start hex> [count 1..32], e.g. YEEPROM? 0068 16");
      return true;
    }
    uint8_t word[2] = {0};
    long wordAddr = -1;
    for (long addr = start; addr < start + count && addr <= 0xFFFF; ++addr) {
      if ((addr - start) % 16 == 0) {
        if (addr != start) Serial.println();
        char buf[8];
        snprintf(buf, sizeof(buf), "%04lX:", addr);
        Serial.print(buf);
      }
      if ((addr & ~1L) != wordAddr) {
        wordAddr = addr & ~1L;
        if (!yaesuCatReadEepromWord((uint16_t)wordAddr, word, 300)) {
          Serial.println(" --");
          reportCommandFailure("YEEPROM?", "no reply");
          return true;
        }
      }
      Serial.print(' ');
      Serial.print(byteToUpperHex(word[addr & 1]));
    }
    Serial.println();
    return true;
  }
  // CAUTION: writes two EEPROM bytes, at addr and addr + 1. A bad write can wipe the radio's
  // memories and calibration.
  if (upper.startsWith("YEEPROM! ")) {
    String args = line.substring(9);
    args.trim();
    const int space = args.indexOf(' ');
    const String addrArg = space < 0 ? args : args.substring(0, space);
    char* endPtr = nullptr;
    const long addr = strtol(addrArg.c_str(), &endPtr, 16);
    uint8_t data[2] = {0};
    if (!addrArg.length() || *endPtr != '\0' || addr < 0 || addr > 0xFFFE || space < 0 ||
        !parseTwoHexByteArgs(args.substring(space + 1), data[0], data[1])) {
      Serial.println("YEEPROM! -> use <addr hex> <byte hex> <byte hex>, e.g. YEEPROM! 0068 1F 00");
      return true;
    }
    uint8_t txStatus = 0;
    if (!yaesuCatQueryTxStatusRaw(txStatus, 300)) {
      reportCommandFailure("YEEPROM!", "no reply");
      return true;
    }
    if (yaesuCatTxStatusTransmitting(txStatus)) {
      Serial.println("YEEPROM! -> not while transmitting");
      return true;
    }
    yaesuCatWriteEeprom2((uint16_t)addr, data);
    char buf[32];
    snprintf(buf, sizeof(buf), "YEEPROM! %04X: %02X %02X", (unsigned)addr, data[0], data[1]);
    Serial.println(buf);
    return true;
  }
  if (upper.startsWith("YSCAN1 ")) {
    uint8_t first = 0;
    uint8_t last = 0;
    if (!parseTwoHexByteArgs(line.substring(7), first, last)) {
      Serial.println("YSCAN1 -> use two hex bytes, e.g. E0 EF or 00 1F");
      return true;
    }
    if (first > last) {
      uint8_t tmp = first;
      first = last;
      last = tmp;
    }
    Serial.print("YSCAN1 0x");
    if (first < 0x10) Serial.print('0');
    Serial.print(first, HEX);
    Serial.print("..0x");
    if (last < 0x10) Serial.print('0');
    Serial.println(last, HEX);
    for (uint16_t op = first; op <= last; ++op) {
      const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)op};
      uint8_t rsp = 0;
      Serial.print("  0x");
      if (op < 0x10) Serial.print('0');
      Serial.print(op, HEX);
      Serial.print(" -> ");
      // 0xBC writes the EEPROM and 0xBE is a factory reset.
      if (op == 0xBC || op == 0xBE) {
        Serial.println("skipped");
        continue;
      }
      if (!yaesuCatTransact1(cmd, rsp, 160)) {
        Serial.println("--");
      } else {
        Serial.print("0x");
        if (rsp < 0x10) Serial.print('0');
        Serial.println(rsp, HEX);
      }
      delay(10);
    }
    return true;
  }
  return false;
}

// Relative and toggle forms of radio settings: what a keypad key does with a
// fixed step, here with the step as an argument.
static bool handleConsoleAdjustCommands(const String& line, const String& upper) {
  int32_t step = 0;
  if (upper == "ROUND" || upper.startsWith("ROUND ")) {
    uint32_t stepHz = 500;
    if (upper != "ROUND") {
      if (!parseStepArg(line, upper, "ROUND ", step) || step < 1 || step > 1000000) {
        Serial.println("ROUND -> invalid (use 1..1000000 Hz)");
        return true;
      }
      stepHz = (uint32_t)step;
    }
    uint64_t hz = 0;
    if (!queryFrequency(hz, 800)) { reportCommandFailure("ROUND", "no reply"); return true; }
    const uint64_t rounded = RadioFrequency::fromHz(hz).roundedTo(stepHz).hz();
    if (rounded != hz && !applyFrequencyAndTrack(rounded, true)) { reportCommandFailure("ROUND", "failed"); return true; }
    Serial.print("ROUND: ");
    Serial.print(hzToMHzString3(hz));
    Serial.print(" -> ");
    Serial.print(hzToMHzString3(rounded));
    Serial.println(" MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(rounded));
    rememberAnnouncedFrequency(rounded);
    return true;
  }
  if (upper.startsWith("NRLEVEL STEP")) {
    if (!parseStepArg(line, upper, "NRLEVEL STEP ", step)) { Serial.println("NRLEVEL STEP -> use a signed percent, e.g. NRLEVEL STEP 10"); return true; }
    uint16_t raw = 0;
    if (!queryNrLevel(raw, 800)) { reportCommandFailure("NRLEVEL STEP", "no reply"); return true; }
    const int percent = (int)clampInt32((int32_t)levelRawToPercent(raw) + step, 0, 100);
    bool wrote = setNrLevel(levelPercentToRaw(percent));
    if (!wrote) {
      // Some radios take the level only while NR is on.
      bool nrOn = false;
      if (queryNr(nrOn, 800) && !nrOn && setNr(true)) wrote = setNrLevel(levelPercentToRaw(percent));
    }
    if (!wrote) { reportCommandFailure("NRLEVEL STEP", "failed"); return true; }
    Serial.print("NRLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    speakFeatureValue("noisereduction", (uint8_t)percent);
    return true;
  }
  if (upper.startsWith("NBLEVEL STEP")) {
    if (!parseStepArg(line, upper, "NBLEVEL STEP ", step)) { Serial.println("NBLEVEL STEP -> use a signed percent, e.g. NBLEVEL STEP 10"); return true; }
    uint16_t raw = 0;
    if (!queryNbLevel(raw, 800)) { reportCommandFailure("NBLEVEL STEP", "no reply"); return true; }
    const int percent = (int)clampInt32((int32_t)levelRawToPercent(raw) + step, 0, 100);
    if (!setNbLevel(levelPercentToRaw(percent))) { reportCommandFailure("NBLEVEL STEP", "failed"); return true; }
    Serial.print("NBLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    speakTokenPercent("noiseblanker", (uint8_t)percent);
    return true;
  }
  if (upper.startsWith("MONLEVEL STEP")) {
    if (!parseStepArg(line, upper, "MONLEVEL STEP ", step)) { Serial.println("MONLEVEL STEP -> use a signed percent, e.g. MONLEVEL STEP 10"); return true; }
    uint16_t raw = 0;
    if (!queryMonitorLevel(raw, 800)) { reportCommandFailure("MONLEVEL STEP", "no reply"); return true; }
    const int percent = (int)clampInt32((int32_t)levelRawToPercent(raw) + step, 0, 100);
    if (!setMonitorLevel(levelPercentToRaw(percent))) { reportCommandFailure("MONLEVEL STEP", "failed"); return true; }
    Serial.print("MONLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    speakFeatureValue("monitor", (uint8_t)percent);
    return true;
  }
  const bool pbt1Step = upper.startsWith("PBT1 STEP");
  if (pbt1Step || upper.startsWith("PBT2 STEP")) {
    const char* label = pbt1Step ? "PBT1" : "PBT2";
    if (!parseStepArg(line, upper, pbt1Step ? "PBT1 STEP " : "PBT2 STEP ", step)) {
      Serial.print(label);
      Serial.println(" STEP -> use a signed step, e.g. STEP -10");
      return true;
    }
    uint16_t raw = 0;
    if (!(pbt1Step ? queryPbtInner(raw, 800) : queryPbtOuter(raw, 800))) { reportCommandFailure(label, "no reply"); return true; }
    const int value = (int)clampInt32((int32_t)pbtRawToOffset(raw) + step, -128, 127);
    if (!(pbt1Step ? setPbtInner(pbtOffsetToRaw(value)) : setPbtOuter(pbtOffsetToRaw(value)))) { reportCommandFailure(label, "failed"); return true; }
    Serial.print(label);
    Serial.print(' ');
    Serial.print(value);
    Serial.println(" step");
    speakSignedStepValue("pbt", value);
    return true;
  }
  if (upper.startsWith("RIT STEP")) {
    if (!parseStepArg(line, upper, "RIT STEP ", step)) { Serial.println("RIT STEP -> use signed Hz, e.g. RIT STEP -100"); return true; }
    int32_t offset = 0;
    if (!queryRitOffsetHz(offset, 800)) { reportCommandFailure("RIT STEP", "no reply"); return true; }
    const int32_t hz = clampInt32(offset + step, -9999, 9999);
    if (!setRitOffsetHz(hz)) { reportCommandFailure("RIT STEP", "failed"); return true; }
    Serial.print("RIT ");
    Serial.print(hz);
    Serial.println(" Hz");
    speakRitStateAndOffset(true, hz);
    return true;
  }
  if (upper == "RIT TOGGLE") {
    bool on = false;
    if (!toggleRitEnabled(on, 800)) { reportCommandFailure("RIT TOGGLE", "failed"); return true; }
    Serial.println(on ? "RIT ON" : "RIT OFF");
    speakTokenState("rit", on);
    return true;
  }
  if (upper == "MONITOR TOGGLE") {
    bool on = false;
    if (!queryMonitorEnabled(on, 800)) { reportCommandFailure("MONITOR TOGGLE", "no reply"); return true; }
    if (!setMonitorEnabled(!on)) { reportCommandFailure("MONITOR TOGGLE", "failed"); return true; }
    Serial.println(!on ? "MONITOR ON" : "MONITOR OFF");
    if (g_speechEnabled) speakTokenState("monitor", !on);
    return true;
  }
  if (upper == "TRANSCEIVE TOGGLE") {
    bool on = false;
    if (!queryTransceiveEnabled(on, 800)) { reportCommandFailure("TRANSCEIVE TOGGLE", "no reply"); return true; }
    if (!setTransceiveEnabled(!on)) { reportCommandFailure("TRANSCEIVE TOGGLE", "failed"); return true; }
    Serial.println(!on ? "TRANSCEIVE ON" : "TRANSCEIVE OFF");
    if (g_speechEnabled) speakTokenState("transceiver", !on);
    return true;
  }
  if (upper == "FILSHAPE TOGGLE") {
    bool soft = false;
    if (!queryFilterShape(soft, 800)) { reportCommandFailure("FILSHAPE TOGGLE", "no reply"); return true; }
    if (!setFilterShape(!soft)) { reportCommandFailure("FILSHAPE TOGGLE", "failed"); return true; }
    Serial.println(!soft ? "FILSHAPE SOFT" : "FILSHAPE SHARP");
    if (g_speechEnabled) {
      speakLabel("filtershape");
      speakToken(!soft ? "soft" : "sharp");
    }
    return true;
  }
  if (upper == "FILWIDTH NEXT" || upper == "FILWIDTH PREV") {
    uint8_t mode = 0xFF;
    uint8_t filter = 0xFF;
    if (!queryCurrentFilterSlot(filter) || !queryCurrentModeValue(mode)) { reportCommandFailure("FILWIDTH", "no reply"); return true; }
    int next = (int)filter + (upper == "FILWIDTH NEXT" ? 1 : -1);
    if (next < 1) next = 3;
    if (next > 3) next = 1;
    if (!setMode(mode, (uint8_t)next)) { reportCommandFailure("FILWIDTH", "failed"); return true; }
    Serial.print("FILWIDTH ");
    Serial.println(next);
    if (g_speechEnabled) {
      speakLabel("filterwidth");
      playDigit((uint8_t)next);
    }
    return true;
  }
  return false;
}

static bool handleConsoleRadioCommands(const String& line, const String& upper) {
  const RadioProfile& sp = currentProfile();
  if (upper == "LFREQ") {
    if (!live.freqValid) { Serial.println("No live frequency yet."); return true; }
    Serial.print("Last live: ");
    Serial.print(hzToMHzString3(live.freqHz));
    Serial.println(" MHz");
    return true;
  }
  if (upper == "FREQ?") {
    if (!refreshLiveFrequency()) { reportCommandFailure("FREQ?", "no reply"); return true; }
    Serial.print("Query FREQ: ");
    Serial.print(hzToMHzString3(live.freqHz));
    Serial.println(" MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(live.freqHz));
    return true;
  }
  if (upper.startsWith("FREQMHZ ")) {
    double mhz = line.substring(8).toDouble();
    if (mhz <= 0.0) { Serial.println("FREQMHZ -> invalid value"); return true; }
    uint64_t hzSet = (uint64_t)(mhz * 1000000.0 + 0.5);
    if (!applyFrequencyAndTrack(hzSet, true)) { reportCommandFailure("SET FREQ", "no reply"); return true; }
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hzSet));
    return true;
  }
  if (upper.startsWith("FREQHZ ")) {
    uint64_t hzSet = strtoull(line.substring(7).c_str(), nullptr, 10);
    if (hzSet == 0) { Serial.println("FREQHZ -> invalid value"); return true; }
    if (!applyFrequencyAndTrack(hzSet, true)) { reportCommandFailure("SET FREQ", "no reply"); return true; }
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hzSet));
    return true;
  }
  if (upper.startsWith("FREQ ")) {
    uint64_t khz = strtoull(line.substring(5).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    if (!applyFrequencyAndTrack(hzSet, true)) { reportCommandFailure("SET FREQ", "no reply"); return true; }
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hzSet));
    return true;
  }
  if (upper == "MODE?") {
    if (!refreshLiveMode()) { reportCommandFailure("MODE?", "no reply"); return true; }
    Serial.println(modeToString(live.mode));
    speakModeReply(live.mode);
    return true;
  }
  if (upper.startsWith("MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(5), mode) || !applyModeAndTrack(mode, 1)) {
      reportCommandFailure("SET MODE", "failed");
      return true;
    }
    Serial.println("SET MODE -> command sent");
    speakModeReply(mode);
    return true;
  }
  if (upper == "FB?") {
    String rsp;
    uint64_t hz = 0;
    if (!transactAsciiCommand("FB;", rsp, "FB", 800) || !parseAsciiUnsignedResponse(rsp, "FB", hz)) {
      reportCommandFailure("FB?", "no reply");
      return true;
    }
    Serial.print("FB: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    return true;
  }
  if (upper.startsWith("FBMHZ ")) {
    double mhz = line.substring(6).toDouble();
    if (mhz <= 0.0) { Serial.println("FBMHZ -> invalid value"); return true; }
    uint64_t hzSet = (uint64_t)(mhz * 1000000.0 + 0.5);
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "FB%011llu;", (unsigned long long)hzSet);
    if (!asciiPacketSendCommand(cmd)) { reportCommandFailure("SET FB", "failed"); return true; }
    Serial.println("SET FB -> command sent");
    return true;
  }
  if (upper.startsWith("FB ")) {
    uint64_t khz = strtoull(line.substring(3).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "FB%011llu;", (unsigned long long)hzSet);
    if (!asciiPacketSendCommand(cmd)) { reportCommandFailure("SET FB", "failed"); return true; }
    Serial.println("SET FB -> command sent");
    return true;
  }
  if (upper == "IF?") {
    String rsp;
    if (!asciiQueryStatusLine(sp, rsp, 800)) { reportCommandFailure("IF?", "no reply"); return true; }
    Serial.print("IF: ");
    Serial.println(rsp);
    speakConsoleTokenOrGap("if");
    return true;
  }
  if (upper == "ID?") {
    String rsp;
    if (!asciiQueryIdLine(sp, rsp, 800)) { reportCommandFailure("ID?", "no reply"); return true; }
    Serial.print("ID: ");
    printAsciiReplyPayload(rsp, sp.commands->idReplyPrefix);
    speakConsoleTokenOrGap("id");
    return true;
  }
  if (upper == "OM?") {
    String rsp;
    if (!asciiQueryOmLine(sp, rsp, 800)) { reportCommandFailure("OM?", "no reply"); return true; }
    Serial.print("OM: ");
    printAsciiReplyPayload(rsp, sp.commands->omReplyPrefix);
    return true;
  }
  if (upper == "FR?") {
    String rsp;
    if (!transactAsciiCommand("FR;", rsp, "FR", 800)) { reportCommandFailure("FR?", "no reply"); return true; }
    Serial.print("FR: ");
    printAsciiReplyPayload(rsp, "FR");
    return true;
  }
  if (upper == "FR0") {
    if (!asciiPacketSendCommand("FR0;")) { reportCommandFailure("FR0", "failed"); return true; }
    Serial.println("FR0");
    return true;
  }
  if (upper == "FT?") {
    String rsp;
    if (!transactAsciiCommand("FT;", rsp, "FT", 800)) { reportCommandFailure("FT?", "no reply"); return true; }
    Serial.print("FT: ");
    printAsciiReplyPayload(rsp, "FT");
    return true;
  }
  if (upper == "FT A") {
    if (!asciiPacketSendCommand("FT0;")) { reportCommandFailure("FT A", "failed"); return true; }
    Serial.println("FT A");
    return true;
  }
  if (upper == "FT B") {
    if (!asciiPacketSendCommand("FT1;")) { reportCommandFailure("FT B", "failed"); return true; }
    Serial.println("FT B");
    return true;
  }
  if (upper == "RX") {
    if (!asciiPacketSendCommand("RX;")) { reportCommandFailure("RX", "failed"); return true; }
    Serial.println("RX");
    return true;
  }
  if (upper == "TX") {
    if (!asciiPacketSendCommand("TX;")) { reportCommandFailure("TX", "failed"); return true; }
    Serial.println("TX");
    return true;
  }
  if (upper == "AK?") {
    String rsp;
    if (!transactAsciiCommand("AK;", rsp, "AK", 800)) { reportCommandFailure("AK?", "no reply"); return true; }
    Serial.print("AK: ");
    printAsciiReplyPayload(rsp, "AK");
    return true;
  }
  if (upper.startsWith("SWT ")) {
    int nn = line.substring(4).toInt();
    if (nn < 0 || nn > 99) { Serial.println("SWT -> invalid (use 00..99)"); return true; }
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "SWT%02d;", nn);
    if (!asciiPacketSendCommand(cmd)) { reportCommandFailure("SWT", "failed"); return true; }
    Serial.print("SENT ");
    Serial.println(cmd);
    return true;
  }
  if (upper.startsWith("SWH ")) {
    int nn = line.substring(4).toInt();
    if (nn < 0 || nn > 99) { Serial.println("SWH -> invalid (use 00..99)"); return true; }
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "SWH%02d;", nn);
    if (!asciiPacketSendCommand(cmd)) { reportCommandFailure("SWH", "failed"); return true; }
    Serial.print("SENT ");
    Serial.println(cmd);
    return true;
  }
  if (upper == "SM?") {
    if (!refreshLiveSmeter()) { reportCommandFailure("SM?", "no reply"); return true; }
    Serial.print("SM: raw=");
    Serial.print(live.smRaw);
    Serial.print("  ");
    Serial.println(live.sm.toString());
    if (g_speechEnabled) speakSValue(live.sm);
    return true;
  }
  if (upper == "SWR?") {
    if (!refreshLiveSwr()) { reportCommandFailure("SWR?", "no reply"); return true; }
    float swr = swrRawToValue(live.swrRaw);
    Serial.println(swr, 2);
    if (g_speechEnabled) {
      speakLabel("swr");
      speakDigitsAndPoint(String(swr, 2));
    }
    return true;
  }
  if (upper == "PO?") {
    if (!refreshLivePower()) { reportCommandFailure("PO?", "no reply"); return true; }
    Serial.println(live.powerRaw);
    if (g_speechEnabled) {
      speakLabel("power");
      speakDigitsAndPoint(String(live.powerRaw));
    }
    return true;
  }
  if (upper == "RFPOWER?") {
    uint16_t raw = 0;
    if (!queryRfPowerLevel(raw, 800)) { reportCommandFailure("RFPOWER?", "no reply"); return true; }
    const uint16_t watts = rfPowerRawToWatts(raw);
    Serial.print("RFPOWER ");
    Serial.print((int)watts);
    Serial.println(" W");
    if (g_speechEnabled) {
      speakLabel("power");
      speakDigitsAndPoint(String((int)watts));
      playSilenceMs(60);
      speakToken("watts");
    }
    return true;
  }
  if (upper.startsWith("RFPOWER ")) {
    int watts = line.substring(8).toInt();
    const uint16_t maxWatts = currentProfile().rfPowerMaxWatts ? currentProfile().rfPowerMaxWatts : 100;
    if (watts < 0 || watts > maxWatts) {
      Serial.print("RFPOWER -> invalid (use 0..");
      Serial.print((int)maxWatts);
      Serial.println(" W)");
      return true;
    }
    if (!setRfPowerLevel(rfPowerWattsToRaw(watts))) { reportCommandFailure("RFPOWER", "failed"); return true; }
    Serial.print("RFPOWER ");
    Serial.print(watts);
    Serial.println(" W");
    if (g_speechEnabled) {
      speakLabel("power");
      speakDigitsAndPoint(String(watts));
      playSilenceMs(60);
      speakToken("watts");
    }
    return true;
  }
  if (upper == "TUNER?") {
    bool on = false;
    if (!queryTuner(on, 800)) { reportCommandFailure("TUNER?", "no reply"); return true; }
    Serial.println(on ? "TUNER ON" : "TUNER OFF");
    speakTokenState("tuner", on);
    return true;
  }
  if (upper == "TUNER ON") {
    if (!setTuner(true)) { reportCommandFailure("TUNER ON", "failed"); return true; }
    Serial.println("TUNER ON");
    speakTokenState("tuner", true);
    return true;
  }
  if (upper == "TUNER OFF") {
    if (!setTuner(false)) { reportCommandFailure("TUNER OFF", "failed"); return true; }
    Serial.println("TUNER OFF");
    speakTokenState("tuner", false);
    return true;
  }
  if (upper == "TUNER TOGGLE") {
    bool on = false;
    if (!queryTuner(on, 800)) { reportCommandFailure("TUNER TOGGLE", "no reply"); return true; }
    if (!setTuner(!on)) { reportCommandFailure("TUNER TOGGLE", "failed"); return true; }
    Serial.println(!on ? "TUNER ON" : "TUNER OFF");
    speakTokenState("tuner", !on);
    return true;
  }
  if (upper == "TUNE") {
    if (!startTune()) { reportCommandFailure("TUNE", "failed"); return true; }
    Serial.println("TUNE");
    if (g_speechEnabled) speakToken("tune");
    return true;
  }
  if (upper == "RXTX?") {
    bool tx = false;
    if (!queryRxTxStatus(tx, 800)) { reportCommandFailure("RXTX?", "no reply"); return true; }
    Serial.println(tx ? "TX" : "RX");
    if (g_speechEnabled) speakToken(tx ? "tx" : "rx");
    return true;
  }
  if (upper == "TXFREQ?") {
    uint64_t hz = 0;
    if (!queryTxFrequency(hz, 800)) { reportCommandFailure("TXFREQ?", "no reply"); return true; }
    Serial.print("TXFREQ: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    return true;
  }
  if (upper == "VFO A") {
    if (!selectVfoA()) { reportCommandFailure("VFO A", "failed"); return true; }
    Serial.println("VFO A");
    speakVfoLabel('A');
    return true;
  }
  if (upper == "VFO B") {
    if (!selectVfoB()) { reportCommandFailure("VFO B", "failed"); return true; }
    Serial.println("VFO B");
    speakVfoLabel('B');
    return true;
  }
  if (upper == "MAIN?") {
    uint64_t hz = 0;
    if (!queryVfoFrequency(true, hz, 800)) { reportCommandFailure("MAIN?", "no reply"); return true; }
    Serial.print("MAIN: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    return true;
  }
  if (upper == "SUB?") {
    uint64_t hz = 0;
    if (!queryVfoFrequency(false, hz, 800)) { reportCommandFailure("SUB?", "no reply"); return true; }
    Serial.print("SUB: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    return true;
  }
  if (upper.startsWith("MAIN MODE?")) {
    uint8_t mode = 0xFF;
    uint8_t filter = 0xFF;
    if (!queryVfoMode(true, mode, filter, 800)) { reportCommandFailure("MAIN MODE?", "no reply"); return true; }
    Serial.print("MAIN MODE: ");
    Serial.println(modeToString(mode));
    speakModeReply(mode);
    return true;
  }
  if (upper.startsWith("SUB MODE?")) {
    uint8_t mode = 0xFF;
    uint8_t filter = 0xFF;
    if (!queryVfoMode(false, mode, filter, 800)) { reportCommandFailure("SUB MODE?", "no reply"); return true; }
    Serial.print("SUB MODE: ");
    Serial.println(modeToString(mode));
    speakModeReply(mode);
    return true;
  }
  if (upper.startsWith("MAIN MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(10), mode) || !setVfoMode(true, mode, 1)) { reportCommandFailure("MAIN MODE", "failed"); return true; }
    Serial.println("MAIN MODE -> command sent");
    return true;
  }
  if (upper.startsWith("SUB MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(9), mode) || !setVfoMode(false, mode, 1)) { reportCommandFailure("SUB MODE", "failed"); return true; }
    Serial.println("SUB MODE -> command sent");
    return true;
  }
  if (upper.startsWith("MAIN ")) {
    uint64_t khz = strtoull(line.substring(5).c_str(), nullptr, 10);
    if (!setVfoFrequency(true, khz * 1000ULL)) { reportCommandFailure("MAIN", "failed"); return true; }
    return true;
  }
  if (upper.startsWith("SUB ")) {
    uint64_t khz = strtoull(line.substring(4).c_str(), nullptr, 10);
    if (!setVfoFrequency(false, khz * 1000ULL)) { reportCommandFailure("SUB", "failed"); return true; }
    return true;
  }
  if (upper == "VFOA?") {
    uint64_t hz = 0;
    if (!queryVfoFrequency(true, hz, 800)) { reportCommandFailure("VFOA?", "no reply"); return true; }
    Serial.print("VFOA: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    speakVfoFrequency('A', hz);
    return true;
  }
  if (upper == "VFOB?") {
    uint64_t hz = 0;
    if (!queryVfoFrequency(false, hz, 800)) { reportCommandFailure("VFOB?", "no reply"); return true; }
    Serial.print("VFOB: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    speakVfoFrequency('B', hz);
    return true;
  }
  if (upper.startsWith("VFOA MODE?")) {
    uint8_t mode = 0xFF;
    uint8_t filter = 0xFF;
    if (!queryVfoMode(true, mode, filter, 800)) { reportCommandFailure("VFOA MODE?", "no reply"); return true; }
    Serial.print("VFOA MODE: ");
    Serial.println(modeToString(mode));
    speakModeReply(mode);
    return true;
  }
  if (upper.startsWith("VFOB MODE?")) {
    uint8_t mode = 0xFF;
    uint8_t filter = 0xFF;
    if (!queryVfoMode(false, mode, filter, 800)) { reportCommandFailure("VFOB MODE?", "no reply"); return true; }
    Serial.print("VFOB MODE: ");
    Serial.println(modeToString(mode));
    speakModeReply(mode);
    return true;
  }
  if (upper.startsWith("VFOA MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(10), mode) || !setVfoMode(true, mode, 1)) { reportCommandFailure("VFOA MODE", "failed"); return true; }
    Serial.println("VFOA MODE -> command sent");
    return true;
  }
  if (upper.startsWith("VFOB MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(10), mode) || !setVfoMode(false, mode, 1)) { reportCommandFailure("VFOB MODE", "failed"); return true; }
    Serial.println("VFOB MODE -> command sent");
    return true;
  }
  if (upper.startsWith("VFOAHZ ")) {
    uint64_t hzSet = strtoull(line.substring(7).c_str(), nullptr, 10);
    if (!setVfoFrequency(true, hzSet)) { reportCommandFailure("VFOA", "failed"); return true; }
    return true;
  }
  if (upper.startsWith("VFOBHZ ")) {
    uint64_t hzSet = strtoull(line.substring(7).c_str(), nullptr, 10);
    if (!setVfoFrequency(false, hzSet)) { reportCommandFailure("VFOB", "failed"); return true; }
    return true;
  }
  if (upper.startsWith("VFOA ")) {
    uint64_t khz = strtoull(line.substring(5).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    if (!setVfoFrequency(true, hzSet)) { reportCommandFailure("VFOA", "failed"); return true; }
    return true;
  }
  if (upper.startsWith("VFOB ")) {
    uint64_t khz = strtoull(line.substring(5).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    if (!setVfoFrequency(false, hzSet)) { reportCommandFailure("VFOB", "failed"); return true; }
    return true;
  }
  if (upper == "SPLIT?") {
    bool on = false;
    if (!querySplit(on, 800)) { reportCommandFailure("SPLIT?", "no reply"); return true; }
    Serial.println(on ? "SPLIT ON" : "SPLIT OFF");
    speakTokenState("split", on);
    return true;
  }
  if (upper == "SPLIT ON") {
    if (!setSplit(true)) { reportCommandFailure("SPLIT ON", "failed"); return true; }
    Serial.println("SPLIT ON");
    speakTokenState("split", true);
    return true;
  }
  if (upper == "SPLIT OFF") {
    if (!setSplit(false)) { reportCommandFailure("SPLIT OFF", "failed"); return true; }
    Serial.println("SPLIT OFF");
    speakTokenState("split", false);
    return true;
  }
  if (upper == "RIT?") {
    bool on = false;
    int32_t offset = 0;
    if (!queryRitEnabled(on, 800)) { reportCommandFailure("RIT?", "no reply"); return true; }
    if (!queryRitOffsetHz(offset, 800)) {
      Serial.println(on ? "RIT ON" : "RIT OFF");
      speakTokenState("rit", on);
      return true;
    }
    Serial.print(on ? "RIT ON " : "RIT OFF ");
    Serial.print(offset);
    Serial.println(" Hz");
    speakRitStateAndOffset(on, offset);
    return true;
  }
  if (upper == "RIT ON") {
    if (!setRitEnabled(true)) { reportCommandFailure("RIT ON", "failed"); return true; }
    Serial.println("RIT ON");
    speakTokenState("rit", true);
    return true;
  }
  if (upper == "RIT OFF") {
    if (!setRitEnabled(false)) { reportCommandFailure("RIT OFF", "failed"); return true; }
    Serial.println("RIT OFF");
    speakTokenState("rit", false);
    return true;
  }
  if (upper.startsWith("RIT ")) {
    int32_t hz = line.substring(4).toInt();
    if (hz < -9999 || hz > 9999) { Serial.println("RIT -> invalid (use -9999..9999 Hz)"); return true; }
    if (!setRitOffsetHz(hz)) { reportCommandFailure("RIT", "failed"); return true; }
    Serial.print("RIT ");
    Serial.print(hz);
    Serial.println(" Hz");
    speakRitStateAndOffset(true, hz);
    return true;
  }
  if (handleConsoleFeatureCommand(upper)) return true;
  if (upper == "PBT1?") {
    uint16_t raw = 0;
    if (!queryPbtInner(raw, 800)) { reportCommandFailure("PBT1?", "no reply"); return true; }
    Serial.print("PBT1 ");
    Serial.print(pbtRawToOffset(raw));
    Serial.println(" step");
    speakSignedStepValue("pbt", pbtRawToOffset(raw));
    return true;
  }
  if (upper == "PBT2?") {
    uint16_t raw = 0;
    if (!queryPbtOuter(raw, 800)) { reportCommandFailure("PBT2?", "no reply"); return true; }
    Serial.print("PBT2 ");
    Serial.print(pbtRawToOffset(raw));
    Serial.println(" step");
    speakSignedStepValue("pbt", pbtRawToOffset(raw));
    return true;
  }
  if (upper == "LOCK?") {
    bool on = false;
    if (!queryDialLock(on, 800)) { reportCommandFailure("LOCK?", "no reply"); return true; }
    Serial.println(on ? "LOCK ON" : "LOCK OFF");
    speakTokenState("lock", on);
    return true;
  }
  if (upper == "LOCK ON") {
    if (!setDialLock(true)) { reportCommandFailure("LOCK ON", "failed"); return true; }
    Serial.println("LOCK ON");
    speakTokenState("lock", true);
    return true;
  }
  if (upper == "LOCK OFF") {
    if (!setDialLock(false)) { reportCommandFailure("LOCK OFF", "failed"); return true; }
    Serial.println("LOCK OFF");
    speakTokenState("lock", false);
    return true;
  }
  if (upper == "LOCK TOGGLE") {
    bool on = false;
    if (!queryDialLock(on, 800)) { reportCommandFailure("LOCK TOGGLE", "no reply"); return true; }
    if (!setDialLock(!on)) { reportCommandFailure("LOCK TOGGLE", "failed"); return true; }
    Serial.println(!on ? "LOCK ON" : "LOCK OFF");
    speakTokenState("lock", !on);
    return true;
  }
  if (upper == "FILSHAPE?") {
    bool soft = false;
    if (!queryFilterShape(soft, 800)) { reportCommandFailure("FILSHAPE?", "no reply"); return true; }
    Serial.println(soft ? "FILSHAPE SOFT" : "FILSHAPE SHARP");
    if (g_speechEnabled) {
      speakLabel("filtershape");
      speakToken(soft ? "soft" : "sharp");
    }
    return true;
  }
  if (upper == "FILSHAPE SHARP") {
    if (!setFilterShape(false)) { reportCommandFailure("FILSHAPE SHARP", "failed"); return true; }
    Serial.println("FILSHAPE SHARP");
    if (g_speechEnabled) {
      speakLabel("filtershape");
      speakToken("sharp");
    }
    return true;
  }
  if (upper == "FILSHAPE SOFT") {
    if (!setFilterShape(true)) { reportCommandFailure("FILSHAPE SOFT", "failed"); return true; }
    Serial.println("FILSHAPE SOFT");
    if (g_speechEnabled) {
      speakLabel("filtershape");
      speakToken("soft");
    }
    return true;
  }
  if (upper == "FILWIDTH?") {
    uint8_t filter = 0xFF;
    if (!queryCurrentFilterSlot(filter)) { reportCommandFailure("FILWIDTH?", "no reply"); return true; }
    Serial.print("FILWIDTH ");
    Serial.println((int)filter);
    if (g_speechEnabled) {
      speakLabel("filterwidth");
      playDigit(filter);
    }
    return true;
  }
  if (upper == "MONITOR?") {
    bool on = false;
    if (!queryMonitorEnabled(on, 800)) { reportCommandFailure("MONITOR?", "no reply"); return true; }
    Serial.println(on ? "MONITOR ON" : "MONITOR OFF");
    if (g_speechEnabled) speakTokenState("monitor", on);
    return true;
  }
  if (upper == "MONITOR ON") {
    if (!setMonitorEnabled(true)) { reportCommandFailure("MONITOR ON", "failed"); return true; }
    Serial.println("MONITOR ON");
    if (g_speechEnabled) speakTokenState("monitor", true);
    return true;
  }
  if (upper == "MONITOR OFF") {
    if (!setMonitorEnabled(false)) { reportCommandFailure("MONITOR OFF", "failed"); return true; }
    Serial.println("MONITOR OFF");
    if (g_speechEnabled) speakTokenState("monitor", false);
    return true;
  }
  if (upper == "MONLEVEL?") {
    uint16_t raw = 0;
    if (!queryMonitorLevel(raw, 800)) { reportCommandFailure("MONLEVEL?", "no reply"); return true; }
    Serial.print("MONLEVEL ");
    Serial.print((int)levelRawToPercent(raw));
    Serial.println("%");
    speakFeatureValue("monitor", levelRawToPercent(raw));
    return true;
  }
  if (upper == "TRANSCEIVE?") {
    bool on = false;
    if (!queryTransceiveEnabled(on, 800)) { reportCommandFailure("TRANSCEIVE?", "no reply"); return true; }
    Serial.println(on ? "TRANSCEIVE ON" : "TRANSCEIVE OFF");
    if (g_speechEnabled) speakTokenState("transceiver", on);
    return true;
  }
  if (upper == "NRLEVEL?") {
    uint16_t raw = 0;
    if (!queryNrLevel(raw, 800)) { reportCommandFailure("NRLEVEL?", "no reply"); return true; }
    Serial.print("NRLEVEL ");
    Serial.print((int)levelRawToPercent(raw));
    Serial.println("%");
    speakFeatureValue("noisereduction", levelRawToPercent(raw));
    return true;
  }
  if (upper.startsWith("NRLEVEL ")) {
    int percent = line.substring(8).toInt();
    if (percent < 0 || percent > 100) { Serial.println("NRLEVEL -> invalid (use 0..100)"); return true; }
    if (!setNrLevel(levelPercentToRaw(percent))) { reportCommandFailure("NRLEVEL", "failed"); return true; }
    Serial.print("NRLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    speakFeatureValue("noisereduction", (uint8_t)percent);
    return true;
  }
  if (upper == "NBLEVEL?") {
    uint16_t raw = 0;
    if (!queryNbLevel(raw, 800)) { reportCommandFailure("NBLEVEL?", "no reply"); return true; }
    Serial.print("NBLEVEL ");
    Serial.print((int)levelRawToPercent(raw));
    Serial.println("%");
    if (g_speechEnabled) {
      speakLabel("noiseblanker");
      speakDigitsAndPoint(String((int)levelRawToPercent(raw)));
      playSilenceMs(60);
      speakToken("percent");
    }
    return true;
  }
  if (upper.startsWith("NBLEVEL ")) {
    int percent = line.substring(8).toInt();
    if (percent < 0 || percent > 100) { Serial.println("NBLEVEL -> invalid (use 0..100)"); return true; }
    if (!setNbLevel(levelPercentToRaw(percent))) { reportCommandFailure("NBLEVEL", "failed"); return true; }
    Serial.print("NBLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    if (g_speechEnabled) {
      speakLabel("noiseblanker");
      speakDigitsAndPoint(String(percent));
      playSilenceMs(60);
      speakToken("percent");
    }
    return true;
  }
  if (upper.startsWith("PBT1 ")) {
    String arg = upper.substring(5);
    int value = (arg == "CENTER") ? 0 : line.substring(5).toInt();
    if (arg != "CENTER" && (value < -128 || value > 127)) { Serial.println("PBT1 -> invalid (use CENTER or -128..127)"); return true; }
    if (!setPbtInner(pbtOffsetToRaw(value))) { reportCommandFailure("PBT1", "failed"); return true; }
    Serial.print("PBT1 ");
    Serial.print(value);
    Serial.println(" step");
    speakSignedStepValue("pbt", value);
    return true;
  }
  if (upper.startsWith("PBT2 ")) {
    String arg = upper.substring(5);
    int value = (arg == "CENTER") ? 0 : line.substring(5).toInt();
    if (arg != "CENTER" && (value < -128 || value > 127)) { Serial.println("PBT2 -> invalid (use CENTER or -128..127)"); return true; }
    if (!setPbtOuter(pbtOffsetToRaw(value))) { reportCommandFailure("PBT2", "failed"); return true; }
    Serial.print("PBT2 ");
    Serial.print(value);
    Serial.println(" step");
    speakSignedStepValue("pbt", value);
    return true;
  }
  if (upper.startsWith("FILWIDTH ")) {
    int filter = line.substring(9).toInt();
    uint8_t mode = 0xFF;
    if (filter < 1 || filter > 3) { Serial.println("FILWIDTH -> invalid (use 1..3)"); return true; }
    if (!queryCurrentModeValue(mode)) { Serial.println("FILWIDTH -> no mode"); return true; }
    if (!setMode(mode, (uint8_t)filter)) { reportCommandFailure("FILWIDTH", "failed"); return true; }
    Serial.print("FILWIDTH ");
    Serial.println(filter);
    if (g_speechEnabled) {
      speakLabel("filterwidth");
      playDigit((uint8_t)filter);
    }
    return true;
  }
  if (upper.startsWith("MONLEVEL ")) {
    int percent = line.substring(9).toInt();
    if (percent < 0 || percent > 100) { Serial.println("MONLEVEL -> invalid (use 0..100)"); return true; }
    if (!setMonitorLevel(levelPercentToRaw(percent))) { reportCommandFailure("MONLEVEL", "failed"); return true; }
    Serial.print("MONLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    speakFeatureValue("monitor", (uint8_t)percent);
    return true;
  }
  if (upper == "TRANSCEIVE ON") {
    if (!setTransceiveEnabled(true)) { reportCommandFailure("TRANSCEIVE ON", "failed"); return true; }
    Serial.println("TRANSCEIVE ON");
    if (g_speechEnabled) speakTokenState("transceiver", true);
    return true;
  }
  if (upper == "TRANSCEIVE OFF") {
    if (!setTransceiveEnabled(false)) { reportCommandFailure("TRANSCEIVE OFF", "failed"); return true; }
    Serial.println("TRANSCEIVE OFF");
    if (g_speechEnabled) speakTokenState("transceiver", false);
    return true;
  }
  if (upper == "PA?") {
    bool on = false;
    if (!asciiQueryPreamp(sp, on, 800)) { reportCommandFailure("PA?", "no reply"); return true; }
    Serial.println(on ? "PA ON" : "PA OFF");
    if (g_speechEnabled) speakTokenState("pa", on);
    return true;
  }
  if (upper == "PA ON") {
    if (!asciiSetPreamp(sp, true)) { reportCommandFailure("PA ON", "failed"); return true; }
    Serial.println("PA ON");
    if (g_speechEnabled) speakTokenState("pa", true);
    return true;
  }
  if (upper == "PA OFF") {
    if (!asciiSetPreamp(sp, false)) { reportCommandFailure("PA OFF", "failed"); return true; }
    Serial.println("PA OFF");
    if (g_speechEnabled) speakTokenState("pa", false);
    return true;
  }
  if (upper == "PA TOGGLE") {
    bool on = false;
    if (!asciiQueryPreamp(sp, on, 800)) { reportCommandFailure("PA TOGGLE", "no reply"); return true; }
    if (!asciiSetPreamp(sp, !on)) { reportCommandFailure("PA TOGGLE", "failed"); return true; }
    Serial.println(!on ? "PA ON" : "PA OFF");
    if (g_speechEnabled) speakTokenState("pa", !on);
    return true;
  }
  if (upper == "GT?") {
    String rsp;
    if (!asciiQueryAgcLine(sp, rsp, 800)) { reportCommandFailure("GT?", "no reply"); return true; }
    Serial.print("GT: ");
    printAsciiReplyPayload(rsp, sp.commands->agcReplyPrefix);
    speakConsoleTokenOrGap("gt");
    return true;
  }
  if (upper == "GT FAST") {
    if (!asciiSetAgcCommand(sp, sp.commands->agcFastCmd)) { reportCommandFailure("GT FAST", "failed"); return true; }
    Serial.println("GT FAST");
    speakConsoleTokenOrGap("gt");
    playSilenceMs(60);
    speakConsoleTokenOrGap("fast");
    return true;
  }
  if (upper == "GT SLOW") {
    if (!asciiSetAgcCommand(sp, sp.commands->agcSlowCmd)) { reportCommandFailure("GT SLOW", "failed"); return true; }
    Serial.println("GT SLOW");
    speakConsoleTokenOrGap("gt");
    playSilenceMs(60);
    speakConsoleTokenOrGap("slow");
    return true;
  }
  if (upper == "GT OFF") {
    if (!asciiSetAgcCommand(sp, sp.commands->agcOffCmd)) { reportCommandFailure("GT OFF", "failed"); return true; }
    Serial.println("GT OFF");
    if (g_speechEnabled) speakTokenState("gt", false);
    return true;
  }
  if (upper == "PS?") {
    bool on = false;
    if (!asciiQueryPowerState(sp, on, 800)) { reportCommandFailure("PS?", "no reply"); return true; }
    Serial.println(on ? "PS ON" : "PS OFF");
    if (g_speechEnabled) speakTokenState("ps", on);
    return true;
  }
  if (upper == "PS ON") {
    if (!asciiSetPowerState(sp, true)) { reportCommandFailure("PS ON", "failed"); return true; }
    Serial.println("PS ON");
    if (g_speechEnabled) speakTokenState("ps", true);
    return true;
  }
  if (upper == "PS OFF") {
    if (!asciiSetPowerState(sp, false)) { reportCommandFailure("PS OFF", "failed"); return true; }
    Serial.println("PS OFF");
    if (g_speechEnabled) speakTokenState("ps", false);
    return true;
  }
  if (upper == "SPLIT TOGGLE") {
    bool on = false;
    if (!querySplit(on, 800)) { reportCommandFailure("SPLIT TOGGLE", "no reply"); return true; }
    if (!setSplit(!on)) { reportCommandFailure("SPLIT TOGGLE", "failed"); return true; }
    Serial.println(!on ? "SPLIT ON" : "SPLIT OFF");
    speakTokenState("split", !on);
    return true;
  }
  return false;
}

static bool handleConsoleBankCommands(const String& line, const String& upper) {
  if (upper == "BANK NEXT") {
    uint8_t nextBank = (uiGetBank() >= 9) ? 1 : (uint8_t)(uiGetBank() + 1);
    uiSetBank(nextBank);
    Serial.print("OK BANK ");
    Serial.println((int)nextBank);
    speakBankNumber();
    return true;
  }
  if (upper == "BANK PREV") {
    uint8_t prevBank = (uiGetBank() <= 1) ? 9 : (uint8_t)(uiGetBank() - 1);
    uiSetBank(prevBank);
    Serial.print("OK BANK ");
    Serial.println((int)prevBank);
    speakBankNumber();
    return true;
  }
  if (upper.startsWith("BANK ")) {
    int b = line.substring(5).toInt();
    if (b < 1 || b > 9) { Serial.println("BANK -> invalid (use 1..9)"); speakError(); return true; }
    uiSetBank((uint8_t)b);
    Serial.print("OK BANK ");
    Serial.println((int)b);
    speakBankNumber();
    return true;
  }
  return false;
}

// Keeps polling off the radio line while the device restarts into the bootloader.
static constexpr uint32_t BOOTLOADER_POLL_HOLD_MS = 5000;

static bool handleHamtrcServiceCommand(const String& upper, Print& output) {
  if (upper == "HAMTRC?") {
    output.print("LX1WJ-HAMTRC;protocol=1;chip=esp32;version=");
    output.println(HAMTRC_FIRMWARE_VERSION);
    return true;
  }

  if (upper == "HAMTRC_BOOTLOADER") {
    if (g_audioPlaying) audioAbortNow();
    g_suspendPollingUntilMs = millis() + BOOTLOADER_POLL_HOLD_MS;
    serialTransportFlushInput();
    serialTransportFlushOutput();
    output.println("OK HAMTRC_BOOTLOADER;action=restarting");
    Serial.flush();
    delay(200);
    ESP.restart();
    return true;
  }

  return false;
}

bool processHamtrcServiceCommand(String line, Print& output) {
  line.trim();
  if (!line.length()) return false;
  String upper = upperCopy(line);
  return handleHamtrcServiceCommand(upper, output);
}

void processCommand(String line) {
  line.trim();
  if (!line.length()) return;
  // A timeout left by background polling must not be blamed on this command.
  g_radioReplyTimedOut = false;

  String upper = upperCopy(line);
  if (handleHamtrcServiceCommand(upper, Serial)) return;

  // Typed input interrupts speech like a key press does. Keypad-issued commands
  // were already interrupted at the key press and may have queued their label.
  if (!g_keypadExecuting) audioAbortNow();

  if (usbConsoleReady()) {
    Serial.print("> ");
    Serial.println(line);
  }

  if (handleConsoleInfoCommands(upper)) return;
  if (handleConsoleProfileCommands(line, upper)) return;
  if (handleConsoleConnectionCommands(line, upper)) return;
  if (handleFtdx10BlockedConsoleCommand(upper)) return;
  if (handleConsoleToggleCommands(line, upper)) return;
  if (handleConsoleYaesuFt8x7Commands(line, upper)) return;
  if (handleConsoleFt847Commands(upper)) return;
  if (handleConsoleAdjustCommands(line, upper)) return;
  if (handleConsoleRadioCommands(line, upper)) return;
  if (handleConsoleBankCommands(line, upper)) return;
  if (usbConsoleReady()) Serial.println("Unknown. Type HELP");
}
