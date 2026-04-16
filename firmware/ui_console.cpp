#include "ui_console.h"

#include "radio_catalog.h"
#include "radio_mode.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_yaesu.h"
#include "protocol_yaesu_cat.h"
#include "radio_profile.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "sd_slots.h"
#include "ui_speech.h"
#include "ui_console_support.h"
#include "ui_keypad.h"

static void speakBinaryFeatureState(const uint8_t* featureData, size_t featureLen, bool on) {
  if (!g_speechEnabled) return;
  playClipProgmem(featureData, featureLen);
  playSilenceMs(60);
  playClipProgmem(on ? voice_on : voice_off, on ? voice_on_len : voice_off_len);
}

static uint8_t levelRawToPercent(uint16_t raw) {
  if (raw >= 255) return 100;
  return (uint8_t)((raw * 100U + 127U) / 255U);
}

static uint16_t levelPercentToRaw(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return (uint16_t)((percent * 255 + 50) / 100);
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

static void printYesNoUnknown(const char* label, bool known, bool on) {
  Serial.print(label);
  if (!known) {
    Serial.println("unknown");
    return;
  }
  Serial.println(on ? "on" : "off");
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
    char label[4];
    snprintf(label, sizeof(label), "%03u", (unsigned)live.dcsCode);
    Serial.println(label);
  }
}

static void printYaesuFt817FmContext() {
  const uint32_t savedSuspendPollingUntilMs = g_suspendPollingUntilMs;
  g_suspendPollingUntilMs = millis() + 2500;
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
    printYesNoUnknown("  SPLIT CACHE: ", g_ft8x7SplitKnown, g_ft8x7SplitOn);
  }

  printLiveToneStateSummary();
  Serial.println("  CLAR query:  unavailable");
  Serial.print("  TRACE:       ");
  Serial.println(g_yaesuCatTrace ? "on" : "off");
  g_suspendPollingUntilMs = savedSuspendPollingUntilMs;
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

static void speakVfoLabel(char which) {
  if (!g_speechEnabled) return;
  speakToken("vfo");
  playSilenceMs(60);
  if (which == 'A') speakToken("a");
  else if (which == 'B') speakToken("b");
}

static void speakVfoFrequencyLabel(char which) {
  if (!g_speechEnabled) return;
  speakVfoLabel(which);
  playSilenceMs(60);
  speakToken("frequency");
}

static void speakRitStateAndOffset(bool on, int32_t offset) {
  if (!g_speechEnabled) return;
  speakToken("rit");
  playSilenceMs(60);
  speakToken(on ? "on" : "off");
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

static void speakFeatureValue(const uint8_t* featureData, size_t featureLen, uint8_t value) {
  if (!g_speechEnabled) return;
  playClipProgmem(featureData, featureLen);
  playSilenceMs(60);
  speakDigitsAndPoint(String((int)value));
}

static void speakNotchCycleState(bool on, NotchWidth width) {
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

static void speakSimpleBinaryState(bool on) {
  if (!g_speechEnabled) return;
  playClipProgmem(on ? voice_on : voice_off, on ? voice_on_len : voice_off_len);
}

static void speakSignedStepValue(const String& label, int value) {
  if (!g_speechEnabled) return;
  speakToken(label);
  playSilenceMs(60);
  if (value < 0) {
    speakToken("minus");
    playSilenceMs(60);
    value = -value;
  }
  speakDigitsAndPoint(String(value));
  playSilenceMs(60);
  speakToken("step");
}

static void speakBandStackLabel(uint8_t reg) {
  if (!g_speechEnabled) return;
  speakToken("b");
  playSilenceMs(60);
  speakToken("stack");
  playSilenceMs(60);
  playDigit((int)reg);
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

String readLine() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String r = line;
      line = "";
      r.trim();
      return r;
    }
    line += c;
  }
  return "";
}

String upperCopy(String s) {
  s.toUpperCase();
  return s;
}

static bool isFtdx10ConsoleProfile() {
  const StoredProfile& sp = currentStoredProfile();
  return sp.protocolType == PROTO_YAESU_FTDX_ASCII &&
         strcmp(sp.voiceVendor, "yaesu") == 0 &&
         strcmp(sp.voiceDigits, "10") == 0;
}

static bool handleFtdx10BlockedConsoleCommand(const String& upper) {
  if (!isFtdx10ConsoleProfile()) return false;
  if (upper == "MONITOR?" || upper == "MONITOR ON" || upper == "MONITOR OFF") {
    Serial.println("MONITOR -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "MONLEVEL?" || upper.startsWith("MONLEVEL ")) {
    Serial.println("MONLEVEL -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "TRANSCEIVE?" || upper == "TRANSCEIVE ON" || upper == "TRANSCEIVE OFF") {
    Serial.println("TRANSCEIVE -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "PBT1?" || upper.startsWith("PBT1 ")) {
    Serial.println("PBT1 -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "PBT2?" || upper.startsWith("PBT2 ")) {
    Serial.println("PBT2 -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "FILSHAPE?" || upper == "FILSHAPE SHARP" || upper == "FILSHAPE SOFT") {
    Serial.println("FILSHAPE -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "FILWIDTH?" || upper.startsWith("FILWIDTH ")) {
    Serial.println("FILWIDTH -> hidden on FTDX10 (use documented Yaesu CAT later)");
    return true;
  }
  if (upper == "RIT?" || upper == "RIT ON" || upper == "RIT OFF" || upper.startsWith("RIT ")) {
    Serial.println("RIT -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "NBLEVEL?" || upper.startsWith("NBLEVEL ")) {
    Serial.println("NBLEVEL -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "NRLEVEL?" || upper.startsWith("NRLEVEL ")) {
    Serial.println("NRLEVEL -> hidden on FTDX10 (no clean Yaesu path here)");
    return true;
  }
  if (upper == "NOTCH NAR" || upper == "NOTCH MID" || upper == "NOTCH WIDE") {
    Serial.println("NOTCH width -> hidden on FTDX10 (only clean ON/OFF path is exposed)");
    return true;
  }
  return false;
}

static bool parseConsoleModeToken(String token, uint8_t& modeOut) {
  token.trim();
  if (!token.length()) return false;

  if (token.length() == 1) {
    modeOut = 0xFF;
    (void)profileModeFromDigit(token[0], modeOut);
    if (modeOut != 0xFF) return true;
  }

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
  Serial.println();
  Serial.println("Commands (case-insensitive):");
  Serial.println("  General:");
  Serial.println("    AK?");
  Serial.println("    BANK?");
  Serial.println("    BANK NEXT | PREV");
  if (!ftdx10) {
    Serial.println("    BSTACK <1..3>  (hamTRC internal)");
    Serial.println("    BSTACK? <1..3>  (hamTRC internal)");
  }
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
  Serial.println("    PROFILE <1..24>");
  Serial.println("    PROFILE NEXT | PREV");
  Serial.println("    PROFILE?");
  Serial.println("    QUIET OFF | QUIET ON");
  Serial.println("    QUIET?");
  Serial.println("    RX | TX");
  Serial.println("    SAY <digits>");
  Serial.println("    SLOTS?");
  Serial.println("    SPEECH OFF | SPEECH ON");
  Serial.println("    SPEECH?");
  Serial.println("    STATUS?");
  Serial.println("    SWT <nn> | SWH <nn>");
  Serial.println("    TEST");
  Serial.println("    TUNINGSPEECH OFF | ON");
  Serial.println("    TUNINGSPEECH?");
  Serial.println("    VOICE <name>");
  Serial.println("    VOLUME <0..3>");
  Serial.println("    VOLUME?");
  Serial.println();
  if (!ftdx10) {
    Serial.println("  IC-7300 / CI-V Extensions:");
    Serial.println("    NBLEVEL <0..100>");
    Serial.println("    NBLEVEL?");
    Serial.println("    NB OFF | ON | TOGGLE");
    Serial.println("    NB?");
    Serial.println("    FILSHAPE SHARP | SOFT");
    Serial.println("    FILSHAPE?");
    Serial.println("    FILWIDTH <1..3>");
    Serial.println("    FILWIDTH?");
    Serial.println("    LOCK OFF | ON | TOGGLE");
    Serial.println("    LOCK?");
    Serial.println("    MONITOR OFF | ON");
    Serial.println("    MONITOR?");
    Serial.println("    MONLEVEL <0..100>");
    Serial.println("    MONLEVEL?");
    Serial.println("    NOTCH MID | NAR | WIDE");
    Serial.println("    NOTCH OFF | ON | TOGGLE");
    Serial.println("    NOTCH?");
    Serial.println("    NRLEVEL <0..100>");
    Serial.println("    NRLEVEL?");
    Serial.println("    NR OFF | ON | TOGGLE");
    Serial.println("    NR?");
    Serial.println("    PBT1 <-128..127> | CENTER");
    Serial.println("    PBT1?");
    Serial.println("    PBT2 <-128..127> | CENTER");
    Serial.println("    PBT2?");
    Serial.println("    RIT <Hz>");
    Serial.println("    RIT OFF | ON");
    Serial.println("    RIT?");
    Serial.println("    RXTX?");
    Serial.println("    SPLIT OFF | ON | TOGGLE");
    Serial.println("    SPLIT?");
    Serial.println("    TUNE");
    Serial.println("    TRANSCEIVE OFF | ON");
    Serial.println("    TRANSCEIVE?");
    Serial.println("    TUNER OFF | ON");
    Serial.println("    TUNER? | TUNER OFF | ON | TOGGLE");
    Serial.println("    TXFREQ?");
    Serial.println("    VFO A | B");
    Serial.println("    VFOA <kHz> | VFOA?");
    Serial.println("    VFOA MODE <n> | VFOA MODE?");
    Serial.println("    VFOB <kHz> | VFOB?");
    Serial.println("    VFOB MODE <n> | VFOB MODE?");
    Serial.println();
    Serial.println("  ASCII / Yaesu Extensions:");
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
  if (!ftdx10) {
    Serial.println("    ALC? | VOL? | SQL?");
    Serial.println("    AGC <hex byte>");
    Serial.println("    CLAR OFFSET <8 hex digits>");
    Serial.println("    CLAR OFF | ON");
    Serial.println("    GT? | GT FAST | SLOW | OFF");
    Serial.println("    LOCKDOC OFF | ON");
    Serial.println("    MEM READ RAW | MEM WRITE  (experimental)");
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
  if (upper == "VOLUME?") {
    Serial.print("VOLUME ");
    Serial.println((int)g_volumeLevel);
    if (g_speechEnabled) speakVolumeLevel(g_volumeLevel);
    return true;
  }
  return false;
}

static bool handleConsoleProfileCommands(const String& line, const String& upper) {
  if (upper == "PROFILE NEXT") {
    uint8_t next = findAdjacentValidProfile(1);
    applyProfile(next);
    Serial.print("OK PROFILE ");
    Serial.println((int)next);
    speakCurrentProfile();
    return true;
  }
  if (upper == "PROFILE PREV") {
    uint8_t prev = findAdjacentValidProfile(-1);
    applyProfile(prev);
    Serial.print("OK PROFILE ");
    Serial.println((int)prev);
    speakCurrentProfile();
    return true;
  }
  if (upper.startsWith("PROFILE ")) {
    int slot = line.substring(8).toInt();
    if (slot < 1 || slot > MAX_PROFILE_SLOTS || !storedProfileForId((uint8_t)slot)) {
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
      Serial.println("BSTACK? -> hidden on FTDX10 (hamTRC internal, not Yaesu CAT)");
      return true;
    }
    int reg = line.substring(8).toInt();
    uint64_t hz = 0;
    uint8_t bandCode = 0;
    BandStackEntry entry;
    if (reg < 1 || reg > 3) { Serial.println("BSTACK? -> invalid register (use 1..3)"); return true; }
    if (!queryCurrentFrequencyValue(hz) || !bandCodeFromFrequency(hz, bandCode)) { Serial.println("BSTACK? -> no current band"); return true; }
    if (!queryBandStackEntry(bandCode, (uint8_t)reg, entry, 800)) { Serial.println("BSTACK? -> no reply"); return true; }
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
      g_suppressModePrefixOnce = true;
      speakMode(entry.mode);
    }
    return true;
  }
  if (upper.startsWith("BSTACK ")) {
    if (isFtdx10ConsoleProfile()) {
      Serial.println("BSTACK -> hidden on FTDX10 (hamTRC internal, not Yaesu CAT)");
      return true;
    }
    int reg = line.substring(7).toInt();
    uint64_t hz = 0;
    uint8_t bandCode = 0;
    BandStackEntry entry;
    if (reg < 1 || reg > 3) { Serial.println("BSTACK -> invalid register (use 1..3)"); return true; }
    if (!queryCurrentFrequencyValue(hz) || !bandCodeFromFrequency(hz, bandCode)) { Serial.println("BSTACK -> no current band"); return true; }
    if (!queryBandStackEntry(bandCode, (uint8_t)reg, entry, 800)) { Serial.println("BSTACK -> no reply"); return true; }
    if (!setFrequency(entry.freqHz) || !setMode(entry.mode, entry.filter)) { Serial.println("BSTACK -> failed"); return true; }
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
      g_suppressModePrefixOnce = true;
      speakMode(entry.mode);
    }
    return true;
  }
  return false;
}

static bool handleConsoleToggleCommands(const String& line, const String& upper) {
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
  if (upper.startsWith("VOLUME ")) {
    int lvl = line.substring(7).toInt();
    if (lvl < 0 || lvl > 3) {
      Serial.println("VOLUME -> invalid (use 0..3)");
      speakError();
      return true;
    }
    applyVolumeLevel((uint8_t)lvl);
    Serial.print("OK VOLUME ");
    Serial.println(lvl);
    if (g_speechEnabled) speakVolumeLevel((uint8_t)lvl);
    return true;
  }
  if (upper.startsWith("VOICE ")) { playNamedVoice(line.substring(6)); return true; }
  if (upper == "TEST") { voiceTest(); return true; }
  if (upper.startsWith("SAY ")) { if (g_speechEnabled) speakDigitsAndPoint(line.substring(4)); return true; }
  return false;
}

static bool handleConsoleYaesuFt8x7Commands(const String& line, const String& upper) {
  if (!isCurrentYaesuFt8x7()) return false;

  if (upper == "YALL?") {
    Serial.println("[YAESU FT8X7]");
    Serial.print("  VARIANT: ");
    Serial.println(currentProfileVariant()[0] ? currentProfileVariant() : "(default)");
    const bool isFt817 = currentProfileVariantIs("ft817");
    const bool isFt857Family = currentProfileVariantIs("ft857_897");

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
    if (yaesuCatQuerySMeterRaw(currentStoredProfile(), raw, 800)) {
      Serial.print("  SM: ");
      Serial.println(raw);
    } else {
      Serial.println("  SM: no reply");
    }
    if (yaesuCatQueryPoMeterRaw(currentStoredProfile(), raw, 800)) {
      Serial.print("  PO: ");
      Serial.println(raw);
    } else {
      Serial.println("  PO: no reply");
    }
    if (yaesuCatQuerySWRRaw(currentStoredProfile(), raw, 800)) {
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
      Serial.println("  VOL: variant unknown");
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
      Serial.println("  SQL: variant unknown");
    }

    if (isFt817) {
      uint8_t status = 0;
      if (yaesuCatQueryStatusRaw(status, 800)) {
        Serial.print("  STATUS: 0x");
        if (status < 0x10) Serial.print('0');
        Serial.println(status, HEX);
      } else {
        Serial.println("  STATUS: no reply");
      }
    } else if (isFt857Family) {
      uint8_t status = 0;
      if (yaesuCatQueryTxStatusRaw(status, 800)) {
        Serial.print("  STATUS: 0x");
        if (status < 0x10) Serial.print('0');
        Serial.println(status, HEX);
      } else {
        Serial.println("  STATUS: no reply");
      }
    } else {
      Serial.println("  STATUS: variant unknown");
    }
    return true;
  }

  if (upper == "YVAR?") {
    const char* variant = currentProfileVariant();
    Serial.print("YVAR: ");
    Serial.println(variant[0] ? variant : "(default)");
    if (currentProfileVariantIs("ft817")) {
      Serial.println("  Tone/DCS family: FT-817 style (simple Tone/DCS layout)");
    } else if (currentProfileVariantIs("ft857_897")) {
      Serial.println("  Tone/DCS family: FT-857/897 style (separate TX/RX Tone/DCS layout)");
    } else {
      Serial.println("  Tone/DCS family: unknown variant");
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
    if (currentProfileVariantIs("ft817")) {
      if (!(modeByte == 0x0A || modeByte == 0x2A || modeByte == 0x4A || modeByte == 0x8A)) {
        Serial.println("YTMODE -> mode not documented for FT-817 variant");
        return true;
      }
    } else if (currentProfileVariantIs("ft857_897")) {
      if (!(modeByte == 0x0A || modeByte == 0x0B || modeByte == 0x0C || modeByte == 0x2A || modeByte == 0x3A || modeByte == 0x4A || modeByte == 0x8A)) {
        Serial.println("YTMODE -> mode not documented for FT-857/897 variant");
        return true;
      }
    } else {
      Serial.println("YTMODE -> unknown FT8x7 variant");
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
    if (currentProfileVariantIs("ft817")) {
      uint8_t pair[2] = {0};
      ok = parseHexNybbleString(arg, pair, 2);
      data[0] = pair[0];
      data[1] = pair[1];
    } else if (currentProfileVariantIs("ft857_897")) {
      ok = parseHexNybbleString(arg, data, 4);
    }
    if (!ok) {
      if (currentProfileVariantIs("ft817")) Serial.println("YTONE -> FT-817 expects 4 hex digits, e.g. 0885");
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

  if (upper.startsWith("YDCS ")) {
    uint8_t data[4] = {0};
    String arg = line.substring(5);
    bool ok = false;
    if (currentProfileVariantIs("ft817")) {
      uint8_t pair[2] = {0};
      ok = parseHexNybbleString(arg, pair, 2);
      data[0] = pair[0];
      data[1] = pair[1];
    } else if (currentProfileVariantIs("ft857_897")) {
      ok = parseHexNybbleString(arg, data, 4);
    }
    if (!ok) {
      if (currentProfileVariantIs("ft817")) Serial.println("YDCS -> FT-817 expects 4 hex digits, e.g. 0023");
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
    if (!currentProfileVariantIs("ft817")) {
      Serial.println("YPOWER -> documented only for FT-817 variant");
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
    double mhz = arg.toDouble();
    if (mhz <= 0.0) {
      Serial.println("YRPTSHIFT -> use MHz, e.g. 0.600 or 5.000");
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
      Serial.println("YMODEBYTE? -> no reply");
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
    const bool ft817Debug = currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817");
    const uint32_t savedSuspendPollingUntilMs = g_suspendPollingUntilMs;
    if (ft817Debug) g_suspendPollingUntilMs = millis() + 2500;
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
      g_suspendPollingUntilMs = savedSuspendPollingUntilMs;
    }
    return true;
  }

  if (upper.startsWith("YSETMODEQ ")) {
    uint8_t modeByte = 0;
    if (!parseHexByteString(line.substring(10), modeByte)) {
      Serial.println("YSETMODEQ -> use hex byte, e.g. 08, 04, 02");
      return true;
    }
    const bool ft817Quiet = currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817");
    const uint32_t savedSuspendPollingUntilMs = g_suspendPollingUntilMs;
    if (ft817Quiet) g_suspendPollingUntilMs = millis() + 2500;
    yaesuCatFlushInput();
    delay(120);
    yaesuCatSetModeRawByte(modeByte);
    Serial.print("YSETMODEQ 0x");
    if (modeByte < 0x10) Serial.print('0');
    Serial.println(modeByte, HEX);
    Serial.println("  quiet wait...");
    delay(ft817Quiet ? 1400 : 600);
    if (ft817Quiet) g_suspendPollingUntilMs = savedSuspendPollingUntilMs;
    return true;
  }

  if (upper == "YFMCTX?") {
    if (!currentProfileVariantIs("ft817")) {
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

  if (upper == "YRXSTATUS?") {
    uint8_t raw = 0;
    if (!yaesuCatQueryRxStatusRaw(raw, 800)) { Serial.println("YRXSTATUS? -> no reply"); return true; }
    Serial.print("YRXSTATUS: 0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
    return true;
  }
  if (upper == "YTXSTATUS?" || upper == "YSTATUS?") {
    uint8_t raw = 0;
    if (!yaesuCatQueryTxStatusRaw(raw, 800)) {
      Serial.println((upper == "YSTATUS?") ? "YSTATUS? -> no reply" : "YTXSTATUS? -> no reply");
      return true;
    }
    Serial.print((upper == "YSTATUS?") ? "YSTATUS: 0x" : "YTXSTATUS: 0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
    return true;
  }
  if (upper == "RXTX?" && currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897")) {
    bool tx = false;
    if (!queryRxTxStatus(tx, 800)) { Serial.println("RXTX? -> no reply"); return true; }
    Serial.println(tx ? "TX" : "RX");
    return true;
  }
  if (upper == "SPLIT?" && currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897")) {
    Serial.println("SPLIT? -> not reliable on FT-857/897");
    return true;
  }
  if (upper == "ALC?") {
    int32_t raw = 0;
    if (!yaesuCatQueryAlcRaw(raw, 800)) { Serial.println("ALC? -> no reply"); return true; }
    Serial.print("ALC: ");
    Serial.println(raw);
    return true;
  }
  if (upper == "VOL?") {
    if (currentProfileVariantIs("ft857_897")) {
      Serial.println("VOL? -> unsupported on verified FT-857/897 path");
      return true;
    }
    int32_t raw = 0;
    if (!yaesuCatQueryVolumeRaw(raw, 800)) { Serial.println("VOL? -> no reply"); return true; }
    Serial.print("VOL: ");
    Serial.println(raw);
    return true;
  }
  if (upper == "SQL?") {
    if (currentProfileVariantIs("ft857_897")) {
      Serial.println("SQL? -> unsupported on verified FT-857/897 path");
      return true;
    }
    int32_t raw = 0;
    if (!yaesuCatQuerySquelchRaw(raw, 800)) { Serial.println("SQL? -> no reply"); return true; }
    Serial.print("SQL: ");
    Serial.println(raw);
    return true;
  }
  if (upper == "VFO TOGGLE") {
    yaesuCatToggleVfo();
    Serial.println("VFO TOGGLE");
    return true;
  }
  if (upper == "VFO A") {
    if (!currentProfileVariantIs("ft817")) {
      Serial.println("VFO A -> raw FT8x7 VFO select is currently enabled only for FT-817");
      return true;
    }
    yaesuCatSelectVfoA();
    Serial.println("VFO A");
    return true;
  }
  if (upper == "VFO B") {
    if (!currentProfileVariantIs("ft817")) {
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
    rememberSplitState(true);
    Serial.println("SPLIT ON");
    return true;
  }
  if (upper == "SPLIT OFF") {
    yaesuCatSetSplit(false);
    rememberSplitState(false);
    Serial.println("SPLIT OFF");
    return true;
  }
  if (upper == "CLAR ON") {
    yaesuCatSetClarifier(true);
    Serial.println("CLAR ON");
    return true;
  }
  if (upper == "CLAR OFF") {
    yaesuCatSetClarifier(false);
    Serial.println("CLAR OFF");
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
  if (upper == "LOCK?" && currentProtocolType() == PROTO_YAESU_FT8X7) {
    Serial.println("LOCK? -> no safe read command known here; documented write bytes collide with VFO A/B");
    return true;
  }
  if ((upper == "LOCK ON" || upper == "LOCK OFF") && currentProtocolType() == PROTO_YAESU_FT8X7) {
    const bool on = (upper == "LOCK ON");
    yaesuCatSetLockDocumentedRaw(on);
    rememberDialLockState(on);
    Serial.println(on ? "LOCK ON -> sent documented FT8x7 raw bytes 00 00 00 00 00" : "LOCK OFF -> sent documented FT8x7 raw bytes 00 00 00 00 80");
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
    Serial.println("MEM WRITE -> experimental FT8x7 raw path; not confirmed as documented normal CAT on FT-857/897");
    yaesuCatMemoryWrite();
    return true;
  }
  if (upper == "MEM READ RAW") {
    Serial.println("MEM READ RAW -> experimental FT8x7 raw path; not confirmed as documented normal CAT on FT-857/897");
    uint8_t rsp[5] = {0};
    if (!yaesuCatMemoryReadRaw(rsp, 800)) { Serial.println("MEM READ RAW -> no reply"); return true; }
    Serial.print("MEM RAW: ");
    yaesuCatPrintFrame(rsp);
    Serial.println();
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
      Serial.println("YCAT? -> no reply");
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
      Serial.println("YCAT1? -> no reply");
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

static bool handleConsoleRadioCommands(const String& line, const String& upper) {
  const StoredProfile& sp = currentStoredProfile();
  if (upper == "LFREQ") {
    if (!live.freqValid) { Serial.println("No live frequency yet."); return true; }
    Serial.print("Last live: ");
    Serial.print(hzToMHzString3(live.freqHz));
    Serial.println(" MHz");
    return true;
  }
  if (upper == "FREQ?") {
    if (!refreshLiveFrequency()) { Serial.println("FREQ? -> no reply"); return true; }
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
    if (!applyFrequencyAndTrack(hzSet, true)) { Serial.println("SET FREQ -> no reply"); return true; }
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hzSet));
    return true;
  }
  if (upper.startsWith("FREQ ")) {
    uint64_t khz = strtoull(line.substring(5).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    if (!applyFrequencyAndTrack(hzSet, true)) { Serial.println("SET FREQ -> no reply"); return true; }
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hzSet));
    return true;
  }
  if (upper == "MODE?") {
    if (!refreshLiveMode()) { Serial.println("MODE? -> no reply"); return true; }
    Serial.println(modeToString(live.mode));
    if (g_keypadExecuting) g_suppressModePrefixOnce = true;
    speakMode(live.mode);
    return true;
  }
  if (upper.startsWith("MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(5), mode) || !applyModeAndTrack(mode, 1)) {
      Serial.println("SET MODE -> failed");
      return true;
    }
    Serial.println("SET MODE -> command sent");
    if (g_keypadExecuting) g_suppressModePrefixOnce = true;
    speakMode(mode);
    return true;
  }
  if (upper == "FB?") {
    String rsp;
    uint64_t hz = 0;
    if (!transactAsciiCommand("FB;", rsp, "FB", 800) || !parseAsciiUnsignedResponse(rsp, "FB", hz)) {
      Serial.println("FB? -> no reply");
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
    if (!asciiPacketSendCommand(cmd)) { Serial.println("SET FB -> failed"); return true; }
    Serial.println("SET FB -> command sent");
    return true;
  }
  if (upper.startsWith("FB ")) {
    uint64_t khz = strtoull(line.substring(3).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "FB%011llu;", (unsigned long long)hzSet);
    if (!asciiPacketSendCommand(cmd)) { Serial.println("SET FB -> failed"); return true; }
    Serial.println("SET FB -> command sent");
    return true;
  }
  if (upper == "IF?") {
    String rsp;
    if (!asciiQueryStatusLine(sp, rsp, 800)) { Serial.println("IF? -> no reply"); return true; }
    Serial.print("IF: ");
    Serial.println(rsp);
    return true;
  }
  if (upper == "ID?") {
    String rsp;
    if (!asciiQueryIdLine(sp, rsp, 800)) { Serial.println("ID? -> no reply"); return true; }
    Serial.print("ID: ");
    printAsciiReplyPayload(rsp, sp.ascii.idReplyPrefix);
    return true;
  }
  if (upper == "OM?") {
    String rsp;
    if (!asciiQueryOmLine(sp, rsp, 800)) { Serial.println("OM? -> no reply"); return true; }
    Serial.print("OM: ");
    printAsciiReplyPayload(rsp, sp.ascii.omReplyPrefix);
    return true;
  }
  if (upper == "FR?") {
    String rsp;
    if (!transactAsciiCommand("FR;", rsp, "FR", 800)) { Serial.println("FR? -> no reply"); return true; }
    Serial.print("FR: ");
    printAsciiReplyPayload(rsp, "FR");
    return true;
  }
  if (upper == "FR0") {
    if (!asciiPacketSendCommand("FR0;")) { Serial.println("FR0 -> failed"); return true; }
    Serial.println("FR0");
    return true;
  }
  if (upper == "FT?") {
    String rsp;
    if (!transactAsciiCommand("FT;", rsp, "FT", 800)) { Serial.println("FT? -> no reply"); return true; }
    Serial.print("FT: ");
    printAsciiReplyPayload(rsp, "FT");
    return true;
  }
  if (upper == "FT A") {
    if (!asciiPacketSendCommand("FT0;")) { Serial.println("FT A -> failed"); return true; }
    Serial.println("FT A");
    return true;
  }
  if (upper == "FT B") {
    if (!asciiPacketSendCommand("FT1;")) { Serial.println("FT B -> failed"); return true; }
    Serial.println("FT B");
    return true;
  }
  if (upper == "RX") {
    if (!asciiPacketSendCommand("RX;")) { Serial.println("RX -> failed"); return true; }
    Serial.println("RX");
    return true;
  }
  if (upper == "TX") {
    if (!asciiPacketSendCommand("TX;")) { Serial.println("TX -> failed"); return true; }
    Serial.println("TX");
    return true;
  }
  if (upper == "AK?") {
    String rsp;
    if (!transactAsciiCommand("AK;", rsp, "AK", 800)) { Serial.println("AK? -> no reply"); return true; }
    Serial.print("AK: ");
    printAsciiReplyPayload(rsp, "AK");
    return true;
  }
  if (upper.startsWith("SWT ")) {
    int nn = line.substring(4).toInt();
    if (nn < 0 || nn > 99) { Serial.println("SWT -> invalid (use 00..99)"); return true; }
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "SWT%02d;", nn);
    if (!asciiPacketSendCommand(cmd)) { Serial.println("SWT -> failed"); return true; }
    Serial.print("SENT ");
    Serial.println(cmd);
    return true;
  }
  if (upper.startsWith("SWH ")) {
    int nn = line.substring(4).toInt();
    if (nn < 0 || nn > 99) { Serial.println("SWH -> invalid (use 00..99)"); return true; }
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "SWH%02d;", nn);
    if (!asciiPacketSendCommand(cmd)) { Serial.println("SWH -> failed"); return true; }
    Serial.print("SENT ");
    Serial.println(cmd);
    return true;
  }
  if (upper == "SM?") {
    if (!refreshLiveSmeter()) { Serial.println("SM? -> no reply"); return true; }
    Serial.print("SM: raw=");
    Serial.println(live.smRaw);
    if (g_speechEnabled) speakSValue(live.smS);
    return true;
  }
  if (upper == "SWR?") {
    if (!refreshLiveSwr()) { Serial.println("SWR? -> no reply"); return true; }
    float swr = swrRawToValue(live.swrRaw);
    Serial.println(swr, 2);
    if (g_speechEnabled) {
      speakToken("swr");
      playSilenceMs(60);
      speakDigitsAndPoint(String(swr, 2));
    }
    return true;
  }
  if (upper == "PO?") {
    if (!refreshLivePower()) { Serial.println("PO? -> no reply"); return true; }
    Serial.println(live.powerRaw);
    if (g_speechEnabled) {
      playClipProgmem(voice_power, voice_power_len);
      speakDigitsAndPoint(String(live.powerRaw));
    }
    return true;
  }
  if (upper == "TUNER?") {
    bool on = false;
    if (!queryTuner(on, 800)) { Serial.println("TUNER? -> no reply"); return true; }
    Serial.println(on ? "TUNER ON" : "TUNER OFF");
    speakTokenState("tuner", on);
    return true;
  }
  if (upper == "TUNER ON") {
    if (!setTuner(true)) { Serial.println("TUNER ON -> failed"); return true; }
    Serial.println("TUNER ON");
    speakTokenState("tuner", true);
    return true;
  }
  if (upper == "TUNER OFF") {
    if (!setTuner(false)) { Serial.println("TUNER OFF -> failed"); return true; }
    Serial.println("TUNER OFF");
    speakTokenState("tuner", false);
    return true;
  }
  if (upper == "TUNER TOGGLE") {
    bool on = false;
    if (!queryTuner(on, 800)) { Serial.println("TUNER TOGGLE -> no reply"); return true; }
    if (!setTuner(!on)) { Serial.println("TUNER TOGGLE -> failed"); return true; }
    Serial.println(!on ? "TUNER ON" : "TUNER OFF");
    speakTokenState("tuner", !on);
    return true;
  }
  if (upper == "TUNE") {
    if (!startTune()) { Serial.println("TUNE -> failed"); return true; }
    Serial.println("TUNE");
    if (g_speechEnabled) speakToken("tune");
    return true;
  }
  if (upper == "RXTX?") {
    bool tx = false;
    if (!queryRxTxStatus(tx, 800)) { Serial.println("RXTX? -> no reply"); return true; }
    Serial.println(tx ? "TX" : "RX");
    if (g_speechEnabled) speakToken(tx ? "tx" : "rx");
    return true;
  }
  if (upper == "TXFREQ?") {
    uint64_t hz = 0;
    if (!queryTxFrequency(hz, 800)) { Serial.println("TXFREQ? -> no reply"); return true; }
    Serial.print("TXFREQ: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    return true;
  }
  if (upper == "VFO A") {
    if (!selectVfoA()) { Serial.println("VFO A -> failed"); return true; }
    Serial.println("VFO A");
    speakVfoLabel('A');
    return true;
  }
  if (upper == "VFO B") {
    if (!selectVfoB()) { Serial.println("VFO B -> failed"); return true; }
    Serial.println("VFO B");
    speakVfoLabel('B');
    return true;
  }
  if (upper == "VFOA?") {
    uint64_t hz = 0;
    if (!queryVfoFrequency(true, hz, 800)) { Serial.println("VFOA? -> no reply"); return true; }
    Serial.print("VFOA: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    if (g_speechEnabled) {
      speakVfoFrequencyLabel('A');
      playSilenceMs(60);
      speakDigitsAndPoint(hzToMHzString3(hz));
    }
    return true;
  }
  if (upper == "VFOB?") {
    uint64_t hz = 0;
    if (!queryVfoFrequency(false, hz, 800)) { Serial.println("VFOB? -> no reply"); return true; }
    Serial.print("VFOB: ");
    Serial.print(hzToMHzString3(hz));
    Serial.println(" MHz");
    if (g_speechEnabled) {
      speakVfoFrequencyLabel('B');
      playSilenceMs(60);
      speakDigitsAndPoint(hzToMHzString3(hz));
    }
    return true;
  }
  if (upper.startsWith("VFOA MODE?")) {
    uint8_t mode = 0xFF;
    uint8_t filter = 0xFF;
    if (!queryVfoMode(true, mode, filter, 800)) { Serial.println("VFOA MODE? -> no reply"); return true; }
    Serial.print("VFOA MODE: ");
    Serial.println(modeToString(mode));
    if (g_keypadExecuting) g_suppressModePrefixOnce = true;
    speakMode(mode);
    return true;
  }
  if (upper.startsWith("VFOB MODE?")) {
    uint8_t mode = 0xFF;
    uint8_t filter = 0xFF;
    if (!queryVfoMode(false, mode, filter, 800)) { Serial.println("VFOB MODE? -> no reply"); return true; }
    Serial.print("VFOB MODE: ");
    Serial.println(modeToString(mode));
    if (g_keypadExecuting) g_suppressModePrefixOnce = true;
    speakMode(mode);
    return true;
  }
  if (upper.startsWith("VFOA MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(10), mode) || !setVfoMode(true, mode, 1)) { Serial.println("VFOA MODE -> failed"); return true; }
    Serial.println("VFOA MODE -> command sent");
    return true;
  }
  if (upper.startsWith("VFOB MODE ")) {
    uint8_t mode = 0xFF;
    if (!parseConsoleModeToken(line.substring(10), mode) || !setVfoMode(false, mode, 1)) { Serial.println("VFOB MODE -> failed"); return true; }
    Serial.println("VFOB MODE -> command sent");
    return true;
  }
  if (upper.startsWith("VFOA ")) {
    uint64_t khz = strtoull(line.substring(5).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    if (!setVfoFrequency(true, hzSet)) { Serial.println("VFOA -> failed"); return true; }
    return true;
  }
  if (upper.startsWith("VFOB ")) {
    uint64_t khz = strtoull(line.substring(5).c_str(), nullptr, 10);
    uint64_t hzSet = khz * 1000ULL;
    if (!setVfoFrequency(false, hzSet)) { Serial.println("VFOB -> failed"); return true; }
    return true;
  }
  if (upper == "SPLIT?") {
    bool on = false;
    if (!querySplit(on, 800)) { Serial.println("SPLIT? -> no reply"); return true; }
    Serial.println(on ? "SPLIT ON" : "SPLIT OFF");
    return true;
  }
  if (upper == "SPLIT ON") {
    if (!setSplit(true)) { Serial.println("SPLIT ON -> failed"); return true; }
    Serial.println("SPLIT ON");
    return true;
  }
  if (upper == "SPLIT OFF") {
    if (!setSplit(false)) { Serial.println("SPLIT OFF -> failed"); return true; }
    Serial.println("SPLIT OFF");
    return true;
  }
  if (upper == "RIT?") {
    bool on = false;
    int32_t offset = 0;
    if (!queryRitEnabled(on, 800)) { Serial.println("RIT? -> no reply"); return true; }
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
    if (!setRitEnabled(true)) { Serial.println("RIT ON -> failed"); return true; }
    Serial.println("RIT ON");
    speakTokenState("rit", true);
    return true;
  }
  if (upper == "RIT OFF") {
    if (!setRitEnabled(false)) { Serial.println("RIT OFF -> failed"); return true; }
    Serial.println("RIT OFF");
    speakTokenState("rit", false);
    return true;
  }
  if (upper.startsWith("RIT ")) {
    int32_t hz = line.substring(4).toInt();
    if (hz < -9999 || hz > 9999) { Serial.println("RIT -> invalid (use -9999..9999 Hz)"); return true; }
    if (!setRitOffsetHz(hz)) { Serial.println("RIT -> failed"); return true; }
    Serial.print("RIT ");
    Serial.print(hz);
    Serial.println(" Hz");
    speakRitStateAndOffset(true, hz);
    return true;
  }
  if (upper == "NR?") {
    if (!sp.caps.getNr) { if (usbConsoleReady()) Serial.println("NR? -> unsupported"); return true; }
    if (!refreshLiveNr()) { if (usbConsoleReady()) Serial.println("NR? -> no reply"); return true; }
    if (usbConsoleReady()) Serial.println(live.nrOn ? "NR ON" : "NR OFF");
    speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, live.nrOn);
    return true;
  }
  if (upper == "NB?") {
    if (!sp.caps.getNb) { if (usbConsoleReady()) Serial.println("NB? -> unsupported"); return true; }
    if (!refreshLiveNb()) { if (usbConsoleReady()) Serial.println("NB? -> no reply"); return true; }
    if (usbConsoleReady()) Serial.println(live.nbOn ? "NB ON" : "NB OFF");
    speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, live.nbOn);
    return true;
  }
  if (upper == "PBT1?") {
    uint16_t raw = 0;
    if (!queryPbtInner(raw, 800)) { Serial.println("PBT1? -> no reply"); return true; }
    Serial.print("PBT1 ");
    Serial.print(pbtRawToOffset(raw));
    Serial.println(" step");
    speakSignedStepValue("pbt", pbtRawToOffset(raw));
    return true;
  }
  if (upper == "PBT2?") {
    uint16_t raw = 0;
    if (!queryPbtOuter(raw, 800)) { Serial.println("PBT2? -> no reply"); return true; }
    Serial.print("PBT2 ");
    Serial.print(pbtRawToOffset(raw));
    Serial.println(" step");
    speakSignedStepValue("pbt", pbtRawToOffset(raw));
    return true;
  }
  if (upper == "LOCK?") {
    bool on = false;
    if (!queryDialLock(on, 800)) { Serial.println("LOCK? -> no reply"); return true; }
    Serial.println(on ? "LOCK ON" : "LOCK OFF");
    speakTokenState("lock", on);
    return true;
  }
  if (upper == "LOCK ON") {
    if (!setDialLock(true)) { Serial.println("LOCK ON -> failed"); return true; }
    Serial.println("LOCK ON");
    speakTokenState("lock", true);
    return true;
  }
  if (upper == "LOCK OFF") {
    if (!setDialLock(false)) { Serial.println("LOCK OFF -> failed"); return true; }
    Serial.println("LOCK OFF");
    speakTokenState("lock", false);
    return true;
  }
  if (upper == "LOCK TOGGLE") {
    bool on = false;
    if (!queryDialLock(on, 800)) { Serial.println("LOCK TOGGLE -> no reply"); return true; }
    if (!setDialLock(!on)) { Serial.println("LOCK TOGGLE -> failed"); return true; }
    Serial.println(!on ? "LOCK ON" : "LOCK OFF");
    speakTokenState("lock", !on);
    return true;
  }
  if (upper == "FILSHAPE?") {
    bool soft = false;
    if (!queryFilterShape(soft, 800)) { Serial.println("FILSHAPE? -> no reply"); return true; }
    Serial.println(soft ? "FILSHAPE SOFT" : "FILSHAPE SHARP");
    if (g_speechEnabled) {
      speakToken("filtershape");
      playSilenceMs(60);
      speakToken(soft ? "soft" : "sharp");
    }
    return true;
  }
  if (upper == "FILSHAPE SHARP") {
    if (!setFilterShape(false)) { Serial.println("FILSHAPE SHARP -> failed"); return true; }
    Serial.println("FILSHAPE SHARP");
    if (g_speechEnabled) {
      speakToken("filtershape");
      playSilenceMs(60);
      speakToken("sharp");
    }
    return true;
  }
  if (upper == "FILSHAPE SOFT") {
    if (!setFilterShape(true)) { Serial.println("FILSHAPE SOFT -> failed"); return true; }
    Serial.println("FILSHAPE SOFT");
    if (g_speechEnabled) {
      speakToken("filtershape");
      playSilenceMs(60);
      speakToken("soft");
    }
    return true;
  }
  if (upper == "FILWIDTH?") {
    uint8_t filter = 0xFF;
    if (!queryCurrentFilterSlot(filter)) { Serial.println("FILWIDTH? -> no reply"); return true; }
    Serial.print("FILWIDTH ");
    Serial.println((int)filter);
    if (g_speechEnabled) {
      speakToken("filterwidth");
      playSilenceMs(60);
      playDigit(filter);
    }
    return true;
  }
  if (upper == "MONITOR?") {
    bool on = false;
    if (!queryMonitorEnabled(on, 800)) { Serial.println("MONITOR? -> no reply"); return true; }
    Serial.println(on ? "MONITOR ON" : "MONITOR OFF");
    if (g_speechEnabled) speakTokenState("monitor", on);
    return true;
  }
  if (upper == "MONITOR ON") {
    if (!setMonitorEnabled(true)) { Serial.println("MONITOR ON -> failed"); return true; }
    Serial.println("MONITOR ON");
    if (g_speechEnabled) speakTokenState("monitor", true);
    return true;
  }
  if (upper == "MONITOR OFF") {
    if (!setMonitorEnabled(false)) { Serial.println("MONITOR OFF -> failed"); return true; }
    Serial.println("MONITOR OFF");
    if (g_speechEnabled) speakTokenState("monitor", false);
    return true;
  }
  if (upper == "MONLEVEL?") {
    uint16_t raw = 0;
    if (!queryMonitorLevel(raw, 800)) { Serial.println("MONLEVEL? -> no reply"); return true; }
    Serial.print("MONLEVEL ");
    Serial.print((int)levelRawToPercent(raw));
    Serial.println("%");
    speakFeatureValue(voice_monitor, voice_monitor_len, levelRawToPercent(raw));
    return true;
  }
  if (upper == "TRANSCEIVE?") {
    bool on = false;
    if (!queryTransceiveEnabled(on, 800)) { Serial.println("TRANSCEIVE? -> no reply"); return true; }
    Serial.println(on ? "TRANSCEIVE ON" : "TRANSCEIVE OFF");
    if (g_speechEnabled) speakTokenState("transceiver", on);
    return true;
  }
  if (upper == "NRLEVEL?") {
    uint16_t raw = 0;
    if (!queryNrLevel(raw, 800)) { Serial.println("NRLEVEL? -> no reply"); return true; }
    Serial.print("NRLEVEL ");
    Serial.print((int)levelRawToPercent(raw));
    Serial.println("%");
    speakFeatureValue(voice_noisereduction, voice_noisereduction_len, levelRawToPercent(raw));
    return true;
  }
  if (upper.startsWith("NRLEVEL ")) {
    int percent = line.substring(8).toInt();
    if (percent < 0 || percent > 100) { Serial.println("NRLEVEL -> invalid (use 0..100)"); return true; }
    if (!setNrLevel(levelPercentToRaw(percent))) { Serial.println("NRLEVEL -> failed"); return true; }
    Serial.print("NRLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    speakFeatureValue(voice_noisereduction, voice_noisereduction_len, (uint8_t)percent);
    return true;
  }
  if (upper == "NBLEVEL?") {
    uint16_t raw = 0;
    if (!queryNbLevel(raw, 800)) { Serial.println("NBLEVEL? -> no reply"); return true; }
    Serial.print("NBLEVEL ");
    Serial.print((int)levelRawToPercent(raw));
    Serial.println("%");
    if (g_speechEnabled) {
      playClipProgmem(voice_noiseblanker, voice_noiseblanker_len);
      playSilenceMs(60);
      speakDigitsAndPoint(String((int)levelRawToPercent(raw)));
      playSilenceMs(60);
      speakToken("percent");
    }
    return true;
  }
  if (upper.startsWith("NBLEVEL ")) {
    int percent = line.substring(8).toInt();
    if (percent < 0 || percent > 100) { Serial.println("NBLEVEL -> invalid (use 0..100)"); return true; }
    if (!setNbLevel(levelPercentToRaw(percent))) { Serial.println("NBLEVEL -> failed"); return true; }
    Serial.print("NBLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    if (g_speechEnabled) {
      playClipProgmem(voice_noiseblanker, voice_noiseblanker_len);
      playSilenceMs(60);
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
    if (!setPbtInner(pbtOffsetToRaw(value))) { Serial.println("PBT1 -> failed"); return true; }
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
    if (!setPbtOuter(pbtOffsetToRaw(value))) { Serial.println("PBT2 -> failed"); return true; }
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
    if (!setMode(mode, (uint8_t)filter)) { Serial.println("FILWIDTH -> failed"); return true; }
    Serial.print("FILWIDTH ");
    Serial.println(filter);
    if (g_speechEnabled) {
      speakToken("filterwidth");
      playSilenceMs(60);
      playDigit((uint8_t)filter);
    }
    return true;
  }
  if (upper.startsWith("MONLEVEL ")) {
    int percent = line.substring(9).toInt();
    if (percent < 0 || percent > 100) { Serial.println("MONLEVEL -> invalid (use 0..100)"); return true; }
    if (!setMonitorLevel(levelPercentToRaw(percent))) { Serial.println("MONLEVEL -> failed"); return true; }
    Serial.print("MONLEVEL ");
    Serial.print(percent);
    Serial.println("%");
    speakFeatureValue(voice_monitor, voice_monitor_len, (uint8_t)percent);
    return true;
  }
  if (upper == "TRANSCEIVE ON") {
    if (!setTransceiveEnabled(true)) { Serial.println("TRANSCEIVE ON -> failed"); return true; }
    Serial.println("TRANSCEIVE ON");
    if (g_speechEnabled) speakTokenState("transceiver", true);
    return true;
  }
  if (upper == "TRANSCEIVE OFF") {
    if (!setTransceiveEnabled(false)) { Serial.println("TRANSCEIVE OFF -> failed"); return true; }
    Serial.println("TRANSCEIVE OFF");
    if (g_speechEnabled) speakTokenState("transceiver", false);
    return true;
  }
  if (upper == "NOTCH?") {
    if (!sp.caps.getNotch) { if (usbConsoleReady()) Serial.println("NOTCH? -> unsupported"); return true; }
    if (!refreshLiveNotch()) { if (usbConsoleReady()) Serial.println("NOTCH? -> no reply"); return true; }
    if (!live.notchOn) {
      if (usbConsoleReady()) Serial.println("NOTCH OFF");
      speakNotchCycleState(false, NOTCH_WIDTH_UNKNOWN);
      return true;
    }
    if (live.notchWidthValid) {
      if (usbConsoleReady()) {
        if (live.notchWidth == NOTCH_WIDTH_NAR) Serial.println("NOTCH NAR");
        else if (live.notchWidth == NOTCH_WIDTH_MID) Serial.println("NOTCH MID");
        else if (live.notchWidth == NOTCH_WIDTH_WIDE) Serial.println("NOTCH WIDE");
        else Serial.println("NOTCH ON");
      }
      speakNotchCycleState(true, live.notchWidth);
      return true;
    }
    if (usbConsoleReady()) Serial.println("NOTCH ON");
    speakTokenState("notch filter", true);
    return true;
  }
  if (upper == "NR ON") {
    if (!sp.caps.setNr) { Serial.println("NR ON -> unsupported"); return true; }
    if (!applyNrAndTrack(true)) { Serial.println("NR ON -> failed"); return true; }
    Serial.println("NR ON");
    speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, true);
    return true;
  }
  if (upper == "NR OFF") {
    if (!sp.caps.setNr) { Serial.println("NR OFF -> unsupported"); return true; }
    if (!applyNrAndTrack(false)) { Serial.println("NR OFF -> failed"); return true; }
    Serial.println("NR OFF");
    speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, false);
    return true;
  }
  if (upper == "NR TOGGLE") {
    if (!sp.caps.getNr || !sp.caps.setNr) { Serial.println("NR TOGGLE -> unsupported"); return true; }
    if (!refreshLiveNr()) { Serial.println("NR TOGGLE -> no reply"); return true; }
    const bool next = !live.nrOn;
    if (!applyNrAndTrack(next)) { Serial.println("NR TOGGLE -> failed"); return true; }
    Serial.println(next ? "NR ON" : "NR OFF");
    speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, next);
    return true;
  }
  if (upper == "NB ON") {
    if (!sp.caps.setNb) { Serial.println("NB ON -> unsupported"); return true; }
    if (!applyNbAndTrack(true)) { Serial.println("NB ON -> failed"); return true; }
    Serial.println("NB ON");
    speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, true);
    return true;
  }
  if (upper == "NB OFF") {
    if (!sp.caps.setNb) { Serial.println("NB OFF -> unsupported"); return true; }
    if (!applyNbAndTrack(false)) { Serial.println("NB OFF -> failed"); return true; }
    Serial.println("NB OFF");
    speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, false);
    return true;
  }
  if (upper == "NB TOGGLE") {
    if (!sp.caps.getNb || !sp.caps.setNb) { Serial.println("NB TOGGLE -> unsupported"); return true; }
    if (!refreshLiveNb()) { Serial.println("NB TOGGLE -> no reply"); return true; }
    const bool next = !live.nbOn;
    if (!applyNbAndTrack(next)) { Serial.println("NB TOGGLE -> failed"); return true; }
    Serial.println(next ? "NB ON" : "NB OFF");
    speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, next);
    return true;
  }
  if (upper == "PA?") {
    bool on = false;
    if (!asciiQueryPreamp(sp, on, 800)) { Serial.println("PA? -> no reply"); return true; }
    Serial.println(on ? "PA ON" : "PA OFF");
    return true;
  }
  if (upper == "PA ON") {
    if (!asciiSetPreamp(sp, true)) { Serial.println("PA ON -> failed"); return true; }
    Serial.println("PA ON");
    return true;
  }
  if (upper == "PA OFF") {
    if (!asciiSetPreamp(sp, false)) { Serial.println("PA OFF -> failed"); return true; }
    Serial.println("PA OFF");
    return true;
  }
  if (upper == "PA TOGGLE") {
    bool on = false;
    if (!asciiQueryPreamp(sp, on, 800)) { Serial.println("PA TOGGLE -> no reply"); return true; }
    if (!asciiSetPreamp(sp, !on)) { Serial.println("PA TOGGLE -> failed"); return true; }
    Serial.println(!on ? "PA ON" : "PA OFF");
    return true;
  }
  if (upper == "GT?") {
    String rsp;
    if (!asciiQueryAgcLine(sp, rsp, 800)) { Serial.println("GT? -> no reply"); return true; }
    Serial.print("GT: ");
    printAsciiReplyPayload(rsp, sp.ascii.agcReplyPrefix);
    return true;
  }
  if (upper == "GT FAST") {
    if (!asciiSetAgcCommand(sp, sp.ascii.agcFastCmd)) { Serial.println("GT FAST -> failed"); return true; }
    Serial.println("GT FAST");
    return true;
  }
  if (upper == "GT SLOW") {
    if (!asciiSetAgcCommand(sp, sp.ascii.agcSlowCmd)) { Serial.println("GT SLOW -> failed"); return true; }
    Serial.println("GT SLOW");
    return true;
  }
  if (upper == "GT OFF") {
    if (!asciiSetAgcCommand(sp, sp.ascii.agcOffCmd)) { Serial.println("GT OFF -> failed"); return true; }
    Serial.println("GT OFF");
    return true;
  }
  if (upper == "PS?") {
    bool on = false;
    if (!asciiQueryPowerState(sp, on, 800)) { Serial.println("PS? -> no reply"); return true; }
    Serial.println(on ? "PS ON" : "PS OFF");
    return true;
  }
  if (upper == "PS ON") {
    if (!asciiSetPowerState(sp, true)) { Serial.println("PS ON -> failed"); return true; }
    Serial.println("PS ON");
    return true;
  }
  if (upper == "PS OFF") {
    if (!asciiSetPowerState(sp, false)) { Serial.println("PS OFF -> failed"); return true; }
    Serial.println("PS OFF");
    return true;
  }
  if (upper == "SPLIT TOGGLE") {
    bool on = false;
    if (!querySplit(on, 800)) { Serial.println("SPLIT TOGGLE -> no reply"); return true; }
    if (!setSplit(!on)) { Serial.println("SPLIT TOGGLE -> failed"); return true; }
    Serial.println(!on ? "SPLIT ON" : "SPLIT OFF");
    speakTokenState("split", !on);
    return true;
  }
  if (upper == "NOTCH ON") {
    if (!sp.caps.setNotch) { Serial.println("NOTCH ON -> unsupported"); return true; }
    if (!applyNotchAndTrack(true)) { Serial.println("NOTCH ON -> failed"); return true; }
    Serial.println("NOTCH ON");
    speakTokenState("notch filter", true);
    return true;
  }
  if (upper == "NOTCH NAR") {
    if (currentProtocolType() != PROTO_CIV || !sp.caps.setNotch) { Serial.println("NOTCH NAR -> unsupported"); return true; }
    if (!applyNotchAndTrack(true) || !applyNotchWidthAndTrack(NOTCH_WIDTH_NAR)) { Serial.println("NOTCH NAR -> failed"); return true; }
    Serial.println("NOTCH NAR");
    speakNotchCycleState(true, NOTCH_WIDTH_NAR);
    return true;
  }
  if (upper == "NOTCH MID") {
    if (currentProtocolType() != PROTO_CIV || !sp.caps.setNotch) { Serial.println("NOTCH MID -> unsupported"); return true; }
    if (!applyNotchAndTrack(true) || !applyNotchWidthAndTrack(NOTCH_WIDTH_MID)) { Serial.println("NOTCH MID -> failed"); return true; }
    Serial.println("NOTCH MID");
    speakNotchCycleState(true, NOTCH_WIDTH_MID);
    return true;
  }
  if (upper == "NOTCH WIDE") {
    if (currentProtocolType() != PROTO_CIV || !sp.caps.setNotch) { Serial.println("NOTCH WIDE -> unsupported"); return true; }
    if (!applyNotchAndTrack(true) || !applyNotchWidthAndTrack(NOTCH_WIDTH_WIDE)) { Serial.println("NOTCH WIDE -> failed"); return true; }
    Serial.println("NOTCH WIDE");
    speakNotchCycleState(true, NOTCH_WIDTH_WIDE);
    return true;
  }
  if (upper == "NOTCH OFF") {
    if (!sp.caps.setNotch) { Serial.println("NOTCH OFF -> unsupported"); return true; }
    if (!applyNotchAndTrack(false)) { Serial.println("NOTCH OFF -> failed"); return true; }
    Serial.println("NOTCH OFF");
    speakTokenState("notch filter", false);
    return true;
  }
  if (upper == "NOTCH TOGGLE") {
    if (!sp.caps.getNotch || !sp.caps.setNotch) { Serial.println("NOTCH TOGGLE -> unsupported"); return true; }
    if (!refreshLiveNotch()) { Serial.println("NOTCH TOGGLE -> no reply"); return true; }
    const bool next = !live.notchOn;
    if (!applyNotchAndTrack(next)) { Serial.println("NOTCH TOGGLE -> failed"); return true; }
    Serial.println(next ? "NOTCH ON" : "NOTCH OFF");
    speakTokenState("notch filter", next);
    return true;
  }
  return false;
}

static bool handleConsoleBankCommands(const String& line, const String& upper) {
  if (upper == "BANK NEXT") {
    uint8_t nextBank = (uiGetBank() >= 3) ? 1 : (uint8_t)(uiGetBank() + 1);
    uiSetBank(nextBank);
    Serial.print("OK BANK ");
    Serial.println((int)nextBank);
    speakBankNumber();
    return true;
  }
  if (upper == "BANK PREV") {
    uint8_t prevBank = (uiGetBank() <= 1) ? 3 : (uint8_t)(uiGetBank() - 1);
    uiSetBank(prevBank);
    Serial.print("OK BANK ");
    Serial.println((int)prevBank);
    speakBankNumber();
    return true;
  }
  if (upper.startsWith("BANK ")) {
    int b = line.substring(5).toInt();
    if (b < 1 || b > 3) { Serial.println("BANK -> invalid (use 1..3)"); speakError(); return true; }
    uiSetBank((uint8_t)b);
    Serial.print("OK BANK ");
    Serial.println((int)b);
    speakBankNumber();
    return true;
  }
  return false;
}

void processCommand(String line) {
  line.trim();
  if (!line.length()) return;
  if (g_audioPlaying) audioAbortNow();

  if (usbConsoleReady()) {
    Serial.print("> ");
    Serial.println(line);
  }

  String upper = upperCopy(line);
  if (handleConsoleInfoCommands(upper)) return;
  if (handleConsoleProfileCommands(line, upper)) return;
  if (handleFtdx10BlockedConsoleCommand(upper)) return;
  if (handleConsoleToggleCommands(line, upper)) return;
  if (handleConsoleYaesuFt8x7Commands(line, upper)) return;
  if (handleConsoleRadioCommands(line, upper)) return;
  if (handleConsoleBankCommands(line, upper)) return;
  if (usbConsoleReady()) Serial.println("Unknown. Type HELP");
}
