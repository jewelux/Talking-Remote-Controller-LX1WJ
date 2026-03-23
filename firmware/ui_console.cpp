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

static bool usbConsoleReady() {
  return (bool)Serial;
}

static void speakNotchCycleState(bool on, NotchWidth width) {
  if (!g_speechEnabled) return;
  playClipProgmem(voice_notchfilter, voice_notchfilter_len);
  playSilenceMs(60);
  if (!on) {
    playClipProgmem(voice_off, voice_off_len);
    return;
  }
  switch (width) {
    case NOTCH_WIDTH_NAR: playClipProgmem(voice_one, voice_one_len); break;
    case NOTCH_WIDTH_MID: playClipProgmem(voice_two, voice_two_len); break;
    case NOTCH_WIDTH_WIDE: playClipProgmem(voice_three, voice_three_len); break;
    default: playClipProgmem(voice_on, voice_on_len); break;
  }
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

void printHelp() {
  Serial.println();
  Serial.println("Commands (case-insensitive):");
  Serial.println("  HELP");
  Serial.println("  STATUS?");
  Serial.println("  MODE LIST");
  Serial.println("  QUIET ON | QUIET OFF");
  Serial.println("  QUIET?");
  Serial.println("  SPEECH ON | SPEECH OFF");
  Serial.println("  SPEECH?");
  Serial.println("  BANK?");
  Serial.println("  BANK NEXT | PREV");
  Serial.println("  LFREQ");
  Serial.println("  FREQ?");
  Serial.println("  FREQ <kHz>");
  Serial.println("  FREQMHZ <MHz>");
  Serial.println("  MODE?");
  Serial.println("  MODE <n>");
  Serial.println("  IF? | ID? | OM?");
  Serial.println("  FB? | FB <kHz> | FBMHZ <MHz>");
  Serial.println("  FR? | FR0");
  Serial.println("  FT? | FT A | FT B");
  Serial.println("  RX | TX");
  Serial.println("  AK?");
  Serial.println("  SWT <nn> | SWH <nn>");
  Serial.println("  SM?");
  Serial.println("  SWR?");
  Serial.println("  PO?");
  Serial.println("  NR?");
  Serial.println("  NR ON | OFF");
  Serial.println("  NB?");
  Serial.println("  NB ON | OFF");
  Serial.println("  PA? | PA ON | OFF");
  Serial.println("  GT? | GT FAST | SLOW");
  Serial.println("  PS? | PS ON | OFF");
  Serial.println("  NOTCH?");
  Serial.println("  NOTCH ON | OFF");
  Serial.println("  NOTCH NAR | MID | WIDE");
  Serial.println("  YSTATUS?");
  Serial.println("  YALL?");
  Serial.println("  ALC? | VOL? | SQL?");
  Serial.println("  VFO TOGGLE | A | B");
  Serial.println("  PTT ON | OFF");
  Serial.println("  SPLIT ON | OFF");
  Serial.println("  CLAR ON | OFF");
  Serial.println("  CLAR OFFSET <8 hex digits>");
  Serial.println("  MEM WRITE | MEM READ RAW");
  Serial.println("  AGC <hex byte>");
  Serial.println("  LOCKDOC ON | OFF");
  Serial.println("  YCAT <10 hex digits>");
  Serial.println("  YCAT? <10 hex digits>");
  Serial.println("  PROFILE <1..9>");
  Serial.println("  PROFILE NEXT | PREV");
  Serial.println("  SAY <digits>");
  Serial.println("  TEST");
  Serial.println("  BANK 1|2|3");
  Serial.println("  PROFILE?");
  Serial.println("  SLOTS?");
  Serial.println("  TUNINGSPEECH?");
  Serial.println("  TUNINGSPEECH ON | OFF");
  Serial.println("  VOLUME?");
  Serial.println("  VOLUME <0..3>");
  Serial.println("  LISTVOICES");
  Serial.println("  VOICE <name>");
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
    if (slot < 1 || slot > 9 || !storedProfileForId((uint8_t)slot)) {
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
    if (yaesuCatQueryVolumeRaw(raw, 800)) {
      Serial.print("  VOL: ");
      Serial.println(raw);
    } else {
      Serial.println("  VOL: no reply");
    }
    if (yaesuCatQuerySquelchRaw(raw, 800)) {
      Serial.print("  SQL: ");
      Serial.println(raw);
    } else {
      Serial.println("  SQL: no reply");
    }

    uint8_t status = 0;
    if (yaesuCatQueryStatusRaw(status, 800)) {
      Serial.print("  STATUS: 0x");
      if (status < 0x10) Serial.print('0');
      Serial.println(status, HEX);
    } else {
      Serial.println("  STATUS: no reply");
    }
    return true;
  }

  if (upper == "YSTATUS?") {
    uint8_t raw = 0;
    if (!yaesuCatQueryStatusRaw(raw, 800)) { Serial.println("YSTATUS? -> no reply"); return true; }
    Serial.print("YSTATUS: 0x");
    if (raw < 0x10) Serial.print('0');
    Serial.println(raw, HEX);
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
    int32_t raw = 0;
    if (!yaesuCatQueryVolumeRaw(raw, 800)) { Serial.println("VOL? -> no reply"); return true; }
    Serial.print("VOL: ");
    Serial.println(raw);
    return true;
  }
  if (upper == "SQL?") {
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
    yaesuCatSelectVfoA();
    Serial.println("VFO A");
    return true;
  }
  if (upper == "VFO B") {
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
  if (upper == "LOCK?") {
    Serial.println("LOCK? -> no safe read command known here; documented write bytes collide with VFO A/B");
    return true;
  }
  if (upper == "LOCK ON" || upper == "LOCK OFF") {
    Serial.println("LOCK -> not sent via generic LOCK; documented bytes collide with VFO A/B");
    Serial.println("Use LOCKDOC ON/OFF if you want to send the documented raw bytes deliberately.");
    return true;
  }
  if (upper == "LOCKDOC ON") {
    yaesuCatSetLockDocumentedRaw(true);
    Serial.println("LOCKDOC ON -> sent documented raw bytes 00 00 00 01 00");
    return true;
  }
  if (upper == "LOCKDOC OFF") {
    yaesuCatSetLockDocumentedRaw(false);
    Serial.println("LOCKDOC OFF -> sent documented raw bytes 00 00 00 00 00");
    return true;
  }
  if (upper == "MEM WRITE") {
    yaesuCatMemoryWrite();
    Serial.println("MEM WRITE");
    return true;
  }
  if (upper == "MEM READ RAW") {
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
    char k = line.substring(5)[0];
    uint8_t mode = 0xFF;
    (void)profileModeFromDigit(k, mode);
    if (mode == 0xFF || !applyModeAndTrack(mode, 1)) { Serial.println("SET MODE -> failed"); return true; }
    Serial.println("SET MODE -> command sent");
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
      playClipProgmem(voice_swr, voice_swr_len);
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
    speakBinaryFeatureState(voice_notchfilter, voice_notchfilter_len, true);
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
  if (upper == "NOTCH ON") {
    if (!sp.caps.setNotch) { Serial.println("NOTCH ON -> unsupported"); return true; }
    if (!applyNotchAndTrack(true)) { Serial.println("NOTCH ON -> failed"); return true; }
    Serial.println("NOTCH ON");
    speakBinaryFeatureState(voice_notchfilter, voice_notchfilter_len, true);
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
    speakBinaryFeatureState(voice_notchfilter, voice_notchfilter_len, false);
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
  if (handleConsoleToggleCommands(line, upper)) return;
  if (handleConsoleYaesuFt8x7Commands(line, upper)) return;
  if (handleConsoleRadioCommands(line, upper)) return;
  if (handleConsoleBankCommands(line, upper)) return;
  if (usbConsoleReady()) Serial.println("Unknown. Type HELP");
}
