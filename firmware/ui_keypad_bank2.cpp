// Bank 2 keypad actions: noise reduction, noise blanker, notch, PBT and filter.
#include "ui_keypad_bank.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "radio_monitor.h"
#include "radio_runtime.h"

static void speakNrLevel(int level) {
  if (!g_speechEnabled) return;
  playClipProgmem(voice_noisereduction, voice_noisereduction_len);
  playSilenceMs(60);
  if (level <= 0) playClipProgmem(voice_off, voice_off_len);
  else if (level == 1) playClipProgmem(voice_one, voice_one_len);
  else playClipProgmem(voice_two, voice_two_len);
}

static int pbtRawToOffset(uint16_t raw) {
  if (raw > 255) raw = 255;
  return (int)raw - 128;
}

static uint16_t pbtOffsetToRaw(int offset) {
  if (offset < -128) offset = -128;
  if (offset > 127) offset = 127;
  return (uint16_t)(offset + 128);
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

void queryBank2Nr() {
  printKeypadCommand("BANK2 1 SHORT -> NR?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NR?");
    return;
  }
  if (keypadReportIfUnsupported(currentStoredProfile().caps.getNr, "NR?")) return;
  g_suspendPollingUntilMs = millis() + 900;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  cancelPendingFreqAnnouncement();
  if (!refreshLiveNr()) {
    if (!keypadReportIfTimedOut("NR?")) {
      printKeypadStatus("NR? -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }
  printKeypadStatus(live.nrOn ? "NR ON" : "NR OFF");
  speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, live.nrOn);
}

void queryBank2Nb() {
  printKeypadCommand("BANK2 2 SHORT -> NB?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NB?");
    return;
  }
  if (keypadReportIfUnsupported(currentStoredProfile().caps.getNb, "NB?")) return;
  g_suspendPollingUntilMs = millis() + 900;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  cancelPendingFreqAnnouncement();
  if (!refreshLiveNb()) {
    if (!keypadReportIfTimedOut("NB?")) {
      printKeypadStatus("NB? -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }
  printKeypadStatus(live.nbOn ? "NB ON" : "NB OFF");
  speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, live.nbOn);
}

void queryBank2Notch() {
  printKeypadCommand("BANK2 3 SHORT -> NOTCH?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NOTCH?");
    return;
  }
  if (keypadReportIfUnsupported(currentStoredProfile().caps.getNotch, "NOTCH?")) return;
  g_suspendPollingUntilMs = millis() + 900;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  cancelPendingFreqAnnouncement();
  if (!refreshLiveNotch()) {
    if (!keypadReportIfTimedOut("NOTCH?")) {
      printKeypadStatus("NOTCH? -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }
  if (!live.notchOn) {
    printKeypadStatus("NOTCH OFF");
    speakNotchCycleState(false, NOTCH_WIDTH_UNKNOWN);
    return;
  }
  if (live.notchWidthValid) {
    if (live.notchWidth == NOTCH_WIDTH_NAR) printKeypadStatus("NOTCH NAR");
    else if (live.notchWidth == NOTCH_WIDTH_MID) printKeypadStatus("NOTCH MID");
    else if (live.notchWidth == NOTCH_WIDTH_WIDE) printKeypadStatus("NOTCH WIDE");
    speakNotchCycleState(true, live.notchWidth);
    return;
  }
  printKeypadStatus("NOTCH ON");
  speakTokenState("notch filter", true);
}

void toggleBank2Nr() {
  printKeypadCommand("BANK2 1 LONG -> NR");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NR TOGGLE");
    return;
  }
  if (keypadReportIfUnsupported(currentStoredProfile().caps.setNr, "NR")) return;
  prepareKeypadSpeechResponse();
  if (currentProtocolType() == PROTO_KENWOOD_ASCII && String(currentProfile().name).indexOf("TS-480") >= 0) {
    String line;
    int nextLevel = 1;
    if (transactAsciiCommand(currentStoredProfile().ascii.nrGet, line, currentStoredProfile().ascii.nrReplyPrefix, 800)) {
      int start = (int)strlen(currentStoredProfile().ascii.nrReplyPrefix);
      int semi = line.indexOf(';', start);
      if (semi < 0) semi = line.length();
      String value = line.substring(start, semi);
      value.trim();
      int currentLevel = value.toInt();
      if (currentLevel <= 0) nextLevel = 1;
      else if (currentLevel == 1) nextLevel = 2;
      else nextLevel = 0;
    }
    const char* cmd = (nextLevel == 0) ? "NR0;" : (nextLevel == 1) ? "NR1;" : "NR2;";
    if (asciiPacketSendCommand(cmd)) {
      live.nrOn = nextLevel != 0;
      live.nrValid = true;
      printKeypadStatus(nextLevel == 0 ? "NR OFF" : (nextLevel == 1 ? "NR 1" : "NR 2"));
      speakNrLevel(nextLevel);
    }
    return;
  }
  if (!live.nrValid && !refreshLiveNr()) { keypadReportIfTimedOut("NR"); return; }
  bool next = !live.nrOn;
  if (!applyNrAndTrack(next)) { keypadReportIfTimedOut("NR"); return; }
  printKeypadStatus(next ? "NR ON" : "NR OFF");
  speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, next);
}

void toggleBank2Nb() {
  printKeypadCommand("BANK2 2 LONG -> NB");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NB TOGGLE");
    return;
  }
  if (keypadReportIfUnsupported(currentStoredProfile().caps.setNb, "NB")) return;
  prepareKeypadSpeechResponse();
  if (!live.nbValid && !refreshLiveNb()) { keypadReportIfTimedOut("NB"); return; }
  bool next = !live.nbOn;
  if (!applyNbAndTrack(next)) { keypadReportIfTimedOut("NB"); return; }
  printKeypadStatus(next ? "NB ON" : "NB OFF");
  speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, next);
}

void toggleBank2Notch() {
  printKeypadCommand("BANK2 3 LONG -> NOTCH");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NOTCH TOGGLE");
    return;
  }
  if (keypadReportIfUnsupported(currentStoredProfile().caps.setNotch, "NOTCH")) return;
  prepareKeypadSpeechResponse();

  if (currentProtocolType() != PROTO_CIV) {
    if (!live.notchValid && !refreshLiveNotch()) { keypadReportIfTimedOut("NOTCH"); return; }
    bool next = !live.notchOn;
    if (!applyNotchAndTrack(next)) { keypadReportIfTimedOut("NOTCH"); return; }
    printKeypadStatus(next ? "NOTCH ON" : "NOTCH OFF");
    speakTokenState("notch filter", next);
    return;
  }

  if (!live.notchValid && !refreshLiveNotch()) { keypadReportIfTimedOut("NOTCH"); return; }

  if (!live.notchOn) {
    if (!applyNotchAndTrack(true)) { keypadReportIfTimedOut("NOTCH"); return; }
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_NAR)) { keypadReportIfTimedOut("NOTCH"); return; }
    printKeypadStatus("NOTCH NAR");
    speakNotchCycleState(true, NOTCH_WIDTH_NAR);
    return;
  }

  if (!live.notchWidthValid && !refreshLiveNotchWidth()) { keypadReportIfTimedOut("NOTCH"); return; }

  if (live.notchWidth == NOTCH_WIDTH_NAR) {
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_MID)) { keypadReportIfTimedOut("NOTCH"); return; }
    printKeypadStatus("NOTCH MID");
    speakNotchCycleState(true, NOTCH_WIDTH_MID);
    return;
  }

  if (live.notchWidth == NOTCH_WIDTH_MID) {
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_WIDE)) { keypadReportIfTimedOut("NOTCH"); return; }
    printKeypadStatus("NOTCH WIDE");
    speakNotchCycleState(true, NOTCH_WIDTH_WIDE);
    return;
  }

  if (!applyNotchAndTrack(false)) { keypadReportIfTimedOut("NOTCH"); return; }
  printKeypadStatus("NOTCH OFF");
  speakNotchCycleState(false, NOTCH_WIDTH_UNKNOWN);
}

void queryBank2NrLevel() {
  printKeypadCommand("BANK2 4 SHORT -> NRLEVEL?");
  uint16_t raw = 0;
  if (!queryNrLevel(raw, 800)) {
    if (!keypadReportIfTimedOut("NRLEVEL?")) {
      printKeypadStatus("NRLEVEL? -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }
  uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus(String("NRLEVEL ") + String((int)percent) + "%");
  speakFeatureValue(voice_noisereduction, voice_noisereduction_len, percent);
}

void adjustBank2NrLevel(int deltaPercent) {
  printKeypadCommand(String("BANK2 4 ") + (deltaPercent > 0 ? "LONG" : "DOUBLE") + " -> NRLEVEL");
  uint16_t raw = 0;
  if (!queryNrLevel(raw, 800)) { keypadReportIfTimedOut("NRLEVEL"); return; }
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  const uint16_t targetRaw = levelPercentToRaw(percent);

  bool wrote = setNrLevel(targetRaw);
  if (!wrote) {
    bool nrOn = false;
    if (queryNr(nrOn, 800) && !nrOn) {
      (void)setNr(true);
      wrote = setNrLevel(targetRaw);
    }
  }

  uint16_t readBack = 0;
  if (!queryNrLevel(readBack, 800)) {
    if (!keypadReportIfTimedOut("NRLEVEL")) printKeypadStatus("NRLEVEL -> failed");
    return;
  }

  const uint8_t readPercent = levelRawToPercent(readBack);
  printKeypadStatus(String("NRLEVEL ") + String((int)readPercent) + "%");
  if (!wrote && (bool)Serial) {
    Serial.println("WARN NRLEVEL write not confirmed; using readback value");
  }
  speakFeatureValue(voice_noisereduction, voice_noisereduction_len, readPercent);
}

void queryBank2NbLevel() {
  printKeypadCommand("BANK2 5 SHORT -> NBLEVEL?");
  uint16_t raw = 0;
  if (!queryNbLevel(raw, 800)) {
    if (!keypadReportIfTimedOut("NBLEVEL?")) {
      printKeypadStatus("NBLEVEL? -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }
  uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus(String("NBLEVEL ") + String((int)percent) + "%");
  speakTokenPercent("noiseblanker", percent);
}

void adjustBank2NbLevel(int deltaPercent) {
  printKeypadCommand(String("BANK2 5 ") + (deltaPercent > 0 ? "LONG" : "DOUBLE") + " -> NBLEVEL");
  uint16_t raw = 0;
  if (!queryNbLevel(raw, 800)) { keypadReportIfTimedOut("NBLEVEL"); return; }
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  if (!setNbLevel(levelPercentToRaw(percent))) { keypadReportIfTimedOut("NBLEVEL"); return; }
  printKeypadStatus(String("NBLEVEL ") + String(percent) + "%");
  speakTokenPercent("noiseblanker", (uint8_t)percent);
}

static bool ensureActiveVfoKnownForKeypad() {
  if (live.activeVfoKnown) return true;
  uint64_t dummy = 0;
  return queryVfoFrequency(true, dummy, 800);
}

static bool queryCurrentFilterSlotForKeypad(uint8_t& filterOut) {
  if (!ensureActiveVfoKnownForKeypad()) return false;
  uint8_t mode = 0xFF;
  return queryVfoMode(live.activeVfoA, mode, filterOut, 800);
}

void queryBank2PbtInner() {
  printKeypadCommand("BANK2 6 SHORT -> PBT1?");
  uint16_t raw = 0;
  if (!queryPbtInner(raw, 800)) {
    if (!keypadReportIfTimedOut("PBT1?")) {
      printKeypadStatus("PBT1? -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }
  printKeypadStatus(String("PBT1 ") + String(pbtRawToOffset(raw)) + " step");
  speakSignedStepValue("pbt", pbtRawToOffset(raw));
}

void adjustBank2PbtInner(int delta) {
  printKeypadCommand(String("BANK2 6 ") + (delta > 0 ? "LONG" : "DOUBLE") + " -> PBT1");
  uint16_t raw = 0;
  if (!queryPbtInner(raw, 800)) { keypadReportIfTimedOut("PBT1"); return; }
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtInner(pbtOffsetToRaw(next))) { keypadReportIfTimedOut("PBT1"); return; }
  queryBank2PbtInner();
}

void queryBank2PbtOuter() {
  printKeypadCommand("BANK2 7 SHORT -> PBT2?");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) { keypadReportIfTimedOut("PBT2?"); return; }
  printKeypadStatus(String("PBT2 ") + String(pbtRawToOffset(raw)) + " step");
  speakSignedStepValue("pbt", pbtRawToOffset(raw));
}

void adjustBank2PbtOuter(int delta) {
  printKeypadCommand(String("BANK2 7 ") + (delta > 0 ? "LONG" : "DOUBLE") + " -> PBT2");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) { keypadReportIfTimedOut("PBT2"); return; }
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtOuter(pbtOffsetToRaw(next))) { keypadReportIfTimedOut("PBT2"); return; }
  queryBank2PbtOuter();
}

void sendBank2FilterShapeQuery() { sendKeypadCommand("BANK2 8 SHORT -> FILSHAPE?", "FILSHAPE?"); }

void toggleBank2FilterShape() {
  printKeypadCommand("BANK2 8 LONG -> FILSHAPE");
  bool soft = false;
  if (!queryFilterShape(soft, 800)) { keypadReportIfTimedOut("FILSHAPE"); return; }
  if (!setFilterShape(!soft)) { keypadReportIfTimedOut("FILSHAPE"); return; }
  printKeypadStatus(!soft ? "FILSHAPE SOFT" : "FILSHAPE SHARP");
  if (g_speechEnabled) {
    speakToken("filtershape");
    playSilenceMs(60);
    speakToken(!soft ? "soft" : "sharp");
  }
}

void queryBank2FilterWidth() {
  printKeypadCommand("BANK2 9 SHORT -> FILWIDTH?");
  uint8_t filter = 0xFF;
  if (!queryCurrentFilterSlotForKeypad(filter)) { keypadReportIfTimedOut("FILWIDTH?"); return; }
  printKeypadStatus(String("FILWIDTH ") + String((int)filter));
  if (g_speechEnabled) {
    speakToken("filterwidth");
    playSilenceMs(60);
    playDigit(filter);
  }
}

void cycleBank2FilterWidth(int delta) {
  printKeypadCommand(String("BANK2 9 ") + (delta > 0 ? "LONG" : "DOUBLE") + " -> FILWIDTH");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!ensureActiveVfoKnownForKeypad()) { keypadReportIfTimedOut("FILWIDTH"); return; }
  if (!queryVfoMode(live.activeVfoA, mode, filter, 800)) { keypadReportIfTimedOut("FILWIDTH"); return; }
  int next = (int)filter + delta;
  if (next < 1) next = 3;
  if (next > 3) next = 1;
  if (!setMode(mode, (uint8_t)next)) { keypadReportIfTimedOut("FILWIDTH"); return; }
  printKeypadStatus(String("FILWIDTH ") + String(next));
  if (g_speechEnabled) {
    speakToken("filterwidth");
    playSilenceMs(60);
    playDigit((uint8_t)next);
  }
}

void ftdx10QueryAgc() { sendKeypadCommand("BANK2 4 SHORT -> GT?", "GT?"); }

void ftdx10AgcFast() { sendKeypadCommand("BANK2 4 LONG -> GT FAST", "GT FAST"); }

void ftdx10AgcSlow() { sendKeypadCommand("BANK2 4 DOUBLE -> GT SLOW", "GT SLOW"); }

void ftdx10QueryPowerState() { sendKeypadCommand("BANK2 5 SHORT -> PS?", "PS?"); }

void ftdx10PowerOff() { sendKeypadCommand("BANK2 5 LONG -> PS OFF", "PS OFF"); }

void ftdx10PowerOn() { sendKeypadCommand("BANK2 5 DOUBLE -> PS ON", "PS ON"); }

void ftdx10QueryInfo() { sendKeypadCommand("BANK2 6 SHORT -> IF?", "IF?"); }

void ftdx10QueryId() { sendKeypadCommand("BANK2 7 SHORT -> ID?", "ID?"); }
