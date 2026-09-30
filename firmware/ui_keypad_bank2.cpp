// Bank 2 keypad actions: noise reduction, noise blanker, notch, PBT and filter,
// and the FT-857/897 settings kept in the EEPROM.
#include "ui_keypad_bank.h"
#include "ui_features.h"
#include "radio_monitor.h"

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
  printKeypadAction("NR?");
  prepareKeypadSpeechResponse();
  NrState state;
  if (keypadReportFeatureFailure(nrQuery(state), "NR?")) return;
  printKeypadStatus(nrStateText(state));
  speakNrState(state);
}

void queryBank2Nb() {
  printKeypadAction("NB?");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(nbQuery(on), "NB?")) return;
  printKeypadStatus(nbStateText(on));
  speakNbState(on);
}

void queryBank2Notch() {
  printKeypadAction("NOTCH?");
  prepareKeypadSpeechResponse();
  NotchState state;
  if (keypadReportFeatureFailure(notchQuery(state), "NOTCH?")) return;
  printKeypadStatus(notchStateText(state));
  speakNotchState(state);
}

static void queryBank2Ft857Setting(Ft8x7Setting setting) {
  const char* label = ft8x7SettingLabel(setting);
  printKeypadAction(label);
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  if (keypadReportFeatureFailure(ft8x7SettingQuery(setting, state), label)) return;
  printKeypadStatus(ft8x7SettingText(state));
  speakFt8x7Setting(state);
}

void queryBank2Ft8x7Agc() { queryBank2Ft857Setting(Ft8x7Setting::Agc); }
void queryBank2Ft857Ipo() { queryBank2Ft857Setting(Ft8x7Setting::Ipo); }
void queryBank2Ft857Att() { queryBank2Ft857Setting(Ft8x7Setting::Att); }
void queryBank2Ft857Dbf() { queryBank2Ft857Setting(Ft8x7Setting::Dbf); }
void queryBank2Ft8x7BreakIn() { queryBank2Ft857Setting(Ft8x7Setting::BreakIn); }
void queryBank2Ft8x7Keyer() { queryBank2Ft857Setting(Ft8x7Setting::Keyer); }
void queryBank2Ft857Nar() { queryBank2Ft857Setting(Ft8x7Setting::Nar); }
void queryBank2Ft8x7Menu() { queryBank2Ft857Setting(Ft8x7Setting::Menu); }
void queryBank2Ft8x7Row() { queryBank2Ft857Setting(Ft8x7Setting::Row); }

void toggleBank2Nr() {
  printKeypadAction("NR");
  prepareKeypadSpeechResponse();
  NrState state;
  if (keypadReportFeatureFailure(nrToggle(state), "NR")) return;
  printKeypadStatus(nrStateText(state));
  speakNrState(state);
}

void toggleBank2Nb() {
  printKeypadAction("NB");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(nbToggle(on), "NB")) return;
  printKeypadStatus(nbStateText(on));
  speakNbState(on);
}

void toggleBank2Notch() {
  printKeypadAction("NOTCH");
  prepareKeypadSpeechResponse();
  NotchState state;
  if (keypadReportFeatureFailure(notchToggle(state), "NOTCH")) return;
  printKeypadStatus(notchStateText(state));
  speakNotchState(state);
}

void queryBank2NrLevel() {
  printKeypadAction("NRLEVEL?");
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
  printKeypadAction("NRLEVEL");
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
  printKeypadAction("NBLEVEL?");
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
  printKeypadAction("NBLEVEL");
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
  printKeypadAction("PBT1?");
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
  printKeypadAction("PBT1");
  uint16_t raw = 0;
  if (!queryPbtInner(raw, 800)) { keypadReportIfTimedOut("PBT1"); return; }
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtInner(pbtOffsetToRaw(next))) { keypadReportIfTimedOut("PBT1"); return; }
  queryBank2PbtInner();
}

void queryBank2PbtOuter() {
  printKeypadAction("PBT2?");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) { keypadReportIfTimedOut("PBT2?"); return; }
  printKeypadStatus(String("PBT2 ") + String(pbtRawToOffset(raw)) + " step");
  speakSignedStepValue("pbt", pbtRawToOffset(raw));
}

void adjustBank2PbtOuter(int delta) {
  printKeypadAction("PBT2");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) { keypadReportIfTimedOut("PBT2"); return; }
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtOuter(pbtOffsetToRaw(next))) { keypadReportIfTimedOut("PBT2"); return; }
  queryBank2PbtOuter();
}

void toggleBank2FilterShape() {
  printKeypadAction("FILSHAPE");
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
  printKeypadAction("FILWIDTH?");
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
  printKeypadAction("FILWIDTH");
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
