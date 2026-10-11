// Bank 2 keypad actions: noise reduction, noise blanker, notch, PBT and filter,
// and the FT-8x7 settings kept in the EEPROM, among them the FT-817 antenna jack.
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
  speakLabel(label);
  if (value < 0) {
    speakToken("minus");
    playSilenceMs(60);
    value = -value;
  }
  speakNumber(String(value));
  playSilenceMs(60);
  speakToken("step");
}

void queryBank2Nr() {
  printKeypadAction("NR?");
  prepareKeypadSpeechResponse();
  NrState state;
  if (keypadReportFeatureFailure(nrQuery(state), "NR?")) return;
  printKeypadStatus("{}", nrStateText(state).c_str());
  speakNrState(state);
}

void queryBank2Nb() {
  printKeypadAction("NB?");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(nbQuery(on), "NB?")) return;
  printKeypadStatus("{}", nbStateText(on).c_str());
  speakNbState(on);
}

void queryBank2Notch() {
  printKeypadAction("NOTCH?");
  prepareKeypadSpeechResponse();
  NotchState state;
  if (keypadReportFeatureFailure(notchQuery(state), "NOTCH?")) return;
  printKeypadStatus("{}", notchStateText(state).c_str());
  speakNotchState(state);
}

void queryBank2Ft8x7NrLevel() { queryKeypadFt8x7Setting(Ft8x7Setting::NrLevel); }
void queryBank2Ft8x7NbLevel() { queryKeypadFt8x7Setting(Ft8x7Setting::NbLevel); }
void queryBank2Ft8x7Dbf() { queryKeypadFt8x7Setting(Ft8x7Setting::Dbf); }
void queryBank2Ft8x7LowCut() { queryKeypadFt8x7Setting(Ft8x7Setting::LowCut); }
void queryBank2Ft8x7HighCut() { queryKeypadFt8x7Setting(Ft8x7Setting::HighCut); }
void queryBank2Ft8x7MicEq() { queryKeypadFt8x7Setting(Ft8x7Setting::MicEq); }
void queryBank2Ft8x7Ipo() { queryKeypadFt8x7Setting(Ft8x7Setting::Ipo); }
void queryBank2Ft8x7Att() { queryKeypadFt8x7Setting(Ft8x7Setting::Att); }
void queryBank2Ft8x7Agc() { queryKeypadFt8x7Setting(Ft8x7Setting::Agc); }
void queryBank2Ft817Antenna() { queryKeypadFt8x7Setting(Ft8x7Setting::Antenna); }

static void stepKeypadFt8x7Level(Ft8x7Setting setting, const char* label, int delta) {
  printKeypadAction("{} STEP {}", label, delta);
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(ft8x7LevelStep(setting, delta, state), label, state);
}

void stepBank2Ft8x7NrLevel(int delta) { stepKeypadFt8x7Level(Ft8x7Setting::NrLevel, "NRLEVEL", delta); }
void stepBank2Ft8x7NbLevel(int delta) { stepKeypadFt8x7Level(Ft8x7Setting::NbLevel, "NBLEVEL", delta); }
void stepBank2Ft8x7LowCut(int delta) { stepKeypadFt8x7Level(Ft8x7Setting::LowCut, "HPF", delta); }
void stepBank2Ft8x7HighCut(int delta) { stepKeypadFt8x7Level(Ft8x7Setting::HighCut, "LPF", delta); }

void toggleBank2Ft8x7Dbf() {
  printKeypadAction("DBF");
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(ft8x7DbfToggle(state), "DBF", state);
}

static constexpr ChoiceOption kFt8x7AgcKeys[] = {
    {'0', (uint8_t)YaesuAgc::Off},
    {'1', (uint8_t)YaesuAgc::Auto},
    {'2', (uint8_t)YaesuAgc::Fast},
    {'3', (uint8_t)YaesuAgc::Slow},
};

static void sayPickedFt8x7Agc(char key, uint8_t agc) {
  printKeypadCommand("AGC SELECT -> {}", key);
  printKeypadStatus("AGC STAGE: {}", ft8x7AgcName((YaesuAgc)agc));
  if (g_speechEnabled) speakToken(ft8x7AgcName((YaesuAgc)agc));
}

static void commitFt8x7Agc(uint8_t agc, TargetVfo) {
  printKeypadCommand("ENTER -> AGC {}", ft8x7AgcName((YaesuAgc)agc));
  prepareKeypadRadioWrite();
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(ft8x7AgcSet((YaesuAgc)agc, state), "AGC", state);
}

static constexpr ChoiceSpec kFt8x7AgcChoice = {
    "AGC SELECT", kFt8x7AgcKeys, sizeof(kFt8x7AgcKeys) / sizeof(kFt8x7AgcKeys[0]),
    nullptr, sayPickedFt8x7Agc, commitFt8x7Agc};

void beginBank2Ft8x7AgcSelect() {
  if (!currentProfile().caps.setRadioMenu) {
    printKeypadStatus("AGC -> unavailable");
    speakKeypadFailure("AGC", KeypadFailure::NotAvailable);
    return;
  }
  printKeypadAction("AGC");
  keypadBeginChoiceSelect(kFt8x7AgcChoice);
  printKeypadStatus("AGC PLEASE");
  speakPrompt("agc");
}

void toggleBank2Nr() {
  printKeypadAction("NR");
  prepareKeypadSpeechResponse();
  NrState state;
  if (keypadReportFeatureFailure(nrToggle(state), "NR")) return;
  printKeypadStatus("{}", nrStateText(state).c_str());
  speakNrState(state);
}

void toggleBank2Nb() {
  printKeypadAction("NB");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(nbToggle(on), "NB")) return;
  printKeypadStatus("{}", nbStateText(on).c_str());
  speakNbState(on);
}

void toggleBank2Notch() {
  printKeypadAction("NOTCH");
  prepareKeypadSpeechResponse();
  NotchState state;
  if (keypadReportFeatureFailure(notchToggle(state), "NOTCH")) return;
  printKeypadStatus("{}", notchStateText(state).c_str());
  speakNotchState(state);
}

void queryBank2NrLevel() {
  printKeypadAction("NRLEVEL?");
  uint16_t raw = 0;
  if (!queryNrLevel(raw, 800)) {
    keypadReportFailure("NRLEVEL?");
    return;
  }
  uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus("NRLEVEL {}%", percent);
  speakFeatureValue("noisereduction", percent);
}

void adjustBank2NrLevel(int deltaPercent) {
  printKeypadAction("NRLEVEL");
  uint16_t raw = 0;
  if (!queryNrLevel(raw, 800)) { keypadReportFailure("NRLEVEL"); return; }
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
    keypadReportFailure("NRLEVEL");
    return;
  }

  const uint8_t readPercent = levelRawToPercent(readBack);
  printKeypadStatus("NRLEVEL {}%", readPercent);
  if (!wrote && (bool)Serial) {
    Serial.println("WARN NRLEVEL write not confirmed; using readback value");
  }
  speakFeatureValue("noisereduction", readPercent);
}

void queryBank2NbLevel() {
  printKeypadAction("NBLEVEL?");
  uint16_t raw = 0;
  if (!queryNbLevel(raw, 800)) {
    keypadReportFailure("NBLEVEL?");
    return;
  }
  uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus("NBLEVEL {}%", percent);
  speakTokenPercent("noiseblanker", percent);
}

void adjustBank2NbLevel(int deltaPercent) {
  printKeypadAction("NBLEVEL");
  uint16_t raw = 0;
  if (!queryNbLevel(raw, 800)) { keypadReportFailure("NBLEVEL"); return; }
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  if (!setNbLevel(levelPercentToRaw(percent))) { keypadReportFailure("NBLEVEL"); return; }
  printKeypadStatus("NBLEVEL {}%", percent);
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
    keypadReportFailure("PBT1?");
    return;
  }
  printKeypadStatus("PBT1 {} step", pbtRawToOffset(raw));
  speakSignedStepValue("pbt", pbtRawToOffset(raw));
}

void adjustBank2PbtInner(int delta) {
  printKeypadAction("PBT1");
  uint16_t raw = 0;
  if (!queryPbtInner(raw, 800)) { keypadReportFailure("PBT1"); return; }
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtInner(pbtOffsetToRaw(next))) { keypadReportFailure("PBT1"); return; }
  queryBank2PbtInner();
}

void queryBank2PbtOuter() {
  printKeypadAction("PBT2?");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) { keypadReportFailure("PBT2?"); return; }
  printKeypadStatus("PBT2 {} step", pbtRawToOffset(raw));
  speakSignedStepValue("pbt", pbtRawToOffset(raw));
}

void adjustBank2PbtOuter(int delta) {
  printKeypadAction("PBT2");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) { keypadReportFailure("PBT2"); return; }
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtOuter(pbtOffsetToRaw(next))) { keypadReportFailure("PBT2"); return; }
  queryBank2PbtOuter();
}

void toggleBank2FilterShape() {
  printKeypadAction("FILSHAPE");
  bool soft = false;
  if (!queryFilterShape(soft, 800)) { keypadReportFailure("FILSHAPE"); return; }
  if (!setFilterShape(!soft)) { keypadReportFailure("FILSHAPE"); return; }
  printKeypadStatus("FILSHAPE {}", !soft ? "SOFT" : "SHARP");
  if (g_speechEnabled) {
    speakLabel("filtershape");
    speakToken(!soft ? "soft" : "sharp");
  }
}

void queryBank2FilterWidth() {
  printKeypadAction("FILWIDTH?");
  uint8_t filter = 0xFF;
  if (!queryCurrentFilterSlotForKeypad(filter)) { keypadReportFailure("FILWIDTH?"); return; }
  printKeypadStatus("FILWIDTH {}", filter);
  if (g_speechEnabled) {
    speakLabel("filterwidth");
    playDigit(filter);
  }
}

void cycleBank2FilterWidth(int delta) {
  printKeypadAction("FILWIDTH");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!ensureActiveVfoKnownForKeypad()) { keypadReportFailure("FILWIDTH"); return; }
  if (!queryVfoMode(live.activeVfoA, mode, filter, 800)) { keypadReportFailure("FILWIDTH"); return; }
  int next = (int)filter + delta;
  if (next < 1) next = 3;
  if (next > 3) next = 1;
  if (!setMode(mode, (uint8_t)next)) { keypadReportFailure("FILWIDTH"); return; }
  printKeypadStatus("FILWIDTH {}", next);
  if (g_speechEnabled) {
    speakLabel("filterwidth");
    playDigit((uint8_t)next);
  }
}
