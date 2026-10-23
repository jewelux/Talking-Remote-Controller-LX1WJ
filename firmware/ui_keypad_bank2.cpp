// Bank 2 keypad actions: the NR, NB and notch switches, AGC, preamp and
// attenuator, the Icom filter shape, and the FT-8x7 settings kept in the EEPROM,
// among them the FT-817 antenna jack. The levels, PBT and filters are Settings
// (keypad_setting_ops_rx.cpp).
#include "ui_keypad_bank.h"
#include "ui_features.h"
#include "radio_monitor.h"

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

void queryBank2Ft8x7Dbf() { queryKeypadFt8x7Setting(Ft8x7Setting::Dbf); }
void queryBank2Ft8x7Ipo() { queryKeypadFt8x7Setting(Ft8x7Setting::Ipo); }
void queryBank2Ft817Antenna() { queryKeypadFt8x7Setting(Ft8x7Setting::Antenna); }

void toggleBank2Ft8x7Dbf() {
  printKeypadAction("DBF");
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(ft8x7FlagSettingToggle(Ft8x7Setting::Dbf, state), "DBF", state);
}

void toggleBank2Ft817Antenna() {
  printKeypadAction("ANT");
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(ft817AntennaToggle(state), "ANT", state);
}

void queryBank2Agc() {
  printKeypadAction("AGC?");
  prepareKeypadSpeechResponse();
  AgcMode mode = AgcMode::Off;
  if (keypadReportFeatureFailure(agcQuery(mode), "AGC?")) return;
  printKeypadStatus("{}", agcText(mode).c_str());
  speakAgc(mode);
}

void queryBank2AutoNotch() {
  printKeypadAction("ANOTCH?");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(autoNotchQuery(on), "ANOTCH?")) return;
  printKeypadStatus("{}", autoNotchText(on).c_str());
  speakAutoNotch(on);
}

void toggleBank2AutoNotch() {
  printKeypadAction("ANOTCH");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(autoNotchToggle(on), "ANOTCH")) return;
  printKeypadStatus("{}", autoNotchText(on).c_str());
  speakAutoNotch(on);
}

void queryBank2Att() {
  printKeypadAction("ATT?");
  prepareKeypadSpeechResponse();
  AttState state;
  if (keypadReportFeatureFailure(attQuery(state), "ATT?")) return;
  printKeypadStatus("{}", attText(state).c_str());
  speakAtt(state);
}

void stepBank2Att() {
  printKeypadAction("ATT");
  prepareKeypadSpeechResponse();
  AttState state;
  if (keypadReportFeatureFailure(attStep(state), "ATT")) return;
  printKeypadStatus("{}", attText(state).c_str());
  speakAtt(state);
}

void queryBank2Preamp() {
  printKeypadAction("PREAMP?");
  prepareKeypadSpeechResponse();
  uint8_t level = 0;
  if (keypadReportFeatureFailure(preampQuery(level), "PREAMP?")) return;
  printKeypadStatus("{}", preampText(level).c_str());
  speakPreamp(level);
}

void stepBank2Preamp() {
  printKeypadAction("PREAMP");
  prepareKeypadSpeechResponse();
  uint8_t level = 0;
  if (keypadReportFeatureFailure(preampStep(level), "PREAMP")) return;
  printKeypadStatus("{}", preampText(level).c_str());
  speakPreamp(level);
}

// 0 off, 1 auto, 2 fast, 3 slow, 4 mid; a key for a speed the radio cannot be set to is rejected.
static bool pickAgc(char key, uint8_t& value) {
  static constexpr AgcMode kKeys[] = {AgcMode::Off, AgcMode::Auto, AgcMode::Fast, AgcMode::Slow, AgcMode::Mid};
  if (key < '0' || key > '4') return false;
  const AgcMode mode = kKeys[key - '0'];
  if (!agcCanSet(mode)) return false;
  value = (uint8_t)mode;
  return true;
}

static void sayPickedAgc(char key, uint8_t value) {
  printKeypadCommand("AGC SELECT -> {}", key);
  printKeypadStatus("AGC STAGE: {}", agcModeName((AgcMode)value));
  if (g_speechEnabled) speakToken(agcModeName((AgcMode)value));
}

static void commitAgc(uint8_t value, TargetVfo) {
  printKeypadCommand("ENTER -> AGC {}", agcModeName((AgcMode)value));
  prepareKeypadRadioWrite();
  AgcMode mode = AgcMode::Off;
  if (keypadReportFeatureFailure(agcSet((AgcMode)value, mode), "AGC")) return;
  printKeypadStatus("{}", agcText(mode).c_str());
  speakAgc(mode);
}

static constexpr ChoiceSpec kAgcChoice = {"AGC SELECT", nullptr, 0, pickAgc, sayPickedAgc, commitAgc, true};

static bool agcCanSetAny() {
  for (const AgcMode mode : {AgcMode::Off, AgcMode::Fast, AgcMode::Mid, AgcMode::Slow, AgcMode::Auto}) {
    if (agcCanSet(mode)) return true;
  }
  return false;
}

void beginBank2AgcSelect() {
  if (!agcCanSetAny()) {
    printKeypadStatus("AGC -> unavailable");
    speakKeypadFailure("AGC", KeypadFailure::NotAvailable);
    return;
  }
  printKeypadAction("AGC");
  keypadBeginChoiceSelect(kAgcChoice);
  printKeypadStatus("AGC PLEASE");
  speakPrompt("agc");
}

// The speed after mode in the select's key order (off, auto, fast, slow, mid) that the radio
// can be set to, going round; false when there is no other.
static bool nextAgc(AgcMode mode, int8_t dir, AgcMode& out) {
  static constexpr AgcMode kOrder[] = {AgcMode::Off, AgcMode::Auto, AgcMode::Fast, AgcMode::Slow, AgcMode::Mid};
  constexpr int kCount = sizeof(kOrder) / sizeof(kOrder[0]);
  int at = 0;
  while (at < kCount && kOrder[at] != mode) ++at;
  if (at == kCount) at = dir > 0 ? kCount - 1 : 0;
  for (int n = 1; n < kCount; ++n) {
    const AgcMode next = kOrder[((at + dir * n) % kCount + kCount) % kCount];
    if (agcCanSet(next)) {
      out = next;
      return next != mode;
    }
  }
  return false;
}

void nudgeBank2Agc(int8_t dir, bool) {
  printKeypadAction("AGC {}", dir > 0 ? "UP" : "DOWN");
  if (keypadReportIfUnsupported(agcCanSetAny(), "AGC")) return;
  prepareKeypadSpeechResponse();
  AgcMode mode = AgcMode::Off;
  if (keypadReportFeatureFailure(agcQuery(mode), "AGC")) return;
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    EntrySeed seed;
    seed.known = true;
    seed.value = (uint8_t)mode;
    if (!keypadNudgeChoice(kAgcChoice, seed, dir)) {
      keypadValueLimit("AGC", dir > 0);
      return;
    }
    speakPlease();
    return;
  }
  AgcMode next = mode;
  if (!nextAgc(mode, dir, next)) {
    keypadValueLimit("AGC", dir > 0);
    return;
  }
  prepareKeypadRadioWrite();
  if (keypadReportFeatureFailure(agcSet(next, mode), "AGC")) return;
  printKeypadStatus("{}", agcText(mode).c_str());
  LabelModeScope labels(keypadActiveFocusIsNew() ? LabelMode::Always : LabelMode::Never);
  speakAgc(mode);
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

void toggleBank2FilterShape() {
  printKeypadAction("FILSHAPE");
  bool soft = false;
  if (!queryFilterShape(soft, kCatReplyTimeoutMs)) { keypadReportFailure("FILSHAPE"); return; }
  if (!setFilterShape(!soft)) { keypadReportFailure("FILSHAPE"); return; }
  printKeypadStatus("FILSHAPE {}", !soft ? "SOFT" : "SHARP");
  if (g_speechEnabled) {
    speakLabel("filtershape");
    speakToken(!soft ? "soft" : "sharp");
  }
}
