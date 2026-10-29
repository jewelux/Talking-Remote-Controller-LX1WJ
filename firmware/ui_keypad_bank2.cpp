// Bank 2 keypad actions: the NR, NB and notch switches, AGC, preamp and
// attenuator, the Icom filter shape, and the FT-8x7 settings kept in the EEPROM,
// among them the FT-817 antenna jack. The levels, PBT and filters are Settings
// (keypad_setting_ops_rx.cpp).
#include "ui_keypad_bank.h"
#include "ui_features.h"
#include "radio_monitor.h"

void queryBank2Nr() {
  printKeypadAction("NR?");
  KeypadAnswer answer("NR?");
  NrState state;
  if (answer.failed(nrQuery(state))) return;
  printKeypadStatus("{}", nrStateText(state).c_str());
  speakNrState(state);
}

void queryBank2Nb() {
  printKeypadAction("NB?");
  KeypadAnswer answer("NB?");
  bool on = false;
  if (answer.failed(nbQuery(on))) return;
  printKeypadStatus("{}", nbStateText(on).c_str());
  speakNbState(on);
}

void queryBank2Notch() {
  printKeypadAction("NOTCH?");
  KeypadAnswer answer("NOTCH?");
  NotchState state;
  if (answer.failed(notchQuery(state))) return;
  printKeypadStatus("{}", notchStateText(state).c_str());
  speakNotchState(state);
}

void queryBank2Ft8x7Dbf() { queryKeypadFt8x7Setting(Ft8x7Setting::Dbf); }
void queryBank2Ft8x7Ipo() { queryKeypadFt8x7Setting(Ft8x7Setting::Ipo); }
void queryBank2Ft817Antenna() { queryKeypadFt8x7Setting(Ft8x7Setting::Antenna); }

void toggleBank2Ft8x7Dbf() {
  printKeypadAction("DBF");
  KeypadAnswer answer("DBF");
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(answer, ft8x7FlagSettingToggle(Ft8x7Setting::Dbf, state), state);
}

void toggleBank2Ft817Antenna() {
  printKeypadAction("ANT");
  KeypadAnswer answer("ANT");
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(answer, ft817AntennaToggle(state), state);
}

void queryBank2Agc() {
  printKeypadAction("AGC?");
  KeypadAnswer answer("AGC?");
  AgcMode mode = AgcMode::Off;
  if (answer.failed(agcQuery(mode))) return;
  printKeypadStatus("{}", agcText(mode).c_str());
  speakAgc(mode);
}

void queryBank2AutoNotch() {
  printKeypadAction("ANOTCH?");
  KeypadAnswer answer("ANOTCH?");
  bool on = false;
  if (answer.failed(autoNotchQuery(on))) return;
  printKeypadStatus("{}", autoNotchText(on).c_str());
  speakAutoNotch(on);
}

void toggleBank2AutoNotch() {
  printKeypadAction("ANOTCH");
  KeypadAnswer answer("ANOTCH");
  bool on = false;
  if (answer.failed(autoNotchToggle(on))) return;
  printKeypadStatus("{}", autoNotchText(on).c_str());
  speakAutoNotch(on);
}

void queryBank2Att() {
  printKeypadAction("ATT?");
  KeypadAnswer answer("ATT?");
  AttState state;
  if (answer.failed(attQuery(state))) return;
  printKeypadStatus("{}", attText(state).c_str());
  speakAtt(state);
}

void stepBank2Att() {
  printKeypadAction("ATT");
  KeypadAnswer answer("ATT");
  AttState state;
  if (answer.failed(attStep(state))) return;
  printKeypadStatus("{}", attText(state).c_str());
  speakAtt(state);
}

void queryBank2Preamp() {
  printKeypadAction("PREAMP?");
  KeypadAnswer answer("PREAMP?");
  uint8_t level = 0;
  if (answer.failed(preampQuery(level))) return;
  printKeypadStatus("{}", preampText(level).c_str());
  speakPreamp(level);
}

void stepBank2Preamp() {
  printKeypadAction("PREAMP");
  KeypadAnswer answer("PREAMP");
  uint8_t level = 0;
  if (answer.failed(preampStep(level))) return;
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
  KeypadAnswer answer("AGC", AnswerNaming::OnFailure);
  prepareKeypadRadioWrite();
  AgcMode mode = AgcMode::Off;
  if (answer.failed(agcSet((AgcMode)value, mode))) return;
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
  printKeypadAction("AGC");
  if (!agcCanSetAny()) {
    keypadReportNotAvailable("AGC");
    return;
  }
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
  KeypadAnswer answer("AGC", keypadStepNaming());
  if (answer.unavailable(agcCanSetAny())) return;
  AgcMode mode = AgcMode::Off;
  if (answer.failed(agcQuery(mode))) return;
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
  if (answer.failed(agcSet(next, mode))) return;
  printKeypadStatus("{}", agcText(mode).c_str());
  speakAgc(mode);
}

void toggleBank2Nr() {
  printKeypadAction("NR");
  KeypadAnswer answer("NR");
  NrState state;
  if (answer.failed(nrToggle(state))) return;
  printKeypadStatus("{}", nrStateText(state).c_str());
  speakNrState(state);
}

void toggleBank2Nb() {
  printKeypadAction("NB");
  KeypadAnswer answer("NB");
  bool on = false;
  if (answer.failed(nbToggle(on))) return;
  printKeypadStatus("{}", nbStateText(on).c_str());
  speakNbState(on);
}

void toggleBank2Notch() {
  printKeypadAction("NOTCH");
  KeypadAnswer answer("NOTCH");
  NotchState state;
  if (answer.failed(notchToggle(state))) return;
  printKeypadStatus("{}", notchStateText(state).c_str());
  speakNotchState(state);
}

// The FT-857/897 DSP notch is an auto notch: the notch operations, spoken as the auto notch.
void queryBank2Ft8x7AutoNotch() {
  printKeypadAction("ANOTCH?");
  KeypadAnswer answer("ANOTCH?");
  NotchState state;
  if (answer.failed(notchQuery(state))) return;
  printKeypadStatus("{}", autoNotchText(state.on).c_str());
  speakAutoNotch(state.on);
}

void toggleBank2Ft8x7AutoNotch() {
  printKeypadAction("ANOTCH");
  KeypadAnswer answer("ANOTCH");
  NotchState state;
  if (answer.failed(notchToggle(state))) return;
  printKeypadStatus("{}", autoNotchText(state.on).c_str());
  speakAutoNotch(state.on);
}

void toggleBank2FilterShape() {
  printKeypadAction("FILSHAPE");
  KeypadAnswer answer("FILSHAPE");
  bool soft = false;
  if (!queryFilterShape(soft, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  if (!setFilterShape(!soft)) { answer.radioFailed(); return; }
  printKeypadStatus("FILSHAPE {}", !soft ? "SOFT" : "SHARP");
  if (g_speechEnabled) {
    speakLabel("filtershape");
    speakToken(!soft ? "soft" : "sharp");
  }
}
