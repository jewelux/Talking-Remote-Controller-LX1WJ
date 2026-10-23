// The Bank 2 receive settings the keypad steps and enters, one SettingOps each:
// the NR and NB levels, PBT, the FT-857/897 DSP filter, the Icom filter and
// notch, the preamp and the attenuator.
#include "keypad_setting_ops.h"

#include "cat_setting_codec.h"
#include "radio_catalog.h"
#include "radio_features_internal.h"
#include "radio_globals.h"
#include "radio_protocol.h"
#include "ui_features.h"
#include "ui_speech.h"

namespace {

bool isCiv() { return currentProtocolType() == PROTO_CIV; }
bool isYaesuAscii() { return currentProtocolType() == PROTO_YAESU_FTDX_ASCII; }
bool always() { return true; }

// ---- FT-857/897 DSP menus, kept in the EEPROM ----
//
// Read on any FT-857/897; set only experimentally, so B and C open the entry, staged for Enter.

Ft8x7SettingState ft8x7State(Ft8x7Setting setting, int32_t value) {
  Ft8x7SettingState state;
  state.setting = setting;
  state.value = (uint16_t)value;
  return state;
}

YaesuFt857Level ft857Level(Ft8x7Setting setting) {
  switch (setting) {
    case Ft8x7Setting::NrLevel: return YaesuFt857Level::NrLevel;
    case Ft8x7Setting::NbLevel: return YaesuFt857Level::NbLevel;
    case Ft8x7Setting::BpfWidth: return YaesuFt857Level::BpfWidth;
    case Ft8x7Setting::LowCut: return YaesuFt857Level::HpfCutoff;
    default: return YaesuFt857Level::LpfCutoff;
  }
}

// What lets ft8x7LevelSet write it: caps.setNrLevel, caps.setNbLevel, else caps.setRadioMenu.
bool ft857LevelWritable(Ft8x7Setting setting) {
  const RadioCapabilities& caps = currentProfile().caps;
  if (setting == Ft8x7Setting::NrLevel) return caps.setNrLevel;
  if (setting == Ft8x7Setting::NbLevel) return caps.setNbLevel;
  return caps.setRadioMenu;
}

template <Ft8x7Setting S>
int32_t stepFt857(int32_t value, int8_t dir) {
  return ft857LevelStep(ft857Level(S), (uint16_t)value, dir);
}

template <Ft8x7Setting S>
FeatureStatus ft857Limits(ValueSpec& spec, Setting setting) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!currentIsFt857Family() || !ft857LevelWritable(S) || !ft8x7LevelRange(S, min, max)) {
    return FeatureStatus::Unsupported;
  }
  spec.name = settingInfo(setting).label;
  spec.setting = setting;
  spec.min = min;
  spec.max = max;
  spec.maxDigits = max >= 1000 ? 4 : 3;
  if (S == Ft8x7Setting::NbLevel) {
    spec.step = 5;  // 0..100
  } else {
    spec.stepFn = stepFt857<S>;
  }
  // Three widths: a big step is one too.
  spec.bigSteps = S == Ft8x7Setting::BpfWidth ? 1 : 5;
  spec.unit = S == Ft8x7Setting::NrLevel || S == Ft8x7Setting::NbLevel ? "" : " Hz";
  return FeatureStatus::Ok;
}

template <Ft8x7Setting S>
FeatureStatus ft857Read(int32_t& value) {
  if (!currentIsFt857Family()) return FeatureStatus::Unsupported;
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7SettingQuery(S, state);
  value = state.value;
  return status;
}

template <Ft8x7Setting S>
FeatureStatus ft857Write(int32_t value, int32_t& readBack) {
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7LevelSet(S, (uint16_t)value, state);
  readBack = state.value;
  return status;
}

template <Ft8x7Setting S>
String ft857Text(int32_t value) {
  return ft8x7SettingText(ft8x7State(S, value));
}

template <Ft8x7Setting S>
void ft857Speak(int32_t value) {
  speakFt8x7Setting(ft8x7State(S, value));
}

// ---- NR and NB levels ----
//
// Icom in percent, the Yaesu ASCII radios in their RL0 and NL0 steps, the FT-857/897 its
// menus 49 and 63.

template <RxLevel L>
bool rxLevelAvailable() {
  if (currentIsFt857Family()) return true;
  const RadioCapabilities& caps = currentProfile().caps;
  if (!isCiv() && !isYaesuAscii()) return false;
  return L == RxLevel::Nr ? caps.getNrLevel : caps.getNbLevel;
}

template <RxLevel L>
FeatureStatus rxLevelLimits(ValueSpec& spec) {
  constexpr Setting kSetting = L == RxLevel::Nr ? Setting::NrLevel : Setting::NbLevel;
  constexpr Ft8x7Setting kFt857 = L == RxLevel::Nr ? Ft8x7Setting::NrLevel : Ft8x7Setting::NbLevel;
  if (currentIsFt857Family()) return ft857Limits<kFt857>(spec, kSetting);
  const RadioCapabilities& caps = currentProfile().caps;
  if (!rxLevelAvailable<L>() || !(L == RxLevel::Nr ? caps.setNrLevel : caps.setNbLevel)) {
    return FeatureStatus::Unsupported;
  }
  spec.name = settingInfo(kSetting).label;
  spec.setting = kSetting;
  if (isCiv()) {
    spec.min = 0;
    spec.max = 100;
    spec.step = 5;
    spec.maxDigits = 3;
    spec.unit = " %";
    return FeatureStatus::Ok;
  }
  uint16_t min = 0;
  uint16_t max = 0;
  if (!yaesuRxLevelRange(L, min, max)) return FeatureStatus::Unsupported;
  spec.min = min;
  spec.max = max;
  spec.maxDigits = 2;
  return FeatureStatus::Ok;
}

bool civQueryRxLevelPercent(RxLevel level, int32_t& percent) {
  uint16_t raw = 0;
  const bool ok = level == RxLevel::Nr ? queryNrLevel(raw, kCatReplyTimeoutMs) : queryNbLevel(raw, kCatReplyTimeoutMs);
  if (ok) percent = levelRawToPercent(raw);
  return ok;
}

template <RxLevel L>
FeatureStatus rxLevelRead(int32_t& value) {
  if (currentIsFt857Family()) {
    return ft857Read<L == RxLevel::Nr ? Ft8x7Setting::NrLevel : Ft8x7Setting::NbLevel>(value);
  }
  if (!rxLevelAvailable<L>()) return FeatureStatus::Unsupported;
  if (isCiv()) return civQueryRxLevelPercent(L, value) ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
  uint16_t level = 0;
  const FeatureStatus status = yaesuRxLevelQuery(L, level);
  value = level;
  return status;
}

// An Icom with NR off may refuse its level: NR is switched on and the level sent again.
bool civSetNrLevelPercent(int32_t percent) {
  const uint16_t raw = levelPercentToRaw(percent);
  if (setNrLevel(raw)) return true;
  bool on = false;
  if (!queryNr(on, kCatReplyTimeoutMs) || on) return false;
  return setNr(true) && setNrLevel(raw);
}

template <RxLevel L>
FeatureStatus rxLevelWrite(int32_t value, int32_t& readBack) {
  if (currentIsFt857Family()) {
    return ft857Write<L == RxLevel::Nr ? Ft8x7Setting::NrLevel : Ft8x7Setting::NbLevel>(value, readBack);
  }
  if (isCiv()) {
    const bool wrote = L == RxLevel::Nr ? civSetNrLevelPercent(value) : setNbLevel(levelPercentToRaw(value));
    if (!wrote) return failure(FeatureStatus::Failed);
    if (!civQueryRxLevelPercent(L, readBack)) readBack = value;
    return FeatureStatus::Ok;
  }
  uint16_t level = 0;
  const FeatureStatus status = yaesuRxLevelSet(L, (uint16_t)value, level);
  readBack = level;
  return status;
}

template <RxLevel L>
String rxLevelTextOf(int32_t value) {
  if (currentIsFt857Family()) {
    return ft857Text<L == RxLevel::Nr ? Ft8x7Setting::NrLevel : Ft8x7Setting::NbLevel>(value);
  }
  return String(L == RxLevel::Nr ? "NRLEVEL " : "NBLEVEL ") + String(value) + (isCiv() ? "%" : "");
}

// "noise reduction level 8", on every radio.
template <RxLevel L>
void speakRxLevelOf(int32_t value) {
  if (!g_speechEnabled) return;
  speakLabel(L == RxLevel::Nr ? "noisereduction level" : "noiseblanker level");
  speakDigitsAndPoint(String(value));
}

bool rxLevelConfirm() { return currentIsFt857Family(); }

// ---- FT-857/897 low cut, high cut and DSP bandpass width ----

bool ft857Available() { return currentIsFt857Family(); }

FeatureStatus lowCutLimits(ValueSpec& spec) { return ft857Limits<Ft8x7Setting::LowCut>(spec, Setting::RxLowCut); }
FeatureStatus highCutLimits(ValueSpec& spec) { return ft857Limits<Ft8x7Setting::HighCut>(spec, Setting::RxHighCut); }
FeatureStatus bpfWidthLimits(ValueSpec& spec) { return ft857Limits<Ft8x7Setting::BpfWidth>(spec, Setting::BpfWidth); }

// ---- Icom passband tuning: -128..127 steps of the radio's level, 0 the centre ----

template <bool Inner>
bool pbtAvailable() {
  return isCiv() && (Inner ? currentProfile().caps.getPbtInner : currentProfile().caps.getPbtOuter);
}

template <bool Inner>
FeatureStatus pbtLimits(ValueSpec& spec) {
  if (!pbtAvailable<Inner>() || !(Inner ? currentProfile().caps.setPbtInner : currentProfile().caps.setPbtOuter)) {
    return FeatureStatus::Unsupported;
  }
  spec.name = Inner ? "PBT1" : "PBT2";
  spec.setting = Inner ? Setting::PbtInner : Setting::PbtOuter;
  spec.min = -128;
  spec.max = 127;
  spec.sign = true;
  spec.maxDigits = 3;
  spec.step = 10;
  return FeatureStatus::Ok;
}

template <bool Inner>
FeatureStatus pbtRead(int32_t& value) {
  if (!pbtAvailable<Inner>()) return FeatureStatus::Unsupported;
  uint16_t raw = 0;
  const bool ok = Inner ? queryPbtInner(raw, kCatReplyTimeoutMs) : queryPbtOuter(raw, kCatReplyTimeoutMs);
  if (!ok) return failure(FeatureStatus::NoReply);
  value = (int32_t)(raw > 255 ? 255 : raw) - 128;
  return FeatureStatus::Ok;
}

template <bool Inner>
FeatureStatus pbtWrite(int32_t value, int32_t& readBack) {
  const uint16_t raw = (uint16_t)(value + 128);
  if (!(Inner ? setPbtInner(raw) : setPbtOuter(raw))) return failure(FeatureStatus::Failed);
  if (pbtRead<Inner>(readBack) != FeatureStatus::Ok) readBack = value;
  return FeatureStatus::Ok;
}

template <bool Inner>
String pbtText(int32_t value) {
  return String(Inner ? "PBT1 " : "PBT2 ") + String(value);
}

// "pbt one minus 20".
template <bool Inner>
void speakPbt(int32_t value) {
  if (!g_speechEnabled) return;
  speakLabel(Inner ? "pbt one" : "pbt two");
  if (value < 0) {
    speakToken("minus");
    playSilenceMs(60);
  }
  speakDigitsAndPoint(String(value < 0 ? -value : value));
}

// ---- Icom filter 1..3 of the current mode ----

bool ensureActiveVfoKnown() {
  if (live.activeVfoKnown) return true;
  uint64_t hz = 0;
  return queryVfoFrequency(true, hz, kCatReplyTimeoutMs);
}

bool filterSlotAvailable() { return isCiv() && currentProfile().caps.getFilterWidth; }

FeatureStatus filterSlotLimits(ValueSpec& spec) {
  if (!filterSlotAvailable() || !currentProfile().caps.setFilterWidth) return FeatureStatus::Unsupported;
  spec.name = "FILWIDTH";
  spec.setting = Setting::FilterSlot;
  spec.min = 1;
  spec.max = 3;
  spec.maxDigits = 1;
  spec.bigSteps = 1;
  return FeatureStatus::Ok;
}

bool queryModeAndFilter(uint8_t& mode, uint8_t& filter) {
  return ensureActiveVfoKnown() && queryVfoMode(live.activeVfoA, mode, filter, kCatReplyTimeoutMs);
}

FeatureStatus filterSlotRead(int32_t& value) {
  if (!filterSlotAvailable()) return FeatureStatus::Unsupported;
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryModeAndFilter(mode, filter)) return failure(FeatureStatus::NoReply);
  value = filter;
  return FeatureStatus::Ok;
}

// The filter is set with the mode, which is read first and kept.
FeatureStatus filterSlotWrite(int32_t value, int32_t& readBack) {
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryModeAndFilter(mode, filter)) return failure(FeatureStatus::NoReply);
  if (!setMode(mode, (uint8_t)value)) return failure(FeatureStatus::Failed);
  if (filterSlotRead(readBack) != FeatureStatus::Ok) readBack = value;
  return FeatureStatus::Ok;
}

String filterSlotText(int32_t value) { return "FILWIDTH " + String(value); }

void speakFilterSlot(int32_t value) {
  if (!g_speechEnabled) return;
  speakLabel("filterwidth");
  playDigit((int)value);
}

// ---- Icom notch: 0 off, 1 narrow, 2 mid, 3 wide ----

bool notchAvailable() { return isCiv() && currentProfile().caps.getNotchWidth; }

FeatureStatus notchLimits(ValueSpec& spec) {
  const RadioCapabilities& caps = currentProfile().caps;
  if (!notchAvailable() || !caps.setNotch || !caps.setNotchWidth) return FeatureStatus::Unsupported;
  spec.name = "NOTCH";
  spec.setting = Setting::Notch;
  spec.min = 0;
  spec.max = 3;
  spec.maxDigits = 1;
  spec.bigSteps = 1;
  return FeatureStatus::Ok;
}

NotchState notchStateOf(int32_t value) {
  NotchState state;
  state.on = value > 0;
  if (state.on) state.width = (NotchWidth)(value - 1);
  return state;
}

FeatureStatus notchRead(int32_t& value) {
  if (!notchAvailable()) return FeatureStatus::Unsupported;
  NotchState state;
  const FeatureStatus status = notchQuery(state);
  if (status != FeatureStatus::Ok) return status;
  value = !state.on ? 0 : state.width == NOTCH_WIDTH_UNKNOWN ? 1 : (int32_t)state.width + 1;
  return FeatureStatus::Ok;
}

FeatureStatus notchWrite(int32_t value, int32_t& readBack) {
  const FeatureStatus status = value == 0 ? notchSet(false) : notchSetWidth((NotchWidth)(value - 1));
  if (status != FeatureStatus::Ok) return status;
  if (notchRead(readBack) != FeatureStatus::Ok) readBack = value;
  return FeatureStatus::Ok;
}

String notchText(int32_t value) { return notchStateText(notchStateOf(value)); }
void speakNotch(int32_t value) { speakNotchState(notchStateOf(value)); }

// ---- Preamp: 0 off, then its levels ----

bool preampAvailable() { return currentProfile().caps.getPreamp && preampLevels() > 0; }

FeatureStatus preampLimits(ValueSpec& spec) {
  if (!preampAvailable() || !currentProfile().caps.setPreamp) return FeatureStatus::Unsupported;
  spec.name = "PREAMP";
  spec.setting = Setting::Preamp;
  spec.min = 0;
  spec.max = preampLevels();
  spec.maxDigits = 1;
  spec.bigSteps = 1;
  return FeatureStatus::Ok;
}

FeatureStatus preampRead(int32_t& value) {
  uint8_t level = 0;
  const FeatureStatus status = preampQuery(level);
  value = level;
  return status;
}

FeatureStatus preampWrite(int32_t value, int32_t& readBack) {
  uint8_t level = 0;
  const FeatureStatus status = preampSet((uint8_t)value, level);
  readBack = level;
  return status;
}

String preampTextOf(int32_t value) { return preampText((uint8_t)value); }
void speakPreampOf(int32_t value) { speakPreamp((uint8_t)value); }

// ---- Attenuator: 0 off, then its steps ----

bool attAvailable() { return currentProfile().caps.getAtt; }

FeatureStatus attLimits(ValueSpec& spec) {
  if (!attAvailable() || attSteps() == 0) return FeatureStatus::Unsupported;
  spec.name = "ATT";
  spec.setting = Setting::Att;
  spec.min = 0;
  spec.max = attSteps();
  spec.maxDigits = 1;
  spec.bigSteps = 1;
  return FeatureStatus::Ok;
}

AttState attStateOf(int32_t step) {
  AttState state;
  state.on = step > 0;
  if (state.on) state.db = attStepDb((uint8_t)step);
  return state;
}

// The step of the dB read; step 1 where the radio does not say.
int32_t attStepOf(const AttState& state) {
  if (!state.on) return 0;
  for (uint8_t step = 1; step <= attSteps(); ++step) {
    if (attStepDb(step) == state.db) return step;
  }
  return 1;
}

FeatureStatus attRead(int32_t& value) {
  AttState state;
  const FeatureStatus status = attQuery(state);
  value = attStepOf(state);
  return status;
}

FeatureStatus attWrite(int32_t value, int32_t& readBack) {
  AttState state;
  const FeatureStatus status = attSet((uint8_t)value, state);
  readBack = attStepOf(state);
  return status;
}

String attTextOf(int32_t value) { return attText(attStateOf(value)); }
void speakAttOf(int32_t value) { speakAtt(attStateOf(value)); }

constexpr SettingOps kRxOps[] = {
    {Setting::NrLevel, "noisereduction level", rxLevelAvailable<RxLevel::Nr>, rxLevelLimits<RxLevel::Nr>,
     rxLevelRead<RxLevel::Nr>, rxLevelWrite<RxLevel::Nr>, rxLevelTextOf<RxLevel::Nr>, speakRxLevelOf<RxLevel::Nr>,
     rxLevelConfirm},
    {Setting::NbLevel, "noiseblanker level", rxLevelAvailable<RxLevel::Nb>, rxLevelLimits<RxLevel::Nb>,
     rxLevelRead<RxLevel::Nb>, rxLevelWrite<RxLevel::Nb>, rxLevelTextOf<RxLevel::Nb>, speakRxLevelOf<RxLevel::Nb>,
     rxLevelConfirm},
    {Setting::RxLowCut, "lowcut", ft857Available, lowCutLimits, ft857Read<Ft8x7Setting::LowCut>,
     ft857Write<Ft8x7Setting::LowCut>, ft857Text<Ft8x7Setting::LowCut>, ft857Speak<Ft8x7Setting::LowCut>, always},
    {Setting::RxHighCut, "highcut", ft857Available, highCutLimits, ft857Read<Ft8x7Setting::HighCut>,
     ft857Write<Ft8x7Setting::HighCut>, ft857Text<Ft8x7Setting::HighCut>, ft857Speak<Ft8x7Setting::HighCut>, always},
    {Setting::BpfWidth, "cw filterwidth", ft857Available, bpfWidthLimits, ft857Read<Ft8x7Setting::BpfWidth>,
     ft857Write<Ft8x7Setting::BpfWidth>, ft857Text<Ft8x7Setting::BpfWidth>, ft857Speak<Ft8x7Setting::BpfWidth>,
     always},
    {Setting::PbtInner, "pbt one", pbtAvailable<true>, pbtLimits<true>, pbtRead<true>, pbtWrite<true>,
     pbtText<true>, speakPbt<true>, nullptr, 0},
    {Setting::PbtOuter, "pbt two", pbtAvailable<false>, pbtLimits<false>, pbtRead<false>, pbtWrite<false>,
     pbtText<false>, speakPbt<false>, nullptr, 0},
    {Setting::FilterSlot, "filterwidth", filterSlotAvailable, filterSlotLimits, filterSlotRead, filterSlotWrite,
     filterSlotText, speakFilterSlot},
    {Setting::Notch, "notch", notchAvailable, notchLimits, notchRead, notchWrite, notchText, speakNotch},
    {Setting::Preamp, "preamplifier", preampAvailable, preampLimits, preampRead, preampWrite, preampTextOf,
     speakPreampOf},
    {Setting::Att, "attenuator", attAvailable, attLimits, attRead, attWrite, attTextOf, speakAttOf},
};

}  // namespace

const SettingOps* rxSettingOps(Setting setting) {
  for (const SettingOps& ops : kRxOps) {
    if (ops.setting == setting) return &ops;
  }
  return nullptr;
}
