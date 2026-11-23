// The Bank 2 receive settings the keypad steps and enters, one SettingOps each:
// the NR and NB levels, PBT, the FT-857/897 DSP filter, the Icom filter and
// notch, the receive filter settings (low and high cut, width, narrow, notch
// frequency, contour, APF), the preamp, the attenuator and the RF gain.
#include "keypad_setting_ops.h"

#include "cat_setting_codec.h"
#include "radio_catalog.h"
#include "radio_features_internal.h"
#include "radio_globals.h"
#include "radio_protocol.h"
#include "rx_filter_codec.h"
#include "ui_features.h"
#include "ui_speech.h"

namespace {

bool isYaesuAscii() { return currentProtocolType() == PROTO_YAESU_FTDX_ASCII; }

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

template <Ft8x7Setting S>
int32_t stepFt857(int32_t value, int8_t dir) {
  return ft857LevelStep(ft857Level(S), (uint16_t)value, dir);
}

template <Ft8x7Setting S>
FeatureStatus ft857Limits(ValueSpec& spec) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!currentIsFt857Family() || !ft8x7LevelRange(S, min, max)) return FeatureStatus::Unsupported;
  const FeatureStatus access = ft8x7LevelWriteAccess(S);
  if (access != FeatureStatus::Ok) return access;
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
  constexpr Ft8x7Setting kFt857 = L == RxLevel::Nr ? Ft8x7Setting::NrLevel : Ft8x7Setting::NbLevel;
  if (currentIsFt857Family()) return ft857Limits<kFt857>(spec);
  const RadioCapabilities& caps = currentProfile().caps;
  if (!rxLevelAvailable<L>() || !(L == RxLevel::Nr ? caps.setNrLevel : caps.setNbLevel)) {
    return FeatureStatus::Unsupported;
  }
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

// ---- FT-857/897 DSP bandpass width ----

bool ft857Available() { return currentIsFt857Family(); }

FeatureStatus bpfWidthLimits(ValueSpec& spec) { return ft857Limits<Ft8x7Setting::BpfWidth>(spec); }

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

// ---- Receive filter levels: radio_features rxFilterLevel* ----
//
// The width and the cuts step through the radio's table for the current mode, which limits
// reads; the notch and contour frequencies step 50 Hz, the APF offset 10 Hz. 0 is the width
// NORMAL and a cut OFF, typed as 0.

// The table limits last read for L, which its B and C step through with no radio traffic.
template <RxFilterLevel L>
RxFilterScale& lastScale() {
  static RxFilterScale scale;
  return scale;
}

template <RxFilterLevel L>
int32_t stepOnScale(int32_t value, int8_t dir) {
  uint16_t next = 0;
  if (value < 0 || !rxFilterScaleNext(lastScale<L>(), (uint16_t)value, dir, next)) return value;
  return next;
}

template <RxFilterLevel L>
bool rxFilterAvailable() {
  return rxFilterLevelSupported(L);
}

template <RxFilterLevel L>
FeatureStatus rxFilterLimits(ValueSpec& spec) {
  const FeatureStatus access = rxFilterLevelWriteAccess(L);
  if (access != FeatureStatus::Ok) return access;
  spec.unit = " Hz";
  spec.bigSteps = 5;
  if (rxFilterLevelByMode(L)) {
    RxFilterScale& scale = lastScale<L>();
    const FeatureStatus status = rxFilterLevelScale(L, scale);
    if (status != FeatureStatus::Ok) return status;
    spec.min = scale.hasZero() ? 0 : scale.valueOf(scale.firstValueCode());
    spec.max = scale.valueOf(scale.last);
    spec.stepFn = stepOnScale<L>;
  } else {
    RxFilterRange range;
    const FeatureStatus status = rxFilterLevelRange(L, range);
    if (status != FeatureStatus::Ok) return status;
    spec.min = range.min;
    spec.max = range.max;
    spec.sign = range.min < 0;
    spec.step = L == RxFilterLevel::ApfFreq ? 10 : 50;
  }
  spec.maxDigits = spec.max >= 10000 ? 5 : spec.max >= 1000 ? 4 : 3;
  return FeatureStatus::Ok;
}

template <RxFilterLevel L>
FeatureStatus rxFilterRead(int32_t& value) {
  int16_t now = 0;
  const FeatureStatus status = rxFilterLevelQuery(L, now);
  value = now;
  return status;
}

template <RxFilterLevel L>
FeatureStatus rxFilterWrite(int32_t value, int32_t& readBack) {
  if (value < -32768 || value > 32767) return FeatureStatus::Invalid;
  // The entry takes 0 (off, NORMAL) to the highest value; one typed below the lowest value goes to
  // it, the nearest the radio has, as rxFilterLevelSet does with the others.
  const RxFilterScale& scale = lastScale<L>();
  if (rxFilterLevelByMode(L) && scale.values && value > 0) {
    const int32_t lowest = scale.valueOf(scale.firstValueCode());
    if (value < lowest) value = lowest;
  }
  int16_t set = 0;
  const FeatureStatus status = rxFilterLevelSet(L, (int16_t)value, set);
  readBack = set;
  return status;
}

template <RxFilterLevel L>
String rxFilterText(int32_t value) {
  return rxFilterLevelText(L, (int16_t)value);
}

template <RxFilterLevel L>
void speakRxFilter(int32_t value) {
  speakRxFilterLevel(L, (int16_t)value);
}

// The width NORMAL has no place among the widths to step from.
bool widthStepsFrom(int32_t value) { return value != 0; }

// The low and high cut: the FT-857/897 DSP menus (staged for Enter), else the receive filter cut.
template <Ft8x7Setting S, RxFilterLevel L>
bool cutAvailable() {
  return currentIsFt857Family() || rxFilterAvailable<L>();
}
template <Ft8x7Setting S, RxFilterLevel L>
FeatureStatus cutLimits(ValueSpec& spec) {
  return currentIsFt857Family() ? ft857Limits<S>(spec) : rxFilterLimits<L>(spec);
}
template <Ft8x7Setting S, RxFilterLevel L>
FeatureStatus cutRead(int32_t& value) {
  return currentIsFt857Family() ? ft857Read<S>(value) : rxFilterRead<L>(value);
}
template <Ft8x7Setting S, RxFilterLevel L>
FeatureStatus cutWrite(int32_t value, int32_t& readBack) {
  return currentIsFt857Family() ? ft857Write<S>(value, readBack) : rxFilterWrite<L>(value, readBack);
}
template <Ft8x7Setting S, RxFilterLevel L>
String cutText(int32_t value) {
  return currentIsFt857Family() ? ft857Text<S>(value) : rxFilterText<L>(value);
}
template <Ft8x7Setting S, RxFilterLevel L>
void speakCut(int32_t value) {
  if (currentIsFt857Family()) ft857Speak<S>(value);
  else speakRxFilter<L>(value);
}
bool cutConfirm() { return currentIsFt857Family(); }

// ---- Receive filter switches: narrow, contour, APF; 0 off, 1 on ----

template <RxFilterSwitch W>
bool rxSwitchAvailable() {
  return rxFilterSwitchSupported(W);
}

template <RxFilterSwitch W>
FeatureStatus rxSwitchAccess() {
  return rxFilterSwitchWriteAccess(W);
}

template <RxFilterSwitch W>
FeatureStatus rxSwitchLimits(ValueSpec& spec) {
  return switchSettingLimits(spec, rxSwitchAccess<W>);
}

template <RxFilterSwitch W>
FeatureStatus rxSwitchRead(int32_t& value) {
  bool on = false;
  const FeatureStatus status = rxFilterSwitchQuery(W, on);
  value = on ? 1 : 0;
  return status;
}

template <RxFilterSwitch W>
FeatureStatus rxSwitchWrite(int32_t value, int32_t& readBack) {
  bool on = false;
  const FeatureStatus status = rxFilterSwitchSet(W, value != 0, on);
  readBack = on ? 1 : 0;
  return status;
}

template <RxFilterSwitch W>
String rxSwitchText(int32_t value) {
  return rxFilterSwitchText(W, value != 0);
}

template <RxFilterSwitch W>
void speakRxSwitch(int32_t value) {
  speakRxFilterSwitch(W, value != 0);
}

// ---- Preamp: 0 off, then its levels ----

// ---- RF gain, in percent of the radio's range ----
//
// 5 % a step, 25 % a big step; on the FT-891's 31 and the KX2's 61 raw steps each still moves the
// radio (tests/cat level_scale_step_moves_and_stops_at_the_ends), and the read-back is what it took.

FeatureStatus rfGainLimits(ValueSpec& spec) {
  const FeatureStatus access = rfGainWriteAccess();
  if (access != FeatureStatus::Ok) return access;
  spec.min = 0;
  spec.max = 100;
  spec.step = 5;
  spec.maxDigits = 3;
  spec.unit = " %";
  return FeatureStatus::Ok;
}

FeatureStatus rfGainRead(int32_t& value) {
  uint8_t percent = 0;
  const FeatureStatus status = rfGainQuery(percent);
  value = percent;
  return status;
}

FeatureStatus rfGainWrite(int32_t value, int32_t& readBack) {
  uint8_t percent = 0;
  const FeatureStatus status = rfGainSet((uint8_t)value, percent);
  readBack = percent;
  return status;
}

String rfGainTextOf(int32_t value) { return rfGainText((uint8_t)value); }
void speakRfGainOf(int32_t value) { speakRfGain((uint8_t)value); }

bool preampAvailable() { return currentProfile().caps.getPreamp && preampLevels() > 0; }

FeatureStatus preampLimits(ValueSpec& spec) {
  if (!preampAvailable() || !currentProfile().caps.setPreamp) return FeatureStatus::Unsupported;
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

constexpr Ft8x7Setting kLowCut = Ft8x7Setting::LowCut;
constexpr Ft8x7Setting kHighCut = Ft8x7Setting::HighCut;
constexpr RxFilterLevel kRxLowCut = RxFilterLevel::LowCut;
constexpr RxFilterLevel kRxHighCut = RxFilterLevel::HighCut;
constexpr RxFilterLevel kWidth = RxFilterLevel::Width;
constexpr RxFilterLevel kNotchFreq = RxFilterLevel::NotchFreq;
constexpr RxFilterLevel kContourFreq = RxFilterLevel::ContourFreq;
constexpr RxFilterLevel kApfFreq = RxFilterLevel::ApfFreq;
constexpr RxFilterSwitch kNarrow = RxFilterSwitch::Narrow;
constexpr RxFilterSwitch kContour = RxFilterSwitch::Contour;
constexpr RxFilterSwitch kApf = RxFilterSwitch::Apf;

constexpr SettingOps kRxOps[] = {
    {Setting::NrLevel, rxLevelAvailable<RxLevel::Nr>, rxLevelLimits<RxLevel::Nr>,
     rxLevelRead<RxLevel::Nr>, rxLevelWrite<RxLevel::Nr>, rxLevelTextOf<RxLevel::Nr>, speakRxLevelOf<RxLevel::Nr>,
     rxLevelConfirm},
    {Setting::NbLevel, rxLevelAvailable<RxLevel::Nb>, rxLevelLimits<RxLevel::Nb>,
     rxLevelRead<RxLevel::Nb>, rxLevelWrite<RxLevel::Nb>, rxLevelTextOf<RxLevel::Nb>, speakRxLevelOf<RxLevel::Nb>,
     rxLevelConfirm},
    {Setting::RxLowCut, cutAvailable<kLowCut, kRxLowCut>, cutLimits<kLowCut, kRxLowCut>,
     cutRead<kLowCut, kRxLowCut>, cutWrite<kLowCut, kRxLowCut>, cutText<kLowCut, kRxLowCut>,
     speakCut<kLowCut, kRxLowCut>, cutConfirm},
    {Setting::RxHighCut, cutAvailable<kHighCut, kRxHighCut>, cutLimits<kHighCut, kRxHighCut>,
     cutRead<kHighCut, kRxHighCut>, cutWrite<kHighCut, kRxHighCut>, cutText<kHighCut, kRxHighCut>,
     speakCut<kHighCut, kRxHighCut>, cutConfirm},
    {Setting::RxWidth, rxFilterAvailable<kWidth>, rxFilterLimits<kWidth>, rxFilterRead<kWidth>,
     rxFilterWrite<kWidth>, rxFilterText<kWidth>, speakRxFilter<kWidth>, nullptr, 0, widthStepsFrom},
    {Setting::NotchFreq, rxFilterAvailable<kNotchFreq>, rxFilterLimits<kNotchFreq>, rxFilterRead<kNotchFreq>,
     rxFilterWrite<kNotchFreq>, rxFilterText<kNotchFreq>, speakRxFilter<kNotchFreq>},
    {Setting::ContourFreq, rxFilterAvailable<kContourFreq>, rxFilterLimits<kContourFreq>,
     rxFilterRead<kContourFreq>, rxFilterWrite<kContourFreq>, rxFilterText<kContourFreq>,
     speakRxFilter<kContourFreq>},
    {Setting::ApfFreq, rxFilterAvailable<kApfFreq>, rxFilterLimits<kApfFreq>, rxFilterRead<kApfFreq>,
     rxFilterWrite<kApfFreq>, rxFilterText<kApfFreq>, speakRxFilter<kApfFreq>},
    {Setting::Narrow, rxSwitchAvailable<kNarrow>, rxSwitchLimits<kNarrow>, rxSwitchRead<kNarrow>,
     rxSwitchWrite<kNarrow>, rxSwitchText<kNarrow>, speakRxSwitch<kNarrow>},
    {Setting::Contour, rxSwitchAvailable<kContour>, rxSwitchLimits<kContour>, rxSwitchRead<kContour>,
     rxSwitchWrite<kContour>, rxSwitchText<kContour>, speakRxSwitch<kContour>},
    {Setting::Apf, rxSwitchAvailable<kApf>, rxSwitchLimits<kApf>, rxSwitchRead<kApf>, rxSwitchWrite<kApf>,
     rxSwitchText<kApf>, speakRxSwitch<kApf>},
    {Setting::BpfWidth, ft857Available, bpfWidthLimits, ft857Read<Ft8x7Setting::BpfWidth>,
     ft857Write<Ft8x7Setting::BpfWidth>, ft857Text<Ft8x7Setting::BpfWidth>, ft857Speak<Ft8x7Setting::BpfWidth>,
     always},
    {Setting::PbtInner, pbtAvailable<true>, pbtLimits<true>, pbtRead<true>, pbtWrite<true>,
     pbtText<true>, speakPbt<true>},
    {Setting::PbtOuter, pbtAvailable<false>, pbtLimits<false>, pbtRead<false>, pbtWrite<false>,
     pbtText<false>, speakPbt<false>},
    {Setting::FilterSlot, filterSlotAvailable, filterSlotLimits, filterSlotRead, filterSlotWrite,
     filterSlotText, speakFilterSlot},
    {Setting::Notch, notchAvailable, notchLimits, notchRead, notchWrite, notchText, speakNotch},
    {Setting::Preamp, preampAvailable, preampLimits, preampRead, preampWrite, preampTextOf,
     speakPreampOf},
    {Setting::Att, attAvailable, attLimits, attRead, attWrite, attTextOf, speakAttOf},
    {Setting::RfGain, rfGainSupported, rfGainLimits, rfGainRead, rfGainWrite, rfGainTextOf, speakRfGainOf},
};

}  // namespace

const SettingOps* rxSettingOps(Setting setting) {
  for (const SettingOps& ops : kRxOps) {
    if (ops.setting == setting) return &ops;
  }
  return nullptr;
}
