// The Bank 7 CW settings the keypad says, switches, steps and enters, one SettingOps each: the
// keyer and its speed, break-in and the FT-857/897 QSK time, the sidetone, the FT-857/897 keyer
// type, the pitch, weight, key reverse and break-in delay. A switch is 0 off, 1 on.
#include "keypad_setting_ops.h"

#include "cat_setting_codec.h"
#include "ft8x7_eeprom_map.h"
#include "radio_catalog.h"
#include "radio_features_internal.h"
#include "radio_globals.h"
#include "radio_protocol.h"
#include "ui_features.h"
#include "ui_speech.h"

namespace {

// The FT-8x7 keeps these in its EEPROM: B and C open the entry, staged for Enter.

Ft8x7SettingState ft8x7State(Ft8x7Setting setting, int32_t value) {
  Ft8x7SettingState state;
  state.setting = setting;
  state.value = (uint16_t)value;
  state.on = value != 0;
  return state;
}

// ---- The keyer ----

FeatureStatus keyerLimits(ValueSpec& spec) { return switchSettingLimits(spec, keyerCanSet()); }

FeatureStatus keyerRead(int32_t& value) {
  bool on = false;
  const FeatureStatus status = keyerQuery(on);
  value = on ? 1 : 0;
  return status;
}

FeatureStatus keyerWrite(int32_t value, int32_t& readBack) {
  bool on = false;
  const FeatureStatus status = keyerSet(value != 0, on);
  readBack = on ? 1 : 0;
  return status;
}

String keyerTextOf(int32_t value) { return keyerText(value != 0); }
void speakKeyerOf(int32_t value) { speakKeyer(value != 0); }

// ---- Break-in: Icom off, semi, full; the others off and on ----


// Icom: off -> semi -> full, and back.
int32_t stepCivBreakIn(int32_t mode, int8_t dir) {
  constexpr int32_t kOff = (int32_t)BreakInMode::Off;
  constexpr int32_t kSemi = (int32_t)BreakInMode::Semi;
  constexpr int32_t kFull = (int32_t)BreakInMode::Full;
  if (dir > 0) return mode == kOff ? kSemi : kFull;
  return mode == kFull ? kSemi : kOff;
}

FeatureStatus breakInLimits(ValueSpec& spec) {
  const FeatureStatus status = switchSettingLimits(spec, breakInCanSet(BreakInMode::On));
  if (status != FeatureStatus::Ok || !isCiv()) return status;
  spec.max = (int32_t)BreakInMode::Full;
  spec.stepFn = stepCivBreakIn;
  return FeatureStatus::Ok;
}

FeatureStatus breakInRead(int32_t& value) {
  BreakInMode mode = BreakInMode::Off;
  const FeatureStatus status = breakInQuery(mode);
  value = (int32_t)mode;
  return status;
}

FeatureStatus breakInWrite(int32_t value, int32_t& readBack) {
  BreakInMode mode = BreakInMode::Off;
  const FeatureStatus status = breakInSet((BreakInMode)value, mode);
  readBack = (int32_t)mode;
  return status;
}

String breakInTextOf(int32_t value) { return breakInText((BreakInMode)value); }
void speakBreakInOf(int32_t value) { speakBreakIn((BreakInMode)value); }

// ---- Speed, pitch and break-in delay: every radio that has them ----

constexpr Ft8x7Setting ft8x7CwSettingOf(CwLevel level) {
  return level == CwLevel::Pitch ? Ft8x7Setting::CwPitch
         : level == CwLevel::Delay ? Ft8x7Setting::CwDelay
                                   : Ft8x7Setting::CwSpeed;
}

constexpr Ft8x7CwLevel ft8x7CwLevelOf(CwLevel level) {
  return level == CwLevel::Pitch ? Ft8x7CwLevel::Pitch
         : level == CwLevel::Delay ? Ft8x7CwLevel::Delay
                                   : Ft8x7CwLevel::Speed;
}

// One menu step of the FT-8x7.
template <Ft8x7CwLevel L>
int32_t stepFt8x7Cw(int32_t value, int8_t dir) {
  return ft8x7CwLevelStep(currentFt8x7Model(), L, (uint16_t)value, dir);
}

// The FT-8x7 delay in 100 ms rather than its 10 ms menu steps: FULL (0, FT-857/897), the
// shortest delay, then the hundreds up to the longest.
int32_t stepFt8x7CwDelay(int32_t ms, int8_t dir) {
  const Ft8x7Model model = currentFt8x7Model();
  uint16_t min = 0;
  uint16_t max = 0;
  if (!ft8x7CwLevelRange(model, Ft8x7CwLevel::Delay, min, max)) return ms;
  if (dir > 0) {
    if (ms < min) return min;
    const int32_t next = (ms / 100 + 1) * 100;
    return next < max ? next : max;
  }
  if (ms <= min) return ft8x7CwDelayHasFull(model) ? 0 : min;
  const int32_t prev = (ms - 1) / 100 * 100;
  return prev > min ? prev : min;
}

// The FTDX10's SD table, the TS-480's ms steps.
int32_t stepAsciiCwDelay(int32_t ms, int8_t dir) {
  const SettingEncodings& enc = currentProfile().settings;
  if (enc.cwDelay == VoxDelayEncoding::YaesuSteps) return yaesuAsciiVoxDelayStepBy((uint16_t)ms, dir);
  return asciiVoxDelayMsStepBy((uint16_t)ms, dir, enc.cwDelayMinMs, enc.cwDelayMaxMs, enc.cwDelayStepMs);
}

template <CwLevel L>
bool cwAvailable() {
  return cwLevelSupported(L);
}

// Speed 1 WPM, pitch 10 Hz, the delay the radio's step; on the FT-8x7 its menu step, the delay
// 100 ms and from FULL (0) up on the FT-857/897.
template <CwLevel L>
FeatureStatus cwLimits(ValueSpec& spec) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!cwLevelRange(L, min, max)) return FeatureStatus::Unsupported;
  spec.min = min;
  spec.max = max;
  spec.maxDigits = L == CwLevel::Speed ? 2 : 4;
  spec.unit = L == CwLevel::Speed ? " WPM" : L == CwLevel::Pitch ? " Hz" : " ms";
  if (isFt8x7()) {
    spec.stepFn = L == CwLevel::Delay ? stepFt8x7CwDelay : stepFt8x7Cw<ft8x7CwLevelOf(L)>;
    if (L == CwLevel::Delay && ft8x7CwDelayHasFull(currentFt8x7Model())) spec.min = 0;
  } else if (L == CwLevel::Pitch) {
    spec.step = 10;
  } else if (L == CwLevel::Delay) {
    spec.stepFn = stepAsciiCwDelay;
  }
  return FeatureStatus::Ok;
}

template <CwLevel L>
FeatureStatus cwRead(int32_t& value) {
  uint16_t v = 0;
  const FeatureStatus status = cwLevelQuery(L, v);
  value = v;
  return status;
}

// The FT-8x7 set takes the FT-857/897 FULL delay, which cwLevelRange leaves out.
template <CwLevel L>
FeatureStatus cwWrite(int32_t value, int32_t& readBack) {
  if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7LevelSet(ft8x7CwSettingOf(L), (uint16_t)value, state);
    readBack = state.value;
    return status;
  }
  uint16_t v = 0;
  const FeatureStatus status = cwLevelSet(L, (uint16_t)value, v);
  readBack = v;
  return status;
}

template <CwLevel L>
String cwText(int32_t value) {
  return cwLevelText(L, (uint16_t)value);
}

template <CwLevel L>
void speakCw(int32_t value) {
  speakCwLevel(L, (uint16_t)value);
}

// ---- FT-8x7 menus: sidetone, weight, QSK time; key reverse and the keyer type ----

template <Ft8x7Setting S>
bool ft8x7Available() {
  return ft8x7SettingAvailable(S);
}

template <Ft8x7Setting S>
FeatureStatus ft8x7Read(int32_t& value) {
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7SettingQuery(S, state);
  value = S == Ft8x7Setting::KeyReverse || S == Ft8x7Setting::Paddle ? (state.on ? 1 : 0) : state.value;
  return status;
}

template <Ft8x7Setting S>
String ft8x7Text(int32_t value) {
  return ft8x7SettingText(ft8x7State(S, value));
}

template <Ft8x7Setting S>
void ft8x7Speak(int32_t value) {
  speakFt8x7Setting(ft8x7State(S, value));
}

// Sidetone 0..100 in steps of 5; the weight 1:2.5..1:4.5 in tenths, typed with '*' as its
// point; QSK in the radio's steps.
template <Ft8x7Setting S>
FeatureStatus ft8x7LevelLimits(ValueSpec& spec) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!ft8x7Available<S>() || !currentProfile().caps.setRadioMenu || !ft8x7LevelRange(S, min, max)) {
    return FeatureStatus::Unsupported;
  }
  spec.min = min;
  spec.max = max;
  spec.maxDigits = S == Ft8x7Setting::Sidetone ? 3 : 2;
  if (S == Ft8x7Setting::Sidetone) {
    spec.step = 5;
  } else {
    spec.stepFn = stepFt8x7Cw<S == Ft8x7Setting::Qsk ? Ft8x7CwLevel::Qsk : Ft8x7CwLevel::Weight>;
  }
  if (S == Ft8x7Setting::CwWeight) {
    spec.scale = 1;
    spec.pointDigits = 1;
  }
  if (S == Ft8x7Setting::Qsk) spec.unit = " ms";
  return FeatureStatus::Ok;
}

template <Ft8x7Setting S>
FeatureStatus ft8x7LevelWrite(int32_t value, int32_t& readBack) {
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7LevelSet(S, (uint16_t)value, state);
  readBack = state.value;
  return status;
}

template <Ft8x7Setting S>
FeatureStatus ft8x7FlagLimits(ValueSpec& spec) {
  return switchSettingLimits(spec, ft8x7Available<S>() && currentProfile().caps.setRadioMenu);
}

template <Ft8x7Setting S>
FeatureStatus ft8x7FlagWrite(int32_t value, int32_t& readBack) {
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7FlagSettingSet(S, value != 0, state);
  readBack = state.on ? 1 : 0;
  return status;
}


constexpr SettingOps kCwOps[] = {
    {Setting::Keyer, keyerSupported, keyerLimits, keyerRead, keyerWrite, keyerTextOf, speakKeyerOf},
    {Setting::CwSpeed, cwAvailable<CwLevel::Speed>, cwLimits<CwLevel::Speed>, cwRead<CwLevel::Speed>,
     cwWrite<CwLevel::Speed>, cwText<CwLevel::Speed>, speakCw<CwLevel::Speed>, isFt8x7},
    {Setting::BreakIn, breakInSupported, breakInLimits, breakInRead, breakInWrite, breakInTextOf,
     speakBreakInOf},
    {Setting::Qsk, ft8x7Available<Ft8x7Setting::Qsk>, ft8x7LevelLimits<Ft8x7Setting::Qsk>,
     ft8x7Read<Ft8x7Setting::Qsk>, ft8x7LevelWrite<Ft8x7Setting::Qsk>, ft8x7Text<Ft8x7Setting::Qsk>,
     ft8x7Speak<Ft8x7Setting::Qsk>, always},
    {Setting::Sidetone, ft8x7Available<Ft8x7Setting::Sidetone>,
     ft8x7LevelLimits<Ft8x7Setting::Sidetone>, ft8x7Read<Ft8x7Setting::Sidetone>,
     ft8x7LevelWrite<Ft8x7Setting::Sidetone>, ft8x7Text<Ft8x7Setting::Sidetone>, ft8x7Speak<Ft8x7Setting::Sidetone>,
     always},
    {Setting::Paddle, ft8x7Available<Ft8x7Setting::Paddle>,
     ft8x7FlagLimits<Ft8x7Setting::Paddle>, ft8x7Read<Ft8x7Setting::Paddle>,
     ft8x7FlagWrite<Ft8x7Setting::Paddle>, ft8x7Text<Ft8x7Setting::Paddle>, ft8x7Speak<Ft8x7Setting::Paddle>},
    {Setting::CwPitch, cwAvailable<CwLevel::Pitch>, cwLimits<CwLevel::Pitch>, cwRead<CwLevel::Pitch>,
     cwWrite<CwLevel::Pitch>, cwText<CwLevel::Pitch>, speakCw<CwLevel::Pitch>, isFt8x7},
    {Setting::CwWeight, ft8x7Available<Ft8x7Setting::CwWeight>,
     ft8x7LevelLimits<Ft8x7Setting::CwWeight>, ft8x7Read<Ft8x7Setting::CwWeight>,
     ft8x7LevelWrite<Ft8x7Setting::CwWeight>, ft8x7Text<Ft8x7Setting::CwWeight>, ft8x7Speak<Ft8x7Setting::CwWeight>,
     always},
    {Setting::KeyReverse, ft8x7Available<Ft8x7Setting::KeyReverse>,
     ft8x7FlagLimits<Ft8x7Setting::KeyReverse>, ft8x7Read<Ft8x7Setting::KeyReverse>,
     ft8x7FlagWrite<Ft8x7Setting::KeyReverse>, ft8x7Text<Ft8x7Setting::KeyReverse>,
     ft8x7Speak<Ft8x7Setting::KeyReverse>},
    {Setting::CwDelay, cwAvailable<CwLevel::Delay>, cwLimits<CwLevel::Delay>, cwRead<CwLevel::Delay>,
     cwWrite<CwLevel::Delay>, cwText<CwLevel::Delay>, speakCw<CwLevel::Delay>, isFt8x7},
};

}  // namespace

const SettingOps* cwSettingOps(Setting setting) {
  for (const SettingOps& ops : kCwOps) {
    if (ops.setting == setting) return &ops;
  }
  return nullptr;
}
