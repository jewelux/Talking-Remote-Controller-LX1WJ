// The Bank 4 phone transmit settings the keypad says, switches, steps and enters, one
// SettingOps each: the processor, VOX, monitor, the FT-857/897 TX equalizer, mic gain and VOX
// delay. A switch is 0 off, 1 on.
#include "keypad_setting_ops.h"

#include "cat_setting_codec.h"
#include "radio_catalog.h"
#include "radio_features_internal.h"
#include "radio_globals.h"
#include "radio_protocol.h"
#include "ui_features.h"
#include "ui_speech.h"

namespace {

// The FT-8x7 keeps these in its EEPROM: B and C open the entry, staged for Enter.

// A level of 0..max in steps of 5 (1 up to 9).
FeatureStatus levelLimits(ValueSpec& spec, int32_t min, int32_t max, const char* unit = "") {
  spec.min = min;
  spec.max = max;
  spec.maxDigits = 3;
  spec.step = max <= 9 ? 1 : 5;
  spec.unit = unit;
  return FeatureStatus::Ok;
}

// The switch state, or what read says went wrong.
FeatureStatus switchRead(FeatureStatus (*read)(bool&), int32_t& value) {
  bool on = false;
  const FeatureStatus status = read(on);
  value = on ? 1 : 0;
  return status;
}

// Sets it, then reads it back; value where the read-back fails.
FeatureStatus switchWrite(FeatureStatus (*set)(bool), FeatureStatus (*read)(bool&), int32_t value,
                          int32_t& readBack) {
  const FeatureStatus status = set(value != 0);
  if (status != FeatureStatus::Ok) return status;
  if (switchRead(read, readBack) != FeatureStatus::Ok) readBack = value;
  return FeatureStatus::Ok;
}

// ---- Speech processor and its level ----

FeatureStatus procLimits(ValueSpec& spec) { return switchSettingLimits(spec, procCanSet()); }
FeatureStatus procRead(int32_t& value) { return switchRead(procQuery, value); }
FeatureStatus procWrite(int32_t value, int32_t& readBack) { return switchWrite(procSet, procQuery, value, readBack); }
String procText(int32_t value) { return procStateText(value != 0); }
void speakProc(int32_t value) { speakProcState(value != 0); }

FeatureStatus procLevelLimits(ValueSpec& spec) {
  if (!procCanSet()) return FeatureStatus::Unsupported;
  return levelLimits(spec, 0, 100);
}

FeatureStatus procLevelRead(int32_t& value) {
  uint8_t level = 0;
  const FeatureStatus status = procLevelQuery(level);
  value = level;
  return status;
}

FeatureStatus procLevelWrite(int32_t value, int32_t& readBack) {
  uint8_t level = 0;
  const FeatureStatus status = procLevelSet((uint8_t)value, level);
  readBack = level;
  return status;
}

String procLevelTextOf(int32_t value) { return procLevelText((uint8_t)value); }
void speakProcLevelOf(int32_t value) { speakProcLevel((uint8_t)value); }

// ---- VOX and its gain ----

bool voxCanSet() {
  uint8_t min = 0;
  uint8_t max = 0;
  return voxGainRange(min, max);
}

FeatureStatus voxLimits(ValueSpec& spec) { return switchSettingLimits(spec, voxCanSet()); }
FeatureStatus voxRead(int32_t& value) { return switchRead(voxQuery, value); }
FeatureStatus voxWrite(int32_t value, int32_t& readBack) { return switchWrite(voxSet, voxQuery, value, readBack); }
String voxText(int32_t value) { return voxStateText(value != 0); }
void speakVox(int32_t value) { speakVoxState(value != 0); }

// 1..100 on the FT-8x7, 0..9 on the TS-480, else 0..100.
FeatureStatus voxGainLimits(ValueSpec& spec) {
  uint8_t min = 0;
  uint8_t max = 0;
  if (!voxGainRange(min, max)) return FeatureStatus::Unsupported;
  return levelLimits(spec, min, max);
}

FeatureStatus voxGainRead(int32_t& value) {
  uint8_t level = 0;
  const FeatureStatus status = voxGainQuery(level);
  value = level;
  return status;
}

FeatureStatus voxGainWrite(int32_t value, int32_t& readBack) {
  uint8_t level = 0;
  const FeatureStatus status = voxGainSet((uint8_t)value, level);
  readBack = level;
  return status;
}

String voxGainTextOf(int32_t value) { return voxGainText((uint8_t)value); }
void speakVoxGainOf(int32_t value) { speakVoxGain((uint8_t)value); }

// ---- Monitor and its level: CI-V ----

bool monitorAvailable() { return protocolSupportsMonitor() && currentProfile().caps.getMonitor; }

FeatureStatus monitorLimits(ValueSpec& spec) {
  return switchSettingLimits(spec, monitorAvailable() && currentProfile().caps.setMonitor);
}

FeatureStatus monitorQuery(bool& on) {
  if (!monitorAvailable()) return FeatureStatus::Unsupported;
  return queryMonitorEnabled(on, kCatReplyTimeoutMs) ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus monitorSet(bool on) { return setMonitorEnabled(on) ? FeatureStatus::Ok : failure(FeatureStatus::Failed); }

FeatureStatus monitorRead(int32_t& value) { return switchRead(monitorQuery, value); }
FeatureStatus monitorWrite(int32_t value, int32_t& readBack) {
  return switchWrite(monitorSet, monitorQuery, value, readBack);
}
String monitorText(int32_t value) { return value ? "MONITOR ON" : "MONITOR OFF"; }

void speakMonitor(int32_t value) {
  if (g_speechEnabled) speakTokenState("monitor", value != 0);
}

bool monitorLevelAvailable() { return protocolSupportsMonitor() && currentProfile().caps.getMonitorLevel; }

FeatureStatus monitorLevelLimits(ValueSpec& spec) {
  if (!monitorLevelAvailable() || !currentProfile().caps.setMonitorLevel) return FeatureStatus::Unsupported;
  return levelLimits(spec, 0, 100, " %");
}

FeatureStatus monitorLevelRead(int32_t& value) {
  if (!monitorLevelAvailable()) return FeatureStatus::Unsupported;
  uint16_t raw = 0;
  if (!queryMonitorLevel(raw, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
  value = levelRawToPercent(raw);
  return FeatureStatus::Ok;
}

FeatureStatus monitorLevelWrite(int32_t value, int32_t& readBack) {
  if (!setMonitorLevel(levelPercentToRaw(value))) return failure(FeatureStatus::Failed);
  if (monitorLevelRead(readBack) != FeatureStatus::Ok) readBack = value;
  return FeatureStatus::Ok;
}

String monitorLevelText(int32_t value) { return "MONLEVEL " + String(value) + "%"; }

void speakMonitorLevel(int32_t value) {
  if (!g_speechEnabled) return;
  speakLabel("monitor level");
  speakDigitsAndPoint(String(value));
}

// ---- FT-857/897 TX equalizer (menu 48) and its two filters ----
//
// The radio's LPF cuts the highs and its HPF the lows: TX low cut is the HPF, TX high cut the
// LPF, and the equalizer both of them, 0 off, 1 LPF, 2 HPF, 3 both.

bool txEqAvailable() { return currentIsFt857Family(); }

FeatureStatus readMicEq(YaesuFt857MicEq& eq) {
  if (!txEqAvailable()) return FeatureStatus::Unsupported;
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7SettingQuery(Ft8x7Setting::MicEq, state);
  eq = state.micEq;
  return status;
}

FeatureStatus writeMicEq(YaesuFt857MicEq eq, YaesuFt857MicEq& readBack) {
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7MicEqSet(eq, state);
  readBack = state.micEq;
  return status;
}

Ft8x7SettingState micEqState(int32_t value) {
  Ft8x7SettingState state;
  state.setting = Ft8x7Setting::MicEq;
  state.micEq = (YaesuFt857MicEq)value;
  return state;
}

FeatureStatus txEqLimits(ValueSpec& spec) {
  if (!txEqAvailable() || !currentProfile().caps.setRadioMenu) return FeatureStatus::Unsupported;
  spec.min = (int32_t)YaesuFt857MicEq::Off;
  spec.max = (int32_t)YaesuFt857MicEq::Both;
  spec.maxDigits = 1;
  spec.bigSteps = 1;
  return FeatureStatus::Ok;
}

FeatureStatus txEqRead(int32_t& value) {
  YaesuFt857MicEq eq = YaesuFt857MicEq::Off;
  const FeatureStatus status = readMicEq(eq);
  value = (int32_t)eq;
  return status;
}

FeatureStatus txEqWrite(int32_t value, int32_t& readBack) {
  YaesuFt857MicEq eq = YaesuFt857MicEq::Off;
  const FeatureStatus status = writeMicEq((YaesuFt857MicEq)value, eq);
  readBack = (int32_t)eq;
  return status;
}

String txEqText(int32_t value) { return ft8x7SettingText(micEqState(value)); }
void speakTxEq(int32_t value) { speakFt8x7Setting(micEqState(value)); }

// The equalizer bit of a TX cut: LPF bit 0, HPF bit 1.
template <Setting S>
constexpr uint8_t kTxCutBit = S == Setting::TxHighCut ? 0x01 : 0x02;

template <Setting S>
FeatureStatus txCutLimits(ValueSpec& spec) {
  return switchSettingLimits(spec, txEqAvailable() && currentProfile().caps.setRadioMenu);
}

template <Setting S>
FeatureStatus txCutRead(int32_t& value) {
  YaesuFt857MicEq eq = YaesuFt857MicEq::Off;
  const FeatureStatus status = readMicEq(eq);
  value = ((uint8_t)eq & kTxCutBit<S>) ? 1 : 0;
  return status;
}

// The other filter is kept as the radio has it.
template <Setting S>
FeatureStatus txCutWrite(int32_t value, int32_t& readBack) {
  YaesuFt857MicEq eq = YaesuFt857MicEq::Off;
  FeatureStatus status = readMicEq(eq);
  if (status != FeatureStatus::Ok) return status;
  const uint8_t bits = value ? ((uint8_t)eq | kTxCutBit<S>) : ((uint8_t)eq & ~kTxCutBit<S>);
  status = writeMicEq((YaesuFt857MicEq)bits, eq);
  readBack = ((uint8_t)eq & kTxCutBit<S>) ? 1 : 0;
  return status;
}

template <Setting S>
String txCutText(int32_t value) {
  return String(settingInfo(S).label) + (value ? " ON" : " OFF");
}

template <Setting S>
void speakTxCut(int32_t value) {
  if (g_speechEnabled) speakTokenState(S == Setting::TxLowCut ? "tx lowcut" : "tx highcut", value != 0);
}

// ---- Mic gain, 0..100 ----

// The FT-8x7 mic gain menu the entry was opened in: that mode's gain is set, even after a mode
// change.
Ft8x7MicGain s_micGainMenu = Ft8x7MicGain::Ssb;

FeatureStatus micGainLimits(ValueSpec& spec) {
  if (!micGainCanSet()) return FeatureStatus::Unsupported;
  if (isFt8x7()) {
    const FeatureStatus status = ft8x7MicGainMenu(s_micGainMenu);
    if (status != FeatureStatus::Ok) return status;
  }
  return levelLimits(spec, 0, 100);
}

FeatureStatus micGainRead(int32_t& value) {
  uint8_t level = 0;
  const FeatureStatus status = micGainQuery(level);
  value = level;
  return status;
}

FeatureStatus micGainWrite(int32_t value, int32_t& readBack) {
  uint8_t level = 0;
  const FeatureStatus status = isFt8x7() ? ft8x7MicGainSet(s_micGainMenu, (uint8_t)value, level)
                                                 : micGainSet((uint8_t)value, level);
  readBack = level;
  return status;
}

String micGainTextOf(int32_t value) { return micGainText((uint8_t)value); }
void speakMicGainOf(int32_t value) { speakMicGain((uint8_t)value); }

// ---- VOX delay in ms, in the radio's steps ----

// The FTDX10's VD table, the FT-891's and TS-480's ms steps (at least 100 ms each).
int32_t stepAsciiVoxDelay(int32_t ms, int8_t dir) {
  const SettingEncodings& enc = currentProfile().settings;
  if (enc.voxDelay == VoxDelayEncoding::YaesuSteps) return yaesuAsciiVoxDelayStepBy((uint16_t)ms, dir);
  return asciiVoxDelayMsStepBy((uint16_t)ms, dir, enc.voxDelayMinMs, enc.voxDelayMaxMs, enc.voxDelayStepMs);
}

FeatureStatus voxDelayLimits(ValueSpec& spec) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!voxDelayRange(min, max)) return FeatureStatus::Unsupported;
  spec.min = min;
  spec.max = max;
  spec.maxDigits = 4;
  // CI-V and the FT-8x7: 100 ms.
  if (isAsciiRadio()) spec.stepFn = stepAsciiVoxDelay;
  else spec.step = 100;
  spec.unit = " ms";
  return FeatureStatus::Ok;
}

FeatureStatus voxDelayRead(int32_t& value) {
  uint16_t ms = 0;
  const FeatureStatus status = voxDelayQuery(ms);
  value = ms;
  return status;
}

FeatureStatus voxDelayWrite(int32_t value, int32_t& readBack) {
  uint16_t ms = 0;
  const FeatureStatus status = voxDelaySet((uint16_t)value, ms);
  readBack = ms;
  return status;
}

String voxDelayTextOf(int32_t value) { return voxDelayText((uint16_t)value); }
void speakVoxDelayOf(int32_t value) { speakVoxDelay((uint16_t)value); }


constexpr SettingOps kTxOps[] = {
    {Setting::Proc, procSupported, procLimits, procRead, procWrite, procText, speakProc},
    {Setting::ProcLevel, procSupported, procLevelLimits, procLevelRead, procLevelWrite,
     procLevelTextOf, speakProcLevelOf, isFt8x7},
    {Setting::Vox, voxSupported, voxLimits, voxRead, voxWrite, voxText, speakVox},
    {Setting::VoxGain, voxSupported, voxGainLimits, voxGainRead, voxGainWrite, voxGainTextOf,
     speakVoxGainOf, isFt8x7},
    {Setting::Monitor, monitorAvailable, monitorLimits, monitorRead, monitorWrite, monitorText,
     speakMonitor},
    {Setting::MonitorLevel, monitorLevelAvailable, monitorLevelLimits, monitorLevelRead,
     monitorLevelWrite, monitorLevelText, speakMonitorLevel},
    {Setting::TxEq, txEqAvailable, txEqLimits, txEqRead, txEqWrite, txEqText, speakTxEq, always},
    {Setting::TxLowCut, txEqAvailable, txCutLimits<Setting::TxLowCut>, txCutRead<Setting::TxLowCut>,
     txCutWrite<Setting::TxLowCut>, txCutText<Setting::TxLowCut>, speakTxCut<Setting::TxLowCut>},
    {Setting::TxHighCut, txEqAvailable, txCutLimits<Setting::TxHighCut>,
     txCutRead<Setting::TxHighCut>, txCutWrite<Setting::TxHighCut>, txCutText<Setting::TxHighCut>,
     speakTxCut<Setting::TxHighCut>},
    {Setting::MicGain, micGainSupported, micGainLimits, micGainRead, micGainWrite, micGainTextOf,
     speakMicGainOf, isFt8x7},
    {Setting::VoxDelay, voxSupported, voxDelayLimits, voxDelayRead, voxDelayWrite, voxDelayTextOf,
     speakVoxDelayOf, isFt8x7},
};

}  // namespace

const SettingOps* txSettingOps(Setting setting) {
  for (const SettingOps& ops : kTxOps) {
    if (ops.setting == setting) return &ops;
  }
  return nullptr;
}
