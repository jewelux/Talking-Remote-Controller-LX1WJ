// The settings the keypad steps and enters, one SettingOps each.
#include "keypad_setting_ops.h"

#include "ft8x7_codec.h"
#include "radio_catalog.h"
#include "radio_features_internal.h"
#include "radio_globals.h"
#include "radio_prefs.h"
#include "radio_protocol.h"
#include "ui_features.h"
#include "ui_speech.h"

namespace {



// ---- RF power, in tenths of a watt ----
//
// Whole watts are typed: the radios are set in watts. The IC-705 reads tenths.

// The FT-857/897 band group the entry was opened on: the power of that group is set, even
// after a band change.
uint64_t s_rfPowerHz = 0;

// 1 W up to 20 W, 5 W above.
int32_t stepRfPower(int32_t tenths, int8_t dir) {
  if (dir > 0) return tenths < 200 ? tenths + 10 : (tenths / 50 + 1) * 50;
  return tenths <= 200 ? tenths - 10 : (tenths - 1) / 50 * 50;
}

// The FT-8x7 too, where it is set only experimentally: its limits say so.
bool rfPowerAvailable() { return currentProfile().caps.setRfPower || currentFt8x7Model() != Ft8x7Model::None; }

FeatureStatus rfPowerLimits(ValueSpec& spec) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (isFt8x7()) {
    const FeatureStatus status = ft8x7RfPowerRange(s_rfPowerHz, min, max);
    if (status != FeatureStatus::Ok) return status;
  } else if (!rfPowerRange(min, max)) {
    return FeatureStatus::Unsupported;
  }
  spec.min = min * 10;
  spec.max = max * 10;
  spec.scale = 1;
  spec.maxDigits = 3;
  spec.stepFn = stepRfPower;
  spec.unit = " W";
  return FeatureStatus::Ok;
}

FeatureStatus rfPowerRead(int32_t& value) {
  uint16_t tenths = 0;
  FeatureStatus status;
  if (isFt8x7()) {
    Ft8x7SettingState state;
    status = ft8x7SettingQuery(Ft8x7Setting::RfPower, state);
    tenths = state.wattsTenths;
  } else {
    status = rfPowerQuery(tenths);
  }
  value = tenths;
  return status;
}

FeatureStatus rfPowerWrite(int32_t value, int32_t& readBack) {
  const uint16_t watts = (uint16_t)((value + 5) / 10);
  if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7RfPowerSet(watts, state, s_rfPowerHz);
    readBack = state.wattsTenths;
    return status;
  }
  uint16_t tenths = 0;
  const FeatureStatus status = rfPowerSet(watts, tenths);
  readBack = tenths;
  return status;
}

String rfPowerTextOf(int32_t value) { return rfPowerText((uint16_t)value); }
void speakRfPowerOf(int32_t value) { speakRfPower((uint16_t)value); }

// ---- CTCSS tone, in tenths of a Hz ----
//
// The FT-8x7 cannot read its tone: the one HamTRC last set, else the profile's default. Its
// write fails only for a tone that is not a standard one.

bool ctcssAvailable() { return isFt8x7() || currentProfile().caps.setCtcss; }

int32_t stepCtcss(int32_t tenths, int8_t dir) { return yaesuCtcssToneStep((uint16_t)tenths, dir); }

FeatureStatus ctcssLimits(ValueSpec& spec) {
  if (!ctcssAvailable()) return FeatureStatus::Unsupported;
  uint16_t first = 0;
  uint16_t last = 0;
  yaesuCtcssToneAt(0, first);
  yaesuCtcssToneAt(49, last);
  spec.min = first;
  spec.max = last;
  spec.scale = 1;
  spec.pointDigits = 1;
  spec.bareInUnits = true;
  spec.maxDigits = 4;
  // No unit while typing: "885" is 88.5 Hz.
  spec.stepFn = stepCtcss;
  return FeatureStatus::Ok;
}

FeatureStatus ctcssRead(int32_t& value) {
  if (isFt8x7()) {
    value = live.ctcssValid ? live.ctcssTenths : currentProfile().ft8x7Bank6.ctcssDefaultTenths;
    return FeatureStatus::Ok;
  }
  uint16_t tenths = 0;
  const FeatureStatus status = ctcssToneQuery(tenths);
  value = tenths;
  return status;
}

FeatureStatus ctcssWrite(int32_t value, int32_t& readBack) {
  if (isFt8x7()) {
    if (!yaesuCatSetCtcssTenths((uint16_t)value)) return FeatureStatus::Invalid;
    readBack = value;
    return FeatureStatus::Ok;
  }
  uint16_t tenths = 0;
  const FeatureStatus status = ctcssToneSet((uint16_t)value, tenths);
  readBack = tenths;
  return status;
}

String ctcssTextOf(int32_t value) { return ctcssText((uint16_t)value); }
void speakCtcssOf(int32_t value) { speakCtcss((uint16_t)value); }

// ---- DCS code: FT-8x7 only, written like the tone ----

bool dcsAvailable() { return isFt8x7(); }

int32_t stepDcs(int32_t code, int8_t dir) { return yaesuDcsCodeStep((uint16_t)code, dir); }

FeatureStatus dcsLimits(ValueSpec& spec) {
  if (!dcsAvailable()) return FeatureStatus::Unsupported;
  uint16_t first = 0;
  uint16_t last = 0;
  yaesuDcsCodeAt(0, first);
  yaesuDcsCodeAt(103, last);
  spec.min = first;
  spec.max = last;
  spec.maxDigits = 3;
  spec.stepFn = stepDcs;
  return FeatureStatus::Ok;
}

FeatureStatus dcsRead(int32_t& value) {
  value = live.dcsValid ? live.dcsCode : currentProfile().ft8x7Bank6.dcsDefaultCode;
  return FeatureStatus::Ok;
}

FeatureStatus dcsWrite(int32_t value, int32_t& readBack) {
  if (!yaesuCatSetDcsCode((uint16_t)value)) return FeatureStatus::Invalid;
  readBack = value;
  return FeatureStatus::Ok;
}

String dcsDigits(int32_t value) {
  char digits[8] = "";
  snprintf(digits, sizeof(digits), "%03u", (unsigned)value);
  return digits;
}

String dcsText(int32_t value) { return "DCS " + dcsDigits(value); }

void speakDcs(int32_t value) {
  if (!g_speechEnabled) return;
  speakLabel("dcs");
  speakDigitsAndPoint(dcsDigits(value));
}

// ---- RIT offset in Hz: CI-V ----

bool ritOffsetAvailable() { return protocolSupportsRit(); }

FeatureStatus ritOffsetLimits(ValueSpec& spec) {
  if (!ritOffsetAvailable()) return FeatureStatus::Unsupported;
  spec.min = -9999;
  spec.max = 9999;
  spec.sign = true;
  spec.maxDigits = 4;
  spec.step = 10;
  spec.bigSteps = 10;
  spec.unit = " Hz";
  return FeatureStatus::Ok;
}

FeatureStatus ritOffsetRead(int32_t& value) {
  return queryRitOffsetHz(value, kCatReplyTimeoutMs) ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus ritOffsetWrite(int32_t value, int32_t& readBack) {
  if (!setRitOffsetHz(value)) return failure(FeatureStatus::Failed);
  if (!queryRitOffsetHz(readBack, kCatReplyTimeoutMs)) readBack = value;
  return FeatureStatus::Ok;
}

String ritOffsetText(int32_t value) { return "RIT " + String(value) + " Hz"; }

void speakRitOffset(int32_t hz) {
  if (!g_speechEnabled) return;
  speakLabel("rit");
  if (hz != 0) {
    speakToken(hz > 0 ? "plus" : "minus");
    playSilenceMs(60);
  }
  speakNumber(String(hz < 0 ? -hz : hz));
  playSilenceMs(60);
  speakToken("hertz");
}

// ---- Speech volume 1..9 and speed, kept by HamTRC ----

FeatureStatus volumeLimits(ValueSpec& spec) {
  spec.min = 1;
  spec.max = 9;
  spec.maxDigits = 1;
  spec.bigSteps = 2;
  return FeatureStatus::Ok;
}

FeatureStatus volumeRead(int32_t& value) {
  value = g_volumeLevel;
  return FeatureStatus::Ok;
}

FeatureStatus volumeWrite(int32_t value, int32_t& readBack) {
  applyVolumeLevel((uint8_t)value);
  saveVolumeToNvs((uint8_t)value);
  readBack = g_volumeLevel;
  return FeatureStatus::Ok;
}

String volumeText(int32_t value) { return "VOLUME " + String(value); }

void speakVolume(int32_t value) {
  if (!g_speechEnabled) return;
  speakLabel("volume");
  speakVolumeLevel((uint8_t)value);
}

FeatureStatus speedLimits(ValueSpec& spec) {
  spec.min = (int32_t)SpeechSpeed::Slow;
  spec.max = (int32_t)SpeechSpeed::Fast;
  spec.maxDigits = 1;
  spec.bigSteps = 1;
  return FeatureStatus::Ok;
}

FeatureStatus speedRead(int32_t& value) {
  value = (int32_t)g_speechSpeed;
  return FeatureStatus::Ok;
}

FeatureStatus speedWrite(int32_t value, int32_t& readBack) {
  applySpeechSpeed((SpeechSpeed)value);
  saveSpeechSpeedToNvs((uint8_t)g_speechSpeed);
  readBack = (int32_t)g_speechSpeed;
  return FeatureStatus::Ok;
}

String speedText(int32_t value) { return String("SPEED ") + speechSpeedName((SpeechSpeed)value); }

// Said at the new speed.
void speakSpeed(int32_t value) {
  if (!g_speechEnabled) return;
  speakLabel("speed");
  speakToken(speechSpeedName((SpeechSpeed)value));
}

constexpr SettingOps kOps[] = {
    {Setting::RfPower, rfPowerAvailable, rfPowerLimits, rfPowerRead, rfPowerWrite, rfPowerTextOf,
     speakRfPowerOf, always},
    {Setting::Ctcss, ctcssAvailable, ctcssLimits, ctcssRead, ctcssWrite, ctcssTextOf, speakCtcssOf},
    {Setting::Dcs, dcsAvailable, dcsLimits, dcsRead, dcsWrite, dcsText, speakDcs},
    {Setting::RitOffset, ritOffsetAvailable, ritOffsetLimits, ritOffsetRead, ritOffsetWrite,
     ritOffsetText, speakRitOffset},
    {Setting::Volume, always, volumeLimits, volumeRead, volumeWrite, volumeText, speakVolume},
    {Setting::SpeechSpeed, always, speedLimits, speedRead, speedWrite, speedText, speakSpeed, nullptr,
     (int32_t)SpeechSpeed::Normal},
};

}  // namespace

const SettingOps* settingOps(Setting setting) {
  for (const SettingOps& ops : kOps) {
    if (ops.setting == setting) return &ops;
  }
  if (const SettingOps* rx = rxSettingOps(setting)) return rx;
  if (const SettingOps* tx = txSettingOps(setting)) return tx;
  return cwSettingOps(setting);
}

FeatureStatus switchSettingLimits(ValueSpec& spec, FeatureStatus (*writeAccess)()) {
  const FeatureStatus access = writeAccess();
  if (access != FeatureStatus::Ok) return access;
  spec.min = 0;
  spec.max = 1;
  spec.maxDigits = 1;
  spec.bigSteps = 1;
  return FeatureStatus::Ok;
}
