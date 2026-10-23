// The settings the keypad steps and enters, one SettingOps each.
#include "keypad_setting_ops.h"

#include "ft8x7_codec.h"
#include "radio_catalog.h"
#include "radio_globals.h"
#include "radio_protocol.h"
#include "ui_features.h"
#include "ui_speech.h"

namespace {

bool isFt8x7Protocol() { return currentProtocolType() == PROTO_YAESU_FT8X7; }

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

bool rfPowerAvailable() { return currentProfile().caps.setRfPower; }

FeatureStatus rfPowerLimits(ValueSpec& spec) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (isFt8x7Protocol()) {
    const FeatureStatus status = ft8x7RfPowerRange(s_rfPowerHz, min, max);
    if (status != FeatureStatus::Ok) return status;
  } else if (!rfPowerRange(min, max)) {
    return FeatureStatus::Unsupported;
  }
  spec.name = "RFPOWER";
  spec.setting = Setting::RfPower;
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
  if (isFt8x7Protocol()) {
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
  if (isFt8x7Protocol()) {
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

bool ctcssAvailable() { return isFt8x7Protocol() || currentProfile().caps.setCtcss; }

int32_t stepCtcss(int32_t tenths, int8_t dir) { return yaesuCtcssToneStep((uint16_t)tenths, dir); }

FeatureStatus ctcssLimits(ValueSpec& spec) {
  if (!ctcssAvailable()) return FeatureStatus::Unsupported;
  uint16_t first = 0;
  uint16_t last = 0;
  yaesuCtcssToneAt(0, first);
  yaesuCtcssToneAt(49, last);
  spec.name = "CTCSS";
  spec.setting = Setting::Ctcss;
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
  if (isFt8x7Protocol()) {
    value = live.ctcssValid ? live.ctcssTenths : currentProfile().ft8x7Bank6.ctcssDefaultTenths;
    return FeatureStatus::Ok;
  }
  uint16_t tenths = 0;
  const FeatureStatus status = ctcssToneQuery(tenths);
  value = tenths;
  return status;
}

FeatureStatus ctcssWrite(int32_t value, int32_t& readBack) {
  if (isFt8x7Protocol()) {
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

bool dcsAvailable() { return isFt8x7Protocol(); }

int32_t stepDcs(int32_t code, int8_t dir) { return yaesuDcsCodeStep((uint16_t)code, dir); }

FeatureStatus dcsLimits(ValueSpec& spec) {
  if (!dcsAvailable()) return FeatureStatus::Unsupported;
  uint16_t first = 0;
  uint16_t last = 0;
  yaesuDcsCodeAt(0, first);
  yaesuDcsCodeAt(103, last);
  spec.name = "DCS";
  spec.setting = Setting::Dcs;
  spec.min = first;
  spec.max = last;
  spec.maxDigits = 3;
  spec.stepFn = stepDcs;
  return FeatureStatus::Ok;
}

FeatureStatus dcsRead(int32_t& value) {
  if (!dcsAvailable()) return FeatureStatus::Unsupported;
  value = live.dcsValid ? live.dcsCode : currentProfile().ft8x7Bank6.dcsDefaultCode;
  return FeatureStatus::Ok;
}

FeatureStatus dcsWrite(int32_t value, int32_t& readBack) {
  if (!dcsAvailable()) return FeatureStatus::Unsupported;
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

constexpr SettingOps kOps[] = {
    {Setting::RfPower, "power", rfPowerAvailable, rfPowerLimits, rfPowerRead, rfPowerWrite, rfPowerTextOf,
     speakRfPowerOf},
    {Setting::Ctcss, "ctcss", ctcssAvailable, ctcssLimits, ctcssRead, ctcssWrite, ctcssTextOf, speakCtcssOf},
    {Setting::Dcs, "dcs", dcsAvailable, dcsLimits, dcsRead, dcsWrite, dcsText, speakDcs},
};

}  // namespace

const SettingOps* settingOps(Setting setting) {
  for (const SettingOps& ops : kOps) {
    if (ops.setting == setting) return &ops;
  }
  return nullptr;
}
