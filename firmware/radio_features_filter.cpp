// The receive filter settings (radio_features.h rxFilterLevel*, rxFilterSwitch*): the IF shift,
// width, narrow, the manual notch frequency, contour, APF and the low and high cut. CI-V has the
// width (1A 03), the ASCII radios what settings.rxFilter says, the FT-857/897 its DSP cut menus and
// NAR from the EEPROM. The values each takes are in rx_filter_codec.h.

#include "radio_features.h"
#include "radio_features_internal.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_civ.h"
#include "rx_filter_codec.h"

namespace {

RxFilterEncoding encoding() {
  return currentProfile().settings.rxFilter;
}

// The FT-857/897 menu that holds level, where it has one.
bool ft8x7LevelOf(RxFilterLevel level, Ft8x7Setting& out) {
  if (level == RxFilterLevel::LowCut) out = Ft8x7Setting::LowCut;
  else if (level == RxFilterLevel::HighCut) out = Ft8x7Setting::HighCut;
  else return false;
  return true;
}

// Whether the radio has a scale for level in some mode.
bool hasAnyScale(RxFilterLevel level) {
  static constexpr FilterModeGroup kGroups[] = {
      FilterModeGroup::Ssb, FilterModeGroup::Cw, FilterModeGroup::Rtty,
      FilterModeGroup::Data, FilterModeGroup::Am, FilterModeGroup::Fm,
  };
  RxFilterScale scale;
  for (FilterModeGroup group : kGroups) {
    if (rxFilterScaleFor(encoding(), level, group, scale)) return true;
  }
  return false;
}

FeatureStatus ft8x7Result(FeatureStatus status, const Ft8x7SettingState& state, int16_t& valueOut) {
  if (status == FeatureStatus::Ok) valueOut = (int16_t)state.value;
  return status;
}

// The scale of level in the radio's current mode; Unsupported in a mode without it.
FeatureStatus currentScale(RxFilterLevel level, FilterModeGroup& group, RxFilterScale& scale) {
  const RadioProfile& sp = currentProfile();
  const bool read = isCiv() ? civQueryFilterModeGroup(sp, group, kCatReplyTimeoutMs)
                            : asciiQueryFilterModeGroup(sp, group, kCatReplyTimeoutMs);
  if (!read) return failure(FeatureStatus::NoReply);
  return rxFilterScaleFor(encoding(), level, group, scale) ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}

// NoReply, or Unexpected when the radio holds a code its scale does not have.
FeatureStatus queryIn(RxFilterLevel level, FilterModeGroup group, int16_t& valueOut) {
  const RadioProfile& sp = currentProfile();
  RxFilterRead read = RxFilterRead::NoReply;
  if (isCiv()) {
    uint16_t hz = 0;
    read = civQueryRxWidth(sp, group, hz, kCatReplyTimeoutMs);
    if (read == RxFilterRead::Ok) valueOut = (int16_t)hz;
  } else {
    read = asciiQueryRxFilterLevel(sp, level, group, valueOut, kCatReplyTimeoutMs);
  }
  switch (read) {
    case RxFilterRead::Ok: return FeatureStatus::Ok;
    case RxFilterRead::Unknown: return FeatureStatus::Unexpected;
    default: return failure(FeatureStatus::NoReply);
  }
}

FeatureStatus setIn(RxFilterLevel level, FilterModeGroup group, int16_t value, int16_t& valueOut) {
  const RadioProfile& sp = currentProfile();
  const bool ok = isCiv() ? civSetRxWidth(sp, group, (uint16_t)value) : asciiSetRxFilterLevel(sp, level, group, value);
  if (!ok) return failure(FeatureStatus::Failed);
  valueOut = value;
  return FeatureStatus::Ok;
}

// The value of scale nearest value: 0 for the zero code, Invalid outside its values. A slope takes
// only its exact values.
FeatureStatus scaleValue(RxFilterLevel level, const RxFilterScale& scale, int16_t value, int16_t& out) {
  if (value == 0 && scale.hasZero()) {
    out = 0;
    return FeatureStatus::Ok;
  }
  if (value < (int16_t)scale.valueOf(scale.firstValueCode()) || value > (int16_t)scale.valueOf(scale.last)) {
    return FeatureStatus::Invalid;
  }
  out = (int16_t)scale.valueOf(rxFilterScaleCodeOf(scale, (uint16_t)value));
  if (rxFilterLevelIsSlope(level) && out != value) return FeatureStatus::Invalid;
  return FeatureStatus::Ok;
}

}  // namespace

bool rxFilterLevelSupported(RxFilterLevel level) {
  if (isFt8x7()) {
    Ft8x7Setting setting;
    return ft8x7LevelOf(level, setting) && ft8x7SettingAvailable(setting);
  }
  const RadioProfile& sp = currentProfile();
  if (!rxFilterLevelCap(sp.caps, level, false) || encoding() == RxFilterEncoding::None) return false;
  if (isCiv()) return level == RxFilterLevel::Width && encoding() == RxFilterEncoding::Icom;
  if (!isAsciiRadio()) return false;
  if (rxFilterLevelByMode(level)) return hasAnyScale(level);
  RxFilterSpan span;
  return rxFilterSpanFor(encoding(), level, span);
}

FeatureStatus rxFilterLevelRange(RxFilterLevel level, RxFilterRange& out) {
  if (!rxFilterLevelSupported(level)) return FeatureStatus::Unsupported;
  Ft8x7Setting setting;
  if (isFt8x7() && ft8x7LevelOf(level, setting)) {
    uint16_t lo = 0;
    uint16_t hi = 0;
    if (!ft8x7LevelRange(setting, lo, hi)) return FeatureStatus::Unsupported;
    out = {(int16_t)lo, (int16_t)hi, false};
    return FeatureStatus::Ok;
  }
  if (rxFilterLevelByMode(level)) {
    FilterModeGroup group = FilterModeGroup::Unknown;
    RxFilterScale scale;
    const FeatureStatus status = currentScale(level, group, scale);
    if (status != FeatureStatus::Ok) return status;
    out = {(int16_t)scale.valueOf(scale.firstValueCode()), (int16_t)scale.valueOf(scale.last), scale.hasZero()};
    return FeatureStatus::Ok;
  }
  RxFilterSpan span;
  if (!rxFilterSpanFor(encoding(), level, span)) return FeatureStatus::Unsupported;
  out = {span.min, span.max, false};
  return FeatureStatus::Ok;
}

FeatureStatus rxFilterLevelScale(RxFilterLevel level, RxFilterScale& out) {
  if (!rxFilterLevelSupported(level) || isFt8x7() || !rxFilterLevelByMode(level)) return FeatureStatus::Unsupported;
  FilterModeGroup group = FilterModeGroup::Unknown;
  return currentScale(level, group, out);
}

FeatureStatus rxFilterLevelWriteAccess(RxFilterLevel level) {
  if (!rxFilterLevelSupported(level)) return FeatureStatus::Unsupported;
  Ft8x7Setting setting;
  if (isFt8x7() && ft8x7LevelOf(level, setting)) return ft8x7LevelWriteAccess(setting);
  return writeAccess([level](const RadioCapabilities& caps) { return rxFilterLevelCap(caps, level, true); });
}

FeatureStatus rxFilterLevelQuery(RxFilterLevel level, int16_t& valueOut) {
  if (!rxFilterLevelSupported(level)) return FeatureStatus::Unsupported;
  Ft8x7Setting setting;
  if (isFt8x7() && ft8x7LevelOf(level, setting)) {
    Ft8x7SettingState state;
    return ft8x7Result(ft8x7SettingQuery(setting, state), state, valueOut);
  }
  FilterModeGroup group = FilterModeGroup::Unknown;
  if (rxFilterLevelByMode(level)) {
    RxFilterScale scale;
    const FeatureStatus status = currentScale(level, group, scale);
    if (status != FeatureStatus::Ok) return status;
  }
  return queryIn(level, group, valueOut);
}

FeatureStatus rxFilterLevelSet(RxFilterLevel level, int16_t value, int16_t& valueOut) {
  const FeatureStatus access = rxFilterLevelWriteAccess(level);
  if (access != FeatureStatus::Ok) return access;
  Ft8x7Setting setting;
  if (isFt8x7() && ft8x7LevelOf(level, setting)) {
    uint16_t lo = 0;
    uint16_t hi = 0;
    if (!ft8x7LevelRange(setting, lo, hi) || value < (int16_t)lo || value > (int16_t)hi) return FeatureStatus::Invalid;
    Ft8x7SettingState state;
    return ft8x7Result(ft8x7LevelSet(setting, (uint16_t)value, state), state, valueOut);
  }
  if (rxFilterLevelByMode(level)) {
    FilterModeGroup group = FilterModeGroup::Unknown;
    RxFilterScale scale;
    FeatureStatus status = currentScale(level, group, scale);
    if (status != FeatureStatus::Ok) return status;
    int16_t want = 0;
    status = scaleValue(level, scale, value, want);
    if (status != FeatureStatus::Ok) return status;
    return setIn(level, group, want, valueOut);
  }
  RxFilterSpan span;
  if (!rxFilterSpanFor(encoding(), level, span)) return FeatureStatus::Unsupported;
  if (value < span.min || value > span.max) return FeatureStatus::Invalid;
  return setIn(level, FilterModeGroup::Unknown, rxFilterSpanRound(span, value), valueOut);
}

FeatureStatus rxFilterLevelStep(RxFilterLevel level, int delta, int16_t& valueOut) {
  const FeatureStatus access = rxFilterLevelWriteAccess(level);
  if (access != FeatureStatus::Ok) return access;
  Ft8x7Setting setting;
  if (isFt8x7() && ft8x7LevelOf(level, setting)) {
    Ft8x7SettingState state;
    return ft8x7Result(ft8x7LevelStep(setting, delta, state), state, valueOut);
  }
  if (rxFilterLevelByMode(level)) {
    FilterModeGroup group = FilterModeGroup::Unknown;
    RxFilterScale scale;
    FeatureStatus status = currentScale(level, group, scale);
    if (status != FeatureStatus::Ok) return status;
    int16_t now = 0;
    status = queryIn(level, group, now);
    if (status != FeatureStatus::Ok) return status;
    uint8_t code = 0;
    // No step from the default width, which has no place among the widths.
    if (!rxFilterScaleStep(scale, rxFilterScaleCodeOf(scale, (uint16_t)now), delta, code)) return FeatureStatus::Invalid;
    return setIn(level, group, (int16_t)scale.valueOf(code), valueOut);
  }
  RxFilterSpan span;
  if (!rxFilterSpanFor(encoding(), level, span)) return FeatureStatus::Unsupported;
  int16_t now = 0;
  const FeatureStatus status = queryIn(level, FilterModeGroup::Unknown, now);
  if (status != FeatureStatus::Ok) return status;
  return setIn(level, FilterModeGroup::Unknown, rxFilterSpanRound(span, (int32_t)now + (int32_t)delta * span.step),
               valueOut);
}

bool rxFilterSwitchSupported(RxFilterSwitch sw) {
  if (isFt8x7()) return sw == RxFilterSwitch::Narrow && ft8x7SettingAvailable(Ft8x7Setting::Nar);
  const RadioProfile& sp = currentProfile();
  return rxFilterSwitchCap(sp.caps, sw, false) && sp.protocol == PROTO_YAESU_FTDX_ASCII &&
         rxFilterEncodingIsYaesu(encoding());
}

FeatureStatus rxFilterSwitchQuery(RxFilterSwitch sw, bool& on) {
  if (!rxFilterSwitchSupported(sw)) return FeatureStatus::Unsupported;
  if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7SettingQuery(Ft8x7Setting::Nar, state);
    if (status == FeatureStatus::Ok) on = state.on;
    return status;
  }
  return asciiQueryRxFilterSwitch(currentProfile(), sw, on, kCatReplyTimeoutMs) ? FeatureStatus::Ok
                                                                                  : failure(FeatureStatus::NoReply);
}

FeatureStatus rxFilterSwitchWriteAccess(RxFilterSwitch sw) {
  // The FT-857/897 keeps NAR in its band block, which HamTRC only reads.
  if (!rxFilterSwitchSupported(sw) || isFt8x7()) return FeatureStatus::Unsupported;
  return writeAccess([sw](const RadioCapabilities& caps) { return rxFilterSwitchCap(caps, sw, true); });
}

FeatureStatus rxFilterSwitchSet(RxFilterSwitch sw, bool on, bool& out) {
  const FeatureStatus access = rxFilterSwitchWriteAccess(sw);
  if (access != FeatureStatus::Ok) return access;
  if (!asciiSetRxFilterSwitch(currentProfile(), sw, on)) return failure(FeatureStatus::Failed);
  out = on;
  return FeatureStatus::Ok;
}

FeatureStatus rxFilterSwitchToggle(RxFilterSwitch sw, bool& out) {
  if (!rxFilterSwitchSupported(sw) || isFt8x7()) return FeatureStatus::Unsupported;
  bool on = false;
  const FeatureStatus status = rxFilterSwitchQuery(sw, on);
  if (status != FeatureStatus::Ok) return status;
  return rxFilterSwitchSet(sw, !on, out);
}
