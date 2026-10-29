#include "radio_features.h"
#include "radio_features_internal.h"
#include "cat_setting_codec.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_civ.h"
#include "radio_catalog.h"
#include "radio_globals.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"

// ---- Speech processor ----

// The profile's cap, and a read HamTRC has for the radio. The ASCII radios read PR, PL, MG, VX,
// VG and VD; the profile's caps say which of them has the commands.
static bool procSupported(const RadioCapabilities& caps) {
  if (!caps.getProc) return false;
  return currentProtocolType() == PROTO_CIV || isAsciiRadio() || currentFt8x7Model() == Ft8x7Model::Ft857;
}

bool procSupported() { return procSupported(currentProfile().caps); }

// ---- RF power ----

static bool isAsciiRfPower() {
  return isAsciiRadio() && currentProfile().settings.rfPower != RfPowerEncoding::None;
}

static uint16_t profileMaxWatts() {
  const uint16_t max = currentProfile().rfPowerMaxWatts;
  return max ? max : 100;
}

FeatureStatus rfPowerQuery(uint16_t& tenthsOut) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getRfPower) return FeatureStatus::Unsupported;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    if (!civQueryRfPowerLevel(sp, raw, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
    tenthsOut = rfPowerRawToTenths(raw, profileMaxWatts());
    return FeatureStatus::Ok;
  }
  if (!isAsciiRfPower()) return FeatureStatus::Unsupported;
  return asciiQueryRfPowerTenths(sp, tenthsOut, kCatReplyTimeoutMs) ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

bool rfPowerRange(uint16_t& minOut, uint16_t& maxOut) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getRfPower || !sp.caps.setRfPower) return false;
  if (currentProtocolType() != PROTO_CIV && !isAsciiRfPower()) return false;
  minOut = sp.settings.rfPowerMinWatts;
  maxOut = profileMaxWatts();
  return true;
}

FeatureStatus rfPowerSet(uint16_t watts, uint16_t& tenthsOut) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!rfPowerRange(min, max)) return FeatureStatus::Unsupported;
  if (watts < min || watts > max) return FeatureStatus::Invalid;
  const RadioProfile& sp = currentProfile();
  if (currentProtocolType() == PROTO_CIV) {
    const uint16_t raw = rfPowerWattsToRaw(watts, max);
    if (!civSetRfPowerLevel(sp, raw)) return failure(FeatureStatus::Failed);
    tenthsOut = rfPowerRawToTenths(raw, max);
    return FeatureStatus::Ok;
  }
  if (!asciiSetRfPowerWatts(sp, watts)) return failure(FeatureStatus::Failed);
  tenthsOut = (uint16_t)(watts * 10);
  return FeatureStatus::Ok;
}

// ---- CW break-in and keyer ----

// The FT-8x7 settings written to the EEPROM, experimentally.
static bool ft8x7CanWriteMenu(const RadioCapabilities& caps) {
  return isFt8x7() && caps.setRadioMenu && currentFt8x7Model() != Ft8x7Model::None;
}

static FeatureStatus ft8x7MenuWriteAccess() {
  return writeAccess([](const RadioCapabilities& caps) { return ft8x7CanWriteMenu(caps); });
}

static FeatureStatus ft8x7FlagQuery(Ft8x7Setting setting, bool& on) {
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7SettingQuery(setting, state);
  if (status == FeatureStatus::Ok) on = state.on;
  return status;
}

static FeatureStatus ft8x7FlagWrite(Ft8x7Setting setting, const bool* on, bool& out) {
  Ft8x7SettingState state;
  const FeatureStatus status = on ? ft8x7FlagSettingSet(setting, *on, state) : ft8x7FlagSettingToggle(setting, state);
  if (status == FeatureStatus::Ok) out = state.on;
  return status;
}

static bool isYaesuAscii() {
  return currentProtocolType() == PROTO_YAESU_FTDX_ASCII;
}

FeatureStatus breakInQuery(BreakInMode& out) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getBreakIn) return FeatureStatus::Unsupported;
  if (currentProtocolType() == PROTO_CIV) {
    return civQueryBreakIn(sp, out, kCatReplyTimeoutMs) ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
  }
  bool on = false;
  FeatureStatus status = FeatureStatus::Unsupported;
  if (isYaesuAscii()) {
    status = asciiQueryBreakIn(sp, on, kCatReplyTimeoutMs) ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
  } else if (isFt8x7()) {
    status = ft8x7FlagQuery(Ft8x7Setting::BreakIn, on);
  }
  if (status == FeatureStatus::Ok) out = on ? BreakInMode::On : BreakInMode::Off;
  return status;
}

static bool breakInCanSet(BreakInMode mode, const RadioCapabilities& caps) {
  if (!caps.getBreakIn) return false;
  if (currentProtocolType() == PROTO_CIV) return caps.setBreakIn;
  const bool onOff = mode == BreakInMode::Off || mode == BreakInMode::On;
  if (isYaesuAscii()) return caps.setBreakIn && onOff;
  return ft8x7CanWriteMenu(caps) && onOff;
}

bool breakInCanSet(BreakInMode mode) { return breakInCanSet(mode, currentProfile().caps); }

FeatureStatus breakInWriteAccess(BreakInMode mode) {
  return writeAccess([mode](const RadioCapabilities& caps) { return breakInCanSet(mode, caps); });
}

FeatureStatus breakInSet(BreakInMode mode, BreakInMode& out) {
  if (!breakInCanSet(mode)) return breakInWriteAccess(mode);
  const RadioProfile& sp = currentProfile();
  if (currentProtocolType() == PROTO_CIV) {
    const BreakInMode want = mode == BreakInMode::On ? BreakInMode::Semi : mode;
    if (!civSetBreakIn(sp, want)) return failure(FeatureStatus::Failed);
    out = want;
    return FeatureStatus::Ok;
  }
  const bool on = mode != BreakInMode::Off;
  if (isYaesuAscii()) {
    if (!asciiSetBreakIn(sp, on)) return failure(FeatureStatus::Failed);
    out = mode;
    return FeatureStatus::Ok;
  }
  bool readBack = false;
  const FeatureStatus status = ft8x7FlagWrite(Ft8x7Setting::BreakIn, &on, readBack);
  if (status == FeatureStatus::Ok) out = readBack ? BreakInMode::On : BreakInMode::Off;
  return status;
}

FeatureStatus breakInToggle(BreakInMode& out) {
  if (!breakInCanSet(BreakInMode::On)) return breakInWriteAccess(BreakInMode::On);
  if (isFt8x7()) {
    bool readBack = false;
    const FeatureStatus status = ft8x7FlagWrite(Ft8x7Setting::BreakIn, nullptr, readBack);
    if (status == FeatureStatus::Ok) out = readBack ? BreakInMode::On : BreakInMode::Off;
    return status;
  }
  BreakInMode now = BreakInMode::Off;
  const FeatureStatus status = breakInQuery(now);
  if (status != FeatureStatus::Ok) return status;
  return breakInSet(now == BreakInMode::Off ? BreakInMode::On : BreakInMode::Off, out);
}

FeatureStatus keyerQuery(bool& on) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getKeyer) return FeatureStatus::Unsupported;
  if (isYaesuAscii()) return asciiQueryKeyer(sp, on, kCatReplyTimeoutMs) ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
  if (isFt8x7()) return ft8x7FlagQuery(Ft8x7Setting::Keyer, on);
  return FeatureStatus::Unsupported;
}

FeatureStatus keyerSet(bool on, bool& out) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getKeyer) return FeatureStatus::Unsupported;
  if (isYaesuAscii()) {
    if (!sp.caps.setKeyer) return FeatureStatus::Unsupported;
    if (!asciiSetKeyer(sp, on)) return failure(FeatureStatus::Failed);
    out = on;
    return FeatureStatus::Ok;
  }
  if (!ft8x7CanWriteMenu(currentProfile().caps)) return ft8x7MenuWriteAccess();
  return ft8x7FlagWrite(Ft8x7Setting::Keyer, &on, out);
}

FeatureStatus keyerToggle(bool& out) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getKeyer) return FeatureStatus::Unsupported;
  if (isYaesuAscii()) {
    if (!sp.caps.setKeyer) return FeatureStatus::Unsupported;
    bool now = false;
    const FeatureStatus status = keyerQuery(now);
    if (status != FeatureStatus::Ok) return status;
    return keyerSet(!now, out);
  }
  if (!ft8x7CanWriteMenu(currentProfile().caps)) return ft8x7MenuWriteAccess();
  return ft8x7FlagWrite(Ft8x7Setting::Keyer, nullptr, out);
}

bool breakInSupported() {
  if (!currentProfile().caps.getBreakIn) return false;
  return currentProtocolType() == PROTO_CIV || isYaesuAscii() || ft8x7SettingAvailable(Ft8x7Setting::BreakIn);
}

static bool keyerSupported(const RadioCapabilities& caps) {
  if (!caps.getKeyer) return false;
  return isYaesuAscii() || ft8x7SettingAvailable(Ft8x7Setting::Keyer);
}

bool keyerSupported() { return keyerSupported(currentProfile().caps); }

static bool keyerCanSet(const RadioCapabilities& caps) {
  if (!keyerSupported(caps)) return false;
  return isYaesuAscii() ? caps.setKeyer : ft8x7CanWriteMenu(caps);
}

bool keyerCanSet() { return keyerCanSet(currentProfile().caps); }

FeatureStatus keyerWriteAccess() {
  return writeAccess([](const RadioCapabilities& caps) { return keyerCanSet(caps); });
}

// ---- ALC ----

FeatureStatus alcQuery(uint8_t& percentOut) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getAlc) return FeatureStatus::Unsupported;
  uint16_t raw = 0;
  if (currentProtocolType() == PROTO_CIV) {
    if (!civQueryAlcRaw(sp, raw, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
    percentOut = meterPercent(raw, 120);
    return FeatureStatus::Ok;
  }
  if (currentProtocolType() != PROTO_YAESU_FTDX_ASCII) return FeatureStatus::Unsupported;
  if (!asciiQueryAlcRaw(sp, raw, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
  percentOut = meterPercent(raw, 255);
  return FeatureStatus::Ok;
}

// ---- CW levels ----

static Ft8x7Setting ft8x7CwSetting(CwLevel level) {
  switch (level) {
    case CwLevel::Pitch: return Ft8x7Setting::CwPitch;
    case CwLevel::Delay: return Ft8x7Setting::CwDelay;
    case CwLevel::Speed: break;
  }
  return Ft8x7Setting::CwSpeed;
}

static bool cwCanGet(CwLevel level, const RadioCapabilities& caps) {
  switch (level) {
    case CwLevel::Pitch: return caps.getCwPitch;
    case CwLevel::Delay: return caps.getCwDelay;
    case CwLevel::Speed: break;
  }
  return caps.getCwSpeed;
}

static bool cwCanSetCap(CwLevel level, const RadioCapabilities& caps) {
  switch (level) {
    case CwLevel::Pitch: return caps.setCwPitch;
    case CwLevel::Delay: return caps.setCwDelay;
    case CwLevel::Speed: break;
  }
  return caps.setCwSpeed;
}

FeatureStatus cwLevelQuery(CwLevel level, uint16_t& valueOut) {
  if (!cwCanGet(level, currentProfile().caps)) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7SettingQuery(ft8x7CwSetting(level), state);
    if (status == FeatureStatus::Ok) valueOut = state.value;
    return status;
  }
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    if (level != CwLevel::Pitch) return FeatureStatus::Unsupported;
    ok = civQueryCwPitchHz(sp, valueOut, kCatReplyTimeoutMs);
  } else if (isAsciiRadio()) {
    switch (level) {
      case CwLevel::Speed: ok = asciiQueryCwSpeed(sp, valueOut, kCatReplyTimeoutMs); break;
      case CwLevel::Pitch: ok = asciiQueryCwPitch(sp, valueOut, kCatReplyTimeoutMs); break;
      case CwLevel::Delay: ok = asciiQueryCwDelayMs(sp, valueOut, kCatReplyTimeoutMs); break;
    }
  } else {
    return FeatureStatus::Unsupported;
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

bool cwLevelSupported(CwLevel level) {
  if (!cwCanGet(level, currentProfile().caps)) return false;
  if (isFt8x7()) return ft8x7SettingAvailable(ft8x7CwSetting(level));
  if (currentProtocolType() == PROTO_CIV) return level == CwLevel::Pitch;
  return isAsciiRadio();
}

static bool cwLevelRange(CwLevel level, uint16_t& minOut, uint16_t& maxOut, const RadioCapabilities& caps) {
  if (!cwCanGet(level, caps)) return false;
  if (isFt8x7()) return ft8x7CanWriteMenu(caps) && ft8x7LevelRange(ft8x7CwSetting(level), minOut, maxOut);
  if (!cwCanSetCap(level, caps)) return false;
  const SettingEncodings& enc = currentProfile().settings;
  if (currentProtocolType() == PROTO_CIV) {
    if (level != CwLevel::Pitch) return false;
    minOut = 300;
    maxOut = 900;
    return true;
  }
  if (!isAsciiRadio()) return false;
  switch (level) {
    case CwLevel::Speed:
      if (!enc.cwSpeedMaxWpm) return false;
      minOut = enc.cwSpeedMinWpm;
      maxOut = enc.cwSpeedMaxWpm;
      return true;
    case CwLevel::Pitch:
      if (!enc.cwPitchKp) return false;
      minOut = 300;
      maxOut = 1050;
      return true;
    case CwLevel::Delay:
      if (enc.cwDelay == VoxDelayEncoding::None) return false;
      minOut = enc.cwDelayMinMs;
      maxOut = enc.cwDelayMaxMs;
      return true;
  }
  return false;
}

bool cwLevelRange(CwLevel level, uint16_t& minOut, uint16_t& maxOut) {
  return cwLevelRange(level, minOut, maxOut, currentProfile().caps);
}

FeatureStatus cwLevelWriteAccess(CwLevel level) {
  return writeAccess([level](const RadioCapabilities& caps) {
    uint16_t min = 0;
    uint16_t max = 0;
    return cwLevelRange(level, min, max, caps);
  });
}

// The value the radio keeps nearest value: Icom pitch 5 Hz, Yaesu KP 10 Hz, the delay its step.
static uint16_t cwLevelNearest(CwLevel level, uint16_t value) {
  const SettingEncodings& enc = currentProfile().settings;
  if (level == CwLevel::Pitch) {
    const uint16_t step = currentProtocolType() == PROTO_CIV ? 5 : 10;
    return (uint16_t)((value + step / 2) / step * step);
  }
  if (level == CwLevel::Delay) {
    if (enc.cwDelay == VoxDelayEncoding::YaesuSteps) return yaesuAsciiVoxDelayStepMs(yaesuAsciiVoxDelayNearestStep(value));
    return asciiVoxDelayMsRound(value, enc.cwDelayMinMs, enc.cwDelayMaxMs, enc.cwDelayStepMs);
  }
  return value;
}

FeatureStatus cwLevelSet(CwLevel level, uint16_t value, uint16_t& valueOut) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!cwLevelRange(level, min, max)) return cwLevelWriteAccess(level);
  if (value < min || value > max) return FeatureStatus::Invalid;
  if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7LevelSet(ft8x7CwSetting(level), value, state);
    if (status == FeatureStatus::Ok) valueOut = state.value;
    return status;
  }
  const RadioProfile& sp = currentProfile();
  const uint16_t want = cwLevelNearest(level, value);
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    ok = civSetCwPitchHz(sp, want);
  } else {
    switch (level) {
      case CwLevel::Speed: ok = asciiSetCwSpeed(sp, want); break;
      case CwLevel::Pitch: ok = asciiSetCwPitch(sp, want); break;
      case CwLevel::Delay: ok = asciiSetCwDelayMs(sp, want); break;
    }
  }
  if (!ok) return failure(FeatureStatus::Failed);
  valueOut = want;
  return FeatureStatus::Ok;
}

FeatureStatus cwLevelStep(CwLevel level, int delta, uint16_t& valueOut) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!cwLevelRange(level, min, max)) return cwLevelWriteAccess(level);
  if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7LevelStep(ft8x7CwSetting(level), delta, state);
    if (status == FeatureStatus::Ok) valueOut = state.value;
    return status;
  }
  uint16_t now = 0;
  const FeatureStatus status = cwLevelQuery(level, now);
  if (status != FeatureStatus::Ok) return status;
  const SettingEncodings& enc = currentProfile().settings;
  int next = now;
  if (level == CwLevel::Speed) next = now + delta;
  else if (level == CwLevel::Pitch) next = now + delta * 10;
  else if (enc.cwDelay == VoxDelayEncoding::YaesuSteps) next = yaesuAsciiVoxDelayStepBy(now, delta);
  else next = asciiVoxDelayMsStepBy(now, delta, enc.cwDelayMinMs, enc.cwDelayMaxMs, enc.cwDelayStepMs);
  if (next < min) next = min;
  if (next > max) next = max;
  return cwLevelSet(level, (uint16_t)next, valueOut);
}

FeatureStatus procQuery(bool& on) {
  if (!procSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryProc(sp, on, kCatReplyTimeoutMs);
  else if (isAsciiRadio()) ok = asciiQueryProc(sp, on, kCatReplyTimeoutMs);
  else ok = yaesuFt8x7QueryFlag(Ft8x7Flag::Proc, on, YAESU_CAT_REPLY_TIMEOUT_MS);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus procLevelQuery(uint8_t& level) {
  if (!procSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    ok = civQueryProcLevel(sp, raw, kCatReplyTimeoutMs);
    level = levelRawToPercent(raw);
  } else if (isAsciiRadio()) {
    ok = asciiQueryProcLevel(sp, level, kCatReplyTimeoutMs);
  } else {
    uint16_t value = 0;
    ok = yaesuFt857QueryLevel(YaesuFt857Level::ProcLevel, value, YAESU_CAT_REPLY_TIMEOUT_MS);
    level = value > 100 ? 100 : (uint8_t)value;
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

// ---- Mic gain ----

// The FT-8x7 keeps a mic gain per mode; the raw mode byte also tells PKT and the narrow modes
// apart, which the profile's mode codes do not.
FeatureStatus ft8x7MicGainMenu(Ft8x7MicGain& gain) {
  uint8_t modeByte = 0;
  if (!yaesuCatQueryModeRawByte(modeByte, YAESU_CAT_REPLY_TIMEOUT_MS)) return failure(FeatureStatus::NoReply);
  if (!ft8x7MicGainForMode(modeByte, gain) || !yaesuFt8x7HasMicGain(gain)) return FeatureStatus::Unsupported;
  return FeatureStatus::Ok;
}

static FeatureStatus ft8x7MicGainQuery(uint8_t& level) {
  Ft8x7MicGain gain = Ft8x7MicGain::Ssb;
  const FeatureStatus status = ft8x7MicGainMenu(gain);
  if (status != FeatureStatus::Ok) return status;
  if (!yaesuFt8x7QueryMicGain(gain, level, YAESU_CAT_REPLY_TIMEOUT_MS)) return failure(FeatureStatus::NoReply);
  return FeatureStatus::Ok;
}

static bool micGainSupported(const RadioCapabilities& caps) {
  if (!caps.getMicGain) return false;
  return currentProtocolType() == PROTO_CIV || isAsciiRadio() || currentFt8x7Model() != Ft8x7Model::None;
}

bool micGainSupported() { return micGainSupported(currentProfile().caps); }

FeatureStatus micGainQuery(uint8_t& level) {
  if (!micGainSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    ok = civQueryMicGain(sp, raw, kCatReplyTimeoutMs);
    level = levelRawToPercent(raw);
  } else if (isAsciiRadio()) {
    ok = asciiQueryMicGain(sp, level, kCatReplyTimeoutMs);
  } else {
    return ft8x7MicGainQuery(level);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

// ---- VOX ----

static bool voxSupported(const RadioCapabilities& caps) {
  if (!caps.getVox) return false;
  return currentProtocolType() == PROTO_CIV || isAsciiRadio() || currentFt8x7Model() != Ft8x7Model::None;
}

bool voxSupported() { return voxSupported(currentProfile().caps); }

FeatureStatus voxQuery(bool& on) {
  if (!voxSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryVox(sp, on, kCatReplyTimeoutMs);
  else if (isAsciiRadio()) ok = asciiQueryVox(sp, on, kCatReplyTimeoutMs);
  else ok = yaesuFt8x7QueryFlag(Ft8x7Flag::Vox, on, YAESU_CAT_REPLY_TIMEOUT_MS);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus voxGainQuery(uint8_t& level) {
  if (!voxSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    ok = civQueryVoxGain(sp, raw, kCatReplyTimeoutMs);
    level = levelRawToPercent(raw);
  } else if (isAsciiRadio()) {
    ok = asciiQueryVoxGain(sp, level, kCatReplyTimeoutMs);
  } else {
    ok = yaesuFt8x7QueryVoxGain(level, YAESU_CAT_REPLY_TIMEOUT_MS);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus voxDelayQuery(uint16_t& ms) {
  if (!voxSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  uint16_t civAddr = 0;
  if (currentProtocolType() == PROTO_CIV && !civVoxDelayAddr(sp.model, civAddr)) return FeatureStatus::Unsupported;
  if (isAsciiRadio() && sp.settings.voxDelay == VoxDelayEncoding::None) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryVoxDelayMs(sp, ms, kCatReplyTimeoutMs);
  else if (isAsciiRadio()) ok = asciiQueryVoxDelayMs(sp, ms, kCatReplyTimeoutMs);
  else ok = yaesuFt8x7QueryVoxDelayMs(ms, YAESU_CAT_REPLY_TIMEOUT_MS);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

// ---- Speech processor, mic gain and VOX writes ----

// Where a write goes: over CAT where the profile has the write cap, or, experimentally
// (caps.setRadioMenu), to the FT-8x7 EEPROM. ft8x7: the FT-8x7 model keeps the setting.
enum class TxWrite : uint8_t { None, Civ, Ascii, Ft8x7 };

static TxWrite txWriteTarget(bool readable, bool capSet, bool ft8x7, const RadioCapabilities& caps) {
  if (!readable) return TxWrite::None;
  if (currentProtocolType() == PROTO_CIV) return capSet ? TxWrite::Civ : TxWrite::None;
  if (isAsciiRadio()) return capSet ? TxWrite::Ascii : TxWrite::None;
  return ft8x7 && caps.setRadioMenu ? TxWrite::Ft8x7 : TxWrite::None;
}

// A CAT write, which reads the setting back.
static FeatureStatus catWriteStatus(bool ok) {
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::Failed);
}

// Sets a CAT level of 0..100 percent: CI-V scaled to 0..255, Yaesu ASCII as it is. levelOut is the
// level set.
template <typename CivSet, typename AsciiSet>
static FeatureStatus catLevelSet(TxWrite target, uint8_t level, CivSet civSet, AsciiSet asciiSet,
                                 uint8_t& levelOut) {
  if (level > 100) return FeatureStatus::Invalid;
  const RadioProfile& sp = currentProfile();
  const bool ok = target == TxWrite::Civ ? civSet(sp, levelPercentToRaw(level)) : asciiSet(sp, level);
  if (!ok) return failure(FeatureStatus::Failed);
  levelOut = level;
  return FeatureStatus::Ok;
}

// The CAT level read, plus delta, kept within 0..max, then set.
template <typename Query, typename CivSet, typename AsciiSet>
static FeatureStatus catLevelStep(TxWrite target, int delta, Query query, CivSet civSet, AsciiSet asciiSet,
                                  uint8_t& levelOut, uint8_t max = 100) {
  uint8_t level = 0;
  const FeatureStatus status = query(level);
  if (status != FeatureStatus::Ok) return status;
  const uint8_t stepped = percentStep(level, delta);
  return catLevelSet(target, stepped > max ? max : stepped, civSet, asciiSet, levelOut);
}

static TxWrite procWriteTarget(const RadioCapabilities& caps) {
  return txWriteTarget(procSupported(caps), caps.setProc, currentIsFt857Family(), caps);
}

static TxWrite procWriteTarget() { return procWriteTarget(currentProfile().caps); }

bool procCanSet() { return procWriteTarget() != TxWrite::None; }

FeatureStatus procWriteAccess() {
  return writeAccess([](const RadioCapabilities& caps) { return procWriteTarget(caps) != TxWrite::None; });
}

FeatureStatus procSet(bool on) {
  const RadioProfile& sp = currentProfile();
  switch (procWriteTarget()) {
    case TxWrite::None: return procWriteAccess();
    case TxWrite::Civ: return catWriteStatus(civSetProc(sp, on));
    case TxWrite::Ascii: return catWriteStatus(asciiSetProc(sp, on));
    case TxWrite::Ft8x7: break;
  }
  return eepromWriteStatus(yaesuFt8x7SetFlag(Ft8x7Flag::Proc, on, kFt8x7TimeoutMs));
}

FeatureStatus procToggle(bool& on) {
  const TxWrite target = procWriteTarget();
  if (target == TxWrite::None) return procWriteAccess();
  if (target == TxWrite::Ft8x7) return ft8x7ToggleFlag(Ft8x7Flag::Proc, on);
  bool current = false;
  const FeatureStatus status = procQuery(current);
  if (status != FeatureStatus::Ok) return status;
  const FeatureStatus set = procSet(!current);
  if (set == FeatureStatus::Ok) on = !current;
  return set;
}

FeatureStatus procLevelSet(uint8_t level, uint8_t& levelOut) {
  const TxWrite target = procWriteTarget();
  if (target == TxWrite::None) return procWriteAccess();
  if (target != TxWrite::Ft8x7) return catLevelSet(target, level, civSetProcLevel, asciiSetProcLevel, levelOut);
  uint16_t read = 0;
  const FeatureStatus status =
      eepromWriteStatus(yaesuFt857SetLevel(YaesuFt857Level::ProcLevel, level, read, kFt8x7TimeoutMs));
  if (status == FeatureStatus::Ok) levelOut = (uint8_t)read;
  return status;
}

FeatureStatus procLevelStep(int delta, uint8_t& levelOut) {
  const TxWrite target = procWriteTarget();
  if (target == TxWrite::None) return procWriteAccess();
  if (target != TxWrite::Ft8x7) {
    return catLevelStep(target, delta, procLevelQuery, civSetProcLevel, asciiSetProcLevel, levelOut);
  }
  uint16_t read = 0;
  const FeatureStatus status =
      eepromWriteStatus(yaesuFt857StepLevel(YaesuFt857Level::ProcLevel, delta, read, kFt8x7TimeoutMs));
  if (status == FeatureStatus::Ok) levelOut = (uint8_t)read;
  return status;
}

static TxWrite micGainWriteTarget(const RadioCapabilities& caps) {
  return txWriteTarget(micGainSupported(caps), caps.setMicGain, true, caps);
}

static TxWrite micGainWriteTarget() { return micGainWriteTarget(currentProfile().caps); }

bool micGainCanSet() { return micGainWriteTarget() != TxWrite::None; }

FeatureStatus micGainWriteAccess() {
  return writeAccess([](const RadioCapabilities& caps) { return micGainWriteTarget(caps) != TxWrite::None; });
}

FeatureStatus ft8x7MicGainSet(Ft8x7MicGain gain, uint8_t level, uint8_t& levelOut) {
  if (micGainWriteTarget() != TxWrite::Ft8x7) {
    return writeAccess([](const RadioCapabilities& caps) { return micGainWriteTarget(caps) == TxWrite::Ft8x7; });
  }
  return eepromWriteStatus(yaesuFt8x7SetMicGain(gain, level, levelOut, kFt8x7TimeoutMs));
}

FeatureStatus micGainSet(uint8_t level, uint8_t& levelOut) {
  const TxWrite target = micGainWriteTarget();
  if (target == TxWrite::None) return micGainWriteAccess();
  if (target != TxWrite::Ft8x7) return catLevelSet(target, level, civSetMicGain, asciiSetMicGain, levelOut);
  // The FT-8x7 keeps a mic gain per mode: the current mode's, as micGainQuery picks it.
  Ft8x7MicGain gain = Ft8x7MicGain::Ssb;
  const FeatureStatus status = ft8x7MicGainMenu(gain);
  if (status != FeatureStatus::Ok) return status;
  return eepromWriteStatus(yaesuFt8x7SetMicGain(gain, level, levelOut, kFt8x7TimeoutMs));
}

FeatureStatus micGainStep(int delta, uint8_t& levelOut) {
  const TxWrite target = micGainWriteTarget();
  if (target == TxWrite::None) return micGainWriteAccess();
  if (target != TxWrite::Ft8x7) {
    return catLevelStep(target, delta, micGainQuery, civSetMicGain, asciiSetMicGain, levelOut);
  }
  Ft8x7MicGain gain = Ft8x7MicGain::Ssb;
  const FeatureStatus status = ft8x7MicGainMenu(gain);
  if (status != FeatureStatus::Ok) return status;
  return eepromWriteStatus(yaesuFt8x7StepMicGain(gain, delta, levelOut, kFt8x7TimeoutMs));
}

static TxWrite voxWriteTarget(const RadioCapabilities& caps) {
  return txWriteTarget(voxSupported(caps), caps.setVox, true, caps);
}

static TxWrite voxWriteTarget() { return voxWriteTarget(currentProfile().caps); }

FeatureStatus voxWriteAccess() {
  return writeAccess([](const RadioCapabilities& caps) { return voxWriteTarget(caps) != TxWrite::None; });
}

FeatureStatus voxSet(bool on) {
  const RadioProfile& sp = currentProfile();
  switch (voxWriteTarget()) {
    case TxWrite::None: return voxWriteAccess();
    case TxWrite::Civ: return catWriteStatus(civSetVox(sp, on));
    case TxWrite::Ascii: return catWriteStatus(asciiSetVox(sp, on));
    case TxWrite::Ft8x7: break;
  }
  return eepromWriteStatus(yaesuFt8x7SetFlag(Ft8x7Flag::Vox, on, kFt8x7TimeoutMs));
}

FeatureStatus voxToggle(bool& on) {
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::None) return voxWriteAccess();
  if (target == TxWrite::Ft8x7) return ft8x7ToggleFlag(Ft8x7Flag::Vox, on);
  bool current = false;
  const FeatureStatus status = voxQuery(current);
  if (status != FeatureStatus::Ok) return status;
  const FeatureStatus set = voxSet(!current);
  if (set == FeatureStatus::Ok) on = !current;
  return set;
}

bool voxGainRange(uint8_t& minOut, uint8_t& maxOut) {
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::None) return false;
  minOut = target == TxWrite::Ft8x7 ? 1 : 0;
  maxOut = target == TxWrite::Ascii ? currentProfile().settings.voxGainMax : 100;
  return true;
}

FeatureStatus voxGainSet(uint8_t level, uint8_t& levelOut) {
  uint8_t lo = 0;
  uint8_t hi = 0;
  if (!voxGainRange(lo, hi)) return voxWriteAccess();
  if (level < lo || level > hi) return FeatureStatus::Invalid;
  const TxWrite target = voxWriteTarget();
  if (target != TxWrite::Ft8x7) return catLevelSet(target, level, civSetVoxGain, asciiSetVoxGain, levelOut);
  return eepromWriteStatus(yaesuFt8x7SetVoxGain(level, levelOut, kFt8x7TimeoutMs));
}

FeatureStatus voxGainStep(int delta, uint8_t& levelOut) {
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::None) return voxWriteAccess();
  if (target != TxWrite::Ft8x7) {
    uint8_t lo = 0;
    uint8_t hi = 100;
    (void)voxGainRange(lo, hi);
    return catLevelStep(target, delta, voxGainQuery, civSetVoxGain, asciiSetVoxGain, levelOut, hi);
  }
  return eepromWriteStatus(yaesuFt8x7StepVoxGain(delta, levelOut, kFt8x7TimeoutMs));
}

static bool voxDelayRange(uint16_t& minOut, uint16_t& maxOut, const RadioCapabilities& caps) {
  const TxWrite target = voxWriteTarget(caps);
  const RadioProfile& sp = currentProfile();
  uint16_t civAddr = 0;
  if (target == TxWrite::None || (target == TxWrite::Civ && !civVoxDelayAddr(sp.model, civAddr)) ||
      (target == TxWrite::Ascii && sp.settings.voxDelay == VoxDelayEncoding::None)) {
    return false;
  }
  minOut = 100;
  maxOut = 3000;
  if (target == TxWrite::Civ) {
    minOut = 0;
    maxOut = 2000;
  } else if (target == TxWrite::Ascii) {
    minOut = sp.settings.voxDelayMinMs;
    maxOut = sp.settings.voxDelayMaxMs;
  } else if (currentIsFt817Family()) {
    maxOut = 2500;
  }
  return true;
}

bool voxDelayRange(uint16_t& minOut, uint16_t& maxOut) {
  return voxDelayRange(minOut, maxOut, currentProfile().caps);
}

FeatureStatus voxDelayWriteAccess() {
  return writeAccess([](const RadioCapabilities& caps) {
    uint16_t min = 0;
    uint16_t max = 0;
    return voxDelayRange(min, max, caps);
  });
}

FeatureStatus voxDelaySet(uint16_t ms, uint16_t& msOut) {
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!voxDelayRange(lo, hi)) return voxDelayWriteAccess();
  if (ms < lo || ms > hi) return FeatureStatus::Invalid;
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::Ft8x7) return eepromWriteStatus(yaesuFt8x7SetVoxDelayMs(ms, msOut, kFt8x7TimeoutMs));
  const RadioProfile& sp = currentProfile();
  const bool ok = target == TxWrite::Civ ? civSetVoxDelayMs(sp, ms) : asciiSetVoxDelayMs(sp, ms);
  if (!ok) return failure(FeatureStatus::Failed);
  // The step the radio took.
  return voxDelayQuery(msOut);
}

FeatureStatus voxDelayStep(int delta, uint16_t& msOut) {
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!voxDelayRange(lo, hi)) return voxDelayWriteAccess();
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::Ft8x7) return eepromWriteStatus(yaesuFt8x7StepVoxDelay(delta, msOut, kFt8x7TimeoutMs));
  uint16_t ms = 0;
  const FeatureStatus status = voxDelayQuery(ms);
  if (status != FeatureStatus::Ok) return status;
  const SettingEncodings& enc = currentProfile().settings;
  if (target == TxWrite::Ascii && enc.voxDelay == VoxDelayEncoding::YaesuSteps) {
    return voxDelaySet(yaesuAsciiVoxDelayStepBy(ms, delta), msOut);
  }
  if (target == TxWrite::Ascii) {
    return voxDelaySet(asciiVoxDelayMsStepBy(ms, delta, lo, hi, enc.voxDelayStepMs), msOut);
  }
  // CI-V: 100 ms steps.
  if (delta > 20) delta = 20;
  if (delta < -20) delta = -20;
  const int stepped = (int)ms + delta * 100;
  return voxDelaySet((uint16_t)(stepped < lo ? lo : stepped > hi ? hi : stepped), msOut);
}
