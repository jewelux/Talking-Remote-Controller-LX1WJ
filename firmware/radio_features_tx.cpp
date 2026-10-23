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

static bool isYaesuAscii() {
  return currentProtocolType() == PROTO_YAESU_FTDX_ASCII;
}

// The profile's cap, and a read HamTRC has for the radio. The Yaesu ASCII radios read PR, PL,
// MG, VX, VG and VD; the profile's caps say which of them has the commands.
static bool procSupported() {
  if (!currentProfile().caps.getProc) return false;
  return currentProtocolType() == PROTO_CIV || isYaesuAscii() || currentFt8x7Model() == Ft8x7Model::Ft857;
}

FeatureStatus procQuery(bool& on) {
  if (!procSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryProc(sp, on, 800);
  else if (isYaesuAscii()) ok = asciiQueryYaesuProc(sp, on, 800);
  else ok = yaesuFt8x7QueryFlag(Ft8x7Flag::Proc, on, YAESU_CAT_REPLY_TIMEOUT_MS);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus procLevelQuery(uint8_t& level) {
  if (!procSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    ok = civQueryProcLevel(sp, raw, 800);
    level = levelRawToPercent(raw);
  } else if (isYaesuAscii()) {
    ok = asciiQueryYaesuProcLevel(sp, level, 800);
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
static FeatureStatus ft8x7CurrentMicGain(Ft8x7MicGain& gain) {
  uint8_t modeByte = 0;
  if (!yaesuCatQueryModeRawByte(modeByte, YAESU_CAT_REPLY_TIMEOUT_MS)) return failure(FeatureStatus::NoReply);
  if (!ft8x7MicGainForMode(modeByte, gain) || !yaesuFt8x7HasMicGain(gain)) return FeatureStatus::Unsupported;
  return FeatureStatus::Ok;
}

static FeatureStatus ft8x7MicGainQuery(uint8_t& level) {
  Ft8x7MicGain gain = Ft8x7MicGain::Ssb;
  const FeatureStatus status = ft8x7CurrentMicGain(gain);
  if (status != FeatureStatus::Ok) return status;
  if (!yaesuFt8x7QueryMicGain(gain, level, YAESU_CAT_REPLY_TIMEOUT_MS)) return failure(FeatureStatus::NoReply);
  return FeatureStatus::Ok;
}

static bool micGainSupported() {
  if (!currentProfile().caps.getMicGain) return false;
  return currentProtocolType() == PROTO_CIV || isYaesuAscii() || currentFt8x7Model() != Ft8x7Model::None;
}

FeatureStatus micGainQuery(uint8_t& level) {
  if (!micGainSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    ok = civQueryMicGain(sp, raw, 800);
    level = levelRawToPercent(raw);
  } else if (isYaesuAscii()) {
    ok = asciiQueryYaesuMicGain(sp, level, 800);
  } else {
    return ft8x7MicGainQuery(level);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

// ---- VOX ----

static bool voxSupported() {
  if (!currentProfile().caps.getVox) return false;
  return currentProtocolType() == PROTO_CIV || isYaesuAscii() || currentFt8x7Model() != Ft8x7Model::None;
}

FeatureStatus voxQuery(bool& on) {
  if (!voxSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryVox(sp, on, 800);
  else if (isYaesuAscii()) ok = asciiQueryYaesuVox(sp, on, 800);
  else ok = yaesuFt8x7QueryFlag(Ft8x7Flag::Vox, on, YAESU_CAT_REPLY_TIMEOUT_MS);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus voxGainQuery(uint8_t& level) {
  if (!voxSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    ok = civQueryVoxGain(sp, raw, 800);
    level = levelRawToPercent(raw);
  } else if (isYaesuAscii()) {
    ok = asciiQueryYaesuVoxGain(sp, level, 800);
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
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryVoxDelayMs(sp, ms, 800);
  else if (isYaesuAscii()) ok = asciiQueryYaesuVoxDelayMs(sp, ms, 800);
  else ok = yaesuFt8x7QueryVoxDelayMs(ms, YAESU_CAT_REPLY_TIMEOUT_MS);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

// ---- Speech processor, mic gain and VOX writes ----

// Where a write goes: over CAT where the profile has the write cap, or, experimentally
// (caps.setRadioMenu), to the FT-8x7 EEPROM. ft8x7: the FT-8x7 model keeps the setting.
enum class TxWrite : uint8_t { None, Civ, YaesuAscii, Ft8x7 };

static TxWrite txWriteTarget(bool readable, bool capSet, bool ft8x7) {
  if (!readable) return TxWrite::None;
  if (currentProtocolType() == PROTO_CIV) return capSet ? TxWrite::Civ : TxWrite::None;
  if (isYaesuAscii()) return capSet ? TxWrite::YaesuAscii : TxWrite::None;
  return ft8x7 && currentProfile().caps.setRadioMenu ? TxWrite::Ft8x7 : TxWrite::None;
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

// The CAT level read, plus delta, kept within 0..100, then set.
template <typename Query, typename CivSet, typename AsciiSet>
static FeatureStatus catLevelStep(TxWrite target, int delta, Query query, CivSet civSet, AsciiSet asciiSet,
                                  uint8_t& levelOut) {
  uint8_t level = 0;
  const FeatureStatus status = query(level);
  if (status != FeatureStatus::Ok) return status;
  return catLevelSet(target, ft8x7PercentStep(level, delta), civSet, asciiSet, levelOut);
}

static TxWrite procWriteTarget() {
  return txWriteTarget(procSupported(), currentProfile().caps.setProc, currentIsFt857Family());
}

FeatureStatus procSet(bool on) {
  const RadioProfile& sp = currentProfile();
  switch (procWriteTarget()) {
    case TxWrite::None: return FeatureStatus::Unsupported;
    case TxWrite::Civ: return catWriteStatus(civSetProc(sp, on));
    case TxWrite::YaesuAscii: return catWriteStatus(asciiSetYaesuProc(sp, on));
    case TxWrite::Ft8x7: break;
  }
  return eepromWriteStatus(yaesuFt8x7SetFlag(Ft8x7Flag::Proc, on, kFt8x7TimeoutMs));
}

FeatureStatus procToggle(bool& on) {
  const TxWrite target = procWriteTarget();
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
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
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
  if (target != TxWrite::Ft8x7) return catLevelSet(target, level, civSetProcLevel, asciiSetYaesuProcLevel, levelOut);
  uint16_t read = 0;
  const FeatureStatus status =
      eepromWriteStatus(yaesuFt857SetLevel(YaesuFt857Level::ProcLevel, level, read, kFt8x7TimeoutMs));
  if (status == FeatureStatus::Ok) levelOut = (uint8_t)read;
  return status;
}

FeatureStatus procLevelStep(int delta, uint8_t& levelOut) {
  const TxWrite target = procWriteTarget();
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
  if (target != TxWrite::Ft8x7) {
    return catLevelStep(target, delta, procLevelQuery, civSetProcLevel, asciiSetYaesuProcLevel, levelOut);
  }
  uint16_t read = 0;
  const FeatureStatus status =
      eepromWriteStatus(yaesuFt857StepLevel(YaesuFt857Level::ProcLevel, delta, read, kFt8x7TimeoutMs));
  if (status == FeatureStatus::Ok) levelOut = (uint8_t)read;
  return status;
}

static TxWrite micGainWriteTarget() {
  return txWriteTarget(micGainSupported(), currentProfile().caps.setMicGain, true);
}

FeatureStatus micGainSet(uint8_t level, uint8_t& levelOut) {
  const TxWrite target = micGainWriteTarget();
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
  if (target != TxWrite::Ft8x7) return catLevelSet(target, level, civSetMicGain, asciiSetYaesuMicGain, levelOut);
  // The FT-8x7 keeps a mic gain per mode: the current mode's, as micGainQuery picks it.
  Ft8x7MicGain gain = Ft8x7MicGain::Ssb;
  const FeatureStatus status = ft8x7CurrentMicGain(gain);
  if (status != FeatureStatus::Ok) return status;
  return eepromWriteStatus(yaesuFt8x7SetMicGain(gain, level, levelOut, kFt8x7TimeoutMs));
}

FeatureStatus micGainStep(int delta, uint8_t& levelOut) {
  const TxWrite target = micGainWriteTarget();
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
  if (target != TxWrite::Ft8x7) {
    return catLevelStep(target, delta, micGainQuery, civSetMicGain, asciiSetYaesuMicGain, levelOut);
  }
  Ft8x7MicGain gain = Ft8x7MicGain::Ssb;
  const FeatureStatus status = ft8x7CurrentMicGain(gain);
  if (status != FeatureStatus::Ok) return status;
  return eepromWriteStatus(yaesuFt8x7StepMicGain(gain, delta, levelOut, kFt8x7TimeoutMs));
}

static TxWrite voxWriteTarget() {
  return txWriteTarget(voxSupported(), currentProfile().caps.setVox, true);
}

FeatureStatus voxSet(bool on) {
  const RadioProfile& sp = currentProfile();
  switch (voxWriteTarget()) {
    case TxWrite::None: return FeatureStatus::Unsupported;
    case TxWrite::Civ: return catWriteStatus(civSetVox(sp, on));
    case TxWrite::YaesuAscii: return catWriteStatus(asciiSetYaesuVox(sp, on));
    case TxWrite::Ft8x7: break;
  }
  return eepromWriteStatus(yaesuFt8x7SetFlag(Ft8x7Flag::Vox, on, kFt8x7TimeoutMs));
}

FeatureStatus voxToggle(bool& on) {
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
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
  maxOut = 100;
  return true;
}

FeatureStatus voxGainSet(uint8_t level, uint8_t& levelOut) {
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
  if (target != TxWrite::Ft8x7) return catLevelSet(target, level, civSetVoxGain, asciiSetYaesuVoxGain, levelOut);
  return eepromWriteStatus(yaesuFt8x7SetVoxGain(level, levelOut, kFt8x7TimeoutMs));
}

FeatureStatus voxGainStep(int delta, uint8_t& levelOut) {
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::None) return FeatureStatus::Unsupported;
  if (target != TxWrite::Ft8x7) {
    return catLevelStep(target, delta, voxGainQuery, civSetVoxGain, asciiSetYaesuVoxGain, levelOut);
  }
  return eepromWriteStatus(yaesuFt8x7StepVoxGain(delta, levelOut, kFt8x7TimeoutMs));
}

bool voxDelayRange(uint16_t& minOut, uint16_t& maxOut) {
  const TxWrite target = voxWriteTarget();
  uint16_t civAddr = 0;
  if (target == TxWrite::None || (target == TxWrite::Civ && !civVoxDelayAddr(currentProfile().model, civAddr))) {
    return false;
  }
  minOut = 100;
  maxOut = 3000;
  if (target == TxWrite::Civ) {
    minOut = 0;
    maxOut = 2000;
  } else if (target == TxWrite::YaesuAscii) {
    minOut = 30;
  } else if (currentIsFt817Family()) {
    maxOut = 2500;
  }
  return true;
}

FeatureStatus voxDelaySet(uint16_t ms, uint16_t& msOut) {
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!voxDelayRange(lo, hi)) return FeatureStatus::Unsupported;
  if (ms < lo || ms > hi) return FeatureStatus::Invalid;
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::Ft8x7) return eepromWriteStatus(yaesuFt8x7SetVoxDelayMs(ms, msOut, kFt8x7TimeoutMs));
  const RadioProfile& sp = currentProfile();
  const bool ok = target == TxWrite::Civ ? civSetVoxDelayMs(sp, ms) : asciiSetYaesuVoxDelayMs(sp, ms);
  if (!ok) return failure(FeatureStatus::Failed);
  // The step the radio took.
  return voxDelayQuery(msOut);
}

FeatureStatus voxDelayStep(int delta, uint16_t& msOut) {
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!voxDelayRange(lo, hi)) return FeatureStatus::Unsupported;
  const TxWrite target = voxWriteTarget();
  if (target == TxWrite::Ft8x7) return eepromWriteStatus(yaesuFt8x7StepVoxDelay(delta, msOut, kFt8x7TimeoutMs));
  uint16_t ms = 0;
  const FeatureStatus status = voxDelayQuery(ms);
  if (status != FeatureStatus::Ok) return status;
  if (target == TxWrite::YaesuAscii) return voxDelaySet(yaesuAsciiVoxDelayStepBy(ms, delta), msOut);
  // CI-V: 100 ms steps.
  if (delta > 20) delta = 20;
  if (delta < -20) delta = -20;
  const int stepped = (int)ms + delta * 100;
  return voxDelaySet((uint16_t)(stepped < lo ? lo : stepped > hi ? hi : stepped), msOut);
}
