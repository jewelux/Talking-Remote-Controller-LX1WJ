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

// ---- FT-8x7 EEPROM settings ----

const char* ft8x7SettingLabel(Ft8x7Setting setting) {
  switch (setting) {
    case Ft8x7Setting::Agc: return "AGC?";
    case Ft8x7Setting::Ipo: return "IPO?";
    case Ft8x7Setting::Att: return "ATT?";
    case Ft8x7Setting::Nar: return "NAR?";
    case Ft8x7Setting::Dbf: return "DBF?";
    case Ft8x7Setting::BreakIn: return "BK?";
    case Ft8x7Setting::Keyer: return "KYR?";
    case Ft8x7Setting::RfPower: return "RFPOWER?";
    case Ft8x7Setting::Menu: return "MENU?";
    case Ft8x7Setting::Row: return "ROW?";
    case Ft8x7Setting::IfShift: return "IFSHIFT?";
    case Ft8x7Setting::NrLevel: return "NRLEVEL?";
    case Ft8x7Setting::NbLevel: return "NBLEVEL?";
    case Ft8x7Setting::BpfWidth: return "BPF?";
    case Ft8x7Setting::LowCut: return "HPF?";
    case Ft8x7Setting::HighCut: return "LPF?";
    case Ft8x7Setting::MicEq: return "MICEQ?";
    case Ft8x7Setting::Antenna: return "ANT?";
    case Ft8x7Setting::CwSpeed: return "CWSPEED?";
    case Ft8x7Setting::CwPitch: return "CWPITCH?";
    case Ft8x7Setting::CwDelay: return "CWDELAY?";
    case Ft8x7Setting::CwWeight: return "CWWEIGHT?";
    case Ft8x7Setting::Sidetone: return "SIDETONE?";
    case Ft8x7Setting::Qsk: return "QSK?";
    case Ft8x7Setting::KeyReverse: return "KEYREV?";
    case Ft8x7Setting::CwAutoMode: return "CWAUTO?";
    case Ft8x7Setting::Paddle: return "PADDLE?";
    case Ft8x7Setting::CwBfo: return "CWBFO?";
  }
  return "?";
}

static void setFt817PowerLevel(Ft817PowerLevel level, Ft8x7SettingState& out) {
  out.hasPowerLevel = true;
  out.powerLevel = level;
  out.wattsTenths = ft817RfPowerTenths((uint8_t)level, currentFt8x7Model() == Ft8x7Model::Ft818);
}

static FeatureStatus ft8x7BandSettingQuery(Ft8x7Setting setting, Ft8x7SettingState& out) {
  uint64_t hz = 0;
  if (!queryFrequency(hz, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  if (setting == Ft8x7Setting::RfPower) {
    uint8_t watts = 0;
    if (!yaesuFt857QueryRfPowerWatts(hz, watts, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
    out.wattsTenths = (uint16_t)(watts * 10);
    return FeatureStatus::Ok;
  }
  YaesuFt857BandFlags flags;
  if (!yaesuFt857QueryBandFlags(hz, flags, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  if (setting == Ft8x7Setting::Nar) {
    out.on = flags.nar;
    return FeatureStatus::Ok;
  }
  if (!flags.hasIpoAtt) return FeatureStatus::Unsupported;
  out.on = setting == Ft8x7Setting::Ipo ? flags.ipo : flags.att;
  return FeatureStatus::Ok;
}

// The settings that are one EEPROM flag.
static bool ft8x7SettingFlag(Ft8x7Setting setting, Ft8x7Flag& flagOut) {
  switch (setting) {
    case Ft8x7Setting::Dbf: flagOut = Ft8x7Flag::Dbf; return true;
    case Ft8x7Setting::BreakIn: flagOut = Ft8x7Flag::BreakIn; return true;
    case Ft8x7Setting::Keyer: flagOut = Ft8x7Flag::Keyer; return true;
    case Ft8x7Setting::IfShift: flagOut = Ft8x7Flag::IfShift; return true;
    case Ft8x7Setting::KeyReverse: flagOut = Ft8x7Flag::KeyReverse; return true;
    case Ft8x7Setting::CwAutoMode: flagOut = Ft8x7Flag::CwAutoMode; return true;
    case Ft8x7Setting::Paddle: flagOut = Ft8x7Flag::PaddleMicKey; return true;
    default: return false;
  }
}

// The settings that are a CW level.
static bool ft8x7SettingCwLevel(Ft8x7Setting setting, Ft8x7CwLevel& levelOut) {
  switch (setting) {
    case Ft8x7Setting::CwSpeed: levelOut = Ft8x7CwLevel::Speed; return true;
    case Ft8x7Setting::CwPitch: levelOut = Ft8x7CwLevel::Pitch; return true;
    case Ft8x7Setting::CwDelay: levelOut = Ft8x7CwLevel::Delay; return true;
    case Ft8x7Setting::CwWeight: levelOut = Ft8x7CwLevel::Weight; return true;
    case Ft8x7Setting::Sidetone: levelOut = Ft8x7CwLevel::Sidetone; return true;
    case Ft8x7Setting::Qsk: levelOut = Ft8x7CwLevel::Qsk; return true;
    default: return false;
  }
}

// A flag or a CW level is supported where the EEPROM map has it.
static bool ft8x7SettingSupported(Ft8x7Setting setting, Ft8x7Model model) {
  Ft8x7Flag flag = Ft8x7Flag::Nb;
  if (ft8x7SettingFlag(setting, flag)) {
    Ft8x7FlagField field;
    return ft8x7FlagField(model, flag, field);
  }
  Ft8x7CwLevel cwLevel = Ft8x7CwLevel::Speed;
  if (ft8x7SettingCwLevel(setting, cwLevel)) {
    uint16_t addr = 0;
    return ft8x7CwLevelAddr(model, cwLevel, addr);
  }
  switch (setting) {
    case Ft8x7Setting::Agc:
    case Ft8x7Setting::Menu:
    case Ft8x7Setting::Row:
    case Ft8x7Setting::RfPower: return model != Ft8x7Model::None;
    case Ft8x7Setting::Antenna: return ft8x7IsFt817Family(model);
    default: return model == Ft8x7Model::Ft857;  // IPO, ATT, NAR, the DSP menu levels, mic EQ
  }
}

bool ft8x7SettingAvailable(Ft8x7Setting setting) {
  return isFt8x7() && ft8x7SettingSupported(setting, currentFt8x7Model());
}

FeatureStatus ft8x7SettingQuery(Ft8x7Setting setting, Ft8x7SettingState& out) {
  const Ft8x7Model model = currentFt8x7Model();
  if (!ft8x7SettingSupported(setting, model)) return FeatureStatus::Unsupported;
  if (setting == Ft8x7Setting::RfPower && !currentProfile().caps.getRfPower) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = setting;
  Ft8x7Flag flag = Ft8x7Flag::Nb;
  if (ft8x7SettingFlag(setting, flag)) {
    if (!yaesuFt8x7QueryFlag(flag, out.on, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
    return FeatureStatus::Ok;
  }
  Ft8x7CwLevel cwLevel = Ft8x7CwLevel::Speed;
  if (ft8x7SettingCwLevel(setting, cwLevel)) {
    return eepromReadStatus(yaesuFt8x7QueryCwLevel(cwLevel, out.value, kFt8x7TimeoutMs));
  }
  if (setting == Ft8x7Setting::CwBfo) return eepromReadStatus(yaesuFt857QueryCwBfo(out.cwBfo, kFt8x7TimeoutMs));
  bool ok = false;
  switch (setting) {
    case Ft8x7Setting::Agc: ok = yaesuFt8x7QueryAgc(out.agc, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::Menu:
    case Ft8x7Setting::Row: {
      uint8_t menu = 0;
      uint8_t row = 0;
      ok = yaesuFt8x7QueryMenuAndRow(menu, row, kFt8x7TimeoutMs);
      out.number = setting == Ft8x7Setting::Menu ? menu : row;
      break;
    }
    case Ft8x7Setting::NrLevel: ok = yaesuFt857QueryLevel(YaesuFt857Level::NrLevel, out.value, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::NbLevel: ok = yaesuFt857QueryLevel(YaesuFt857Level::NbLevel, out.value, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::BpfWidth: ok = yaesuFt857QueryLevel(YaesuFt857Level::BpfWidth, out.value, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::LowCut: ok = yaesuFt857QueryLevel(YaesuFt857Level::HpfCutoff, out.value, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::HighCut: ok = yaesuFt857QueryLevel(YaesuFt857Level::LpfCutoff, out.value, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::MicEq: ok = yaesuFt857QueryMicEq(out.micEq, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::Antenna: {
      uint64_t hz = 0;
      ok = queryFrequency(hz, kFt8x7TimeoutMs) && yaesuFt817QueryRearAntenna(hz, out.on, kFt8x7TimeoutMs);
      break;
    }
    case Ft8x7Setting::RfPower: {
      if (!ft8x7IsFt817Family(model)) return ft8x7BandSettingQuery(setting, out);
      Ft817PowerLevel level = Ft817PowerLevel::High;
      ok = yaesuFt817QueryPowerLevel(level, kFt8x7TimeoutMs);
      if (ok) setFt817PowerLevel(level, out);
      break;
    }
    default: return ft8x7BandSettingQuery(setting, out);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus ft8x7MenuRowSet(Ft8x7Setting setting, uint8_t number, Ft8x7SettingState& out) {
  const bool row = setting == Ft8x7Setting::Row;
  if ((!row && setting != Ft8x7Setting::Menu) || !currentProfile().caps.setRadioMenu ||
      currentFt8x7Model() == Ft8x7Model::None) {
    return FeatureStatus::Unsupported;
  }
  out = Ft8x7SettingState();
  out.setting = setting;
  return eepromWriteStatus(yaesuFt8x7SetMenuOrRow(row, number, out.number, kFt8x7TimeoutMs));
}

// The menu level of an FT-857/897 setting that can be set.
static bool ft8x7SettableLevel(Ft8x7Setting setting, YaesuFt857Level& levelOut) {
  switch (setting) {
    case Ft8x7Setting::NrLevel: levelOut = YaesuFt857Level::NrLevel; return true;
    case Ft8x7Setting::NbLevel: levelOut = YaesuFt857Level::NbLevel; return true;
    case Ft8x7Setting::BpfWidth: levelOut = YaesuFt857Level::BpfWidth; return true;
    case Ft8x7Setting::LowCut: levelOut = YaesuFt857Level::HpfCutoff; return true;
    case Ft8x7Setting::HighCut: levelOut = YaesuFt857Level::LpfCutoff; return true;
    default: return false;
  }
}

// caps.setNrLevel, caps.setNbLevel, or for the width and the cuts caps.setRadioMenu; no FT-8x7
// profile sets them.
static bool ft8x7LevelSetAllowed(Ft8x7Setting setting) {
  const RadioCapabilities& caps = currentProfile().caps;
  if (setting == Ft8x7Setting::NrLevel) return caps.setNrLevel;
  if (setting == Ft8x7Setting::NbLevel) return caps.setNbLevel;
  return caps.setRadioMenu;
}

bool ft8x7LevelRange(Ft8x7Setting setting, uint16_t& minOut, uint16_t& maxOut) {
  Ft8x7CwLevel cwLevel = Ft8x7CwLevel::Speed;
  if (ft8x7SettingCwLevel(setting, cwLevel)) return ft8x7CwLevelRange(currentFt8x7Model(), cwLevel, minOut, maxOut);
  YaesuFt857Level level = YaesuFt857Level::NrLevel;
  return ft8x7SettableLevel(setting, level) && ft857LevelRange(level, minOut, maxOut);
}

FeatureStatus ft8x7LevelSet(Ft8x7Setting setting, uint16_t value, Ft8x7SettingState& out) {
  Ft8x7CwLevel cwLevel = Ft8x7CwLevel::Speed;
  const bool cw = ft8x7SettingCwLevel(setting, cwLevel);
  YaesuFt857Level level = YaesuFt857Level::NrLevel;
  if (cw ? !ft8x7SettingSupported(setting, currentFt8x7Model())
         : !ft8x7SettableLevel(setting, level) || !currentIsFt857Family()) {
    return FeatureStatus::Unsupported;
  }
  if (!ft8x7LevelSetAllowed(setting)) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = setting;
  if (cw) return eepromWriteStatus(yaesuFt8x7SetCwLevel(cwLevel, value, out.value, kFt8x7TimeoutMs));
  return eepromWriteStatus(yaesuFt857SetLevel(level, value, out.value, kFt8x7TimeoutMs));
}

FeatureStatus ft8x7LevelStep(Ft8x7Setting setting, int delta, Ft8x7SettingState& out) {
  Ft8x7CwLevel cwLevel = Ft8x7CwLevel::Speed;
  const bool cw = ft8x7SettingCwLevel(setting, cwLevel);
  YaesuFt857Level level = YaesuFt857Level::NrLevel;
  if (cw ? !ft8x7SettingSupported(setting, currentFt8x7Model())
         : !ft8x7SettableLevel(setting, level) || !currentIsFt857Family()) {
    return FeatureStatus::Unsupported;
  }
  if (!ft8x7LevelSetAllowed(setting)) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = setting;
  if (cw) return eepromWriteStatus(yaesuFt8x7StepCwLevel(cwLevel, delta, out.value, kFt8x7TimeoutMs));
  return eepromWriteStatus(yaesuFt857StepLevel(level, delta, out.value, kFt8x7TimeoutMs));
}

// DBF, BK, KYR and the CW flags, where the model keeps them; IF shift is read only.
static bool ft8x7FlagSettable(Ft8x7Setting setting, Ft8x7Flag& flag) {
  if (!currentProfile().caps.setRadioMenu || setting == Ft8x7Setting::IfShift) return false;
  return ft8x7SettingFlag(setting, flag) && ft8x7SettingSupported(setting, currentFt8x7Model());
}

FeatureStatus ft8x7FlagSettingSet(Ft8x7Setting setting, bool on, Ft8x7SettingState& out) {
  Ft8x7Flag flag = Ft8x7Flag::Nb;
  if (!ft8x7FlagSettable(setting, flag)) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = setting;
  const FeatureStatus status = eepromWriteStatus(yaesuFt8x7SetFlag(flag, on, kFt8x7TimeoutMs));
  if (status != FeatureStatus::Ok) return status;
  out.on = on;
  return FeatureStatus::Ok;
}

FeatureStatus ft8x7FlagSettingToggle(Ft8x7Setting setting, Ft8x7SettingState& out) {
  Ft8x7Flag flag = Ft8x7Flag::Nb;
  if (!ft8x7FlagSettable(setting, flag)) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = setting;
  return ft8x7ToggleFlag(flag, out.on);
}

// rear: nullptr toggles.
static FeatureStatus ft817AntennaWrite(const bool* rear, Ft8x7SettingState& out) {
  if (!currentProfile().caps.setRadioMenu || !currentIsFt817Family()) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = Ft8x7Setting::Antenna;
  uint64_t hz = 0;
  if (!queryFrequency(hz, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  return eepromWriteStatus(rear ? yaesuFt817SetRearAntenna(hz, *rear, out.on, kFt8x7TimeoutMs)
                                : yaesuFt817ToggleRearAntenna(hz, out.on, kFt8x7TimeoutMs));
}

FeatureStatus ft817AntennaSet(bool rear, Ft8x7SettingState& out) {
  return ft817AntennaWrite(&rear, out);
}

FeatureStatus ft817AntennaToggle(Ft8x7SettingState& out) {
  return ft817AntennaWrite(nullptr, out);
}

FeatureStatus ft8x7RfPowerRange(uint64_t& hzOut, uint16_t& minOut, uint16_t& maxOut) {
  if (!currentProfile().caps.setRfPower || !currentIsFt857Family()) return FeatureStatus::Unsupported;
  if (!queryFrequency(hzOut, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  uint8_t lo = 0;
  uint8_t hi = 0;
  ft857RfPowerRange(ft8x7BandGroupForHz(hzOut), lo, hi);
  minOut = lo;
  maxOut = hi;
  return FeatureStatus::Ok;
}

FeatureStatus ft8x7RfPowerSet(uint16_t watts, Ft8x7SettingState& out, uint64_t hz) {
  if (!currentProfile().caps.setRfPower || !currentIsFt857Family()) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = Ft8x7Setting::RfPower;
  // Above any band's maximum; also keeps the cast below from wrapping.
  if (watts > 0xFF) return FeatureStatus::Invalid;
  if (!hz && !queryFrequency(hz, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  uint8_t written = 0;
  const FeatureStatus status =
      eepromWriteStatus(yaesuFt857SetRfPowerWatts(hz, (uint8_t)watts, written, kFt8x7TimeoutMs));
  out.wattsTenths = (uint16_t)(written * 10);
  return status;
}

FeatureStatus ft817PowerLevelSet(Ft817PowerLevel level, Ft8x7SettingState& out) {
  if (!currentProfile().caps.setRfPower || !currentIsFt817Family()) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = Ft8x7Setting::RfPower;
  Ft817PowerLevel read = level;
  const FeatureStatus status = eepromWriteStatus(yaesuFt817SetPowerLevel(level, read, kFt8x7TimeoutMs));
  if (status == FeatureStatus::Ok) setFt817PowerLevel(read, out);
  return status;
}

FeatureStatus ft8x7AgcSet(YaesuAgc agc, Ft8x7SettingState& out) {
  if (!currentProfile().caps.setRadioMenu || currentFt8x7Model() == Ft8x7Model::None) {
    return FeatureStatus::Unsupported;
  }
  out = Ft8x7SettingState();
  out.setting = Ft8x7Setting::Agc;
  return eepromWriteStatus(yaesuFt8x7SetAgc(agc, out.agc, kFt8x7TimeoutMs));
}

FeatureStatus ft8x7MicEqSet(YaesuFt857MicEq eq, Ft8x7SettingState& out) {
  if (!currentProfile().caps.setRadioMenu || !currentIsFt857Family()) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = Ft8x7Setting::MicEq;
  return eepromWriteStatus(yaesuFt857SetMicEq(eq, out.micEq, kFt8x7TimeoutMs));
}

FeatureStatus ft8x7CwBfoSet(Ft857CwBfo bfo, Ft8x7SettingState& out) {
  if (!currentProfile().caps.setRadioMenu || !currentIsFt857Family()) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = Ft8x7Setting::CwBfo;
  return eepromWriteStatus(yaesuFt857SetCwBfo(bfo, out.cwBfo, kFt8x7TimeoutMs));
}
