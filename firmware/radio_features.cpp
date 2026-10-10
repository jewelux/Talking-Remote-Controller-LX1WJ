#include "radio_features.h"
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

// A radio exchange that went wrong: a timeout when the radio said nothing.
static FeatureStatus failure(FeatureStatus status) {
  return g_radioReplyTimedOut ? FeatureStatus::Timeout : status;
}

// The FT-817/818 has no DSP: no noise reduction and no notch, whatever the profile says.
static bool isFt817WithoutDsp() {
  return currentIsFt817Family();
}

static bool isTs480() {
  return currentRadioModel() == RadioModel::Ts480;
}

// ---- Noise reduction ----

// The TS-480 NR has two levels, which the plain on/off commands cannot reach.
static constexpr uint8_t kTs480NrLevels = 2;

static bool ts480ReadNrLevel(uint8_t& level) {
  const RadioProfile& sp = currentProfile();
  String line;
  if (!transactAsciiCommand(sp.commands->nrGet, line, sp.commands->nrReplyPrefix, 800)) return false;
  int start = (int)strlen(sp.commands->nrReplyPrefix);
  int semi = line.indexOf(';', start);
  if (semi < 0) semi = line.length();
  String value = line.substring(start, semi);
  value.trim();
  const int parsed = value.toInt();
  level = parsed <= 0 ? 0 : parsed >= kTs480NrLevels ? kTs480NrLevels : (uint8_t)parsed;
  return true;
}

static FeatureStatus ts480WriteNrLevel(uint8_t level, NrState& out) {
  char cmd[] = "NR0;";
  cmd[2] = (char)('0' + level);
  if (!asciiPacketSendCommand(cmd)) return failure(FeatureStatus::Failed);
  rememberLiveNr(level != 0, millis());
  out.on = level != 0;
  out.level = level;
  return FeatureStatus::Ok;
}

// Steps off -> 1 -> 2 -> off from the level the radio reports (to 1 when it
// does not answer).
static FeatureStatus ts480NrStep(NrState& out) {
  uint8_t level = 0;
  (void)ts480ReadNrLevel(level);
  return ts480WriteNrLevel(level >= kTs480NrLevels ? 0 : level + 1, out);
}

FeatureStatus nrQuery(NrState& out) {
  if (!currentProfile().caps.getNr || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (isTs480()) {
    uint8_t level = 0;
    if (!ts480ReadNrLevel(level)) return failure(FeatureStatus::NoReply);
    rememberLiveNr(level != 0, millis());
    out.on = level != 0;
    out.level = level;
    return FeatureStatus::Ok;
  }
  if (!refreshLiveNr()) return failure(FeatureStatus::NoReply);
  out.on = live.nrOn;
  out.level = 0;
  return FeatureStatus::Ok;
}

FeatureStatus nrSet(bool on) {
  if (!currentProfile().caps.setNr) return FeatureStatus::Unsupported;
  if (!applyNrAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

uint8_t nrLevelCount() {
  return isTs480() ? kTs480NrLevels : 0;
}

FeatureStatus nrSetLevel(uint8_t level, NrState& out) {
  if (!currentProfile().caps.setNr || nrLevelCount() == 0 || level > nrLevelCount()) return FeatureStatus::Unsupported;
  return ts480WriteNrLevel(level, out);
}

FeatureStatus nrToggle(NrState& out) {
  if (!currentProfile().caps.setNr || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (isTs480()) return ts480NrStep(out);
  if (!live.nrValid && !refreshLiveNr()) return failure(FeatureStatus::NoReply);
  const bool next = !live.nrOn;
  if (!applyNrAndTrack(next)) return failure(FeatureStatus::Failed);
  out.on = next;
  out.level = 0;
  return FeatureStatus::Ok;
}

// ---- Noise blanker ----

FeatureStatus nbQuery(bool& on) {
  if (!currentProfile().caps.getNb) return FeatureStatus::Unsupported;
  if (!refreshLiveNb()) return failure(FeatureStatus::NoReply);
  on = live.nbOn;
  return FeatureStatus::Ok;
}

FeatureStatus nbSet(bool on) {
  if (!currentProfile().caps.setNb) return FeatureStatus::Unsupported;
  if (!applyNbAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

FeatureStatus nbToggle(bool& on) {
  if (!currentProfile().caps.setNb) return FeatureStatus::Unsupported;
  if (!live.nbValid && !refreshLiveNb()) return failure(FeatureStatus::NoReply);
  const bool next = !live.nbOn;
  if (!applyNbAndTrack(next)) return failure(FeatureStatus::Failed);
  on = next;
  return FeatureStatus::Ok;
}

// ---- Notch filter ----

FeatureStatus notchQuery(NotchState& out) {
  if (!currentProfile().caps.getNotch || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!refreshLiveNotch()) return failure(FeatureStatus::NoReply);
  out.on = live.notchOn;
  out.width = (live.notchOn && live.notchWidthValid) ? live.notchWidth : NOTCH_WIDTH_UNKNOWN;
  return FeatureStatus::Ok;
}

FeatureStatus notchSet(bool on) {
  if (!currentProfile().caps.setNotch) return FeatureStatus::Unsupported;
  if (!applyNotchAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

FeatureStatus notchSetWidth(NotchWidth width) {
  if (currentProtocolType() != PROTO_CIV || !currentProfile().caps.setNotch) return FeatureStatus::Unsupported;
  if (!applyNotchAndTrack(true) || !applyNotchWidthAndTrack(width)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

// Sets the notch and reports it in out.
static FeatureStatus applyNotchState(bool on, NotchWidth width, NotchState& out) {
  if (!applyNotchAndTrack(on)) return failure(FeatureStatus::Failed);
  if (on && width != NOTCH_WIDTH_UNKNOWN && !applyNotchWidthAndTrack(width)) return failure(FeatureStatus::Failed);
  out.on = on;
  out.width = on ? width : NOTCH_WIDTH_UNKNOWN;
  return FeatureStatus::Ok;
}

// Moves a notch that is on to width.
static FeatureStatus applyNotchWidth(NotchWidth width, NotchState& out) {
  if (!applyNotchWidthAndTrack(width)) return failure(FeatureStatus::Failed);
  out.on = true;
  out.width = width;
  return FeatureStatus::Ok;
}

FeatureStatus notchToggle(NotchState& out) {
  if (!currentProfile().caps.setNotch || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!live.notchValid && !refreshLiveNotch()) return failure(FeatureStatus::NoReply);
  if (currentProtocolType() != PROTO_CIV) return applyNotchState(!live.notchOn, NOTCH_WIDTH_UNKNOWN, out);

  if (!live.notchOn) return applyNotchState(true, NOTCH_WIDTH_NAR, out);
  if (!live.notchWidthValid && !refreshLiveNotchWidth()) return failure(FeatureStatus::NoReply);
  if (live.notchWidth == NOTCH_WIDTH_NAR) return applyNotchWidth(NOTCH_WIDTH_MID, out);
  if (live.notchWidth == NOTCH_WIDTH_MID) return applyNotchWidth(NOTCH_WIDTH_WIDE, out);
  return applyNotchState(false, NOTCH_WIDTH_UNKNOWN, out);
}

// ---- Speech processor ----

static bool isFtdx10() {
  return currentRadioModel() == RadioModel::Ftdx10;
}

static bool procSupported() {
  return currentProtocolType() == PROTO_CIV || isFtdx10() || currentFt8x7Model() == Ft8x7Model::Ft857;
}

FeatureStatus procQuery(bool& on) {
  if (!procSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryProc(sp, on, 800);
  else if (isFtdx10()) ok = asciiQueryYaesuProc(sp, on, 800);
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
  } else if (isFtdx10()) {
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
static FeatureStatus ft8x7MicGainQuery(uint8_t& level) {
  uint8_t modeByte = 0;
  if (!yaesuCatQueryModeRawByte(modeByte, YAESU_CAT_REPLY_TIMEOUT_MS)) return failure(FeatureStatus::NoReply);
  Ft8x7MicGain gain = Ft8x7MicGain::Ssb;
  if (!ft8x7MicGainForMode(modeByte, gain) || !yaesuFt8x7HasMicGain(gain)) return FeatureStatus::Unsupported;
  if (!yaesuFt8x7QueryMicGain(gain, level, YAESU_CAT_REPLY_TIMEOUT_MS)) return failure(FeatureStatus::NoReply);
  return FeatureStatus::Ok;
}

FeatureStatus micGainQuery(uint8_t& level) {
  if (currentProtocolType() != PROTO_CIV && !isFtdx10() && currentFt8x7Model() == Ft8x7Model::None) {
    return FeatureStatus::Unsupported;
  }
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    uint16_t raw = 0;
    ok = civQueryMicGain(sp, raw, 800);
    level = levelRawToPercent(raw);
  } else if (isFtdx10()) {
    ok = asciiQueryYaesuMicGain(sp, level, 800);
  } else {
    return ft8x7MicGainQuery(level);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

// ---- VOX ----

static bool voxSupported() {
  return currentProtocolType() == PROTO_CIV || isFtdx10() || currentFt8x7Model() != Ft8x7Model::None;
}

FeatureStatus voxQuery(bool& on) {
  if (!voxSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryVox(sp, on, 800);
  else if (isFtdx10()) ok = asciiQueryYaesuVox(sp, on, 800);
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
  } else if (isFtdx10()) {
    ok = asciiQueryYaesuVoxGain(sp, level, 800);
  } else {
    ok = yaesuFt8x7QueryVoxGain(level, YAESU_CAT_REPLY_TIMEOUT_MS);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

// ---- FT-8x7 EEPROM settings ----

static constexpr uint32_t kFt8x7TimeoutMs = YAESU_CAT_REPLY_TIMEOUT_MS;

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
    case Ft8x7Setting::LowCut: return "HPF?";
    case Ft8x7Setting::HighCut: return "LPF?";
    case Ft8x7Setting::MicEq: return "MICEQ?";
    case Ft8x7Setting::Antenna: return "ANT?";
  }
  return "?";
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
  if (!flags.bandKnown) return FeatureStatus::Unsupported;
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
    default: return false;
  }
}

// A flag is supported where the EEPROM map has it.
static bool ft8x7SettingSupported(Ft8x7Setting setting, Ft8x7Model model) {
  Ft8x7Flag flag = Ft8x7Flag::Nb;
  if (ft8x7SettingFlag(setting, flag)) {
    Ft8x7FlagField field;
    return ft8x7FlagField(model, flag, field);
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
    case Ft8x7Setting::LowCut: ok = yaesuFt857QueryLevel(YaesuFt857Level::HpfCutoff, out.value, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::HighCut: ok = yaesuFt857QueryLevel(YaesuFt857Level::LpfCutoff, out.value, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::MicEq: ok = yaesuFt857QueryMicEq(out.micEq, kFt8x7TimeoutMs); break;
    case Ft8x7Setting::Antenna: {
      uint64_t hz = 0;
      ok = queryFrequency(hz, kFt8x7TimeoutMs) && yaesuFt817QueryRearAntenna(hz, out.on, kFt8x7TimeoutMs);
      break;
    }
    case Ft8x7Setting::RfPower:
      if (ft8x7IsFt817Family(model)) {
        ok = yaesuFt817QueryRfPowerTenths(out.wattsTenths, kFt8x7TimeoutMs);
        break;
      }
      return ft8x7BandSettingQuery(setting, out);
    default: return ft8x7BandSettingQuery(setting, out);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}
