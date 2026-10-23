#include "radio_features.h"
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

static constexpr uint32_t kFt8x7TimeoutMs = YAESU_CAT_REPLY_TIMEOUT_MS;

static FeatureStatus eepromWriteStatus(YaesuEepromWrite result) {
  switch (result) {
    case YaesuEepromWrite::Ok: return FeatureStatus::Ok;
    case YaesuEepromWrite::Unsupported: return FeatureStatus::Unsupported;
    case YaesuEepromWrite::OutOfRange: return FeatureStatus::Invalid;
    case YaesuEepromWrite::NoReply: return failure(FeatureStatus::NoReply);
    case YaesuEepromWrite::UnexpectedBytes: return FeatureStatus::Unexpected;
    case YaesuEepromWrite::Transmitting:
    case YaesuEepromWrite::NotVerified: return FeatureStatus::Failed;
  }
  return FeatureStatus::Failed;
}

static FeatureStatus eepromReadStatus(YaesuEepromRead result) {
  switch (result) {
    case YaesuEepromRead::Ok: return FeatureStatus::Ok;
    case YaesuEepromRead::Unsupported: return FeatureStatus::Unsupported;
    case YaesuEepromRead::NoReply: return failure(FeatureStatus::NoReply);
    case YaesuEepromRead::UnexpectedBytes: return FeatureStatus::Unexpected;
  }
  return FeatureStatus::NoReply;
}

static bool isFt8x7() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
}

// FT-8x7: flips flag from the EEPROM byte read for the write. on is the flag as written.
static FeatureStatus ft8x7ToggleFlag(Ft8x7Flag flag, bool& on) {
  return eepromWriteStatus(yaesuFt8x7ToggleFlag(flag, on, kFt8x7TimeoutMs));
}

// The toggles below start from the radio's state, read each time: nothing keeps the remembered
// one up to date when the radio's own keys change it.

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
  if (!currentProfile().caps.setNr || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
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
  bool next = false;
  if (isFt8x7()) {
    const FeatureStatus status = ft8x7ToggleFlag(Ft8x7Flag::Dnr, next);
    if (status != FeatureStatus::Ok) return status;
    rememberLiveNr(next, millis());
  } else {
    if (!refreshLiveNr()) return failure(FeatureStatus::NoReply);
    next = !live.nrOn;
    if (!applyNrAndTrack(next)) return failure(FeatureStatus::Failed);
  }
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
  if (isFt8x7()) {
    const FeatureStatus status = ft8x7ToggleFlag(Ft8x7Flag::Nb, on);
    if (status == FeatureStatus::Ok) rememberLiveNb(on, millis());
    return status;
  }
  if (!refreshLiveNb()) return failure(FeatureStatus::NoReply);
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
  if (!currentProfile().caps.setNotch || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!applyNotchAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

FeatureStatus notchSetWidth(NotchWidth width) {
  if (currentProtocolType() != PROTO_CIV || !currentProfile().caps.setNotchWidth) return FeatureStatus::Unsupported;
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
  if (isFt8x7()) {
    bool on = false;
    const FeatureStatus status = ft8x7ToggleFlag(Ft8x7Flag::Dnf, on);
    if (status != FeatureStatus::Ok) return status;
    rememberLiveNotch(on, millis(), false, NOTCH_WIDTH_UNKNOWN);
    out.on = on;
    out.width = NOTCH_WIDTH_UNKNOWN;
    return FeatureStatus::Ok;
  }
  if (!refreshLiveNotch()) return failure(FeatureStatus::NoReply);
  // Without a notch width the toggle is just on and off; an Icom with widths steps through them.
  if (currentProtocolType() != PROTO_CIV || !currentProfile().caps.setNotchWidth) {
    return applyNotchState(!live.notchOn, NOTCH_WIDTH_UNKNOWN, out);
  }

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

// The profile's cap, and a read HamTRC has for the radio.
static bool procSupported() {
  if (!currentProfile().caps.getProc) return false;
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
  return currentProtocolType() == PROTO_CIV || isFtdx10() || currentFt8x7Model() != Ft8x7Model::None;
}

FeatureStatus micGainQuery(uint8_t& level) {
  if (!micGainSupported()) return FeatureStatus::Unsupported;
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
  if (!currentProfile().caps.getVox) return false;
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

FeatureStatus voxDelayQuery(uint16_t& ms) {
  if (!voxSupported()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  uint16_t civAddr = 0;
  if (currentProtocolType() == PROTO_CIV && !civVoxDelayAddr(sp.model, civAddr)) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryVoxDelayMs(sp, ms, 800);
  else if (isFtdx10()) ok = asciiQueryYaesuVoxDelayMs(sp, ms, 800);
  else ok = yaesuFt8x7QueryVoxDelayMs(ms, YAESU_CAT_REPLY_TIMEOUT_MS);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

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

FeatureStatus ft8x7RfPowerSet(uint16_t watts, Ft8x7SettingState& out) {
  if (!currentProfile().caps.setRfPower || !currentIsFt857Family()) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = Ft8x7Setting::RfPower;
  // Above any band's maximum; also keeps the cast below from wrapping.
  if (watts > 0xFF) return FeatureStatus::Invalid;
  uint64_t hz = 0;
  if (!queryFrequency(hz, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
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

// ---- Speech processor, mic gain and VOX writes ----

// Where a write goes: over CAT where the profile has the write cap, or, experimentally
// (caps.setRadioMenu), to the FT-8x7 EEPROM. ft8x7: the FT-8x7 model keeps the setting.
enum class TxWrite : uint8_t { None, Civ, Ftdx10, Ft8x7 };

static TxWrite txWriteTarget(bool readable, bool capSet, bool ft8x7) {
  if (!readable) return TxWrite::None;
  if (currentProtocolType() == PROTO_CIV) return capSet ? TxWrite::Civ : TxWrite::None;
  if (isFtdx10()) return capSet ? TxWrite::Ftdx10 : TxWrite::None;
  return ft8x7 && currentProfile().caps.setRadioMenu ? TxWrite::Ft8x7 : TxWrite::None;
}

// A CAT write, which reads the setting back.
static FeatureStatus catWriteStatus(bool ok) {
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::Failed);
}

// Sets a CAT level of 0..100 percent: CI-V scaled to 0..255, FTDX10 as it is. levelOut is the
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
    case TxWrite::Ftdx10: return catWriteStatus(asciiSetYaesuProc(sp, on));
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
    case TxWrite::Ftdx10: return catWriteStatus(asciiSetYaesuVox(sp, on));
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
  } else if (target == TxWrite::Ftdx10) {
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
  if (target == TxWrite::Ftdx10) return voxDelaySet(yaesuAsciiVoxDelayStepBy(ms, delta), msOut);
  // CI-V: 100 ms steps.
  if (delta > 20) delta = 20;
  if (delta < -20) delta = -20;
  const int stepped = (int)ms + delta * 100;
  return voxDelaySet((uint16_t)(stepped < lo ? lo : stepped > hi ? hi : stepped), msOut);
}

// ---- FT-8x7 tone squelch ----

static bool toneSupported() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
}

// The FT-817/818 keeps the tone mode per band block, so the one last set holds only on the band
// it was set on (the amateur band edges are the FT-857/897's), and outside the amateur bands
// only on the same frequency. A switch to the other VFO on the same band is not seen.
static bool ft817ToneModeHolds(uint64_t hz) {
  if (!live.toneModeValid) return false;
  const Ft857BandSlot* slot = ft857BandSlotForHz(live.toneModeHz);
  return slot ? slot == ft857BandSlotForHz(hz) : hz == live.toneModeHz;
}

// hz: on the FT-817/818 the frequency read, also when the mode is not known; a change sets the
// mode with it. 0 on the FT-857/897, which reads the mode from the radio.
static FeatureStatus toneModeQueryAt(uint64_t& hz, YaesuToneMode& out) {
  if (!toneSupported()) return FeatureStatus::Unsupported;
  hz = 0;
  if (!currentIsFt857Family()) {
    if (!queryFrequency(hz, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
    if (!ft817ToneModeHolds(hz)) return FeatureStatus::Unsupported;
    out = live.toneMode;
    return FeatureStatus::Ok;
  }
  bool known = false;
  if (!yaesuFt857QueryToneMode(known, out, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  return known ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}

FeatureStatus toneModeQuery(YaesuToneMode& out) {
  uint64_t hz = 0;
  return toneModeQueryAt(hz, out);
}

FeatureStatus toneOff() {
  if (!toneSupported()) return FeatureStatus::Unsupported;
  // The FT-817/818 remembers off for the band it is on. Off is sent also when the frequency
  // cannot be read; then no band is remembered.
  uint64_t hz = 0;
  if (!currentIsFt857Family() && !queryFrequency(hz, kFt8x7TimeoutMs)) hz = 0;  // not known
  return yaesuCatSetToneMode(YaesuToneMode(), hz) ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}

FeatureStatus toneTypeSet(YaesuToneType type, YaesuToneMode& out) {
  if (!toneSupported() || type == YaesuToneType::None) return FeatureStatus::Unsupported;
  uint64_t hz = 0;
  YaesuToneMode current;
  const FeatureStatus status = toneModeQueryAt(hz, current);
  if (status != FeatureStatus::Ok && status != FeatureStatus::Unsupported) return status;
  // Not known: as from off.
  if (status != FeatureStatus::Ok) current = YaesuToneMode();
  out.tx = current.rxOnly() ? YaesuToneType::None : type;
  out.rx = current.txOnly() ? YaesuToneType::None : type;
  if (yaesuCatSetToneMode(out, hz)) return FeatureStatus::Ok;
  out.tx = type;
  out.rx = type;
  return yaesuCatSetToneMode(out, hz) ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}

FeatureStatus toneDirStep(YaesuToneMode& out) {
  if (!toneSupported()) return FeatureStatus::Unsupported;
  uint64_t hz = 0;
  YaesuToneMode current;
  const FeatureStatus status = toneModeQueryAt(hz, current);
  if (status != FeatureStatus::Ok) return status;
  if (current.isOff()) {
    out = current;
    return FeatureStatus::Ok;
  }
  const bool txOnly = current.txOnly();
  const bool rxOnly = current.rxOnly();
  // A mixed mode counts as both: its sent type goes on alone.
  const YaesuToneType type = current.type();
  out = YaesuToneMode();
  if (!txOnly) out.tx = type;
  if (txOnly || rxOnly) out.rx = type;
  if (yaesuCatSetToneMode(out, hz)) return FeatureStatus::Ok;
  // The FT-817/818 has no RX only: from TX only back to both.
  if (!txOnly) return FeatureStatus::Unsupported;
  out.tx = type;
  out.rx = type;
  return yaesuCatSetToneMode(out, hz) ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}

FeatureStatus splitToneQuery(bool& on) {
  if (!toneSupported() || !currentIsFt857Family()) return FeatureStatus::Unsupported;
  bool known = false;
  if (!yaesuFt857QuerySplitTone(known, on, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  return known ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}
