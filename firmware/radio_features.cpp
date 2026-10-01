#include "radio_features.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "radio_catalog.h"
#include "radio_globals.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"

// A radio exchange that went wrong: a timeout when the radio said nothing.
static FeatureStatus failure(FeatureStatus status) {
  return g_radioReplyTimedOut ? FeatureStatus::Timeout : status;
}

// The FT-817/818 has no DSP: no noise reduction and no notch, whatever the profile says.
static bool isFt817WithoutDsp() {
  return currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817");
}

static bool isTs480() {
  return currentProtocolType() == PROTO_KENWOOD_ASCII && String(currentStoredProfile().name).indexOf("TS-480") >= 0;
}

// ---- Noise reduction ----

// The TS-480 NR has two levels, which the plain on/off commands cannot reach.
static constexpr uint8_t kTs480NrLevels = 2;

static bool ts480ReadNrLevel(uint8_t& level) {
  const StoredProfile& sp = currentStoredProfile();
  String line;
  if (!transactAsciiCommand(sp.ascii.nrGet, line, sp.ascii.nrReplyPrefix, 800)) return false;
  int start = (int)strlen(sp.ascii.nrReplyPrefix);
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
  if (!currentStoredProfile().caps.getNr || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
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
  if (!currentStoredProfile().caps.setNr) return FeatureStatus::Unsupported;
  if (!applyNrAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

uint8_t nrLevelCount() {
  return isTs480() ? kTs480NrLevels : 0;
}

FeatureStatus nrSetLevel(uint8_t level, NrState& out) {
  if (!currentStoredProfile().caps.setNr || nrLevelCount() == 0 || level > nrLevelCount()) return FeatureStatus::Unsupported;
  return ts480WriteNrLevel(level, out);
}

FeatureStatus nrToggle(NrState& out) {
  if (!currentStoredProfile().caps.setNr || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
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
  if (!currentStoredProfile().caps.getNb) return FeatureStatus::Unsupported;
  if (!refreshLiveNb()) return failure(FeatureStatus::NoReply);
  on = live.nbOn;
  return FeatureStatus::Ok;
}

FeatureStatus nbSet(bool on) {
  if (!currentStoredProfile().caps.setNb) return FeatureStatus::Unsupported;
  if (!applyNbAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

FeatureStatus nbToggle(bool& on) {
  if (!currentStoredProfile().caps.setNb) return FeatureStatus::Unsupported;
  if (!live.nbValid && !refreshLiveNb()) return failure(FeatureStatus::NoReply);
  const bool next = !live.nbOn;
  if (!applyNbAndTrack(next)) return failure(FeatureStatus::Failed);
  on = next;
  return FeatureStatus::Ok;
}

// ---- Notch filter ----

FeatureStatus notchQuery(NotchState& out) {
  if (!currentStoredProfile().caps.getNotch || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!refreshLiveNotch()) return failure(FeatureStatus::NoReply);
  out.on = live.notchOn;
  out.width = (live.notchOn && live.notchWidthValid) ? live.notchWidth : NOTCH_WIDTH_UNKNOWN;
  return FeatureStatus::Ok;
}

FeatureStatus notchSet(bool on) {
  if (!currentStoredProfile().caps.setNotch) return FeatureStatus::Unsupported;
  if (!applyNotchAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

FeatureStatus notchSetWidth(NotchWidth width) {
  if (currentProtocolType() != PROTO_CIV || !currentStoredProfile().caps.setNotch) return FeatureStatus::Unsupported;
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
  if (!currentStoredProfile().caps.setNotch || isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!live.notchValid && !refreshLiveNotch()) return failure(FeatureStatus::NoReply);
  if (currentProtocolType() != PROTO_CIV) return applyNotchState(!live.notchOn, NOTCH_WIDTH_UNKNOWN, out);

  if (!live.notchOn) return applyNotchState(true, NOTCH_WIDTH_NAR, out);
  if (!live.notchWidthValid && !refreshLiveNotchWidth()) return failure(FeatureStatus::NoReply);
  if (live.notchWidth == NOTCH_WIDTH_NAR) return applyNotchWidth(NOTCH_WIDTH_MID, out);
  if (live.notchWidth == NOTCH_WIDTH_MID) return applyNotchWidth(NOTCH_WIDTH_WIDE, out);
  return applyNotchState(false, NOTCH_WIDTH_UNKNOWN, out);
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
    case Ft8x7Setting::LowCut: return "HPF?";
    case Ft8x7Setting::HighCut: return "LPF?";
    case Ft8x7Setting::MicEq: return "MICEQ?";
    case Ft8x7Setting::Antenna: return "ANT?";
  }
  return "?";
}

static FeatureStatus ft8x7BandSettingQuery(Ft8x7Setting setting, Ft8x7SettingState& out) {
  uint64_t hz = 0;
  if (!queryFrequency(hz, 800)) return failure(FeatureStatus::NoReply);
  if (setting == Ft8x7Setting::RfPower) {
    uint8_t watts = 0;
    if (!yaesuFt857QueryRfPowerWatts(hz, watts, 800)) return failure(FeatureStatus::NoReply);
    out.wattsTenths = (uint16_t)(watts * 10);
    return FeatureStatus::Ok;
  }
  YaesuFt857BandFlags flags;
  if (!yaesuFt857QueryBandFlags(hz, flags, 800)) return failure(FeatureStatus::NoReply);
  if (!flags.bandKnown) return FeatureStatus::Unsupported;
  if (setting == Ft8x7Setting::Nar) {
    out.on = flags.nar;
    return FeatureStatus::Ok;
  }
  if (!flags.hasIpoAtt) return FeatureStatus::Unsupported;
  out.on = setting == Ft8x7Setting::Ipo ? flags.ipo : flags.att;
  return FeatureStatus::Ok;
}

FeatureStatus ft8x7SettingQuery(Ft8x7Setting setting, Ft8x7SettingState& out) {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return FeatureStatus::Unsupported;
  const bool ft817 = currentProfileVariantIs("ft817");
  if (!ft817 && !currentProfileVariantIs("ft857_897")) return FeatureStatus::Unsupported;
  if (ft817 && setting != Ft8x7Setting::RfPower && setting != Ft8x7Setting::Menu &&
      setting != Ft8x7Setting::Row && setting != Ft8x7Setting::Agc &&
      setting != Ft8x7Setting::BreakIn && setting != Ft8x7Setting::Keyer &&
      setting != Ft8x7Setting::IfShift && setting != Ft8x7Setting::Antenna) {
    return FeatureStatus::Unsupported;
  }
  if (!ft817 && setting == Ft8x7Setting::Antenna) return FeatureStatus::Unsupported;
  if (setting == Ft8x7Setting::RfPower && !currentStoredProfile().caps.getRfPower) return FeatureStatus::Unsupported;
  out = Ft8x7SettingState();
  out.setting = setting;
  bool ok = false;
  switch (setting) {
    case Ft8x7Setting::Agc: ok = ft817 ? yaesuFt817QueryAgc(out.agc, 800) : yaesuFt857QueryAgc(out.agc, 800); break;
    case Ft8x7Setting::Dbf: ok = yaesuFt857QueryDbf(out.on, 800); break;
    case Ft8x7Setting::BreakIn:
      ok = ft817 ? yaesuFt817QueryBreakIn(out.on, 800) : yaesuFt857QueryBreakIn(out.on, 800);
      break;
    case Ft8x7Setting::Keyer:
      ok = ft817 ? yaesuFt817QueryKeyer(out.on, 800) : yaesuFt857QueryKeyer(out.on, 800);
      break;
    case Ft8x7Setting::IfShift:
      ok = ft817 ? yaesuFt817QueryIfShift(out.on, 800) : yaesuFt857QueryIfShift(out.on, 800);
      break;
    case Ft8x7Setting::Menu:
    case Ft8x7Setting::Row: {
      uint8_t menu = 0;
      uint8_t row = 0;
      ok = ft817 ? yaesuFt817QueryMenuAndRow(menu, row, 800) : yaesuFt857QueryMenuAndRow(menu, row, 800);
      out.number = setting == Ft8x7Setting::Menu ? menu : row;
      break;
    }
    case Ft8x7Setting::NrLevel: ok = yaesuFt857QueryLevel(YaesuFt857Level::NrLevel, out.value, 800); break;
    case Ft8x7Setting::NbLevel: ok = yaesuFt857QueryLevel(YaesuFt857Level::NbLevel, out.value, 800); break;
    case Ft8x7Setting::LowCut: ok = yaesuFt857QueryLevel(YaesuFt857Level::HpfCutoff, out.value, 800); break;
    case Ft8x7Setting::HighCut: ok = yaesuFt857QueryLevel(YaesuFt857Level::LpfCutoff, out.value, 800); break;
    case Ft8x7Setting::MicEq: ok = yaesuFt857QueryMicEq(out.micEq, 800); break;
    case Ft8x7Setting::Antenna: {
      uint64_t hz = 0;
      ok = queryFrequency(hz, 800) && yaesuFt817QueryRearAntenna(hz, out.on, 800);
      break;
    }
    case Ft8x7Setting::RfPower:
      if (ft817) {
        ok = yaesuFt817QueryRfPowerTenths(out.wattsTenths, 800);
        break;
      }
      return ft8x7BandSettingQuery(setting, out);
    default: return ft8x7BandSettingQuery(setting, out);
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}
