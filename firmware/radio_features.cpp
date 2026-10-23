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

// Receive features: noise reduction, noise blanker and notch. The transmit settings are in
// radio_features_tx.cpp, the FT-8x7 EEPROM settings in radio_features_ft8x7.cpp and tone squelch
// in radio_features_tone.cpp.

// The FT-817/818 has no DSP: no noise reduction and no notch, whatever the profile says.
static bool isFt817WithoutDsp() {
  return currentIsFt817Family();
}

static bool isTs480() {
  return currentRadioModel() == RadioModel::Ts480;
}

// ---- AGC ----

FeatureStatus agcQuery(AgcMode& out) {
  if (!currentProfile().caps.getAgc) return FeatureStatus::Unsupported;
  if (!isFt8x7()) return FeatureStatus::Unsupported;
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7SettingQuery(Ft8x7Setting::Agc, state);
  if (status == FeatureStatus::Ok) out = ft8x7AgcToMode(state.agc);
  return status;
}

bool agcCanSet(AgcMode mode) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getAgc || !isFt8x7()) return false;
  YaesuAgc agc = YaesuAgc::Off;
  return sp.caps.setRadioMenu && currentFt8x7Model() != Ft8x7Model::None && ft8x7AgcFromMode(mode, agc);
}

FeatureStatus agcSet(AgcMode mode, AgcMode& out) {
  if (!agcCanSet(mode)) return FeatureStatus::Unsupported;
  YaesuAgc agc = YaesuAgc::Off;
  (void)ft8x7AgcFromMode(mode, agc);
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7AgcSet(agc, state);
  if (status == FeatureStatus::Ok) out = ft8x7AgcToMode(state.agc);
  return status;
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
