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

// The ASCII radios with a GT encoding in their profile.
static bool isAsciiAgc() {
  const ProtocolType pt = currentProtocolType();
  return isAsciiProtocol(pt) &&
         currentProfile().settings.agc != AgcEncoding::None;
}

FeatureStatus agcQuery(AgcMode& out) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getAgc) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) {
    ok = civQueryAgc(sp, out, kCatReplyTimeoutMs);
  } else if (isAsciiAgc()) {
    ok = asciiQueryAgc(sp, out, kCatReplyTimeoutMs);
  } else if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7SettingQuery(Ft8x7Setting::Agc, state);
    if (status == FeatureStatus::Ok) out = ft8x7AgcToMode(state.agc);
    return status;
  } else {
    return FeatureStatus::Unsupported;
  }
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

static bool agcCanSet(AgcMode mode, const RadioCapabilities& caps) {
  if (!caps.getAgc) return false;
  if (currentProtocolType() == PROTO_CIV) {
    uint8_t raw = 0;
    return caps.setAgc && civAgcEncode(mode, raw);
  }
  if (isAsciiAgc()) {
    char cmd[8];
    return caps.setAgc && asciiAgcEncode(currentProfile().settings.agc, mode, cmd, sizeof(cmd));
  }
  if (!isFt8x7()) return false;
  YaesuAgc agc = YaesuAgc::Off;
  return caps.setRadioMenu && currentFt8x7Model() != Ft8x7Model::None && ft8x7AgcFromMode(mode, agc);
}

bool agcCanSet(AgcMode mode) { return agcCanSet(mode, currentProfile().caps); }

FeatureStatus agcWriteAccess() {
  return writeAccess([](const RadioCapabilities& caps) {
    for (const AgcMode mode : {AgcMode::Off, AgcMode::Fast, AgcMode::Mid, AgcMode::Slow, AgcMode::Auto}) {
      if (agcCanSet(mode, caps)) return true;
    }
    return false;
  });
}

FeatureStatus agcSet(AgcMode mode, AgcMode& out) {
  if (!agcCanSet(mode)) {
    return writeAccess([mode](const RadioCapabilities& caps) { return agcCanSet(mode, caps); });
  }
  const RadioProfile& sp = currentProfile();
  if (!isFt8x7()) {
    const bool ok = currentProtocolType() == PROTO_CIV ? civSetAgc(sp, mode) : asciiSetAgc(sp, mode);
    if (!ok) return failure(FeatureStatus::Failed);
    out = mode;
    return FeatureStatus::Ok;
  }
  YaesuAgc agc = YaesuAgc::Off;
  (void)ft8x7AgcFromMode(mode, agc);
  Ft8x7SettingState state;
  const FeatureStatus status = ft8x7AgcSet(agc, state);
  if (status == FeatureStatus::Ok) out = ft8x7AgcToMode(state.agc);
  return status;
}

// ---- Yaesu ASCII NR and NB levels ----

bool yaesuRxLevelRange(RxLevel level, uint16_t& minOut, uint16_t& maxOut) {
  if (currentProtocolType() != PROTO_YAESU_FTDX_ASCII) return false;
  const RadioCapabilities& caps = currentProfile().caps;
  if (level == RxLevel::Nr ? !caps.getNrLevel : !caps.getNbLevel) return false;
  minOut = level == RxLevel::Nr ? 1 : 0;
  maxOut = level == RxLevel::Nr ? 15 : 10;
  return true;
}

FeatureStatus yaesuRxLevelQuery(RxLevel level, uint16_t& valueOut) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!yaesuRxLevelRange(level, min, max)) return FeatureStatus::Unsupported;
  return asciiQueryRxLevel(currentProfile(), level, valueOut, kCatReplyTimeoutMs) ? FeatureStatus::Ok
                                                                     : failure(FeatureStatus::NoReply);
}

FeatureStatus yaesuRxLevelSet(RxLevel level, uint16_t value, uint16_t& valueOut) {
  uint16_t min = 0;
  uint16_t max = 0;
  const RadioCapabilities& caps = currentProfile().caps;
  if (!yaesuRxLevelRange(level, min, max) || (level == RxLevel::Nr ? !caps.setNrLevel : !caps.setNbLevel)) {
    return FeatureStatus::Unsupported;
  }
  if (value < min || value > max) return FeatureStatus::Invalid;
  if (!asciiSetRxLevel(currentProfile(), level, value)) return failure(FeatureStatus::Failed);
  valueOut = value;
  return FeatureStatus::Ok;
}

FeatureStatus yaesuRxLevelStep(RxLevel level, int delta, uint16_t& valueOut) {
  uint16_t min = 0;
  uint16_t max = 0;
  if (!yaesuRxLevelRange(level, min, max)) return FeatureStatus::Unsupported;
  uint16_t now = 0;
  const FeatureStatus status = yaesuRxLevelQuery(level, now);
  if (status != FeatureStatus::Ok) return status;
  int next = (int)now + delta;
  if (next < min) next = min;
  if (next > max) next = max;
  return yaesuRxLevelSet(level, (uint16_t)next, valueOut);
}

// ---- Auto notch ----

FeatureStatus autoNotchQuery(bool& on) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getAutoNotch) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryAutoNotch(sp, on, kCatReplyTimeoutMs);
  else if (currentProtocolType() == PROTO_YAESU_FTDX_ASCII) ok = asciiQueryAutoNotch(sp, on, kCatReplyTimeoutMs);
  else return FeatureStatus::Unsupported;
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus autoNotchSet(bool on) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getAutoNotch || !sp.caps.setAutoNotch) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civSetAutoNotch(sp, on);
  else if (currentProtocolType() == PROTO_YAESU_FTDX_ASCII) ok = asciiSetAutoNotch(sp, on);
  else return FeatureStatus::Unsupported;
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::Failed);
}

FeatureStatus autoNotchToggle(bool& on) {
  if (!currentProfile().caps.setAutoNotch) return FeatureStatus::Unsupported;
  bool now = false;
  const FeatureStatus status = autoNotchQuery(now);
  if (status != FeatureStatus::Ok) return status;
  const FeatureStatus set = autoNotchSet(!now);
  if (set == FeatureStatus::Ok) on = !now;
  return set;
}

// ---- Attenuator ----

// The ASCII radios with a PA/RA encoding in their profile.
static bool isAsciiFrontEnd() {
  const ProtocolType pt = currentProtocolType();
  return isAsciiProtocol(pt) &&
         currentProfile().settings.frontEnd != FrontEndEncoding::None;
}

// The CAT radios whose profile gives the attenuator's steps.
static bool hasCatAtt() {
  return currentProfile().settings.attLevels && (currentProtocolType() == PROTO_CIV || isAsciiFrontEnd());
}

FeatureStatus attQuery(AttState& out) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getAtt) return FeatureStatus::Unsupported;
  if (isFt8x7()) {
    Ft8x7SettingState state;
    const FeatureStatus status = ft8x7SettingQuery(Ft8x7Setting::Att, state);
    if (status != FeatureStatus::Ok) return status;
    out.on = state.on;
    out.db = 0;
    return FeatureStatus::Ok;
  }
  if (!hasCatAtt()) return FeatureStatus::Unsupported;
  if (currentProtocolType() == PROTO_CIV) {
    uint8_t db = 0;
    if (!civQueryAttDb(sp, db, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
    out.on = db != 0;
    out.db = db;
    return FeatureStatus::Ok;
  }
  uint8_t step = 0;
  if (!asciiQueryAttStep(sp, step, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
  out.on = step != 0;
  out.db = attStepDb(step);
  return FeatureStatus::Ok;
}

uint8_t attSteps() {
  const RadioProfile& sp = currentProfile();
  return sp.caps.getAtt && sp.caps.setAtt && hasCatAtt() ? sp.settings.attLevels : 0;
}

uint8_t attStepDb(uint8_t step) {
  const SettingEncodings& enc = currentProfile().settings;
  return step >= 1 && step <= enc.attLevels && step <= sizeof(enc.attDb) ? enc.attDb[step - 1] : 0;
}

FeatureStatus attSet(uint8_t step, AttState& out) {
  if (!attSteps()) return FeatureStatus::Unsupported;
  if (step > attSteps()) return FeatureStatus::Invalid;
  const RadioProfile& sp = currentProfile();
  const bool ok = currentProtocolType() == PROTO_CIV ? civSetAttDb(sp, attStepDb(step)) : asciiSetAttStep(sp, step);
  if (!ok) return failure(FeatureStatus::Failed);
  out.on = step != 0;
  out.db = attStepDb(step);
  return FeatureStatus::Ok;
}

FeatureStatus attStep(AttState& out) {
  if (!attSteps()) return FeatureStatus::Unsupported;
  AttState now;
  const FeatureStatus status = attQuery(now);
  if (status != FeatureStatus::Ok) return status;
  const SettingEncodings& enc = currentProfile().settings;
  return attSet(attNextStep(enc.attDb, enc.attLevels, now.on, now.db), out);
}

// ---- Preamp ----

static bool hasCatPreamp() {
  return currentProfile().settings.preampLevels && (currentProtocolType() == PROTO_CIV || isAsciiFrontEnd());
}

uint8_t preampLevels() {
  const RadioProfile& sp = currentProfile();
  return sp.caps.getPreamp && hasCatPreamp() ? sp.settings.preampLevels : 0;
}

FeatureStatus preampQuery(uint8_t& levelOut) {
  if (!preampLevels()) return FeatureStatus::Unsupported;
  const RadioProfile& sp = currentProfile();
  const bool ok = currentProtocolType() == PROTO_CIV ? civQueryPreampLevel(sp, levelOut, kCatReplyTimeoutMs)
                                                     : asciiQueryPreampLevel(sp, levelOut, kCatReplyTimeoutMs);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus preampSet(uint8_t level, uint8_t& levelOut) {
  if (!preampLevels() || !currentProfile().caps.setPreamp) return FeatureStatus::Unsupported;
  if (level > preampLevels()) return FeatureStatus::Invalid;
  const RadioProfile& sp = currentProfile();
  const bool ok = currentProtocolType() == PROTO_CIV ? civSetPreampLevel(sp, level) : asciiSetPreampLevel(sp, level);
  if (!ok) return failure(FeatureStatus::Failed);
  levelOut = level;
  return FeatureStatus::Ok;
}

FeatureStatus preampStep(uint8_t& levelOut) {
  if (!preampLevels() || !currentProfile().caps.setPreamp) return FeatureStatus::Unsupported;
  uint8_t now = 0;
  const FeatureStatus status = preampQuery(now);
  if (status != FeatureStatus::Ok) return status;
  return preampSet(now >= preampLevels() ? 0 : now + 1, levelOut);
}

// The toggles below start from the radio's state, read each time: nothing keeps the remembered
// one up to date when the radio's own keys change it.

// ---- Noise reduction ----

// The TS-480 NR has two levels, which the plain on/off commands cannot reach.
static constexpr uint8_t kTs480NrLevels = 2;

static bool ts480ReadNrLevel(uint8_t& level) {
  const RadioProfile& sp = currentProfile();
  String line;
  if (!transactAsciiCommand(sp.commands->nrGet, line, sp.commands->nrReplyPrefix, kCatReplyTimeoutMs)) return false;
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
  if (isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!currentProfile().caps.setNr) return capWriteAccess(&RadioCapabilities::setNr);
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
  if (isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!currentProfile().caps.setNr) return capWriteAccess(&RadioCapabilities::setNr);
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
  if (!currentProfile().caps.setNb) return capWriteAccess(&RadioCapabilities::setNb);
  if (!applyNbAndTrack(on)) return failure(FeatureStatus::Failed);
  return FeatureStatus::Ok;
}

FeatureStatus nbToggle(bool& on) {
  if (!currentProfile().caps.setNb) return capWriteAccess(&RadioCapabilities::setNb);
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
  if (isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!currentProfile().caps.setNotch) return capWriteAccess(&RadioCapabilities::setNotch);
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
  if (isFt817WithoutDsp()) return FeatureStatus::Unsupported;
  if (!currentProfile().caps.setNotch) return capWriteAccess(&RadioCapabilities::setNotch);
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
