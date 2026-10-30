#include "radio_features.h"
#include "radio_features_internal.h"
#include "cat_setting_codec.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_civ.h"
#include "radio_catalog.h"
#include "radio_globals.h"
#include "radio_prefs.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"

// ---- Tone squelch over CAT ----

static bool isCatTone() {
  const ProtocolType pt = currentProtocolType();
  return currentProfile().caps.getTone && (pt == PROTO_CIV || pt == PROTO_YAESU_FTDX_ASCII);
}

static YaesuToneMode ctcssMode(bool tx, bool rx) {
  YaesuToneMode mode;
  if (tx) mode.tx = YaesuToneType::Ctcss;
  if (rx) mode.rx = YaesuToneType::Ctcss;
  return mode;
}

// A mixed mode counts as both.
static ToneDir toneDirOf(const YaesuToneMode& mode) {
  if (mode.txOnly()) return ToneDir::TxOnly;
  if (mode.rxOnly()) return ToneDir::RxOnly;
  return ToneDir::Both;
}

static YaesuToneMode toneModeOf(YaesuToneType type, ToneDir dir) {
  YaesuToneMode mode;
  if (dir != ToneDir::RxOnly) mode.tx = type;
  if (dir != ToneDir::TxOnly) mode.rx = type;
  return mode;
}

// Both -> TX only -> RX only, where the radio has it -> both.
static ToneDir nextToneDir(ToneDir dir) {
  if (dir == ToneDir::Both) return ToneDir::TxOnly;
  if (dir == ToneDir::TxOnly && currentProfile().caps.setToneDecodeOnly) return ToneDir::RxOnly;
  return ToneDir::Both;
}

// Tone switched on from off goes on with the direction last seen on, also after a restart.
// Saved only when it changes.
static void rememberToneDir(const YaesuToneMode& mode) {
  if (mode.isOff() || toneDirOf(mode) == g_toneDir) return;
  g_toneDir = toneDirOf(mode);
  saveToneDirToNvs(g_toneDir);
}

static FeatureStatus catToneQuery(YaesuToneMode& out) {
  const RadioProfile& sp = currentProfile();
  if (currentProtocolType() == PROTO_CIV) {
    bool tone = false;
    bool tsql = false;
    if (!civQueryTone(sp, tone, tsql, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
    out = ctcssMode(tone || tsql, tsql);
    rememberToneDir(out);
    return FeatureStatus::Ok;
  }
  uint8_t code = 0;
  if (!asciiQueryToneCode(sp, code, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
  out = ctcssMode(code == 1 || code == 2, code == 1);
  if (code == 3) {
    out.tx = YaesuToneType::Dcs;
    out.rx = YaesuToneType::Dcs;
  }
  rememberToneDir(out);
  return FeatureStatus::Ok;
}

// Off, CTCSS both or CTCSS TX only; Unsupported for any other mode.
static FeatureStatus catToneSet(const YaesuToneMode& mode) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.setTone) return FeatureStatus::Unsupported;
  if (!mode.isOff() && (mode.type() != YaesuToneType::Ctcss || mode.mixed() || mode.rxOnly())) {
    return FeatureStatus::Unsupported;
  }
  const bool tx = mode.tx == YaesuToneType::Ctcss;
  const bool rx = mode.rx == YaesuToneType::Ctcss;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civSetTone(sp, tx && !rx, tx && rx);
  else ok = asciiSetToneCode(sp, !tx ? 0 : rx ? 1 : 2);
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::Failed);
}

FeatureStatus ctcssToneQuery(uint16_t& tenthsOut) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getCtcss) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryCtcssTenths(sp, tenthsOut, kCatReplyTimeoutMs);
  else if (currentProtocolType() == PROTO_YAESU_FTDX_ASCII) ok = asciiQueryCtcssTenths(sp, tenthsOut, kCatReplyTimeoutMs);
  else return FeatureStatus::Unsupported;
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus ctcssToneSet(uint16_t tenths, uint16_t& tenthsOut) {
  const RadioProfile& sp = currentProfile();
  const ProtocolType pt = currentProtocolType();
  if (!sp.caps.getCtcss || !sp.caps.setCtcss || (pt != PROTO_CIV && pt != PROTO_YAESU_FTDX_ASCII)) {
    return FeatureStatus::Unsupported;
  }
  if (!yaesuCtcssTenthsValid(tenths)) return FeatureStatus::Invalid;
  const bool ok = pt == PROTO_CIV ? civSetCtcssTenths(sp, tenths) : asciiSetCtcssTenths(sp, tenths);
  if (!ok) return failure(FeatureStatus::Failed);
  tenthsOut = tenths;
  return FeatureStatus::Ok;
}

FeatureStatus rptShiftQuery(RptShift& out) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getRptShift) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civQueryRptShift(sp, out, kCatReplyTimeoutMs);
  else if (currentProtocolType() == PROTO_YAESU_FTDX_ASCII) ok = asciiQueryRptShift(sp, out, kCatReplyTimeoutMs);
  else return FeatureStatus::Unsupported;
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::NoReply);
}

FeatureStatus rptShiftSet(RptShift shift) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.getRptShift || !sp.caps.setRptShift) return FeatureStatus::Unsupported;
  bool ok = false;
  if (currentProtocolType() == PROTO_CIV) ok = civSetRptShift(sp, shift);
  else if (currentProtocolType() == PROTO_YAESU_FTDX_ASCII) ok = asciiSetRptShift(sp, shift);
  else return FeatureStatus::Unsupported;
  return ok ? FeatureStatus::Ok : failure(FeatureStatus::Failed);
}

// ---- Tone squelch mode: the FT-8x7, and the CAT radios through the above ----

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
  hz = 0;
  if (isCatTone()) return catToneQuery(out);
  if (!toneSupported()) return FeatureStatus::Unsupported;
  if (!currentIsFt857Family()) {
    if (!queryFrequency(hz, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
    if (!ft817ToneModeHolds(hz)) return FeatureStatus::Unsupported;
    out = live.toneMode;
    rememberToneDir(out);
    return FeatureStatus::Ok;
  }
  bool known = false;
  if (!yaesuFt857QueryToneMode(known, out, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  if (!known) return FeatureStatus::Unsupported;
  rememberToneDir(out);
  return FeatureStatus::Ok;
}

FeatureStatus toneModeQuery(YaesuToneMode& out) {
  uint64_t hz = 0;
  return toneModeQueryAt(hz, out);
}

// Unsupported for a mode the radio has no command for, and for decode only unless the profile
// has it. Over CAT off, CTCSS both and CTCSS TX only.
static FeatureStatus toneModeSet(const YaesuToneMode& mode, uint64_t hz) {
  if (mode.rxOnly() && !currentProfile().caps.setToneDecodeOnly) return FeatureStatus::Unsupported;
  if (isCatTone()) return catToneSet(mode);
  return yaesuCatSetToneMode(mode, hz) ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}

// Sets mode, or both of its type when the radio cannot set it: always for decode only, for
// encode only when txOnlyToBoth. The direction set is remembered.
static FeatureStatus toneModeSetOrBoth(YaesuToneMode& mode, uint64_t hz, bool txOnlyToBoth) {
  FeatureStatus status = toneModeSet(mode, hz);
  if (status == FeatureStatus::Unsupported && (mode.rxOnly() || (mode.txOnly() && txOnlyToBoth))) {
    mode = toneModeOf(mode.type(), ToneDir::Both);
    status = toneModeSet(mode, hz);
  }
  if (status == FeatureStatus::Ok) rememberToneDir(mode);
  return status;
}

FeatureStatus toneOff() {
  if (!isCatTone() && !toneSupported()) return FeatureStatus::Unsupported;
  // The mode is read first so its direction is remembered; off is sent also without a reply. The
  // FT-817/818 remembers off for the band it is on; then no band is remembered.
  uint64_t hz = 0;
  YaesuToneMode current;
  const FeatureStatus status = toneModeQueryAt(hz, current);
  if (status != FeatureStatus::Ok && status != FeatureStatus::Unsupported) hz = 0;  // not known
  return toneModeSet(YaesuToneMode(), hz);
}

FeatureStatus toneTypeSet(YaesuToneType type, YaesuToneMode& out) {
  if (type == YaesuToneType::None || (!isCatTone() && !toneSupported())) return FeatureStatus::Unsupported;
  uint64_t hz = 0;
  YaesuToneMode current;
  const FeatureStatus status = toneModeQueryAt(hz, current);
  if (status != FeatureStatus::Ok && status != FeatureStatus::Unsupported) return status;
  // On, or from off or not known, the direction last seen goes on: the query remembers it.
  out = toneModeOf(type, g_toneDir);
  return toneModeSetOrBoth(out, hz, true);
}

FeatureStatus toneDirStep(YaesuToneMode& out) {
  if (!isCatTone() && !toneSupported()) return FeatureStatus::Unsupported;
  uint64_t hz = 0;
  YaesuToneMode current;
  const FeatureStatus status = toneModeQueryAt(hz, current);
  if (status != FeatureStatus::Ok) return status;
  if (current.isOff()) {
    out = current;
    return FeatureStatus::Ok;
  }
  // A mixed mode counts as both: its sent type goes on alone.
  out = toneModeOf(current.type(), nextToneDir(toneDirOf(current)));
  return toneModeSetOrBoth(out, hz, false);
}

FeatureStatus splitToneQuery(bool& on) {
  if (!toneSupported() || !currentIsFt857Family()) return FeatureStatus::Unsupported;
  bool known = false;
  if (!yaesuFt857QuerySplitTone(known, on, kFt8x7TimeoutMs)) return failure(FeatureStatus::NoReply);
  return known ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}
