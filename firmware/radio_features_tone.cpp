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

static FeatureStatus catToneQuery(YaesuToneMode& out) {
  const RadioProfile& sp = currentProfile();
  if (currentProtocolType() == PROTO_CIV) {
    bool tone = false;
    bool tsql = false;
    if (!civQueryTone(sp, tone, tsql, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
    out = ctcssMode(tone || tsql, tsql);
    return FeatureStatus::Ok;
  }
  uint8_t code = 0;
  if (!asciiQueryToneCode(sp, code, kCatReplyTimeoutMs)) return failure(FeatureStatus::NoReply);
  out = ctcssMode(code == 1 || code == 2, code == 1);
  if (code == 3) {
    out.tx = YaesuToneType::Dcs;
    out.rx = YaesuToneType::Dcs;
  }
  return FeatureStatus::Ok;
}

// Off, CTCSS both or CTCSS TX only.
static FeatureStatus catToneSet(const YaesuToneMode& mode) {
  const RadioProfile& sp = currentProfile();
  if (!sp.caps.setTone) return FeatureStatus::Unsupported;
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
  hz = 0;
  if (isCatTone()) return catToneQuery(out);
  if (!toneSupported()) return FeatureStatus::Unsupported;
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
  if (isCatTone()) return catToneSet(YaesuToneMode());
  if (!toneSupported()) return FeatureStatus::Unsupported;
  // The FT-817/818 remembers off for the band it is on. Off is sent also when the frequency
  // cannot be read; then no band is remembered.
  uint64_t hz = 0;
  if (!currentIsFt857Family() && !queryFrequency(hz, kFt8x7TimeoutMs)) hz = 0;  // not known
  return yaesuCatSetToneMode(YaesuToneMode(), hz) ? FeatureStatus::Ok : FeatureStatus::Unsupported;
}

FeatureStatus toneTypeSet(YaesuToneType type, YaesuToneMode& out) {
  // Over CAT CTCSS only, keeping TX only; from off or DCS, both.
  if (isCatTone()) {
    if (type != YaesuToneType::Ctcss) return FeatureStatus::Unsupported;
    YaesuToneMode current;
    const FeatureStatus status = catToneQuery(current);
    if (status != FeatureStatus::Ok) return status;
    const YaesuToneMode want = ctcssMode(true, !(current.txOnly() && current.tx == YaesuToneType::Ctcss));
    const FeatureStatus set = catToneSet(want);
    if (set == FeatureStatus::Ok) out = want;
    return set;
  }
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
  // Over CAT: both -> TX only -> both, CTCSS only.
  if (isCatTone()) {
    YaesuToneMode current;
    const FeatureStatus status = catToneQuery(current);
    if (status != FeatureStatus::Ok) return status;
    if (current.isOff()) {
      out = current;
      return FeatureStatus::Ok;
    }
    if (current.type() != YaesuToneType::Ctcss) return FeatureStatus::Unsupported;
    const YaesuToneMode want = ctcssMode(true, current.txOnly());
    const FeatureStatus set = catToneSet(want);
    if (set == FeatureStatus::Ok) out = want;
    return set;
  }
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
