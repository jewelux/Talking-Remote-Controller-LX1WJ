#include "radio_features.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "radio_catalog.h"
#include "radio_globals.h"
#include "radio_runtime.h"

// A radio exchange that went wrong: a timeout when the radio said nothing.
static FeatureStatus failure(FeatureStatus status) {
  return g_radioReplyTimedOut ? FeatureStatus::Timeout : status;
}

static bool isTs480() {
  return currentProtocolType() == PROTO_KENWOOD_ASCII && String(currentProfile().name).indexOf("TS-480") >= 0;
}

// ---- Noise reduction ----

FeatureStatus nrQuery(NrState& out) {
  if (!currentStoredProfile().caps.getNr) return FeatureStatus::Unsupported;
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

// The TS-480 NR has two levels: read the level (1 when that fails), step it.
static FeatureStatus ts480NrStep(NrState& out) {
  const StoredProfile& sp = currentStoredProfile();
  String line;
  int nextLevel = 1;
  if (transactAsciiCommand(sp.ascii.nrGet, line, sp.ascii.nrReplyPrefix, 800)) {
    int start = (int)strlen(sp.ascii.nrReplyPrefix);
    int semi = line.indexOf(';', start);
    if (semi < 0) semi = line.length();
    String value = line.substring(start, semi);
    value.trim();
    int currentLevel = value.toInt();
    if (currentLevel <= 0) nextLevel = 1;
    else if (currentLevel == 1) nextLevel = 2;
    else nextLevel = 0;
  }
  const char* cmd = (nextLevel == 0) ? "NR0;" : (nextLevel == 1) ? "NR1;" : "NR2;";
  if (!asciiPacketSendCommand(cmd)) return failure(FeatureStatus::Failed);
  live.nrOn = nextLevel != 0;
  live.nrValid = true;
  out.on = live.nrOn;
  out.level = (uint8_t)nextLevel;
  return FeatureStatus::Ok;
}

FeatureStatus nrToggle(NrState& out) {
  if (!currentStoredProfile().caps.setNr) return FeatureStatus::Unsupported;
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
  if (!currentStoredProfile().caps.getNotch) return FeatureStatus::Unsupported;
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
  if (!currentStoredProfile().caps.setNotch) return FeatureStatus::Unsupported;
  if (!live.notchValid && !refreshLiveNotch()) return failure(FeatureStatus::NoReply);
  if (currentProtocolType() != PROTO_CIV) return applyNotchState(!live.notchOn, NOTCH_WIDTH_UNKNOWN, out);

  if (!live.notchOn) return applyNotchState(true, NOTCH_WIDTH_NAR, out);
  if (!live.notchWidthValid && !refreshLiveNotchWidth()) return failure(FeatureStatus::NoReply);
  if (live.notchWidth == NOTCH_WIDTH_NAR) return applyNotchWidth(NOTCH_WIDTH_MID, out);
  if (live.notchWidth == NOTCH_WIDTH_MID) return applyNotchWidth(NOTCH_WIDTH_WIDE, out);
  return applyNotchState(false, NOTCH_WIDTH_UNKNOWN, out);
}
