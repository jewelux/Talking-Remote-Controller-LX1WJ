#include "radio_catalog.h"
#include "radio_globals.h"
#include "protocol_ascii.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_civ.h"
#include "protocol_ops_yaesu.h"
#include "radio_protocol.h"
#include "radio_state.h"
#include "radio_utils.h"

static bool isYaesuFtdxAsciiProfile(const StoredProfile& sp) {
  return sp.protocolType == PROTO_YAESU_FTDX_ASCII;
}

static bool ensureYaesuFtdxActiveVfoKnown(const StoredProfile& sp, uint32_t timeoutMs) {
  if (!isYaesuFtdxAsciiProfile(sp)) return false;
  if (live.activeVfoKnown) return true;
  bool activeVfoA = true;
  if (!asciiQueryActiveVfoA(sp, activeVfoA, timeoutMs)) return false;
  rememberActiveVfo(activeVfoA);
  return true;
}

static bool selectYaesuFtdxVfo(const StoredProfile& sp, bool targetVfoA) {
  if (!isYaesuFtdxAsciiProfile(sp)) return false;
  if (!(targetVfoA ? asciiSelectVfoA(sp) : asciiSelectVfoB(sp))) return false;
  rememberActiveVfo(targetVfoA);
  delay(60);
  return true;
}

// FT-8x7 VFO switches. The radio misses a command sent too soon after the A/B toggle, so each
// toggle is followed by a pause.
static constexpr uint32_t FT8X7_VFO_SETTLE_MS = 120;       // after switching to the other VFO
static constexpr uint32_t FT8X7_VFO_RETURN_GAP_MS = 180;   // before and after switching back

// Runs op once, or twice when retry is set and the first try fails.
template <typename Op>
static bool ft8x7RunWithRetry(bool retry, Op op) {
  bool ok = op();
  if (!ok && retry) {
    delay(FT8X7_VFO_SETTLE_MS);
    ok = op();
  }
  return ok;
}

// Runs op on the other VFO, then switches back, keeping the tracked VFO right throughout. A
// query (retry) gets one more try. False when op failed.
template <typename Op>
static bool ft8x7OnOtherVfo(bool retry, Op op) {
  if (!live.activeVfoKnown) rememberActiveVfo(true);
  const bool priorVfoA = live.activeVfoA;
  yaesuCatToggleVfo();
  rememberActiveVfo(!priorVfoA);
  delay(FT8X7_VFO_SETTLE_MS);
  const bool ok = ft8x7RunWithRetry(retry, op);
  delay(FT8X7_VFO_RETURN_GAP_MS);
  yaesuCatToggleVfo();
  rememberActiveVfo(priorVfoA);
  delay(FT8X7_VFO_RETURN_GAP_MS);
  return ok;
}

// Runs op on the target VFO, switching to it and back when it is not the active one. The
// FT-817 cannot report its VFO (0x55 bit 0, which Hamlib reads, does not follow A/B while the
// radio runs), so this goes by the tracked one.
template <typename Op>
static bool ft8x7OnVfo(bool targetVfoA, bool retry, Op op) {
  if (!live.activeVfoKnown) rememberActiveVfo(true);
  if (live.activeVfoA == targetVfoA) return ft8x7RunWithRetry(retry, op);
  return ft8x7OnOtherVfo(retry, op);
}

// Keep in sync with the protocol dispatch in the functions below.
bool protocolSupportsTuner() {
  const ProtocolType pt = currentProtocolType();
  return pt == PROTO_CIV || pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII;
}
bool protocolSupportsMonitor() { return currentProtocolType() == PROTO_CIV; }
bool protocolSupportsTransceive() { return currentProtocolType() == PROTO_CIV; }
bool protocolSupportsBandStack() { return currentProtocolType() == PROTO_CIV; }
// The RIT offset keys too. The FT-8x7 has RIT on/off only (queryRitEnabled, setRitEnabled).
bool protocolSupportsRit() { return currentProtocolType() == PROTO_CIV; }

bool queryFrequency(uint64_t& hzOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryFrequency(sp, hzOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryFrequency(sp, hzOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQueryFrequency(sp, hzOut, timeoutMs);
  return false;
}

bool setFrequency(uint64_t hz) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetFrequency(sp, hz);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetFrequency(sp, hz);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatSetFrequency(sp, hz);
  return false;
}

bool queryMode(uint8_t& modeOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryMode(sp, modeOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryMode(sp, modeOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQueryMode(sp, modeOut, timeoutMs);
  return false;
}

bool setMode(uint8_t mode, uint8_t filter) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetMode(sp, mode, filter);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetMode(sp, mode);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatSetMode(sp, mode);
  return false;
}

bool canSetMode(uint8_t mode) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (!sp.caps.setMode) return false;
  // CI-V sends the mode as it is; the others need the profile's code for it.
  if (pt == PROTO_CIV) return true;
  String code;
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) {
    return sp.ascii.modeSetFormat[0] && profileModeCodeForInternal(sp, mode, code);
  }
  if (pt == PROTO_YAESU_FT8X7) return profileModeCodeForInternal(sp, mode, code);
  return false;
}

bool querySMeterRaw(int32_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQuerySMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQuerySMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQuerySMeterRaw(sp, rawOut, timeoutMs);
  return false;
}

SMeterReading sMeterFromRaw(int32_t raw) {
  if (currentProtocolType() == PROTO_YAESU_FT8X7) return yaesuCatDecodeSMeter((uint8_t)raw);
  SMeterReading reading;
  reading.sUnits = smRawToS(raw);
  return reading;
}

bool queryPoMeterRaw(int32_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryPoMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryPoMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQueryPoMeterRaw(sp, rawOut, timeoutMs);
  return false;
}

bool querySWRRaw(int32_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQuerySWRRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQuerySWRRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQuerySWRRaw(sp, rawOut, timeoutMs);
  return false;
}

bool queryRfPowerLevel(uint16_t& valueOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryRfPowerLevel(sp, valueOut, timeoutMs);
  return false;
}

bool setRfPowerLevel(uint16_t value) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetRfPowerLevel(sp, value);
  return false;
}

bool queryNr(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNr(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryNr(sp, onOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuFt857QueryDnr(onOut, timeoutMs);
  return false;
}

bool setNr(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNr(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetNr(sp, on);
  return false;
}

bool queryNrLevel(uint16_t& valueOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNrLevel(sp, valueOut, timeoutMs);
  return false;
}

bool setNrLevel(uint16_t value) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNrLevel(sp, value);
  return false;
}

bool queryNb(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNb(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryNb(sp, onOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) {
    return currentIsFt817Family() ? yaesuFt817QueryNb(onOut, timeoutMs) : yaesuFt857QueryNb(onOut, timeoutMs);
  }
  return false;
}

bool setNb(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNb(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetNb(sp, on);
  return false;
}

bool queryNbLevel(uint16_t& valueOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNbLevel(sp, valueOut, timeoutMs);
  return false;
}

bool setNbLevel(uint16_t value) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNbLevel(sp, value);
  return false;
}

bool queryNotch(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNotch(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryNotch(sp, onOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuFt857QueryDnf(onOut, timeoutMs);
  return false;
}

bool setNotch(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNotch(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetNotch(sp, on);
  return false;
}

bool queryNotchWidth(NotchWidth& widthOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNotchWidth(sp, widthOut, timeoutMs);
  return false;
}

bool setNotchWidth(NotchWidth width) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNotchWidth(sp, width);
  return false;
}

bool queryPbtInner(uint16_t& valueOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryPbtInner(sp, valueOut, timeoutMs);
  return false;
}

bool setPbtInner(uint16_t value) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetPbtInner(sp, value);
  return false;
}

bool queryPbtOuter(uint16_t& valueOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryPbtOuter(sp, valueOut, timeoutMs);
  return false;
}

bool setPbtOuter(uint16_t value) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetPbtOuter(sp, value);
  return false;
}

bool queryActiveVfo(bool& vfoAOut, uint32_t timeoutMs) {
  const StoredProfile& sp = currentStoredProfile();
  if (currentProtocolType() != PROTO_YAESU_FT8X7 || !sp.caps.getVfo) return false;
  bool vfoB = false;
  if (!yaesuFt857QueryVfoB(vfoB, timeoutMs)) return false;
  vfoAOut = !vfoB;
  rememberActiveVfo(vfoAOut);
  return true;
}

bool queryDialLock(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryDialLock(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryLock(sp, onOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) {
    // The FT-817/818 and FT-857/897 keep the lock in their EEPROM, so a lock set on the front
    // panel counts too. Other variants get the state HamTRC last set.
    bool ok = false;
    if (currentIsFt817Family()) {
      ok = yaesuFt817QueryLock(onOut, timeoutMs);
    } else if (currentIsFt857Family()) {
      ok = yaesuFt857QueryLock(onOut, timeoutMs);
    } else {
      if (!live.lockKnown) return false;
      onOut = live.lockOn;
      return true;
    }
    if (!ok) return false;
    rememberDialLockState(onOut);
    return true;
  }
  return false;
}

bool setDialLock(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetDialLock(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetLock(sp, on);
  if (pt == PROTO_YAESU_FT8X7) {
    yaesuCatSetLockDocumentedRaw(on);
    rememberDialLockState(on);
    return true;
  }
  return false;
}

bool queryFilterShape(bool& softOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryFilterShape(sp, softOut, timeoutMs);
  return false;
}

bool setFilterShape(bool soft) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetFilterShape(sp, soft);
  return false;
}

bool queryFilterWidth(uint8_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryFilterWidth(sp, rawOut, timeoutMs);
  return false;
}

bool setFilterWidth(uint8_t raw) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetFilterWidth(sp, raw);
  return false;
}

bool queryMonitorEnabled(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryMonitorEnabled(sp, onOut, timeoutMs);
  return false;
}

bool setMonitorEnabled(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetMonitorEnabled(sp, on);
  return false;
}

bool queryMonitorLevel(uint16_t& valueOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryMonitorLevel(sp, valueOut, timeoutMs);
  return false;
}

bool setMonitorLevel(uint16_t value) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetMonitorLevel(sp, value);
  return false;
}

bool queryTransceiveEnabled(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryTransceiveEnabled(sp, onOut, timeoutMs);
  return false;
}

bool setTransceiveEnabled(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetTransceiveEnabled(sp, on);
  return false;
}

bool queryBandStackEntry(uint8_t bandCode, uint8_t registerCode, BandStackEntry& entryOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryBandStackEntry(sp, bandCode, registerCode, entryOut, timeoutMs);
  return false;
}

bool queryTuner(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryTuner(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryTuner(sp, onOut, timeoutMs);
  return false;
}

bool setTuner(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetTuner(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetTuner(sp, on);
  return false;
}

bool startTune() {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civStartTune(sp);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiStartTune(sp);
  return false;
}

bool queryRxTxStatus(bool& txOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryRxTxStatus(sp, txOut, timeoutMs);
  if (pt == PROTO_YAESU_FTDX_ASCII) {
    if (!ensureYaesuFtdxActiveVfoKnown(sp, timeoutMs)) return false;
    const char* txCode = live.activeVfoA ? "5" : "6";
    const char* rxCode = live.activeVfoA ? "7" : "8";
    bool flag = false;
    if (asciiQueryYaesuRadioInfoFlag(sp, txCode, flag, timeoutMs) && flag) {
      txOut = true;
      return true;
    }
    if (!asciiQueryYaesuRadioInfoFlag(sp, rxCode, flag, timeoutMs)) return false;
    txOut = false;
    return true;
  }
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getRxTx) {
    uint8_t raw = 0;
    if (!yaesuCatQueryStatusRaw(raw, timeoutMs)) return false;
    txOut = yaesuCatTxStatusTransmitting(raw);
    return true;
  }
  return false;
}

bool queryTxFrequency(uint64_t& hzOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryTxFrequency(sp, hzOut, timeoutMs);
  if (pt == PROTO_YAESU_FTDX_ASCII) {
    bool splitOn = false;
    if (querySplit(splitOn, timeoutMs) && splitOn) return queryVfoFrequency(false, hzOut, timeoutMs);
    return queryVfoFrequency(true, hzOut, timeoutMs);
  }
  return false;
}

bool selectVfoA() {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSelectVfoA(sp);
  if (pt == PROTO_YAESU_FTDX_ASCII) return selectYaesuFtdxVfo(sp, true);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII) return asciiSelectVfoA(sp);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfo && currentIsFt817Family()) {
    yaesuCatSelectVfoA();
    return true;
  }
  return false;
}

bool selectVfoB() {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSelectVfoB(sp);
  if (pt == PROTO_YAESU_FTDX_ASCII) return selectYaesuFtdxVfo(sp, false);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII) return asciiSelectVfoB(sp);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfo && currentIsFt817Family()) {
    yaesuCatSelectVfoB();
    return true;
  }
  return false;
}

bool queryVfoFrequency(bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryVfoFrequency(sp, targetVfoA, hzOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryVfoFrequency(sp, targetVfoA, hzOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getVfo && sp.caps.setVfo && currentIsFt817Family()) {
    return ft8x7OnVfo(targetVfoA, true, [&] { return queryFrequency(hzOut, timeoutMs); });
  }
  return false;
}

bool setVfoFrequency(bool targetVfoA, uint64_t hz) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetVfoFrequency(sp, targetVfoA, hz);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetVfoFrequency(sp, targetVfoA, hz);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfo && sp.caps.setFreq && currentIsFt817Family()) {
    return ft8x7OnVfo(targetVfoA, false, [&] { return setFrequency(hz); });
  }
  return false;
}

bool queryVfoMode(bool targetVfoA, uint8_t& modeOut, uint8_t& filterOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryVfoMode(sp, targetVfoA, modeOut, filterOut, timeoutMs);
  if (pt == PROTO_YAESU_FTDX_ASCII) {
    if (!ensureYaesuFtdxActiveVfoKnown(sp, timeoutMs)) return false;
    const bool priorVfoA = live.activeVfoA;
    if (priorVfoA != targetVfoA && !selectYaesuFtdxVfo(sp, targetVfoA)) return false;
    delay(40);
    bool ok = queryMode(modeOut, timeoutMs);
    if (!ok) {
      delay(60);
      ok = queryMode(modeOut, timeoutMs);
    }
    if (priorVfoA != targetVfoA) (void)selectYaesuFtdxVfo(sp, priorVfoA);
    if (!ok) return false;
    filterOut = 1;
    return true;
  }
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getVfoMode && sp.caps.setVfo && currentIsFt817Family()) {
    filterOut = 1;
    return ft8x7OnVfo(targetVfoA, true, [&] { return queryMode(modeOut, timeoutMs); });
  }
  return false;
}

bool setVfoMode(bool targetVfoA, uint8_t mode, uint8_t filter) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetVfoMode(sp, targetVfoA, mode, filter);
  if (pt == PROTO_YAESU_FTDX_ASCII) {
    (void)filter;
    if (!ensureYaesuFtdxActiveVfoKnown(sp, 800)) return false;
    const bool priorVfoA = live.activeVfoA;
    if (priorVfoA != targetVfoA && !selectYaesuFtdxVfo(sp, targetVfoA)) return false;
    delay(40);
    const bool ok = setMode(mode, 1);
    if (priorVfoA != targetVfoA) (void)selectYaesuFtdxVfo(sp, priorVfoA);
    return ok;
  }
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfoMode && sp.caps.setVfo && currentIsFt817Family()) {
    return ft8x7OnVfo(targetVfoA, false, [&] { return setMode(mode, filter); });
  }
  return false;
}

bool ft8x7CopyActiveVfoToOther() {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return false;
  uint64_t hz = 0;
  uint8_t mode = 0xFF;
  if (!queryFrequency(hz, 800)) return false;
  if (!queryMode(mode, 800)) return false;
  return ft8x7OnOtherVfo(false, [&] {
    if (!setFrequency(hz)) return false;
    delay(FT8X7_VFO_SETTLE_MS);
    return setMode(mode, 1);
  });
}

bool ft8x7QueryOtherVfoFrequency(uint64_t& hzOut, uint32_t timeoutMs) {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return false;
  return ft8x7OnOtherVfo(true, [&] { return queryFrequency(hzOut, timeoutMs); });
}

bool ft8x7SetOtherVfoFrequency(uint64_t hz) {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return false;
  return ft8x7OnOtherVfo(false, [&] { return setFrequency(hz); });
}

bool querySplit(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQuerySplit(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQuerySplit(sp, onOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getSplit) return yaesuCatQuerySplit(onOut, timeoutMs);
  return false;
}

bool setSplit(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetSplit(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetSplit(sp, on);
  if (pt == PROTO_YAESU_FT8X7) {
    yaesuCatSetSplit(on);
    return true;
  }
  return false;
}

bool queryRitEnabled(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryRitEnabled(sp, onOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuFt8x7QueryRit(onOut, timeoutMs);
  return false;
}

bool setRitEnabled(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetRitEnabled(sp, on);
  if (pt == PROTO_YAESU_FT8X7) return yaesuFt8x7SetRit(on, 300);
  return false;
}

bool toggleRitEnabled(bool& onOut, uint32_t timeoutMs) {
  if (currentProtocolType() == PROTO_YAESU_FT8X7) return yaesuFt8x7ToggleRit(onOut, timeoutMs);
  bool on = false;
  if (!queryRitEnabled(on, timeoutMs) || !setRitEnabled(!on)) return false;
  onOut = !on;
  return true;
}

bool queryRitOffsetHz(int32_t& hzOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryRitOffsetHz(sp, hzOut, timeoutMs);
  return false;
}

bool setRitOffsetHz(int32_t hz) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetRitOffsetHz(sp, hz);
  return false;
}
