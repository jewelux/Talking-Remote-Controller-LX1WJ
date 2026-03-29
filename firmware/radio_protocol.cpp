#include "radio_catalog.h"
#include "radio_globals.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_civ.h"
#include "protocol_ops_yaesu.h"
#include "radio_protocol.h"
#include "radio_state.h"

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

bool querySMeterRaw(int32_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQuerySMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQuerySMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQuerySMeterRaw(sp, rawOut, timeoutMs);
  return false;
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

bool queryNr(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNr(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryNr(sp, onOut, timeoutMs);
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

bool queryDialLock(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryDialLock(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryLock(sp, onOut, timeoutMs);
  return false;
}

bool setDialLock(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetDialLock(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetLock(sp, on);
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
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getRxTx) {
    uint8_t raw = 0;
    if (!yaesuCatQueryStatusRaw(raw, timeoutMs)) return false;
    txOut = (raw & 0x01) == 0;
    return true;
  }
  return false;
}

bool queryTxFrequency(uint64_t& hzOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryTxFrequency(sp, hzOut, timeoutMs);
  return false;
}

bool selectVfoA() {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSelectVfoA(sp);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSelectVfoA(sp);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfo && currentProfileVariantIs("ft817")) return yaesuCatSelectVfoA();
  return false;
}

bool selectVfoB() {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSelectVfoB(sp);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSelectVfoB(sp);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfo && currentProfileVariantIs("ft817")) return yaesuCatSelectVfoB();
  return false;
}

bool queryVfoFrequency(bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryVfoFrequency(sp, targetVfoA, hzOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryVfoFrequency(sp, targetVfoA, hzOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getVfo && sp.caps.setVfo && currentProfileVariantIs("ft817")) {
    if (!(targetVfoA ? yaesuCatSelectVfoA() : yaesuCatSelectVfoB())) return false;
    delay(60);
    return queryFrequency(hzOut, timeoutMs);
  }
  return false;
}

bool setVfoFrequency(bool targetVfoA, uint64_t hz) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetVfoFrequency(sp, targetVfoA, hz);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetVfoFrequency(sp, targetVfoA, hz);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfo && sp.caps.setFreq && currentProfileVariantIs("ft817")) {
    if (!(targetVfoA ? yaesuCatSelectVfoA() : yaesuCatSelectVfoB())) return false;
    delay(60);
    return setFrequency(hz);
  }
  return false;
}

bool queryVfoMode(bool targetVfoA, uint8_t& modeOut, uint8_t& filterOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryVfoMode(sp, targetVfoA, modeOut, filterOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getVfoMode && sp.caps.setVfo && currentProfileVariantIs("ft817")) {
    if (!(targetVfoA ? yaesuCatSelectVfoA() : yaesuCatSelectVfoB())) return false;
    delay(60);
    filterOut = 1;
    return queryMode(modeOut, timeoutMs);
  }
  return false;
}

bool setVfoMode(bool targetVfoA, uint8_t mode, uint8_t filter) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetVfoMode(sp, targetVfoA, mode, filter);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.setVfoMode && sp.caps.setVfo && currentProfileVariantIs("ft817")) {
    if (!(targetVfoA ? yaesuCatSelectVfoA() : yaesuCatSelectVfoB())) return false;
    delay(60);
    return setMode(mode, filter);
  }
  return false;
}

bool querySplit(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQuerySplit(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQuerySplit(sp, onOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7 && sp.caps.getSplit) {
    if (currentProfileVariantIs("ft857_897") && g_ft8x7SplitKnown) {
      onOut = g_ft8x7SplitOn;
      return true;
    }
    uint8_t raw = 0;
    if (!yaesuCatQueryStatusRaw(raw, timeoutMs)) return false;
    if (currentProfileVariantIs("ft857_897")) onOut = (raw & 0x04) != 0;
    else onOut = (raw & 0x04) == 0;
    rememberSplitState(onOut);
    return true;
  }
  return false;
}

bool setSplit(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetSplit(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetSplit(sp, on);
  if (pt == PROTO_YAESU_FT8X7) {
    if (!yaesuCatSetSplit(on)) return false;
    if (currentProfileVariantIs("ft857_897")) rememberSplitState(on);
    return true;
  }
  return false;
}

bool queryRitEnabled(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryRitEnabled(sp, onOut, timeoutMs);
  return false;
}

bool setRitEnabled(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetRitEnabled(sp, on);
  return false;
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
