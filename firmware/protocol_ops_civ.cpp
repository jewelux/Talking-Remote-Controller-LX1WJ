#include "protocol_ops_civ.h"

#include "engine_civ.h"
#include "protocol_ascii.h"
#include "protocol_civ.h"
#include "radio_protocol.h"
#include "radio_state.h"
#include "radio_utils.h"

#include <string.h>

static bool civQueryToggleSub(uint8_t subcmd, bool& onOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {subcmd};
  civSend(0x16, sub, 1);
  CivDecoded d;
  if (!waitReply(0x16, d, timeoutMs) || d.payloadLen < 2 || d.payload[0] != subcmd) return false;
  onOut = d.payload[1] != 0x00;
  return true;
}

static bool civSetToggleSub(uint8_t subcmd, bool on) {
  civFlushInput();
  const uint8_t pl[] = {subcmd, (uint8_t)(on ? 0x01 : 0x00)};
  civSend(0x16, pl, 2);
  return true;
}

static bool civQuery1CByte(uint8_t subcmd, uint8_t& valueOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {subcmd};
  civSend(0x1C, sub, 1);
  CivDecoded d;
  if (!waitReply(0x1C, d, timeoutMs) || d.payloadLen < 2 || d.payload[0] != subcmd) return false;
  valueOut = d.payload[1];
  return true;
}

static bool civSet1CByte(uint8_t subcmd, uint8_t value) {
  civFlushInput();
  const uint8_t pl[] = {subcmd, value};
  civSend(0x1C, pl, 2);
  return true;
}

static bool civQuery21Byte(uint8_t subcmd, uint8_t& valueOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {subcmd};
  civSend(0x21, sub, 1);
  CivDecoded d;
  if (!waitReply(0x21, d, timeoutMs) || d.payloadLen < 2 || d.payload[0] != subcmd) return false;
  valueOut = d.payload[1];
  return true;
}

static bool civSet21Byte(uint8_t subcmd, uint8_t value) {
  civFlushInput();
  const uint8_t pl[] = {subcmd, value};
  civSend(0x21, pl, 2);
  return true;
}

static bool civWriteRitOffsetFrame(const uint8_t* payload, size_t payloadLen) {
  civFlushInput();
  civSend(0x21, payload, payloadLen);
  delay(40);
  pumpIncoming(40);
  return true;
}

static void civSettleAfterWrite() {
  delay(40);
  pumpIncoming(40);
}

static bool civQueryMenuByte(uint8_t group, uint16_t item, uint8_t& valueOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {group, (uint8_t)((item >> 8) & 0xFF), (uint8_t)(item & 0xFF)};
  civSend(0x1A, sub, 3);
  CivDecoded d;
  if (!waitReply(0x1A, d, timeoutMs) || d.payloadLen < 4) return false;
  if (d.payload[0] != group || d.payload[1] != ((item >> 8) & 0xFF) || d.payload[2] != (item & 0xFF)) return false;
  valueOut = d.payload[3];
  return true;
}

static bool civSetMenuByte(uint8_t group, uint16_t item, uint8_t value) {
  civFlushInput();
  const uint8_t pl[] = {group, (uint8_t)((item >> 8) & 0xFF), (uint8_t)(item & 0xFF), value};
  civSend(0x1A, pl, 4);
  return true;
}

static bool civQuery14Value(uint8_t subcmd, uint16_t& valueOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {subcmd};
  civSend(0x14, sub, 1);
  CivDecoded d;
  if (!waitReply(0x14, d, timeoutMs) || d.payloadLen < 3 || d.payload[0] != subcmd) return false;
  valueOut = (uint16_t)bcdDigitsToInt(d.payload + 1, d.payloadLen - 1);
  return valueOut <= 255;
}

static bool civSet14Value(uint8_t subcmd, uint16_t value) {
  if (value > 255) return false;
  const uint8_t hundreds = (uint8_t)((value / 100) % 10);
  const uint8_t tens = (uint8_t)((value / 10) % 10);
  const uint8_t ones = (uint8_t)(value % 10);
  civFlushInput();
  const uint8_t pl[] = {subcmd, hundreds, (uint8_t)((tens << 4) | ones)};
  civSend(0x14, pl, 3);
  return true;
}

static bool isIcom7760Profile(const StoredProfile& sp) {
  return sp.protocolType == PROTO_CIV && strcmp(sp.variant, "ic7760") == 0;
}

static uint8_t civMainSubSelector(bool main) {
  return main ? 0x00 : 0x01;
}

static void civSendMainSub(uint8_t mainSub, uint8_t cmd, const uint8_t* data, size_t dataLen) {
  uint8_t pl[32];
  if (dataLen + 2 > sizeof(pl)) return;
  pl[0] = mainSub;
  pl[1] = cmd;
  if (data && dataLen) memcpy(pl + 2, data, dataLen);
  civSend(0x29, pl, dataLen + 2);
}

static bool waitMainSubReply(uint8_t mainSub, uint8_t innerCmd, CivDecoded& out, uint32_t timeoutMs) {
  CivDecoded d;
  if (!waitReply(0x29, d, timeoutMs) || d.payloadLen < 2) return false;
  if (d.payload[0] != mainSub || d.payload[1] != innerCmd) return false;
  out = d;
  out.payload = d.payload + 2;
  out.payloadLen = d.payloadLen - 2;
  return true;
}

static bool civQueryMainSubFrequency(uint8_t mainSub, uint64_t& hzOut, uint32_t timeoutMs) {
  civFlushInput();
  civSendMainSub(mainSub, 0x03, nullptr, 0);
  CivDecoded d;
  if (!waitMainSubReply(mainSub, 0x03, d, timeoutMs) || d.payloadLen < 5) return false;
  hzOut = decodeBcdFrequencyHz(d.payload, 5);
  return true;
}

static bool civSetMainSubFrequency(uint8_t mainSub, uint64_t hz) {
  uint8_t bcd[5] = {0};
  uint64_t v = hz;
  for (int i = 0; i < 5; ++i) {
    uint8_t d1 = v % 10; v /= 10;
    uint8_t d2 = v % 10; v /= 10;
    bcd[i] = (d2 << 4) | d1;
  }
  civFlushInput();
  civSendMainSub(mainSub, 0x05, bcd, 5);
  civSettleAfterWrite();
  uint64_t readBack = 0;
  return civQueryMainSubFrequency(mainSub, readBack, 800) && readBack == hz;
}

static bool civQueryMainSubMode(uint8_t mainSub, uint8_t& modeOut, uint8_t& filterOut, uint32_t timeoutMs) {
  civFlushInput();
  civSendMainSub(mainSub, 0x04, nullptr, 0);
  CivDecoded d;
  if (!waitMainSubReply(mainSub, 0x04, d, timeoutMs) || d.payloadLen < 1) return false;
  modeOut = d.payload[0];
  filterOut = d.payloadLen >= 2 ? d.payload[1] : 1;
  return true;
}

static bool civSetMainSubMode(uint8_t mainSub, uint8_t mode, uint8_t filter) {
  const uint8_t pl[] = {mode, filter};
  civFlushInput();
  civSendMainSub(mainSub, 0x06, pl, 2);
  civSettleAfterWrite();
  uint8_t readMode = 0xFF;
  uint8_t readFilter = 0xFF;
  return civQueryMainSubMode(mainSub, readMode, readFilter, 800) &&
         readMode == mode &&
         readFilter == filter;
}

static bool decodeIcomNotchWidth(uint8_t raw, NotchWidth& widthOut) {
  switch (raw) {
    case 0x00: widthOut = NOTCH_WIDTH_WIDE; return true;
    case 0x01: widthOut = NOTCH_WIDTH_MID; return true;
    case 0x02: widthOut = NOTCH_WIDTH_NAR; return true;
    default: return false;
  }
}

static uint8_t encodeIcomNotchWidth(NotchWidth width) {
  switch (width) {
    case NOTCH_WIDTH_WIDE: return 0x00;
    case NOTCH_WIDTH_MID: return 0x01;
    case NOTCH_WIDTH_NAR: return 0x02;
    default: return 0x02;
  }
}

bool civQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq) return false;
  civFlushInput();
  civSend(0x03, nullptr, 0);
  CivDecoded d;
  if (!waitReply(0x03, d, timeoutMs) || d.payloadLen < 5) return false;
  hzOut = decodeBcdFrequencyHz(d.payload, 5);
  return true;
}

bool civSetFrequency(const StoredProfile& sp, uint64_t hz) {
  if (!sp.caps.setFreq) return false;
  uint8_t bcd[5] = {0};
  uint64_t v = hz;
  for (int i = 0; i < 5; ++i) {
    uint8_t d1 = v % 10; v /= 10;
    uint8_t d2 = v % 10; v /= 10;
    bcd[i] = (d2 << 4) | d1;
  }
  civFlushInput();
  civSend(0x05, bcd, 5);
  delay(40);
  pumpIncoming(40);
  uint64_t readHz = 0;
  return queryFrequency(readHz, 800) && (readHz == hz);
}

bool civQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode) return false;
  civFlushInput();
  civSend(0x04, nullptr, 0);
  CivDecoded d;
  if (!waitReply(0x04, d, timeoutMs) || d.payloadLen < 1) return false;
  modeOut = d.payload[0];
  return true;
}

bool civSetMode(const StoredProfile& sp, uint8_t mode, uint8_t filter) {
  if (!sp.caps.setMode) return false;
  uint8_t pl[2] = {mode, filter};
  civFlushInput();
  civSend(0x06, pl, 2);
  return true;
}

bool civQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter) return false;
  civFlushInput();
  const uint8_t sub[] = {0x02};
  civSend(0x15, sub, 1);
  CivDecoded d;
  if (!waitReply(0x15, d, timeoutMs) || d.payloadLen < 2 || d.payload[0] != 0x02) return false;
  rawOut = bcdDigitsToInt(d.payload + 1, d.payloadLen - 1);
  return true;
}

bool civQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower) return false;
  civFlushInput();
  const uint8_t sub[] = {0x11};
  civSend(0x15, sub, 1);
  CivDecoded d;
  if (!waitReply(0x15, d, timeoutMs) || d.payloadLen < 2 || d.payload[0] != 0x11) return false;
  rawOut = bcdDigitsToInt(d.payload + 1, d.payloadLen - 1);
  return true;
}

bool civQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr) return false;
  civFlushInput();
  const uint8_t sub[] = {0x12};
  civSend(0x15, sub, 1);
  CivDecoded d;
  if (!waitReply(0x15, d, timeoutMs) || d.payloadLen < 2 || d.payload[0] != 0x12) return false;
  rawOut = bcdDigitsToInt(d.payload + 1, d.payloadLen - 1);
  return true;
}

bool civQueryRfPowerLevel(const StoredProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (!sp.caps.getRfPower) return false;
  return civQuery14Value(0x0A, valueOut, timeoutMs);
}

bool civSetRfPowerLevel(const StoredProfile& sp, uint16_t value) {
  if (!sp.caps.setRfPower) return false;
  if (!civSet14Value(0x0A, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryRfPowerLevel(sp, readBack, 800) && readBack == value;
}

bool civQueryNr(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNr) return false;
  return civQueryToggleSub(0x40, onOut, timeoutMs);
}

bool civSetNr(const StoredProfile& sp, bool on) {
  if (!sp.caps.setNr) return false;
  if (!civSetToggleSub(0x40, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryNr(sp, readBack, 800) && (readBack == on);
}

bool civQueryNrLevel(const StoredProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  return civQuery14Value(0x06, valueOut, timeoutMs);
}

bool civSetNrLevel(const StoredProfile& sp, uint16_t value) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet14Value(0x06, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  if (civQueryNrLevel(sp, readBack, 800) && readBack == value) return true;

  bool nrOn = false;
  if (civQueryNr(sp, nrOn, 800) && !nrOn) {
    if (!civSetNr(sp, true)) return false;
    if (!civSet14Value(0x06, value)) return false;
    civSettleAfterWrite();
    if (civQueryNrLevel(sp, readBack, 800) && readBack == value) return true;
  }

  return civQueryNrLevel(sp, readBack, 800) && (readBack >= (value > 1 ? value - 1 : 0)) && (readBack <= value + 1);
}

bool civQueryNb(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNb) return false;
  return civQueryToggleSub(0x22, onOut, timeoutMs);
}

bool civSetNb(const StoredProfile& sp, bool on) {
  if (!sp.caps.setNb) return false;
  if (!civSetToggleSub(0x22, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryNb(sp, readBack, 800) && (readBack == on);
}

bool civQueryNbLevel(const StoredProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  return civQuery14Value(0x12, valueOut, timeoutMs);
}

bool civSetNbLevel(const StoredProfile& sp, uint16_t value) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet14Value(0x12, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryNbLevel(sp, readBack, 800) && (readBack == value);
}

bool civQueryNotch(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNotch) return false;
  return civQueryToggleSub(0x48, onOut, timeoutMs);
}

bool civSetNotch(const StoredProfile& sp, bool on) {
  if (!sp.caps.setNotch) return false;
  if (!civSetToggleSub(0x48, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryNotch(sp, readBack, 800) && (readBack == on);
}

bool civQueryNotchWidth(const StoredProfile& sp, NotchWidth& widthOut, uint32_t timeoutMs) {
  if (!sp.caps.getNotch) return false;
  uint8_t raw = 0;
  if (!civQueryMenuByte(0x05, 0x0024, raw, timeoutMs)) return false;
  return decodeIcomNotchWidth(raw, widthOut);
}

bool civSetNotchWidth(const StoredProfile& sp, NotchWidth width) {
  if (!sp.caps.setNotch || width == NOTCH_WIDTH_UNKNOWN) return false;
  if (!civSetMenuByte(0x05, 0x0024, encodeIcomNotchWidth(width))) return false;
  civSettleAfterWrite();
  NotchWidth readBack = NOTCH_WIDTH_UNKNOWN;
  return civQueryNotchWidth(sp, readBack, 800) && readBack == width;
}

bool civQueryPbtInner(const StoredProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  return civQuery14Value(0x07, valueOut, timeoutMs);
}

bool civSetPbtInner(const StoredProfile& sp, uint16_t value) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet14Value(0x07, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryPbtInner(sp, readBack, 800) &&
         readBack >= (value > 1 ? value - 1 : 0) &&
         readBack <= value + 1;
}

bool civQueryPbtOuter(const StoredProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  return civQuery14Value(0x08, valueOut, timeoutMs);
}

bool civSetPbtOuter(const StoredProfile& sp, uint16_t value) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet14Value(0x08, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryPbtOuter(sp, readBack, 800) &&
         readBack >= (value > 1 ? value - 1 : 0) &&
         readBack <= value + 1;
}

bool civQueryDialLock(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  uint8_t raw = 0;
  if (!civQueryMenuByte(0x05, 0x0022, raw, timeoutMs)) return false;
  if (raw > 0x01) return false;
  onOut = raw == 0x01;
  return true;
}

bool civSetDialLock(const StoredProfile& sp, bool on) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSetMenuByte(0x05, 0x0022, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryDialLock(sp, readBack, 800) && readBack == on;
}

bool civQueryFilterShape(const StoredProfile& sp, bool& softOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  uint8_t raw = 0;
  if (!civQueryMenuByte(0x05, 0x0023, raw, timeoutMs)) return false;
  if (raw > 0x01) return false;
  softOut = raw == 0x01;
  return true;
}

bool civSetFilterShape(const StoredProfile& sp, bool soft) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSetMenuByte(0x05, 0x0023, soft ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryFilterShape(sp, readBack, 800) && readBack == soft;
}

bool civQueryFilterWidth(const StoredProfile& sp, uint8_t& rawOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  return civQueryMenuByte(0x05, 0x0025, rawOut, timeoutMs);
}

bool civSetFilterWidth(const StoredProfile& sp, uint8_t raw) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSetMenuByte(0x05, 0x0025, raw)) return false;
  civSettleAfterWrite();
  uint8_t readBack = 0xFF;
  return civQueryFilterWidth(sp, readBack, 800) && readBack == raw;
}

bool civQueryMonitorEnabled(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  uint8_t value = 0;
  if (!civQuery1CByte(0x02, value, timeoutMs)) return false;
  if (value > 0x01) return false;
  onOut = value == 0x01;
  return true;
}

bool civSetMonitorEnabled(const StoredProfile& sp, bool on) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet1CByte(0x02, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryMonitorEnabled(sp, readBack, 800) && readBack == on;
}

bool civQueryMonitorLevel(const StoredProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  return civQuery14Value(0x15, valueOut, timeoutMs);
}

bool civSetMonitorLevel(const StoredProfile& sp, uint16_t value) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet14Value(0x15, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryMonitorLevel(sp, readBack, 800) && readBack == value;
}

bool civQueryTransceiveEnabled(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  uint8_t raw = 0;
  if (!civQueryMenuByte(0x05, 0x0071, raw, timeoutMs)) return false;
  if (raw > 0x01) return false;
  onOut = raw == 0x01;
  return true;
}

bool civSetTransceiveEnabled(const StoredProfile& sp, bool on) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSetMenuByte(0x05, 0x0071, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryTransceiveEnabled(sp, readBack, 800) && readBack == on;
}

bool civQueryBandStackEntry(const StoredProfile& sp, uint8_t bandCode, uint8_t registerCode, BandStackEntry& entryOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (bandCode < 0x01 || bandCode > 0x11) return false;
  if (registerCode < 0x01 || registerCode > 0x03) return false;
  civFlushInput();
  const uint8_t sub[] = {0x01, bandCode, registerCode};
  civSend(0x1A, sub, 3);
  CivDecoded d;
  if (!waitReply(0x1A, d, timeoutMs) || d.payloadLen < 10) return false;
  if (d.payload[0] != 0x01 || d.payload[1] != bandCode || d.payload[2] != registerCode) return false;
  entryOut.bandCode = bandCode;
  entryOut.registerCode = registerCode;
  entryOut.freqHz = decodeBcdFrequencyHz(d.payload + 3, 5);
  entryOut.mode = d.payload[8];
  entryOut.filter = d.payload[9];
  return true;
}

bool civQueryTuner(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  uint8_t value = 0;
  if (!civQuery1CByte(0x01, value, timeoutMs)) return false;
  if (value > 0x02) return false;
  onOut = value != 0x00;
  return true;
}

bool civSetTuner(const StoredProfile& sp, bool on) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet1CByte(0x01, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryTuner(sp, readBack, 800) && (readBack == on);
}

bool civStartTune(const StoredProfile& sp) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet1CByte(0x01, 0x02)) return false;
  civSettleAfterWrite();
  return true;
}

bool civQueryRxTxStatus(const StoredProfile& sp, bool& txOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  uint8_t value = 0;
  if (!civQuery1CByte(0x00, value, timeoutMs)) return false;
  if (value > 0x01) return false;
  txOut = value == 0x01;
  return true;
}

bool civQueryTxFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  civFlushInput();
  const uint8_t sub[] = {0x03};
  civSend(0x1C, sub, 1);
  CivDecoded d;
  if (!waitReply(0x1C, d, timeoutMs) || d.payloadLen < 6 || d.payload[0] != 0x03) return false;
  hzOut = decodeBcdFrequencyHz(d.payload + 1, 5);
  return true;
}

bool civSelectVfoA(const StoredProfile& sp) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (isIcom7760Profile(sp)) {
    rememberActiveVfo(true);
    return true;
  }
  civFlushInput();
  const uint8_t sub[] = {0x00};
  civSend(0x07, sub, 1);
  civSettleAfterWrite();
  rememberActiveVfo(true);
  return true;
}

bool civSelectVfoB(const StoredProfile& sp) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (isIcom7760Profile(sp)) {
    rememberActiveVfo(false);
    return true;
  }
  civFlushInput();
  const uint8_t sub[] = {0x01};
  civSend(0x07, sub, 1);
  civSettleAfterWrite();
  rememberActiveVfo(false);
  return true;
}

static uint8_t civEncodeVfoSelector(bool targetVfoA) {
  if (!live.activeVfoKnown) return targetVfoA ? 0x00 : 0x01;
  return (targetVfoA == live.activeVfoA) ? 0x00 : 0x01;
}

static bool civQuerySelectedOrUnselectedFrequencyRaw(uint8_t selector, uint64_t& hzOut, uint32_t timeoutMs) {
  civFlushInput();
  civSend(0x25, &selector, 1);
  CivDecoded d;
  if (!waitReply(0x25, d, timeoutMs) || d.payloadLen < 6) return false;
  if (d.payload[0] != selector) return false;
  hzOut = decodeBcdFrequencyHz(d.payload + 1, 5);
  return true;
}

static bool civEnsureActiveVfoKnown(const StoredProfile& sp) {
  if (live.activeVfoKnown) return true;
  if (sp.protocolType != PROTO_CIV) return false;

  uint64_t currentHz = 0;
  if (!civQueryFrequency(sp, currentHz, 800)) return false;

  const bool hadKnown = live.activeVfoKnown;
  const bool priorVfoA = live.activeVfoA;

  if (!civSelectVfoA(sp)) return false;
  uint64_t vfoAHz = 0;
  if (!civQueryFrequency(sp, vfoAHz, 800)) return false;

  bool activeWasA = (vfoAHz == currentHz);
  rememberActiveVfo(activeWasA);

  if (activeWasA) {
    if (!hadKnown || !priorVfoA) {
      // Restore original active VFO if it was not A.
      // If current was already A, no further action is needed.
    }
    return true;
  }

  if (!civSelectVfoB(sp)) return false;
  rememberActiveVfo(false);
  return true;
}

bool civQueryVfoFrequency(const StoredProfile& sp, bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (isIcom7760Profile(sp)) return civQueryMainSubFrequency(civMainSubSelector(targetVfoA), hzOut, timeoutMs);
  if (!civEnsureActiveVfoKnown(sp)) return false;
  const uint8_t selector = civEncodeVfoSelector(targetVfoA);
  return civQuerySelectedOrUnselectedFrequencyRaw(selector, hzOut, timeoutMs);
}

bool civSetVfoFrequency(const StoredProfile& sp, bool targetVfoA, uint64_t hz) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (isIcom7760Profile(sp)) return civSetMainSubFrequency(civMainSubSelector(targetVfoA), hz);
  if (!civEnsureActiveVfoKnown(sp)) return false;
  uint8_t pl[6] = {0};
  pl[0] = civEncodeVfoSelector(targetVfoA);
  uint64_t v = hz;
  for (int i = 0; i < 5; ++i) {
    uint8_t d1 = v % 10; v /= 10;
    uint8_t d2 = v % 10; v /= 10;
    pl[i + 1] = (uint8_t)((d2 << 4) | d1);
  }
  civFlushInput();
  civSend(0x25, pl, 6);
  civSettleAfterWrite();
  uint64_t readBack = 0;
  return civQueryVfoFrequency(sp, targetVfoA, readBack, 800) && readBack == hz;
}

bool civQueryVfoMode(const StoredProfile& sp, bool targetVfoA, uint8_t& modeOut, uint8_t& filterOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (isIcom7760Profile(sp)) return civQueryMainSubMode(civMainSubSelector(targetVfoA), modeOut, filterOut, timeoutMs);
  if (!civEnsureActiveVfoKnown(sp)) return false;
  const uint8_t selector = civEncodeVfoSelector(targetVfoA);
  civFlushInput();
  civSend(0x26, &selector, 1);
  CivDecoded d;
  if (!waitReply(0x26, d, timeoutMs) || d.payloadLen < 4) return false;
  if (d.payload[0] != selector) return false;
  modeOut = d.payload[1];
  filterOut = d.payload[3];
  return true;
}

bool civSetVfoMode(const StoredProfile& sp, bool targetVfoA, uint8_t mode, uint8_t filter) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (isIcom7760Profile(sp)) return civSetMainSubMode(civMainSubSelector(targetVfoA), mode, filter);
  if (!civEnsureActiveVfoKnown(sp)) return false;
  const uint8_t pl[] = {civEncodeVfoSelector(targetVfoA), mode, 0x00, filter};
  civFlushInput();
  civSend(0x26, pl, 4);
  civSettleAfterWrite();
  uint8_t readMode = 0xFF;
  uint8_t readFilter = 0xFF;
  return civQueryVfoMode(sp, targetVfoA, readMode, readFilter, 800) &&
         readMode == mode &&
         readFilter == filter;
}

bool civQuerySplit(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  civFlushInput();
  civSend(0x0F, nullptr, 0);
  CivDecoded d;
  if (!waitReply(0x0F, d, timeoutMs) || d.payloadLen < 1) return false;
  if (d.payload[0] > 0x01) return false;
  onOut = d.payload[0] == 0x01;
  return true;
}

bool civSetSplit(const StoredProfile& sp, bool on) {
  if (sp.protocolType != PROTO_CIV) return false;
  civFlushInput();
  const uint8_t data[] = {(uint8_t)(on ? 0x01 : 0x00)};
  civSend(0x0F, data, 1);
  civSettleAfterWrite();
  bool readBack = false;
  return civQuerySplit(sp, readBack, 800) && (readBack == on);
}

bool civQueryRitEnabled(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  uint8_t value = 0;
  if (!civQuery21Byte(0x01, value, timeoutMs)) return false;
  if (value > 0x01) return false;
  onOut = value == 0x01;
  return true;
}

bool civSetRitEnabled(const StoredProfile& sp, bool on) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (!civSet21Byte(0x01, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryRitEnabled(sp, readBack, 800) && (readBack == on);
}

bool civQueryRitOffsetHz(const StoredProfile& sp, int32_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocolType != PROTO_CIV) return false;
  civFlushInput();
  const uint8_t sub[] = {0x00};
  civSend(0x21, sub, 1);
  CivDecoded d;
  if (!waitReply(0x21, d, timeoutMs) || d.payloadLen < 4 || d.payload[0] != 0x00) return false;
  const uint8_t low = d.payload[1];
  const uint8_t high = d.payload[2];
  int32_t magnitude = 0;
  magnitude += ((high >> 4) & 0x0F) * 1000;
  magnitude += (high & 0x0F) * 100;
  magnitude += ((low >> 4) & 0x0F) * 10;
  magnitude += (low & 0x0F);
  uint8_t sign = d.payload[3];
  if (sign > 0x01) return false;
  hzOut = sign == 0x01 ? -magnitude : magnitude;
  return true;
}

bool civSetRitOffsetHz(const StoredProfile& sp, int32_t hz) {
  if (sp.protocolType != PROTO_CIV) return false;
  if (hz < -9999 || hz > 9999) return false;

  uint16_t magnitude = (uint16_t)(hz < 0 ? -hz : hz);
  const uint8_t sign = (uint8_t)(hz < 0 ? 0x01 : 0x00);

  const uint8_t d1Hz = (uint8_t)(magnitude % 10);
  const uint8_t d10Hz = (uint8_t)((magnitude / 10) % 10);
  const uint8_t d100Hz = (uint8_t)((magnitude / 100) % 10);
  const uint8_t d1kHz = (uint8_t)((magnitude / 1000) % 10);

  const uint8_t pl[] = {
    0x00,
    (uint8_t)((d10Hz << 4) | d1Hz),
    (uint8_t)((d1kHz << 4) | d100Hz),
    sign
  };

  (void)civWriteRitOffsetFrame(pl, sizeof(pl));
  int32_t readBack = 0;
  return civQueryRitOffsetHz(sp, readBack, 800) && (readBack == hz);
}
