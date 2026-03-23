#include "protocol_ops_civ.h"

#include "engine_civ.h"
#include "protocol_civ.h"
#include "radio_protocol.h"
#include "radio_utils.h"

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
