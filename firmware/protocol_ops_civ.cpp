#include "protocol_ops_civ.h"

#include "cat_setting_codec.h"
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
  const std::optional<CivFrame> d = waitReply(0x16, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != subcmd) return false;
  onOut = d->payload[1] != 0x00;
  return true;
}

static bool civSetToggleSub(uint8_t subcmd, bool on) {
  civFlushInput();
  const uint8_t pl[] = {subcmd, (uint8_t)(on ? 0x01 : 0x00)};
  civSend(0x16, pl, 2);
  return true;
}

// A 16 function that takes a value byte (00, 01, 02 ...), not just on and off.
static bool civQuery16Byte(uint8_t subcmd, uint8_t& valueOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {subcmd};
  civSend(0x16, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x16, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != subcmd) return false;
  valueOut = d->payload[1];
  return true;
}

static bool civSet16Byte(uint8_t subcmd, uint8_t value) {
  civFlushInput();
  const uint8_t pl[] = {subcmd, value};
  civSend(0x16, pl, 2);
  return true;
}

static bool civQuery1CByte(uint8_t subcmd, uint8_t& valueOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {subcmd};
  civSend(0x1C, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x1C, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != subcmd) return false;
  valueOut = d->payload[1];
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
  const std::optional<CivFrame> d = waitReply(0x21, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != subcmd) return false;
  valueOut = d->payload[1];
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
  const std::optional<CivFrame> d = waitReply(0x1A, timeoutMs);
  if (!d || d->payloadLen < 4) return false;
  if (d->payload[0] != group || d->payload[1] != ((item >> 8) & 0xFF) || d->payload[2] != (item & 0xFF)) return false;
  valueOut = d->payload[3];
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
  const std::optional<CivFrame> d = waitReply(0x14, timeoutMs);
  if (!d || d->payloadLen < 3 || d->payload[0] != subcmd) return false;
  valueOut = (uint16_t)bcdDigitsToInt(d->payload.data() + 1, d->payloadLen - 1);
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

static bool isIcom7760Profile(const RadioProfile& sp) {
  return sp.model == RadioModel::Ic7760;
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

// The reply to a 0x29 command, with the echoed main/sub selector and inner
// command removed from the payload.
static std::optional<CivFrame> waitMainSubReply(uint8_t mainSub, uint8_t innerCmd, uint32_t timeoutMs) {
  std::optional<CivFrame> d = waitReply(0x29, timeoutMs);
  if (!d || d->payloadLen < 2) return std::nullopt;
  if (d->payload[0] != mainSub || d->payload[1] != innerCmd) return std::nullopt;
  d->dropPayloadPrefix(2);
  return d;
}

static bool civQueryMainSubFrequency(uint8_t mainSub, uint64_t& hzOut, uint32_t timeoutMs) {
  civFlushInput();
  civSendMainSub(mainSub, 0x03, nullptr, 0);
  const std::optional<CivFrame> d = waitMainSubReply(mainSub, 0x03, timeoutMs);
  if (!d || d->payloadLen < 5) return false;
  hzOut = decodeBcdFrequencyHz(d->payload.data(), 5);
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
  const std::optional<CivFrame> d = waitMainSubReply(mainSub, 0x04, timeoutMs);
  if (!d || d->payloadLen < 1) return false;
  modeOut = d->payload[0];
  filterOut = d->payloadLen >= 2 ? d->payload[1] : 1;
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

bool civQueryFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq) return false;
  civFlushInput();
  civSend(0x03, nullptr, 0);
  const std::optional<CivFrame> d = waitReply(0x03, timeoutMs);
  if (!d || d->payloadLen < 5) return false;
  hzOut = decodeBcdFrequencyHz(d->payload.data(), 5);
  return true;
}

bool civSetFrequency(const RadioProfile& sp, uint64_t hz) {
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

bool civQueryMode(const RadioProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode) return false;
  civFlushInput();
  civSend(0x04, nullptr, 0);
  const std::optional<CivFrame> d = waitReply(0x04, timeoutMs);
  if (!d || d->payloadLen < 1) return false;
  modeOut = d->payload[0];
  return true;
}

bool civSetMode(const RadioProfile& sp, uint8_t mode, uint8_t filter) {
  if (!sp.caps.setMode) return false;
  uint8_t pl[2] = {mode, filter};
  civFlushInput();
  civSend(0x06, pl, 2);
  return true;
}

bool civQuerySMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter) return false;
  civFlushInput();
  const uint8_t sub[] = {0x02};
  civSend(0x15, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x15, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != 0x02) return false;
  rawOut = bcdDigitsToInt(d->payload.data() + 1, d->payloadLen - 1);
  return true;
}

bool civQueryPoMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower) return false;
  civFlushInput();
  const uint8_t sub[] = {0x11};
  civSend(0x15, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x15, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != 0x11) return false;
  rawOut = bcdDigitsToInt(d->payload.data() + 1, d->payloadLen - 1);
  return true;
}

bool civQuerySWRRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr) return false;
  civFlushInput();
  const uint8_t sub[] = {0x12};
  civSend(0x15, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x15, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != 0x12) return false;
  rawOut = bcdDigitsToInt(d->payload.data() + 1, d->payloadLen - 1);
  return true;
}

// 15 13: 0000 minimum, 0120 maximum (the IC-7300 manual, the IC-705 and IC-7760 CI-V guides;
// Hamlib's ALC table).
bool civQueryAlcRaw(const RadioProfile& sp, uint16_t& rawOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getAlc) return false;
  civFlushInput();
  const uint8_t sub[] = {0x13};
  civSend(0x15, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x15, timeoutMs);
  if (!d || d->payloadLen < 2 || d->payload[0] != 0x13) return false;
  const int raw = bcdDigitsToInt(d->payload.data() + 1, d->payloadLen - 1);
  if (raw < 0 || raw > 255) return false;
  rawOut = (uint16_t)raw;
  return true;
}

bool civQueryRfPowerLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (!sp.caps.getRfPower) return false;
  return civQuery14Value(0x0A, valueOut, timeoutMs);
}

bool civSetRfPowerLevel(const RadioProfile& sp, uint16_t value) {
  if (!sp.caps.setRfPower) return false;
  if (!civSet14Value(0x0A, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryRfPowerLevel(sp, readBack, 800) && readBack == value;
}

bool civQueryNr(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNr) return false;
  return civQueryToggleSub(0x40, onOut, timeoutMs);
}

bool civSetNr(const RadioProfile& sp, bool on) {
  if (!sp.caps.setNr) return false;
  if (!civSetToggleSub(0x40, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryNr(sp, readBack, 800) && (readBack == on);
}

bool civQueryNrLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getNrLevel) return false;
  return civQuery14Value(0x06, valueOut, timeoutMs);
}

bool civSetNrLevel(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setNrLevel) return false;
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

bool civQueryNb(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNb) return false;
  return civQueryToggleSub(0x22, onOut, timeoutMs);
}

bool civSetNb(const RadioProfile& sp, bool on) {
  if (!sp.caps.setNb) return false;
  if (!civSetToggleSub(0x22, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryNb(sp, readBack, 800) && (readBack == on);
}

bool civQueryNbLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getNbLevel) return false;
  return civQuery14Value(0x12, valueOut, timeoutMs);
}

bool civSetNbLevel(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setNbLevel) return false;
  if (!civSet14Value(0x12, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryNbLevel(sp, readBack, 800) && (readBack == value);
}

bool civQueryProc(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getProc) return false;
  return civQueryToggleSub(0x44, onOut, timeoutMs);
}

bool civQueryProcLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getProc) return false;
  return civQuery14Value(0x0E, valueOut, timeoutMs);
}

bool civQueryMicGain(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getMicGain) return false;
  return civQuery14Value(0x0B, valueOut, timeoutMs);
}

bool civQueryVox(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getVox) return false;
  return civQueryToggleSub(0x46, onOut, timeoutMs);
}

bool civQueryVoxGain(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getVox) return false;
  return civQuery14Value(0x16, valueOut, timeoutMs);
}

// The 1A 05 menu address of the VOX delay, where Hamlib's VOXDELAY and the model's CI-V guide
// agree. Other models keep it elsewhere (the IC-7300 MK2 at 0267, the IC-9700 at 0330). The
// IC-7760 has none here: Hamlib says 0182, its guide 0365, and 0182 is its Internet access line.
bool civVoxDelayAddr(RadioModel model, uint16_t& addrOut) {
  switch (model) {
    case RadioModel::Ic7300: addrOut = 0x0191; return true;
    case RadioModel::Ic705: addrOut = 0x0359; return true;
    default: return false;
  }
}

// 00..20 in BCD, tenths of a second.
bool civQueryVoxDelayMs(const RadioProfile& sp, uint16_t& msOut, uint32_t timeoutMs) {
  uint16_t addr = 0;
  if (sp.protocol != PROTO_CIV || !sp.caps.getVox || !civVoxDelayAddr(sp.model, addr)) return false;
  uint8_t raw = 0;
  if (!civQueryMenuByte(0x05, addr, raw, timeoutMs)) return false;
  const int32_t tenths = bcdDigitsToInt(&raw, 1);
  if (tenths < 0 || tenths > 20) return false;
  msOut = (uint16_t)(tenths * 100);
  return true;
}

// Sets a 16 on/off function and reads it back.
static bool civSetToggleSubChecked(uint8_t subcmd, bool on) {
  if (!civSetToggleSub(subcmd, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryToggleSub(subcmd, readBack, 800) && readBack == on;
}

// Sets a 14 level and reads it back.
static bool civSet14ValueChecked(uint8_t subcmd, uint16_t value) {
  if (!civSet14Value(subcmd, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQuery14Value(subcmd, readBack, 800) && readBack == value;
}

bool civSetProc(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setProc) return false;
  return civSetToggleSubChecked(0x44, on);
}

bool civSetProcLevel(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setProc) return false;
  return civSet14ValueChecked(0x0E, value);
}

bool civSetMicGain(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setMicGain) return false;
  return civSet14ValueChecked(0x0B, value);
}

bool civSetVox(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setVox) return false;
  return civSetToggleSubChecked(0x46, on);
}

bool civSetVoxGain(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setVox) return false;
  return civSet14ValueChecked(0x16, value);
}

bool civSetVoxDelayMs(const RadioProfile& sp, uint16_t ms) {
  uint16_t addr = 0;
  if (sp.protocol != PROTO_CIV || !sp.caps.setVox || !civVoxDelayAddr(sp.model, addr) || ms > 2000) return false;
  const uint8_t tenths = (uint8_t)((ms + 50) / 100);
  if (!civSetMenuByte(0x05, addr, (uint8_t)(((tenths / 10) << 4) | (tenths % 10)))) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryVoxDelayMs(sp, readBack, 800) && readBack == tenths * 100;
}

bool civQueryNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNotch) return false;
  return civQueryToggleSub(0x48, onOut, timeoutMs);
}

bool civSetNotch(const RadioProfile& sp, bool on) {
  if (!sp.caps.setNotch) return false;
  if (!civSetToggleSub(0x48, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryNotch(sp, readBack, 800) && (readBack == on);
}

// 16 57: the manual notch width (00 wide, 01 mid, 02 narrow).
bool civQueryNotchWidth(const RadioProfile& sp, NotchWidth& widthOut, uint32_t timeoutMs) {
  if (!sp.caps.getNotchWidth) return false;
  uint8_t raw = 0;
  if (!civQuery16Byte(0x57, raw, timeoutMs)) return false;
  return decodeIcomNotchWidth(raw, widthOut);
}

bool civSetNotchWidth(const RadioProfile& sp, NotchWidth width) {
  if (!sp.caps.setNotchWidth || width == NOTCH_WIDTH_UNKNOWN) return false;
  if (!civSet16Byte(0x57, encodeIcomNotchWidth(width))) return false;
  civSettleAfterWrite();
  NotchWidth readBack = NOTCH_WIDTH_UNKNOWN;
  return civQueryNotchWidth(sp, readBack, 800) && readBack == width;
}

bool civQueryPbtInner(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getPbtInner) return false;
  return civQuery14Value(0x07, valueOut, timeoutMs);
}

bool civSetPbtInner(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setPbtInner) return false;
  if (!civSet14Value(0x07, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryPbtInner(sp, readBack, 800) &&
         readBack >= (value > 1 ? value - 1 : 0) &&
         readBack <= value + 1;
}

bool civQueryPbtOuter(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getPbtOuter) return false;
  return civQuery14Value(0x08, valueOut, timeoutMs);
}

bool civSetPbtOuter(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setPbtOuter) return false;
  if (!civSet14Value(0x08, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryPbtOuter(sp, readBack, 800) &&
         readBack >= (value > 1 ? value - 1 : 0) &&
         readBack <= value + 1;
}

// 16 12: the AGC time constant preset, 01 fast, 02 mid, 03 slow (the IC-7300, IC-705 and IC-7760
// CI-V guides, Hamlib).
bool civQueryAgc(const RadioProfile& sp, AgcMode& out, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getAgc) return false;
  uint8_t raw = 0;
  return civQuery16Byte(0x12, raw, timeoutMs) && civAgcDecode(raw, out);
}

bool civSetAgc(const RadioProfile& sp, AgcMode mode) {
  uint8_t raw = 0;
  if (sp.protocol != PROTO_CIV || !sp.caps.setAgc || !civAgcEncode(mode, raw)) return false;
  if (!civSet16Byte(0x12, raw)) return false;
  civSettleAfterWrite();
  AgcMode readBack = AgcMode::Off;
  return civQueryAgc(sp, readBack, 800) && readBack == mode;
}

// 16 02: 00 off, 01 P.AMP1, 02 P.AMP2 (the IC-7300, IC-705 and IC-7760 CI-V guides, Hamlib).
bool civQueryPreampLevel(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getPreamp) return false;
  uint8_t raw = 0;
  return civQuery16Byte(0x02, raw, timeoutMs) && civPreampDecode(raw, sp.settings.preampLevels, levelOut);
}

bool civSetPreampLevel(const RadioProfile& sp, uint8_t level) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setPreamp || level > sp.settings.preampLevels) return false;
  if (!civSet16Byte(0x02, level)) return false;
  civSettleAfterWrite();
  uint8_t readBack = 0xFF;
  return civQueryPreampLevel(sp, readBack, 800) && readBack == level;
}

// 11: the attenuator in dB, BCD, with no sub command (the CI-V guides, Hamlib).
bool civQueryAttDb(const RadioProfile& sp, uint8_t& dbOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getAtt) return false;
  civFlushInput();
  civSend(0x11, nullptr, 0);
  const std::optional<CivFrame> d = waitReply(0x11, timeoutMs);
  if (!d || d->payloadLen < 1) return false;
  return civAttDecode(d->payload[0], dbOut);
}

bool civSetAttDb(const RadioProfile& sp, uint8_t db) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setAtt) return false;
  if (db != 0 && !attStepOfDb(sp.settings.attDb, sp.settings.attLevels, db)) return false;
  civFlushInput();
  const uint8_t pl[] = {civAttEncode(db)};
  civSend(0x11, pl, 1);
  civSettleAfterWrite();
  uint8_t readBack = 0xFF;
  return civQueryAttDb(sp, readBack, 800) && readBack == db;
}

// 16 47: 00 off, 01 semi, 02 full (the IC-7300 manual, the IC-705 and IC-7760 CI-V guides,
// Hamlib, flrig).
bool civQueryBreakIn(const RadioProfile& sp, BreakInMode& out, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getBreakIn) return false;
  uint8_t raw = 0;
  return civQuery16Byte(0x47, raw, timeoutMs) && civBreakInDecode(raw, out);
}

bool civSetBreakIn(const RadioProfile& sp, BreakInMode mode) {
  uint8_t raw = 0;
  if (sp.protocol != PROTO_CIV || !sp.caps.setBreakIn || !civBreakInEncode(mode, raw)) return false;
  if (!civSet16Byte(0x47, raw)) return false;
  civSettleAfterWrite();
  BreakInMode readBack = BreakInMode::On;
  return civQueryBreakIn(sp, readBack, 800) && readBack == mode;
}

// 14 09: 0000 = 300 Hz, 0128 = 600 Hz, 0255 = 900 Hz (the IC-7300 manual, the IC-705 and IC-7760
// CI-V guides, Hamlib).
bool civQueryCwPitchHz(const RadioProfile& sp, uint16_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getCwPitch) return false;
  uint16_t raw = 0;
  if (!civQuery14Value(0x09, raw, timeoutMs) || raw > 255) return false;
  hzOut = civCwPitchRawToHz(raw);
  return true;
}

bool civSetCwPitchHz(const RadioProfile& sp, uint16_t hz) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setCwPitch || hz < 300 || hz > 900) return false;
  if (!civSet14Value(0x09, civCwPitchHzToRaw(hz))) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  if (!civQueryCwPitchHz(sp, readBack, 800)) return false;
  return readBack + 5 >= hz && readBack <= hz + 5;
}

// A 16 function that is only off (00) or on (01); any other byte is not a valid reply.
static bool civQuery16OnOff(uint8_t subcmd, bool& onOut, uint32_t timeoutMs) {
  uint8_t raw = 0;
  if (!civQuery16Byte(subcmd, raw, timeoutMs)) return false;
  if (raw > 0x01) return false;
  onOut = raw == 0x01;
  return true;
}

// 16 41: the auto notch, 00 off, 01 on (the IC-7300 manual, the IC-705 and IC-7760 CI-V guides,
// Hamlib).
bool civQueryAutoNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getAutoNotch) return false;
  return civQuery16OnOff(0x41, onOut, timeoutMs);
}

bool civSetAutoNotch(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setAutoNotch) return false;
  if (!civSetToggleSub(0x41, on)) return false;
  civSettleAfterWrite();
  bool readBack = !on;
  return civQueryAutoNotch(sp, readBack, 800) && readBack == on;
}

// 16 42 and 16 43 (the IC-7300 manual, the IC-705 and IC-7760 CI-V guides, Hamlib).
bool civQueryTone(const RadioProfile& sp, bool& toneOut, bool& tsqlOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getTone) return false;
  return civQuery16OnOff(0x42, toneOut, timeoutMs) && civQuery16OnOff(0x43, tsqlOut, timeoutMs);
}

bool civSetTone(const RadioProfile& sp, bool tone, bool tsql) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setTone || (tone && tsql)) return false;
  // The one turned off first, so the two are never on together.
  const uint8_t offFirst = tone ? 0x43 : 0x42;
  if (!civSetToggleSub(offFirst, false)) return false;
  civSettleAfterWrite();
  if (!civSetToggleSub(tone ? 0x42 : 0x43, tone || tsql)) return false;
  civSettleAfterWrite();
  bool toneBack = !tone;
  bool tsqlBack = !tsql;
  return civQueryTone(sp, toneBack, tsqlBack, 800) && toneBack == tone && tsqlBack == tsql;
}

// 1B 00 and 1B 01: 00 08 85 = 88.5 Hz (the CI-V guides, Hamlib).
static bool civQueryToneFreq(uint8_t subcmd, uint16_t& tenthsOut, uint32_t timeoutMs) {
  civFlushInput();
  const uint8_t sub[] = {subcmd};
  civSend(0x1B, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x1B, timeoutMs);
  if (!d || d->payloadLen < 4 || d->payload[0] != subcmd) return false;
  return civToneDecode(d->payload.data() + 1, tenthsOut);
}

static void civSetToneFreq(uint8_t subcmd, uint16_t tenths) {
  uint8_t pl[4] = {subcmd};
  civToneEncode(tenths, pl + 1);
  civFlushInput();
  civSend(0x1B, pl, sizeof(pl));
  civSettleAfterWrite();
}

bool civQueryCtcssTenths(const RadioProfile& sp, uint16_t& tenthsOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getCtcss) return false;
  return civQueryToneFreq(0x00, tenthsOut, timeoutMs);
}

bool civSetCtcssTenths(const RadioProfile& sp, uint16_t tenths) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setCtcss) return false;
  civSetToneFreq(0x00, tenths);
  civSetToneFreq(0x01, tenths);
  uint16_t rptBack = 0;
  uint16_t sqlBack = 0;
  return civQueryToneFreq(0x00, rptBack, 800) && civQueryToneFreq(0x01, sqlBack, 800) && rptBack == tenths &&
         sqlBack == tenths;
}

// 0F: the IC-705 CI-V guide and Hamlib.
bool civQueryRptShift(const RadioProfile& sp, RptShift& out, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getRptShift) return false;
  civFlushInput();
  civSend(0x0F, nullptr, 0);
  const std::optional<CivFrame> d = waitReply(0x0F, timeoutMs);
  if (!d || d->payloadLen < 1) return false;
  switch (d->payload[0]) {
    case 0x00:
    case 0x01:
    case 0x10: out = RptShift::Simplex; return true;
    case 0x11: out = RptShift::Minus; return true;
    case 0x12: out = RptShift::Plus; return true;
    default: return false;
  }
}

bool civSetRptShift(const RadioProfile& sp, RptShift shift) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setRptShift) return false;
  const uint8_t pl[] = {(uint8_t)(shift == RptShift::Minus ? 0x11 : shift == RptShift::Plus ? 0x12 : 0x10)};
  civFlushInput();
  civSend(0x0F, pl, 1);
  civSettleAfterWrite();
  RptShift readBack = shift == RptShift::Simplex ? RptShift::Plus : RptShift::Simplex;
  return civQueryRptShift(sp, readBack, 800) && readBack == shift;
}

// 16 50: the dial lock.
bool civQueryDialLock(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getDialLock) return false;
  return civQuery16OnOff(0x50, onOut, timeoutMs);
}

bool civSetDialLock(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setDialLock) return false;
  if (!civSetToggleSub(0x50, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryDialLock(sp, readBack, 800) && readBack == on;
}

// 16 56: the DSP filter type (00 sharp, 01 soft).
bool civQueryFilterShape(const RadioProfile& sp, bool& softOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getFilterShape) return false;
  return civQuery16OnOff(0x56, softOut, timeoutMs);
}

bool civSetFilterShape(const RadioProfile& sp, bool soft) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setFilterShape) return false;
  if (!civSetToggleSub(0x56, soft)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryFilterShape(sp, readBack, 800) && readBack == soft;
}

// 16 45: the monitor (1C 02 is the transmit frequency monitor, a different function).
bool civQueryMonitorEnabled(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getMonitor) return false;
  return civQuery16OnOff(0x45, onOut, timeoutMs);
}

bool civSetMonitorEnabled(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setMonitor) return false;
  if (!civSetToggleSub(0x45, on)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryMonitorEnabled(sp, readBack, 800) && readBack == on;
}

bool civQueryMonitorLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getMonitorLevel) return false;
  return civQuery14Value(0x15, valueOut, timeoutMs);
}

bool civSetMonitorLevel(const RadioProfile& sp, uint16_t value) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setMonitorLevel) return false;
  if (!civSet14Value(0x15, value)) return false;
  civSettleAfterWrite();
  uint16_t readBack = 0;
  return civQueryMonitorLevel(sp, readBack, 800) && readBack == value;
}

// The 1A 05 menu address of the CI-V transceive setting, as the model's CI-V guide and Hamlib
// give it. The IC-7760's guide says 0150, but Hamlib has none for it.
static bool civTransceiveAddr(RadioModel model, uint16_t& addrOut) {
  switch (model) {
    case RadioModel::Ic7300: addrOut = 0x0071; return true;
    case RadioModel::Ic705: addrOut = 0x0131; return true;
    default: return false;
  }
}

bool civQueryTransceiveEnabled(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  uint16_t addr = 0;
  if (sp.protocol != PROTO_CIV || !sp.caps.getTransceive || !civTransceiveAddr(sp.model, addr)) return false;
  uint8_t raw = 0;
  if (!civQueryMenuByte(0x05, addr, raw, timeoutMs)) return false;
  if (raw > 0x01) return false;
  onOut = raw == 0x01;
  return true;
}

bool civSetTransceiveEnabled(const RadioProfile& sp, bool on) {
  uint16_t addr = 0;
  if (sp.protocol != PROTO_CIV || !sp.caps.setTransceive || !civTransceiveAddr(sp.model, addr)) return false;
  if (!civSetMenuByte(0x05, addr, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryTransceiveEnabled(sp, readBack, 800) && readBack == on;
}

bool civQueryBandStackEntry(const RadioProfile& sp, uint8_t bandCode, uint8_t registerCode, BandStackEntry& entryOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getBandStack) return false;
  if (bandCode < 0x01 || bandCode > 0x11) return false;
  if (registerCode < 0x01 || registerCode > 0x03) return false;
  civFlushInput();
  const uint8_t sub[] = {0x01, bandCode, registerCode};
  civSend(0x1A, sub, 3);
  const std::optional<CivFrame> d = waitReply(0x1A, timeoutMs);
  if (!d || d->payloadLen < 10) return false;
  if (d->payload[0] != 0x01 || d->payload[1] != bandCode || d->payload[2] != registerCode) return false;
  entryOut.bandCode = bandCode;
  entryOut.registerCode = registerCode;
  entryOut.freqHz = decodeBcdFrequencyHz(d->payload.data() + 3, 5);
  entryOut.mode = d->payload[8];
  entryOut.filter = d->payload[9];
  return true;
}

bool civQueryTuner(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getTuner) return false;
  uint8_t value = 0;
  if (!civQuery1CByte(0x01, value, timeoutMs)) return false;
  if (value > 0x02) return false;
  onOut = value != 0x00;
  return true;
}

bool civSetTuner(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setTuner) return false;
  if (!civSet1CByte(0x01, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryTuner(sp, readBack, 800) && (readBack == on);
}

bool civStartTune(const RadioProfile& sp) {
  if (sp.protocol != PROTO_CIV || !sp.caps.startTune) return false;
  if (!civSet1CByte(0x01, 0x02)) return false;
  civSettleAfterWrite();
  return true;
}

bool civQueryRxTxStatus(const RadioProfile& sp, bool& txOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getRxTx) return false;
  uint8_t value = 0;
  if (!civQuery1CByte(0x00, value, timeoutMs)) return false;
  if (value > 0x01) return false;
  txOut = value == 0x01;
  return true;
}

bool civQueryTxFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getTxFreq) return false;
  civFlushInput();
  const uint8_t sub[] = {0x03};
  civSend(0x1C, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x1C, timeoutMs);
  if (!d || d->payloadLen < 6 || d->payload[0] != 0x03) return false;
  hzOut = decodeBcdFrequencyHz(d->payload.data() + 1, 5);
  return true;
}

bool civSelectVfoA(const RadioProfile& sp) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setVfo) return false;
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

bool civSelectVfoB(const RadioProfile& sp) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setVfo) return false;
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
  const std::optional<CivFrame> d = waitReply(0x25, timeoutMs);
  if (!d || d->payloadLen < 6) return false;
  if (d->payload[0] != selector) return false;
  hzOut = decodeBcdFrequencyHz(d->payload.data() + 1, 5);
  return true;
}

static bool civEnsureActiveVfoKnown(const RadioProfile& sp) {
  if (live.activeVfoKnown) return true;
  if (sp.protocol != PROTO_CIV) return false;

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

bool civQueryVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getVfo) return false;
  if (isIcom7760Profile(sp)) return civQueryMainSubFrequency(civMainSubSelector(targetVfoA), hzOut, timeoutMs);
  if (!civEnsureActiveVfoKnown(sp)) return false;
  const uint8_t selector = civEncodeVfoSelector(targetVfoA);
  return civQuerySelectedOrUnselectedFrequencyRaw(selector, hzOut, timeoutMs);
}

bool civSetVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t hz) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setVfo) return false;
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

bool civQueryVfoMode(const RadioProfile& sp, bool targetVfoA, uint8_t& modeOut, uint8_t& filterOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getVfoMode) return false;
  if (isIcom7760Profile(sp)) return civQueryMainSubMode(civMainSubSelector(targetVfoA), modeOut, filterOut, timeoutMs);
  if (!civEnsureActiveVfoKnown(sp)) return false;
  const uint8_t selector = civEncodeVfoSelector(targetVfoA);
  civFlushInput();
  civSend(0x26, &selector, 1);
  const std::optional<CivFrame> d = waitReply(0x26, timeoutMs);
  if (!d || d->payloadLen < 4) return false;
  if (d->payload[0] != selector) return false;
  modeOut = d->payload[1];
  filterOut = d->payload[3];
  return true;
}

bool civSetVfoMode(const RadioProfile& sp, bool targetVfoA, uint8_t mode, uint8_t filter) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setVfoMode) return false;
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

bool civQuerySplit(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getSplit) return false;
  civFlushInput();
  civSend(0x0F, nullptr, 0);
  const std::optional<CivFrame> d = waitReply(0x0F, timeoutMs);
  if (!d || d->payloadLen < 1) return false;
  if (d->payload[0] > 0x01) return false;
  onOut = d->payload[0] == 0x01;
  return true;
}

bool civSetSplit(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setSplit) return false;
  civFlushInput();
  const uint8_t data[] = {(uint8_t)(on ? 0x01 : 0x00)};
  civSend(0x0F, data, 1);
  civSettleAfterWrite();
  bool readBack = false;
  return civQuerySplit(sp, readBack, 800) && (readBack == on);
}

bool civQueryRitEnabled(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getRit) return false;
  uint8_t value = 0;
  if (!civQuery21Byte(0x01, value, timeoutMs)) return false;
  if (value > 0x01) return false;
  onOut = value == 0x01;
  return true;
}

bool civSetRitEnabled(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setRit) return false;
  if (!civSet21Byte(0x01, on ? 0x01 : 0x00)) return false;
  civSettleAfterWrite();
  bool readBack = false;
  return civQueryRitEnabled(sp, readBack, 800) && (readBack == on);
}

bool civQueryRitOffsetHz(const RadioProfile& sp, int32_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_CIV || !sp.caps.getRit) return false;
  civFlushInput();
  const uint8_t sub[] = {0x00};
  civSend(0x21, sub, 1);
  const std::optional<CivFrame> d = waitReply(0x21, timeoutMs);
  if (!d || d->payloadLen < 4 || d->payload[0] != 0x00) return false;
  const uint8_t low = d->payload[1];
  const uint8_t high = d->payload[2];
  int32_t magnitude = 0;
  magnitude += ((high >> 4) & 0x0F) * 1000;
  magnitude += (high & 0x0F) * 100;
  magnitude += ((low >> 4) & 0x0F) * 10;
  magnitude += (low & 0x0F);
  uint8_t sign = d->payload[3];
  if (sign > 0x01) return false;
  hzOut = sign == 0x01 ? -magnitude : magnitude;
  return true;
}

bool civSetRitOffsetHz(const RadioProfile& sp, int32_t hz) {
  if (sp.protocol != PROTO_CIV || !sp.caps.setRit) return false;
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
