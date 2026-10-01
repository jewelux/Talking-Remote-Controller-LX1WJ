#include "protocol_ops_yaesu.h"

#include "protocol_ascii.h"
#include "protocol_yaesu_cat.h"
#include "radio_catalog.h"
#include "radio_protocol.h"
#include "radio_state.h"

static bool yaesuCatQueryMeterByte(uint8_t cmdByte, int32_t& rawOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, cmdByte};
  uint8_t rsp = 0;
  if (!yaesuCatTransact1(cmd, rsp, timeoutMs)) return false;
  rawOut = rsp;
  return true;
}

bool yaesuCatQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
  if (!yaesuCatFreqFieldValid(rsp)) {
    yaesuCatMarkLineDirty();
    return false;
  }
  hzOut = yaesuCatDecodeFreqHz(rsp);
  return true;
}

bool yaesuCatQueryModeRawByte(uint8_t& modeByteOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
  modeByteOut = (uint8_t)(rsp[4] & 0x7F);
  return true;
}

bool yaesuCatSetFrequency(const StoredProfile& sp, uint64_t hz) {
  if (!sp.caps.setFreq) return false;
  uint8_t cmd[5] = {0, 0, 0, 0, 0x01};
  yaesuCatEncodeFreqHz(hz, cmd);
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  // FT-817/857 frequency writes behave like write-only commands in practice.
  delay(YAESU_CAT_FREQ_MODE_SETTLE_MS);
  return true;
}

bool yaesuCatQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
  // The mode byte shares its frame with the frequency; a bad frequency field means the frame is misaligned.
  if (!yaesuCatFreqFieldValid(rsp)) {
    yaesuCatMarkLineDirty();
    return false;
  }
  String code = byteToUpperHex((uint8_t)(rsp[4] & 0x7F));
  return profileInternalModeForCode(sp, code, modeOut);
}

bool yaesuCatSetMode(const StoredProfile& sp, uint8_t mode) {
  if (!sp.caps.setMode) return false;
  String code;
  uint8_t modeByte = 0;
  if (!profileModeCodeForInternal(sp, mode, code)) return false;
  if (!parseHexByteString(code, modeByte)) return false;
  yaesuCatSetModeRawByte(modeByte);
  delay(YAESU_CAT_FREQ_MODE_SETTLE_MS);
  return true;
}

void yaesuCatSetModeRawByte(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x07};
  yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter) return false;
  uint8_t rxStatus = 0;
  if (!yaesuCatQueryRxStatusRaw(rxStatus, timeoutMs)) return false;
  rawOut = rxStatus;
  return true;
}

SMeterReading yaesuCatDecodeSMeter(uint8_t rxStatus) {
  SMeterReading reading;
  yaesuCatDecodeSMeterLevel(rxStatus, reading.sUnits, reading.dbOverS9);
  return reading;
}

// Power and the high-SWR flag come from the TX status. ALC and SWR need the undocumented 0xBD,
// answered by the FT-817/818 and FT-857/897 while transmitting: two bytes of 0..15 meters,
// byte 0 = PWR (high nibble) | ALC (low), byte 1 = SWR (high) | MOD (low), as Hamlib's ft817.c
// reads them. In receive the FT-897 does not answer 0xBD and misses the next polls, so it is sent
// only after the TX status shows PTT on.
bool yaesuCatQueryTxMeters(YaesuTxMeters& out, bool withBdMeters, uint32_t timeoutMs) {
  out = YaesuTxMeters();
  uint8_t txStatus = 0;
  if (!yaesuCatQueryTxStatusRaw(txStatus, timeoutMs)) return false;
  if (!yaesuCatTxStatusTransmitting(txStatus)) return true;
  out.transmitting = true;
  out.highSwr = (txStatus & 0x40) != 0;
  out.po = txStatus & 0x0F;
  if (!withBdMeters) return true;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xBD};
  uint8_t meters[2] = {0};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  if (!yaesuCatRead1(meters[0], timeoutMs) || !yaesuCatRead1(meters[1], timeoutMs)) return false;
  out.alc = meters[0] & 0x0F;
  out.swr = (meters[1] >> 4) & 0x0F;
  return true;
}

bool yaesuCatQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower) return false;
  YaesuTxMeters meters;
  if (!yaesuCatQueryTxMeters(meters, false, timeoutMs)) return false;
  rawOut = meters.po;
  return true;
}

bool yaesuCatQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr) return false;
  YaesuTxMeters meters;
  if (!yaesuCatQueryTxMeters(meters, true, timeoutMs)) return false;
  rawOut = meters.swr;
  return true;
}

bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs) {
  YaesuTxMeters meters;
  if (!yaesuCatQueryTxMeters(meters, true, timeoutMs)) return false;
  rawOut = meters.alc;
  return true;
}

bool yaesuCatQueryVolumeRaw(int32_t& rawOut, uint32_t timeoutMs) {
  return yaesuCatQueryMeterByte(0x13, rawOut, timeoutMs);
}

bool yaesuCatQuerySquelchRaw(int32_t& rawOut, uint32_t timeoutMs) {
  return yaesuCatQueryMeterByte(0x14, rawOut, timeoutMs);
}

bool yaesuCatQueryRxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xE7};
  return yaesuCatTransact1(cmd, rawOut, timeoutMs);
}

bool yaesuCatQueryTxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xF7};
  return yaesuCatTransact1(cmd, rawOut, timeoutMs);
}

// The TX status split bit is valid only while transmitting; the FT-857/897 answer 0xFF in
// receive. Otherwise the EEPROM has it.
bool yaesuCatQuerySplit(bool& onOut, uint32_t timeoutMs) {
  uint8_t txStatus = 0;
  if (!yaesuCatQueryTxStatusRaw(txStatus, timeoutMs)) return false;
  if (yaesuCatTxStatusTransmitting(txStatus)) {
    onOut = (txStatus & 0x20) != 0;
    return true;
  }
  return yaesuFt8x7QueryFlag(Ft8x7Flag::Split, onOut, timeoutMs);
}

// The CAT clarifier commands (05 on, 85 off) switch RIT, the short press of the CLAR key, which
// the manuals also call the clarifier. The radio answers 00 when it switched and F0 when RIT was
// already in that state, also after a front panel change.
static bool ft8x7SetRitReply(bool on, bool& changedOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family() && !currentIsFt817Family()) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x05 : 0x85)};
  uint8_t rsp = 0;
  if (!yaesuCatTransact1(cmd, rsp, timeoutMs)) return false;
  if (rsp != 0x00 && rsp != 0xF0) return false;
  changedOut = rsp == 0x00;
  return true;
}

// RIT on/off is not in the EEPROM (on the FT-857/897 0x6A bit 3 is whether the knob tunes RIT or
// IF shift), so switching RIT off asks: refused means it is off, and if it was on it is switched
// back at once. That hands the knob to RIT if IF shift had it.
bool yaesuFt8x7QueryRit(bool& onOut, uint32_t timeoutMs) {
  bool changed = false;
  if (!ft8x7SetRitReply(false, changed, timeoutMs)) return false;
  if (changed && !ft8x7SetRitReply(true, changed, timeoutMs)) return false;
  onOut = changed;
  return true;
}

bool yaesuFt8x7SetRit(bool on, uint32_t timeoutMs) {
  bool changed = false;
  return ft8x7SetRitReply(on, changed, timeoutMs);
}

bool yaesuFt8x7ToggleRit(bool& onOut, uint32_t timeoutMs) {
  bool changed = false;
  if (!ft8x7SetRitReply(true, changed, timeoutMs)) return false;
  if (changed) {
    onOut = true;
    return true;
  }
  if (!ft8x7SetRitReply(false, changed, timeoutMs) || !changed) return false;
  onOut = false;
  return true;
}

void yaesuCatToggleVfo() {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x81};
  yaesuCatSendWriteOnly(cmd);
}

// The FT-8x7 has no command for VFO A or B, only the A/B toggle, so these toggle when the
// tracked VFO is the other one. An unknown VFO counts as the target.
void yaesuCatSelectVfoA() {
  if (live.activeVfoKnown && !live.activeVfoA) yaesuCatToggleVfo();
  rememberActiveVfo(true);
}

void yaesuCatSelectVfoB() {
  if (live.activeVfoKnown && live.activeVfoA) yaesuCatToggleVfo();
  rememberActiveVfo(false);
}

void yaesuCatSetPtt(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x08 : 0x88)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetSplit(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x02 : 0x82)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetLockDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x00 : 0x80)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetRepeaterShiftRaw(uint8_t shiftByte) {
  const uint8_t cmd[5] = {shiftByte, 0x00, 0x00, 0x00, 0x09};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetRepeaterOffsetHzRaw(uint64_t hz) {
  uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xF9};
  yaesuCatEncodeRepeaterOffsetHz(hz, cmd);
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetPowerDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x0F : 0x8F)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetAgcMode(uint8_t modeByte) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, modeByte, 0xF3};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetClarifierOffsetRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0xF5};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetToneDcsModeRaw(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x0A};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetCtcssToneRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0B};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetDcsCodeRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0C};
  yaesuCatSendWriteOnly(cmd);
}

// FT-857/897 take separate TX and RX values; the FT-817 takes one.
static void fillToneData(uint16_t value, uint8_t data[4]) {
  yaesuEncodeToneData(value, currentIsFt857Family(), data);
}

bool yaesuCatSetCtcssTenths(uint16_t toneTenths) {
  if (!yaesuCtcssTenthsValid(toneTenths)) return false;
  uint8_t data[4];
  fillToneData(toneTenths, data);
  yaesuCatSetCtcssToneRaw(data);
  live.ctcssValid = true;
  live.ctcssTenths = toneTenths;
  return true;
}

bool yaesuCatSetDcsCode(uint16_t dcsCode) {
  if (!yaesuDcsCodeValid(dcsCode)) return false;
  uint8_t data[4];
  fillToneData(dcsCode, data);
  yaesuCatSetDcsCodeRaw(data);
  live.dcsValid = true;
  live.dcsCode = dcsCode;
  return true;
}
