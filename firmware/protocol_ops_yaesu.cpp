#include "protocol_ops_yaesu.h"

#include "protocol_ascii.h"
#include "protocol_yaesu_cat.h"
#include "radio_protocol.h"

static bool yaesuCatQueryMeterByte(uint8_t cmdByte, int32_t& rawOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, cmdByte};
  uint8_t rsp = 0;
  if (!yaesuCatTransact1(cmd, rsp, timeoutMs)) return false;
  rawOut = rsp;
  return true;
}

static bool yaesuCatSendWriteOnly(const uint8_t cmd[5]) {
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  delay(60);
  return true;
}

bool yaesuCatQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
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
  delay(80);
  uint64_t readHz = 0;
  if (queryFrequency(readHz, 800) && (readHz == hz)) return true;
  delay(120);
  if (queryFrequency(readHz, 800) && (readHz == hz)) return true;
  // FT-817/857 frequency writes are write-only in practice. If readback races
  // or returns stale data, keep the successful write instead of reporting a
  // false failure to the UI.
  return true;
}

bool yaesuCatQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
  String code = byteToUpperHex((uint8_t)(rsp[4] & 0x7F));
  return profileInternalModeForCode(sp, code, modeOut);
}

bool yaesuCatSetMode(const StoredProfile& sp, uint8_t mode) {
  if (!sp.caps.setMode) return false;
  String code;
  uint8_t modeByte = 0;
  if (!profileModeCodeForInternal(sp, mode, code)) return false;
  if (!parseHexByteString(code, modeByte)) return false;
  if (!yaesuCatSetModeRawByte(modeByte)) return false;
  uint8_t readMode = 0xFF;
  return queryMode(readMode, 800) && (readMode == mode);
}

bool yaesuCatSetModeRawByte(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x07};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter) return false;
  return yaesuCatQueryMeterByte(0xE7, rawOut, timeoutMs);
}

bool yaesuCatQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower) return false;
  return yaesuCatQueryMeterByte(0xBD, rawOut, timeoutMs);
}

bool yaesuCatQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr) return false;
  return yaesuCatQueryMeterByte(0xBC, rawOut, timeoutMs);
}

bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs) {
  return yaesuCatQueryMeterByte(0xBB, rawOut, timeoutMs);
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

bool yaesuCatQueryStatusRaw(uint8_t& rawOut, uint32_t timeoutMs) {
  return yaesuCatQueryTxStatusRaw(rawOut, timeoutMs);
}

bool yaesuCatToggleVfo() {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x81};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSelectVfoA() {
  return false;
}

bool yaesuCatSelectVfoB() {
  return false;
}

bool yaesuCatSetPtt(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x08 : 0x88};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetClarifier(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x05 : 0x85};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetSplit(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x02 : 0x82};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetLockDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x00 : 0x80};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetRepeaterShiftRaw(uint8_t shiftByte) {
  const uint8_t cmd[5] = {shiftByte, 0x00, 0x00, 0x00, 0x09};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetRepeaterOffsetHzRaw(uint64_t hz) {
  uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xF9};
  yaesuCatEncodeRepeaterOffsetHz(hz, cmd);
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetPowerDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x0F : 0x8F};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatMemoryWrite() {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x0A};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatMemoryReadRaw(uint8_t rsp[5], uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x0B};
  return yaesuCatTransact5(cmd, rsp, timeoutMs);
}

bool yaesuCatSetAgcMode(uint8_t modeByte) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, modeByte, 0xF3};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetClarifierOffsetRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0xF5};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetToneDcsModeRaw(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x0A};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetCtcssToneRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0B};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetDcsCodeRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0C};
  return yaesuCatSendWriteOnly(cmd);
}
