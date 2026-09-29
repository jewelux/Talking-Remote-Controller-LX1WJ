#pragma once

#include "radio_globals.h"

bool yaesuCatQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool yaesuCatQueryModeRawByte(uint8_t& modeByteOut, uint32_t timeoutMs);
bool yaesuCatSetFrequency(const StoredProfile& sp, uint64_t hz);
bool yaesuCatQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool yaesuCatSetMode(const StoredProfile& sp, uint8_t mode);
bool yaesuCatSetModeRawByte(uint8_t modeByte);
bool yaesuCatQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
SMeterReading yaesuCatDecodeSMeter(uint8_t rxStatus);
bool yaesuCatQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryVolumeRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySquelchRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryRxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryTxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
// TX status (0xF7): bit 7 = PTT (0 = transmitting), bit 6 = high SWR, bit 5 = split (1 = on,
// measured on an FT-897; the manuals say 0 = on), bits 3..0 = PO meter.
bool yaesuCatTxStatusTransmitting(uint8_t txStatus);
bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs);
bool yaesuCatQuerySplit(bool& onOut, uint32_t timeoutMs);
bool yaesuCatToggleVfo();
bool yaesuCatSelectVfoA();
bool yaesuCatSelectVfoB();
bool yaesuCatSetPtt(bool on);
bool yaesuCatSetClarifier(bool on);
bool yaesuCatSetSplit(bool on);
bool yaesuCatSetLockDocumentedRaw(bool on);
bool yaesuCatSetRepeaterShiftRaw(uint8_t shiftByte);
bool yaesuCatSetRepeaterOffsetHzRaw(uint64_t hz);
bool yaesuCatSetPowerDocumentedRaw(bool on);
bool yaesuCatMemoryWrite();
bool yaesuCatMemoryReadRaw(uint8_t rsp[5], uint32_t timeoutMs);
bool yaesuCatSetAgcMode(uint8_t modeByte);
bool yaesuCatSetClarifierOffsetRaw(const uint8_t data[4]);
bool yaesuCatSetToneDcsModeRaw(uint8_t modeByte);
bool yaesuCatSetCtcssToneRaw(const uint8_t data[4]);
bool yaesuCatSetDcsCodeRaw(const uint8_t data[4]);
// FT-8x7 CTCSS tone in tenths of Hz (885 = 88.5 Hz) and DCS code (23 = 023).
// Only the standard values are valid. A write updates the live tone cache.
bool yaesuCtcssTenthsValid(uint16_t toneTenths);
bool yaesuDcsCodeValid(uint16_t dcsCode);
bool yaesuCatSetCtcssTenths(uint16_t toneTenths);
bool yaesuCatSetDcsCode(uint16_t dcsCode);
