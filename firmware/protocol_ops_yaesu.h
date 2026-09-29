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
// Meters are 0..15 bars and stay 0 in receive.
struct YaesuTxMeters {
  bool transmitting = false;
  bool highSwr = false;  // TX status bit 6
  uint8_t po = 0;        // TX status bits 3..0
  uint8_t alc = 0;       // 0xBD, only queried withBdMeters
  uint8_t swr = 0;       // 0xBD, only queried withBdMeters
};
bool yaesuCatQueryTxMeters(YaesuTxMeters& out, bool withBdMeters, uint32_t timeoutMs);
float yaesuSwrFromMeter(uint8_t bars);
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
// Reads the EEPROM word at addr & ~1 (0xBB): out[0] is the even byte, out[1] the odd one.
bool yaesuCatReadEepromWord(uint16_t addr, uint8_t out[2], uint32_t timeoutMs);
bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs);
bool yaesuCatQuerySplit(bool& onOut, uint32_t timeoutMs);
// FT-857/897 settings read from the EEPROM (read-only). False on other models.
bool yaesuFt857QueryNb(bool& onOut, uint32_t timeoutMs);
bool yaesuFt857QueryBreakIn(bool& onOut, uint32_t timeoutMs);
bool yaesuFt857QueryKeyer(bool& onOut, uint32_t timeoutMs);
bool yaesuFt857QueryDnr(bool& onOut, uint32_t timeoutMs);
bool yaesuFt857QueryDnf(bool& onOut, uint32_t timeoutMs);
bool yaesuFt857QueryDbf(bool& onOut, uint32_t timeoutMs);
enum class YaesuAgc : uint8_t { Off, Fast, Slow, Auto };
bool yaesuFt857QueryAgc(YaesuAgc& out, uint32_t timeoutMs);
// IPO, ATT and FM narrow are kept per band. bandKnown is false outside the amateur bands.
struct YaesuFt857BandFlags {
  bool bandKnown = false;
  bool hasIpoAtt = false;  // HF and 6 m
  bool ipo = false;
  bool att = false;
  bool nar = false;
};
bool yaesuFt857QueryBandFlags(uint64_t hz, YaesuFt857BandFlags& out, uint32_t timeoutMs);
// Menu 75 RF power for the band group of hz: HF, 6 m, VHF or UHF.
bool yaesuFt857QueryRfPowerWatts(uint64_t hz, uint8_t& wattsOut, uint32_t timeoutMs);
// The menu item and the soft key row as the radio saved them when its menu was last exited,
// counted from 1 like on the display.
bool yaesuFt857QueryMenuAndRow(uint8_t& menuOut, uint8_t& rowOut, uint32_t timeoutMs);
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
