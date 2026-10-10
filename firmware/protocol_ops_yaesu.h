#pragma once

#include "ft8x7_codec.h"
#include "protocol_ft8x7_eeprom.h"
#include "radio_globals.h"

// Pauses that keep the radio's CAT parser from missing a command (the one after a write-only
// command is in protocol_yaesu_cat.h).
// After a frequency or mode write: a readback sooner can take the line before the radio has
// settled the new value.
static constexpr uint32_t YAESU_CAT_FREQ_MODE_SETTLE_MS = 140;
// After the A/B toggle, and before and after toggling back from a look at the other VFO.
static constexpr uint32_t YAESU_CAT_VFO_SETTLE_MS = 120;
static constexpr uint32_t YAESU_CAT_VFO_RETURN_GAP_MS = 180;
// How long a read for the keypad or the console waits for its reply.
static constexpr uint32_t YAESU_CAT_REPLY_TIMEOUT_MS = 800;

bool yaesuCatQueryFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool yaesuCatQueryModeRawByte(uint8_t& modeByteOut, uint32_t timeoutMs);
bool yaesuCatSetFrequency(const RadioProfile& sp, uint64_t hz);
bool yaesuCatQueryMode(const RadioProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool yaesuCatSetMode(const RadioProfile& sp, uint8_t mode);
void yaesuCatSetModeRawByte(uint8_t modeByte);
bool yaesuCatQuerySMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
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
bool yaesuCatQueryPoMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySWRRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryVolumeRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySquelchRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryRxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryTxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
// Split from the TX status while transmitting, else from the EEPROM.
bool yaesuCatQuerySplit(bool& onOut, uint32_t timeoutMs);
// RIT (a short press of the CLAR key), which the CAT commands 05 and 85 switch. Also right after
// a front panel change. IF shift (a long press) can be read (Ft8x7Flag::IfShift) but not
// switched.
bool yaesuFt8x7QueryRit(bool& onOut, uint32_t timeoutMs);
bool yaesuFt8x7SetRit(bool on, uint32_t timeoutMs);
bool yaesuFt8x7ToggleRit(bool& onOut, uint32_t timeoutMs);
// Write-only commands: the radio does not answer them and the UART cannot report a failed
// send, so there is nothing to check after one.
void yaesuCatToggleVfo();
void yaesuCatSelectVfoA();
void yaesuCatSelectVfoB();
void yaesuCatSetPtt(bool on);
void yaesuCatSetSplit(bool on);
void yaesuCatSetLockDocumentedRaw(bool on);
void yaesuCatSetRepeaterShiftRaw(uint8_t shiftByte);
void yaesuCatSetRepeaterOffsetHzRaw(uint64_t hz);
void yaesuCatSetPowerDocumentedRaw(bool on);
void yaesuCatSetAgcMode(uint8_t modeByte);
void yaesuCatSetRitOffsetRaw(const uint8_t data[4]);
void yaesuCatSetToneDcsModeRaw(uint8_t modeByte);
void yaesuCatSetCtcssToneRaw(const uint8_t data[4]);
void yaesuCatSetDcsCodeRaw(const uint8_t data[4]);
// A write of a standard tone or code (ft8x7_codec.h) updates the live tone cache.
bool yaesuCatSetCtcssTenths(uint16_t toneTenths);
bool yaesuCatSetDcsCode(uint16_t dcsCode);
