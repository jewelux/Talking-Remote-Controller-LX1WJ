#pragma once

#include "ft8x7_codec.h"
#include "ft8x7_eeprom_map.h"
#include "radio_globals.h"

// Pauses that keep the radio's CAT parser from missing a command.
// After a command the radio does not answer.
static constexpr uint32_t YAESU_CAT_WRITE_SETTLE_MS = 60;
// After a frequency or mode write: a readback sooner can take the line before the radio has
// settled the new value.
static constexpr uint32_t YAESU_CAT_FREQ_MODE_SETTLE_MS = 140;
// After the A/B toggle, and before and after toggling back from a look at the other VFO.
static constexpr uint32_t YAESU_CAT_VFO_SETTLE_MS = 120;
static constexpr uint32_t YAESU_CAT_VFO_RETURN_GAP_MS = 180;
// How long a read for the keypad or the console waits for its reply.
static constexpr uint32_t YAESU_CAT_REPLY_TIMEOUT_MS = 800;

bool yaesuCatQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool yaesuCatQueryModeRawByte(uint8_t& modeByteOut, uint32_t timeoutMs);
bool yaesuCatSetFrequency(const StoredProfile& sp, uint64_t hz);
bool yaesuCatQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool yaesuCatSetMode(const StoredProfile& sp, uint8_t mode);
void yaesuCatSetModeRawByte(uint8_t modeByte);
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
bool yaesuCatQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryVolumeRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySquelchRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryRxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryTxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
// Reads the EEPROM word at addr & ~1 (0xBB): out[0] is the even byte, out[1] the odd one.
bool yaesuCatReadEepromWord(uint16_t addr, uint8_t out[2], uint32_t timeoutMs);
bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs);
// CAUTION: writes data[0] to the EEPROM at addr and data[1] at addr + 1 (0xBC). A bad write can
// wipe the radio's memories and calibration.
void yaesuCatWriteEeprom2(uint16_t addr, const uint8_t data[2]);
// Split from the TX status while transmitting, else from the EEPROM.
bool yaesuCatQuerySplit(bool& onOut, uint32_t timeoutMs);
// FT-8x7 settings read from the EEPROM (read-only). False when the active model does not keep
// them (ft8x7_eeprom_map.h) or the radio did not answer.
bool yaesuFt8x7QueryFlag(Ft8x7Flag flag, bool& onOut, uint32_t timeoutMs);
// True when the active model keeps flag.
bool yaesuFt8x7HasFlag(Ft8x7Flag flag);
bool yaesuFt8x7QueryAgc(YaesuAgc& out, uint32_t timeoutMs);
// The menu item and the soft key row (FT-817: function row) as the radio saved them when its
// menu was last exited, counted from 1 like on the display.
bool yaesuFt8x7QueryMenuAndRow(uint8_t& menuOut, uint8_t& rowOut, uint32_t timeoutMs);
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
// FT-817/818: the TX power setting in tenths of a watt (FT-817: 50, 25, 10 or 5).
bool yaesuFt817QueryRfPowerTenths(uint16_t& tenthsOut, uint32_t timeoutMs);
// FT-817/818: true when the band group of hz uses the rear antenna jack (menu 07).
bool yaesuFt817QueryRearAntenna(uint64_t hz, bool& rearOut, uint32_t timeoutMs);
bool yaesuFt857QueryLevel(YaesuFt857Level level, uint16_t& valueOut, uint32_t timeoutMs);
// The RIT offset kept in the band block of hz, as of the radio's last save of that block (a
// band change saves it; turning the knob or switching RIT off does not). The offset stays when
// RIT is switched off. The IF shift offset is not kept there.
bool yaesuFt857QueryRitOffsetHz(uint64_t hz, int32_t& offsetOut, uint32_t timeoutMs);
bool yaesuFt857QueryMicEq(YaesuFt857MicEq& out, uint32_t timeoutMs);
// RIT (a short press of the CLAR key, which the manuals also call the clarifier), which the CAT
// clarifier commands switch. Also right after a front panel change. IF shift (a long press)
// can be read (Ft8x7Flag::IfShift) but not switched.
bool yaesuFt8x7QueryRit(bool& onOut, uint32_t timeoutMs);
bool yaesuFt8x7SetRit(bool on, uint32_t timeoutMs);
bool yaesuFt8x7ToggleRit(bool& onOut, uint32_t timeoutMs);
// Write-only commands: the radio does not answer them and the UART cannot report a failed
// send, so there is nothing to check after one.
void yaesuCatToggleVfo();
void yaesuCatSelectVfoA();
void yaesuCatSelectVfoB();
void yaesuCatSetPtt(bool on);
void yaesuCatSetClarifier(bool on);
void yaesuCatSetSplit(bool on);
void yaesuCatSetLockDocumentedRaw(bool on);
void yaesuCatSetRepeaterShiftRaw(uint8_t shiftByte);
void yaesuCatSetRepeaterOffsetHzRaw(uint64_t hz);
void yaesuCatSetPowerDocumentedRaw(bool on);
bool yaesuCatMemoryWrite();
bool yaesuCatMemoryReadRaw(uint8_t rsp[5], uint32_t timeoutMs);
void yaesuCatSetAgcMode(uint8_t modeByte);
void yaesuCatSetClarifierOffsetRaw(const uint8_t data[4]);
void yaesuCatSetToneDcsModeRaw(uint8_t modeByte);
void yaesuCatSetCtcssToneRaw(const uint8_t data[4]);
void yaesuCatSetDcsCodeRaw(const uint8_t data[4]);
// A write of a standard tone or code (ft8x7_codec.h) updates the live tone cache.
bool yaesuCatSetCtcssTenths(uint16_t toneTenths);
bool yaesuCatSetDcsCode(uint16_t dcsCode);
