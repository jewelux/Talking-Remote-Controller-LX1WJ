#pragma once

// Reads (and the one write) of the FT-8x7 EEPROM, where the radio keeps the settings CAT has no
// command for. The addresses and their decoding are in ft8x7_eeprom_map.h.

#include "ft8x7_eeprom_map.h"
#include "radio_globals.h"

// Reads the EEPROM word at addr & ~1 (0xBB): out[0] is the even byte, out[1] the odd one.
bool yaesuCatReadEepromWord(uint16_t addr, uint8_t out[2], uint32_t timeoutMs);
bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs);
// CAUTION: writes data[0] to the EEPROM at addr and data[1] at addr + 1 (0xBC). A bad write can
// wipe the radio's memories and calibration.
void yaesuCatWriteEeprom2(uint16_t addr, const uint8_t data[2]);
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
// True when the active model keeps gain where it can be read (not the FT-857/897's PKT).
bool yaesuFt8x7HasMicGain(Ft8x7MicGain gain);
// 0..100. The FT-817 reads its packet rate to pick the PKT one.
bool yaesuFt8x7QueryMicGain(Ft8x7MicGain gain, uint8_t& valueOut, uint32_t timeoutMs);
