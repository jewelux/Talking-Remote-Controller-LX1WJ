#pragma once

// Reads (and the few writes) of the FT-8x7 EEPROM, where the radio keeps the settings CAT has no
// command for. The addresses and their decoding are in ft8x7_eeprom_map.h.

#include "ft8x7_eeprom_map.h"
#include "radio_globals.h"

// Reads the EEPROM word at addr & ~1 (0xBB): out[0] is the even byte, out[1] the odd one.
bool yaesuCatReadEepromWord(uint16_t addr, uint8_t out[2], uint32_t timeoutMs);
bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs);
// CAUTION: writes data[0] to the EEPROM at addr and data[1] at addr + 1 (0xBC). A bad write can
// wipe the radio's memories and calibration.
void yaesuCatWriteEeprom2(uint16_t addr, const uint8_t data[2]);
// FT-8x7 settings read from the EEPROM. False when the active model does not keep
// them (ft8x7_eeprom_map.h) or the radio did not answer.
bool yaesuFt8x7QueryFlag(Ft8x7Flag flag, bool& onOut, uint32_t timeoutMs);
// True when the active model keeps flag.
bool yaesuFt8x7HasFlag(Ft8x7Flag flag);
bool yaesuFt8x7QueryAgc(YaesuAgc& out, uint32_t timeoutMs);
// The menu item and the soft key row (FT-817: function row) as the radio saved them when its
// menu was last exited, counted from 1 like on the display.
bool yaesuFt8x7QueryMenuAndRow(uint8_t& menuOut, uint8_t& rowOut, uint32_t timeoutMs);
// The settings writes below read the bytes first and change only the setting's bits, write
// nothing while the radio transmits, and read the bytes back.
enum class YaesuEepromWrite : uint8_t {
  Ok,
  Unsupported,      // the active model does not keep the setting where it can be set
  OutOfRange,       // the value is not one the radio offers; nothing written
  NoReply,
  Transmitting,     // nothing written
  UnexpectedBytes,  // the stored value is out of range, so the map does not fit; nothing written
  NotVerified,      // written, but the bytes read back differ
};
// Sets the row (row true) or the menu item, counted from 1, by rewriting the two bytes that
// hold both (ft8x7MenuRowPair keeps every other bit). numberOut is the value read back. The
// radio overwrites it when its menu is open and then exited.
YaesuEepromWrite yaesuFt8x7SetMenuOrRow(bool row, uint8_t number, uint8_t& numberOut,
                                        uint32_t timeoutMs);
// Sets a flag (NB, DNR, DNF, DBF, PROC, VOX, BK, KYR); on sets every bit of the flag. Nothing is written when the
// byte is already so.
YaesuEepromWrite yaesuFt8x7SetFlag(Ft8x7Flag flag, bool on, uint32_t timeoutMs);
// Flips a flag from the byte read for the write, so a change on the radio's own keys counts.
// onOut is the flag as written.
YaesuEepromWrite yaesuFt8x7ToggleFlag(Ft8x7Flag flag, bool& onOut, uint32_t timeoutMs);
// FT-857/897: sets a menu level to the radio's step nearest value, within ft857LevelRange
// (NrLevel, NbLevel, HpfCutoff, LpfCutoff, ProcLevel, the mic gains). valueOut is the value read
// back. Nothing is written when it already has the value.
YaesuEepromWrite yaesuFt857SetLevel(YaesuFt857Level level, uint16_t value, uint16_t& valueOut,
                                    uint32_t timeoutMs);
// The same, steps of the radio's steps from the value read for the write (ft857LevelStep).
YaesuEepromWrite yaesuFt857StepLevel(YaesuFt857Level level, int steps, uint16_t& valueOut,
                                     uint32_t timeoutMs);
// FT-857/897: sets menu 75 power of the band group of hz to watts, within ft857RfPowerRange.
// wattsOut is the value read back.
YaesuEepromWrite yaesuFt857SetRfPowerWatts(uint64_t hz, uint8_t watts, uint8_t& wattsOut,
                                           uint32_t timeoutMs);
// FT-817/818: sets the TX power level. levelOut is the level read back.
YaesuEepromWrite yaesuFt817SetPowerLevel(Ft817PowerLevel level, Ft817PowerLevel& levelOut,
                                         uint32_t timeoutMs);
// Sets AGC. The FT-857/897 keeps off as its own bit: off clears it and keeps the speed, the
// speeds write the speed, then set it, checking for transmit once. When the second write fails,
// AGC stays as it was; if it was off, its speed may have changed. agcOut is the AGC read back.
YaesuEepromWrite yaesuFt8x7SetAgc(YaesuAgc agc, YaesuAgc& agcOut, uint32_t timeoutMs);
// IPO, ATT and FM narrow are kept per band; read from the block of the band the active VFO is
// on, also outside the amateur bands. hz decides whether IPO and ATT exist.
struct YaesuFt857BandFlags {
  bool hasIpoAtt = false;  // HF and 6 m
  bool ipo = false;
  bool att = false;
  bool nar = false;
};
bool yaesuFt857QueryBandFlags(uint64_t hz, YaesuFt857BandFlags& out, uint32_t timeoutMs);
// Menu 75 RF power for the band group of hz: HF, 6 m, VHF or UHF.
bool yaesuFt857QueryRfPowerWatts(uint64_t hz, uint8_t& wattsOut, uint32_t timeoutMs);
// FT-817/818: the TX power level.
bool yaesuFt817QueryPowerLevel(Ft817PowerLevel& levelOut, uint32_t timeoutMs);
// FT-817/818: true when the band group of hz uses the rear antenna jack (menu 07).
bool yaesuFt817QueryRearAntenna(uint64_t hz, bool& rearOut, uint32_t timeoutMs);
bool yaesuFt857QueryLevel(YaesuFt857Level level, uint16_t& valueOut, uint32_t timeoutMs);
// The RIT offset kept in the block of the band the active VFO is on, as of the radio's last save
// of that block (a band change saves it; turning the knob or switching RIT off does not). The
// offset stays when RIT is switched off. The IF shift offset is not kept there.
bool yaesuFt857QueryRitOffsetHz(int32_t& offsetOut, uint32_t timeoutMs);
// The tone mode in the block of the band the active VFO is on, live, also outside the amateur
// bands (the general coverage block). knownOut is false on a memory channel (also one being
// tuned), whose tone is not in the block.
bool yaesuFt857QueryToneMode(bool& knownOut, YaesuToneMode& out, uint32_t timeoutMs);
// Split tone (menu 79) of the same band block; the radio keeps it per band.
bool yaesuFt857QuerySplitTone(bool& knownOut, bool& onOut, uint32_t timeoutMs);
bool yaesuFt857QueryMicEq(YaesuFt857MicEq& out, uint32_t timeoutMs);
// FT-857/897: sets menu 48, keeping the NR level and BPF width that share its byte. eqOut is
// the value read back.
YaesuEepromWrite yaesuFt857SetMicEq(YaesuFt857MicEq eq, YaesuFt857MicEq& eqOut, uint32_t timeoutMs);
// True when the active model keeps gain where it can be read (not the FT-857/897's PKT).
bool yaesuFt8x7HasMicGain(Ft8x7MicGain gain);
// 0..100. The FT-817 reads its packet rate to pick the PKT one.
bool yaesuFt8x7QueryMicGain(Ft8x7MicGain gain, uint8_t& valueOut, uint32_t timeoutMs);
// Sets gain to value, 0..100, as the read above picks it; bit 7 of its byte is kept. valueOut
// is the value read back.
YaesuEepromWrite yaesuFt8x7SetMicGain(Ft8x7MicGain gain, uint8_t value, uint8_t& valueOut,
                                      uint32_t timeoutMs);
// The same, steps from the gain read for the write, kept within 0..100.
YaesuEepromWrite yaesuFt8x7StepMicGain(Ft8x7MicGain gain, int steps, uint8_t& valueOut, uint32_t timeoutMs);
// FT-857/897 menu 88, FT-817 menu 51; 0..100.
bool yaesuFt8x7QueryVoxGain(uint8_t& valueOut, uint32_t timeoutMs);
// FT-857/897 menu 87, FT-817 menu 50; ms.
bool yaesuFt8x7QueryVoxDelayMs(uint16_t& msOut, uint32_t timeoutMs);
// Sets the VOX gain, 1..100; bit 7 of its byte is kept. valueOut is the value read back.
YaesuEepromWrite yaesuFt8x7SetVoxGain(uint8_t value, uint8_t& valueOut, uint32_t timeoutMs);
// The same, steps from the gain read for the write, kept within 1..100.
YaesuEepromWrite yaesuFt8x7StepVoxGain(int steps, uint8_t& valueOut, uint32_t timeoutMs);
// Sets the VOX delay to the 100 ms step nearest ms: FT-857/897 100..3000, FT-817 100..2500.
// msOut is the value read back.
YaesuEepromWrite yaesuFt8x7SetVoxDelayMs(uint16_t ms, uint16_t& msOut, uint32_t timeoutMs);
// The same, steps of 100 ms from the delay read for the write, kept within the range.
YaesuEepromWrite yaesuFt8x7StepVoxDelay(int steps, uint16_t& msOut, uint32_t timeoutMs);
// FT-817/818: sets the jack of the band group of hz (menu 07), keeping the other groups and
// split. rearOut is the jack read back.
YaesuEepromWrite yaesuFt817SetRearAntenna(uint64_t hz, bool rear, bool& rearOut, uint32_t timeoutMs);
YaesuEepromWrite yaesuFt817ToggleRearAntenna(uint64_t hz, bool& rearOut, uint32_t timeoutMs);
