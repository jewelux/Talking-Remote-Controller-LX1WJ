#pragma once

// Where the FT-817/818 and FT-857/897 keep their settings in the EEPROM and how the bytes
// decode, without Arduino so the host tests can check them (tests/ft8x7). The reads are in
// protocol_ops_yaesu.cpp.
//
// FT-857/897 addresses from the yo3ggx FT8x7EE map (https://www.yo3ggx.ro/ft8x7ee/eeprom.html),
// which Hamlib uses too, and measured on an FT-897 by changing one setting at a time. FT-817
// addresses from the KA7OEI map (https://www.ka7oei.com/ft817_memmap.html) and the FT8x7Com
// FT817Setup project, measured on an FT-817.

#include <stdint.h>

#include "ft8x7_codec.h"
#include "ft8x7_model.h"

// Settings kept as one bit, or as a group of bits of which any set means on.
enum class Ft8x7Flag : uint8_t {
  Nb,
  BreakIn,
  Keyer,
  Vox,
  Proc,
  Lock,
  FastTuning,
  IfShift,        // a long press of CLAR; the KA7OEI map's "PBT"
  Dnr,
  Dnf,
  Dbf,
  AgcOn,          // FT-857/897: AGC off is its own bit
  DspRow,         // the DSP soft key row is shown (saved at once, unlike the row number)
  VfoB,           // VFO B is active; follows the A/B key and the CAT toggle
  Filter2,        // filter 2 is chosen on the CFIL row (filter 1 could not be measured)
  KnobIsSquelch,  // menu 80: the RF/SQL knob is squelch, not RF gain
  Split,          // valid in receive; the TX status has it while transmitting
};

struct Ft8x7FlagField {
  uint16_t addr;
  uint8_t mask;
  bool inverted;  // the bit is set when the setting is off
};

// Where model keeps flag. False when it does not (or for Ft8x7Model::None).
bool ft8x7FlagField(Ft8x7Model model, Ft8x7Flag flag, Ft8x7FlagField& out);
bool ft8x7FlagValue(const Ft8x7FlagField& field, uint8_t b);
// The byte that sets a flag on or off, made from old with every other bit kept. On sets every
// bit of the mask, as the FT-897 does for DBF (both bits, in SSB and CW; measured).
uint8_t ft8x7FlagByte(const Ft8x7FlagField& field, bool on, uint8_t old);

enum class YaesuAgc : uint8_t { Off, Fast, Slow, Auto };

// Menu 48.
enum class YaesuFt857MicEq : uint8_t { Off, Lpf, Hpf, Both };

// Menu levels, returned in the units the radio shows.
enum class YaesuFt857Level : uint8_t {
  CwSpeed,       // menu 30, WPM
  AmMicGain,     // menu 5
  DigGain,       // menu 37
  DigVox,        // menu 40
  BpfWidth,      // menu 45, Hz
  HpfCutoff,     // menu 46, Hz
  LpfCutoff,     // menu 47, Hz
  NrLevel,       // menu 49
  FmMicGain,     // menu 51
  NbLevel,       // menu 63
  Pkt1200,       // menu 71
  Pkt9600,       // menu 72
  ProcLevel,     // menu 74
  SsbMicGain,    // menu 81
  VoxDelay,      // menu 87, ms
  VoxGain,       // menu 88
};

// The band groups the radios keep one setting for (FT-857/897 menu 75 power, FT-817 menu 07
// antenna). The FT-857/897 counts FM broadcast and air as VHF.
enum class Ft8x7BandGroup : uint8_t { Hf, SixM, FmBroadcast, Air, Vhf, Uhf };
Ft8x7BandGroup ft8x7BandGroupForHz(uint64_t hz);

// FT-857/897 band blocks: each band keeps a 28-byte block per VFO: +1 bit 3 = NAR, +2 bit 5 =
// IPO, +2 bit 4 = ATT, +10..+11 = RIT offset, +12..+15 = the band's last frequency in 10 Hz
// units, big-endian. 16 bands, in the order of 0x6C below: 160 m 0xBA, 80 m, 5 MHz, 40 m, 30 m,
// 20 m, 17 m, 15 m, 12 m, 10 m, 6 m, FM broadcast, air, 2 m, 70 cm, general coverage 0x25E,
// each 0x1C on; VFO B's blocks follow VFO A's. Block addresses measured on an FT-897; they
// differ from the yo3ggx map only for 5 MHz, which that map puts at 0x25E.
static constexpr uint16_t FT857_BAND_NAR_OFFSET = 1;
static constexpr uint8_t FT857_BAND_NAR_MASK = 0x08;
static constexpr uint16_t FT857_BAND_IPO_ATT_OFFSET = 2;
static constexpr uint8_t FT857_BAND_IPO_MASK = 0x20;
static constexpr uint8_t FT857_BAND_ATT_MASK = 0x10;
static constexpr uint16_t FT857_BAND_RIT_OFFSET = 10;
// Band block +4 bits 3..0: the tone mode, written as soon as it changes. Bits 3..2 are what is
// sent, bits 1..0 what the squelch waits for, each 01 = CTCSS and 10 = DCS. Measured on an
// FT-897 by setting each CAT tone mode and, for the mixed types, with the radio's own keys.
// Bit 4 is split tone (menu 79), kept per band and written on leaving the menu. A mixed mode
// stays when split tone is switched off. Bit 7 is something else.
static constexpr uint16_t FT857_BAND_TONE_OFFSET = 4;
static constexpr uint8_t FT857_BAND_SPLIT_TONE_MASK = 0x10;
YaesuToneMode ft857ToneModeFromByte(uint8_t b);
// FT-857/897 0x68, the byte of Ft8x7Flag::VfoB (bit 0): 0x80 VFO, 0x40 a memory channel, 0x60
// a memory being tuned. On a memory the radio uses the channel's own record, not the band
// block, and which channel is in use cannot be read.
static constexpr uint8_t FT857_MEMORY_MASK = 0x40;
// FT-857/897 0x6C: the band each VFO is on, VFO A bits 3..0 and VFO B bits 7..4, numbered in
// block order (13 = 2 m, 14 = 70 cm, 15 = general coverage, which the radio uses outside its
// other bands). Measured on an FT-897 with the band keys (2 m to 70 cm) and on 26.8 MHz.
static constexpr uint16_t FT857_VFO_BANDS_ADDR = 0x006C;
// The block of the band 0x6C names for VFO A or B.
uint16_t ft857VfoBandBlock(uint8_t vfoBands, bool vfoB);
// IPO and ATT exist on HF and 6 m only, so on the general coverage block too below 76 MHz.
bool ft857HasIpoAtt(uint64_t hz);

// The amateur band edges, which the FT-817/818 tone mode uses to tell bands apart. Null outside
// them.
struct Ft857BandSlot {
  uint32_t lowKhz;
  uint32_t highKhz;
};
const Ft857BandSlot* ft857BandSlotForHz(uint64_t hz);
// Band block +10..+11: signed, 10 Hz units (0xFFE1 = -310 Hz).
int32_t ft857RitOffsetHz(const uint8_t word[2]);

// Menu 75 keeps one maximum power per band group. Bits 6..0 are the watts; bit 7 is set from
// 20 W up (100 W is 0xE4).
uint16_t ft857RfPowerAddr(Ft8x7BandGroup group);
uint8_t ft857RfPowerWatts(uint8_t b);

// AGC off is its own bit (Ft8x7Flag::AgcOn); the speed is 0x6A bits 1..0.
static constexpr uint16_t FT857_AGC_SPEED_ADDR = 0x006A;
YaesuAgc ft857AgcFromSpeed(uint8_t b);

// The menu item and the soft key row (FT-817: function row), counted from 0 in two neighbouring
// bytes that one 0xBC writes together. The FT-857/897 saves both when its menu is exited; the
// FT-817 saves its row at once. The last item depends on the extended menu (FT-857/897 menu 1
// EXT MENU, FT-817 menu 52 EXTEND). Measured at both ends on an FT-897 and an FT-817; the
// FT-857 and FT-818 are taken to match.
//   FT-857/897: 0x88 = item, 0x89 = row, whole bytes; 17 rows; 88 items, 91 with 0x91 bit 7
//   (EXT MENU) on.
//   FT-817/818: 0x75 bits 5..0 = item, 0x76 bits 3..0 = row (FT8x7Com keeps the other bits);
//   12 rows; 52 items, 57 with 0x6B bit 7 (EXTEND) on. 0x6B bits 6..0 are the PKT mic gain.
struct Ft8x7MenuRowMap {
  uint16_t addr;     // the item byte; the row byte is at addr + 1
  uint8_t menuMask;  // the bits of the item byte that hold the item
  uint8_t rowMask;
  uint8_t rows;
  uint8_t menus;     // the last item with the extended menu off
  uint8_t extMenus;  // and with it on
  uint16_t extAddr;  // the extended menu switch
  uint8_t extMask;
};
// False for Ft8x7Model::None.
bool ft8x7MenuRowMap(Ft8x7Model model, Ft8x7MenuRowMap& out);
// pair: the bytes at addr and addr + 1. Counted from 1 like on the display.
uint8_t ft8x7MenuFromPair(const Ft8x7MenuRowMap& map, const uint8_t pair[2]);
uint8_t ft8x7RowFromPair(const Ft8x7MenuRowMap& map, const uint8_t pair[2]);
// The last row (row true) or menu item, extOn when the extended menu is on.
uint8_t ft8x7MenuRowLast(const Ft8x7MenuRowMap& map, bool row, bool extOn);
// The pair that sets the row (row true) or the menu item to number, made from old, the pair as
// read, with every other bit of both bytes kept. False, and out untouched, when number is not
// 1..ft8x7MenuRowLast, or when old does not hold an item and a row in range: the map then does
// not fit the radio, and nothing may be written.
bool ft8x7MenuRowPair(const Ft8x7MenuRowMap& map, bool row, uint8_t number, bool extOn,
                      const uint8_t old[2], uint8_t out[2]);

// 0x93 bits 1..0.
static constexpr uint16_t FT857_MIC_EQ_ADDR = 0x0093;
YaesuFt857MicEq ft857MicEqFromByte(uint8_t b);

// The byte that holds a level; false for an unknown level.
bool ft857LevelAddr(YaesuFt857Level level, uint16_t& addrOut);
// The level from the byte at its address, in the units the radio shows.
uint16_t ft857LevelValue(YaesuFt857Level level, uint8_t b);
// The values a level can be set to, in the units the radio shows: NrLevel 1..16, NbLevel 0..100,
// HpfCutoff 100..1000 Hz, LpfCutoff 1000..6000 Hz. False for the other levels, which cannot be
// set.
bool ft857LevelRange(YaesuFt857Level level, uint16_t& minOut, uint16_t& maxOut);
// The byte that sets level to the radio's step nearest value (HPF steps are 60 Hz, LPF steps
// about 160 Hz), made from old with every other bit kept. False, and out untouched, when value
// is outside ft857LevelRange, or when old does not hold a value in range: the map then does not
// fit the radio, and nothing may be written.
bool ft857LevelByte(YaesuFt857Level level, uint16_t value, uint8_t old, uint8_t& out);
// The value steps of the radio's steps away from the step nearest value, kept within
// ft857LevelRange. value itself for a level that cannot be set.
uint16_t ft857LevelStep(YaesuFt857Level level, uint16_t value, int steps);

// The mic gain a mode transmits with; each mode group has its own.
enum class Ft8x7MicGain : uint8_t { Ssb, Am, Fm, Dig, Pkt };
// From a CAT mode byte (bit 7, the narrow variant, ignored). False for CW and WFM, which have
// none.
bool ft8x7MicGainForMode(uint8_t modeByte, Ft8x7MicGain& out);
// FT-857/897: SSB menu 81, AM menu 5, FM menu 51, DIG gain menu 37. False for PKT, whose level
// (menu 71 or 72) depends on the packet rate in menu 70, which is not in the map.
bool ft857MicGainLevel(Ft8x7MicGain gain, YaesuFt857Level& out);

// FT-817/818 0x79 bits 1..0: TX power High, L3, L2, L1. On an external supply 5, 2.5, 1 and
// 0.5 W on the FT-817, and 6, 5, 2.5 and 1 W on the FT-818 (not measured on an FT-818).
static constexpr uint16_t FT817_RF_POWER_ADDR = 0x0079;
uint16_t ft817RfPowerTenths(uint8_t b, bool ft818);

// FT-817 0x57 bits 1..0: 00 auto, 01 fast, 10 slow, 11 off.
static constexpr uint16_t FT817_AGC_ADDR = 0x0057;
YaesuAgc ft817AgcFromByte(uint8_t b);

// FT-817 0x7A bits 5..0: the antenna of each band group, 1 = rear. Bit 0 HF, 1 6 m, 2 FM
// broadcast, 3 air, 4 2 m, 5 UHF (KA7OEI map). Bit 7 of the same byte is split.
static constexpr uint16_t FT817_ANTENNA_ADDR = 0x007A;
uint8_t ft817RearAntennaMask(Ft8x7BandGroup group);

// FT-817 mic gains, bits 6..0, 0..100 (KA7OEI map): SSB 0x67 (menu 46), AM 0x68 (menu 5), FM
// 0x69 (menu 29), DIG 0x6A (menu 25), PKT 0x6B (menu 39) at 1200 baud and 0x6C (menu 3) at
// 9600. The packet rate is 0x5D bit 2 (menu 40), 1 = 9600.
static constexpr uint16_t FT817_PKT_RATE_ADDR = 0x005D;
static constexpr uint8_t FT817_PKT_RATE_9600_MASK = 0x04;
uint16_t ft817MicGainAddr(Ft8x7MicGain gain, bool pkt9600);

// FT-817 VOX gain (menu 51), 0x63 bits 6..0, 1..100 (KA7OEI map, measured).
static constexpr uint16_t FT817_VOX_GAIN_ADDR = 0x0063;

// The mic gains and the VOX gain: bits 6..0, 0..100.
uint8_t ft817LevelFromByte(uint8_t b);

// FT-817 VOX delay (menu 50), 0x64 bits 4..0: the delay / 100 ms, 1..25 = 100..2500 ms;
// other values are kept within that range.
// Measured: 300 ms is 3 and the default 500 ms is 5; the KA7OEI map says 0 = 100 ms.
static constexpr uint16_t FT817_VOX_DELAY_ADDR = 0x0064;
uint16_t ft817VoxDelayMs(uint8_t b);
