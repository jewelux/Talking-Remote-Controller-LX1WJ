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
// units, big-endian. VFO B's blocks follow VFO A's. Block addresses measured on an FT-897; they
// differ from the yo3ggx map only for 5 MHz, which that map puts at 0x25E (on the FT-897 the
// general coverage block).
struct Ft857BandSlot {
  uint32_t lowKhz;
  uint32_t highKhz;
  uint16_t block;   // VFO A
  bool hasIpoAtt;   // IPO and ATT exist on HF and 6 m only
};
static constexpr uint16_t FT857_BAND_NAR_OFFSET = 1;
static constexpr uint8_t FT857_BAND_NAR_MASK = 0x08;
static constexpr uint16_t FT857_BAND_IPO_ATT_OFFSET = 2;
static constexpr uint8_t FT857_BAND_IPO_MASK = 0x20;
static constexpr uint8_t FT857_BAND_ATT_MASK = 0x10;
static constexpr uint16_t FT857_BAND_RIT_OFFSET = 10;

// Only the amateur bands: the band edges the radio uses outside them are not known. Null
// outside them.
const Ft857BandSlot* ft857BandSlotForHz(uint64_t hz);
uint16_t ft857BandBlock(const Ft857BandSlot& slot, bool vfoB);
// Band block +10..+11: signed, 10 Hz units (0xFFE1 = -310 Hz).
int32_t ft857RitOffsetHz(const uint8_t word[2]);

// Menu 75 keeps one maximum power per band group. Bits 6..0 are the watts; bit 7 is set from
// 20 W up (100 W is 0xE4).
uint16_t ft857RfPowerAddr(Ft8x7BandGroup group);
uint8_t ft857RfPowerWatts(uint8_t b);

// AGC off is its own bit (Ft8x7Flag::AgcOn); the speed is 0x6A bits 1..0.
static constexpr uint16_t FT857_AGC_SPEED_ADDR = 0x006A;
YaesuAgc ft857AgcFromSpeed(uint8_t b);

// 0x88 = menu item, 0x89 = soft key row, both from 0. Measured on an FT-897: the radio writes
// them when its menu is exited, not while the rows are stepped.
static constexpr uint16_t FT857_MENU_ROW_ADDR = 0x0088;

// 0x93 bits 1..0.
static constexpr uint16_t FT857_MIC_EQ_ADDR = 0x0093;
YaesuFt857MicEq ft857MicEqFromByte(uint8_t b);

// The byte that holds a level; false for an unknown level.
bool ft857LevelAddr(YaesuFt857Level level, uint16_t& addrOut);
// The level from the byte at its address, in the units the radio shows.
uint16_t ft857LevelValue(YaesuFt857Level level, uint8_t b);

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

// FT-817 0x75 bits 5..0 = menu item, 0x76 bits 3..0 = function row, both counted from 0.
static constexpr uint16_t FT817_MENU_ADDR = 0x0075;
static constexpr uint16_t FT817_ROW_ADDR = 0x0076;
uint8_t ft817MenuFromByte(uint8_t b);
uint8_t ft817RowFromByte(uint8_t b);

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
