#include "ft8x7_eeprom_map.h"

#include <stddef.h>

struct Ft8x7FlagRow {
  Ft8x7Flag flag;
  Ft8x7FlagField field;
};

// Measured on an FT-897 by changing one setting at a time. Lock and fast tuning are stored
// inverted. Split at 0x8D as Hamlib reads it (0x03 with split off, 0x83 with split on).
static constexpr Ft8x7FlagRow kFt857Flags[] = {
  {Ft8x7Flag::VfoB, {0x0068, 0x01, false}},
  {Ft8x7Flag::IfShift, {0x006A, 0x10, false}},
  {Ft8x7Flag::Nb, {0x006A, 0x20, false}},
  {Ft8x7Flag::Lock, {0x006A, 0x40, true}},
  {Ft8x7Flag::FastTuning, {0x006A, 0x80, true}},
  {Ft8x7Flag::KeyReverse, {0x006B, 0x04, false}},
  {Ft8x7Flag::Keyer, {0x006B, 0x10, false}},
  {Ft8x7Flag::BreakIn, {0x006B, 0x20, false}},
  {Ft8x7Flag::Vox, {0x006B, 0x80, false}},
  {Ft8x7Flag::KnobIsSquelch, {0x0072, 0x80, false}},
  {Ft8x7Flag::PaddleMicKey, {0x007B, 0x80, false}},  // bits 6..0 are the AM mic gain
  {Ft8x7Flag::Split, {0x008D, 0x80, false}},
  {Ft8x7Flag::CwAutoMode, {0x0092, 0x80, false}},
  // 0xA7 bit 1 was set when the built-in filter was chosen in CW but not in USB.
  {Ft8x7Flag::Filter2, {0x00A7, 0x80, false}},
  {Ft8x7Flag::Dnf, {0x00A8, 0x01, false}},
  {Ft8x7Flag::Dnr, {0x00A8, 0x02, false}},
  {Ft8x7Flag::Dbf, {0x00A8, 0x0C, false}},
  {Ft8x7Flag::AgcOn, {0x00A8, 0x20, false}},
  {Ft8x7Flag::DspRow, {0x00A8, 0x80, false}},
  {Ft8x7Flag::Proc, {0x00A9, 0x02, false}},
};

// The bit positions of the KA7OEI map, measured on an FT-817 by switching each setting on the
// radio. Lock and fast tuning are stored inverted, like on the FT-857/897, though the map says
// 1 = on. All follow the front panel at once.
static constexpr Ft8x7FlagRow kFt817Flags[] = {
  {Ft8x7Flag::IfShift, {0x0057, 0x10, false}},
  {Ft8x7Flag::Nb, {0x0057, 0x20, false}},
  {Ft8x7Flag::Lock, {0x0057, 0x40, true}},
  {Ft8x7Flag::FastTuning, {0x0057, 0x80, true}},
  {Ft8x7Flag::KeyReverse, {0x0058, 0x04, false}},  // KA7OEI map, not measured
  {Ft8x7Flag::Keyer, {0x0058, 0x10, false}},
  {Ft8x7Flag::BreakIn, {0x0058, 0x20, false}},
  {Ft8x7Flag::Vox, {0x0058, 0x80, false}},
  {Ft8x7Flag::Split, {0x007A, 0x80, false}},
};

template <size_t N>
static bool findFlag(const Ft8x7FlagRow (&rows)[N], Ft8x7Flag flag, Ft8x7FlagField& out) {
  for (const Ft8x7FlagRow& row : rows) {
    if (row.flag != flag) continue;
    out = row.field;
    return true;
  }
  return false;
}

bool ft8x7FlagField(Ft8x7Model model, Ft8x7Flag flag, Ft8x7FlagField& out) {
  if (ft8x7IsFt817Family(model)) return findFlag(kFt817Flags, flag, out);
  if (model == Ft8x7Model::Ft857) return findFlag(kFt857Flags, flag, out);
  return false;
}

bool ft8x7FlagValue(const Ft8x7FlagField& field, uint8_t b) {
  return ((b & field.mask) != 0) != field.inverted;
}

uint8_t ft8x7FlagByte(const Ft8x7FlagField& field, bool on, uint8_t old) {
  return (on != field.inverted) ? (uint8_t)(old | field.mask) : (uint8_t)(old & ~field.mask);
}

Ft8x7BandGroup ft8x7BandGroupForHz(uint64_t hz) {
  if (hz >= 420000000ULL) return Ft8x7BandGroup::Uhf;
  if (hz >= 137000000ULL) return Ft8x7BandGroup::Vhf;
  if (hz >= 108000000ULL) return Ft8x7BandGroup::Air;
  if (hz >= 76000000ULL) return Ft8x7BandGroup::FmBroadcast;
  if (hz >= 33000000ULL) return Ft8x7BandGroup::SixM;
  return Ft8x7BandGroup::Hf;
}

static constexpr uint16_t kFt857FirstBlock = 0x00BA;
static constexpr uint16_t kFt857BlockSize = 0x1C;
static constexpr uint16_t kFt857VfoBBlockOffset = 0x01C0;  // 16 blocks

static constexpr Ft857BandSlot kFt857BandSlots[] = {
  {1800, 2000},     {3500, 4000},     {5250, 5450},     {7000, 7300},     {10100, 10150},
  {14000, 14350},   {18068, 18168},   {21000, 21450},   {24890, 24990},   {28000, 29700},
  {50000, 54000},   {144000, 148000}, {430000, 450000},
};

const Ft857BandSlot* ft857BandSlotForHz(uint64_t hz) {
  const uint32_t khz = (uint32_t)(hz / 1000ULL);
  for (const Ft857BandSlot& slot : kFt857BandSlots) {
    if (khz >= slot.lowKhz && khz <= slot.highKhz) return &slot;
  }
  return nullptr;
}

uint16_t ft857VfoBandBlock(uint8_t vfoBands, bool vfoB) {
  const uint8_t band = vfoB ? (uint8_t)(vfoBands >> 4) : (uint8_t)(vfoBands & 0x0F);
  const uint16_t block = (uint16_t)(kFt857FirstBlock + band * kFt857BlockSize);
  return vfoB ? (uint16_t)(block + kFt857VfoBBlockOffset) : block;
}

bool ft857HasIpoAtt(uint64_t hz) {
  const Ft8x7BandGroup group = ft8x7BandGroupForHz(hz);
  return group == Ft8x7BandGroup::Hf || group == Ft8x7BandGroup::SixM;
}

int32_t ft857RitOffsetHz(const uint8_t word[2]) {
  return (int32_t)(int16_t)(((uint16_t)word[0] << 8) | word[1]) * 10;
}

static YaesuToneType ft857ToneType(uint8_t bits) {
  switch (bits & 0x03) {
    case 0x01: return YaesuToneType::Ctcss;
    case 0x02: return YaesuToneType::Dcs;
    default: return YaesuToneType::None;
  }
}

YaesuToneMode ft857ToneModeFromByte(uint8_t b) {
  YaesuToneMode mode;
  mode.tx = ft857ToneType((uint8_t)(b >> 2));
  mode.rx = ft857ToneType(b);
  return mode;
}

uint16_t ft857RfPowerAddr(Ft8x7BandGroup group) {
  switch (group) {
    case Ft8x7BandGroup::Hf: return 0x009B;
    case Ft8x7BandGroup::SixM: return 0x00AA;
    case Ft8x7BandGroup::Uhf: return 0x00AC;
    default: return 0x00AB;  // VHF, with FM broadcast and air
  }
}

uint8_t ft857RfPowerWatts(uint8_t b) {
  return b & 0x7F;
}

void ft857RfPowerRange(Ft8x7BandGroup group, uint8_t& minOut, uint8_t& maxOut) {
  switch (group) {
    case Ft8x7BandGroup::Hf:
    case Ft8x7BandGroup::SixM: minOut = 5; maxOut = 100; return;
    case Ft8x7BandGroup::Uhf: minOut = 2; maxOut = 20; return;
    default: minOut = 5; maxOut = 50; return;  // VHF, with FM broadcast and air
  }
}

bool ft857RfPowerByte(Ft8x7BandGroup group, uint8_t watts, uint8_t old, uint8_t& out) {
  uint8_t lo = 0;
  uint8_t hi = 0;
  ft857RfPowerRange(group, lo, hi);
  const uint8_t stored = ft857RfPowerWatts(old);
  if (watts < lo || watts > hi || stored < lo || stored > hi) return false;
  out = (uint8_t)(watts | (watts >= 20 ? 0x80 : 0x00));
  return true;
}

// old with bits 1..0 set to bits, where the FT-857/897 AGC speed and mic EQ and the FT-817 power
// level and AGC are kept.
static uint8_t withLowBits(uint8_t old, uint8_t bits) {
  return (uint8_t)((old & ~0x03) | (bits & 0x03));
}

YaesuAgc ft857AgcFromSpeed(uint8_t b) {
  switch (b & 0x03) {
    case 0x01: return YaesuAgc::Fast;
    case 0x02: return YaesuAgc::Slow;
    default: return YaesuAgc::Auto;
  }
}

uint8_t ft857AgcSpeedByte(YaesuAgc agc, uint8_t old) {
  uint8_t speed = 0;
  switch (agc) {
    case YaesuAgc::Auto: speed = 0x00; break;
    case YaesuAgc::Fast: speed = 0x01; break;
    case YaesuAgc::Slow: speed = 0x02; break;
    default: return old;
  }
  return withLowBits(old, speed);
}

YaesuFt857MicEq ft857MicEqFromByte(uint8_t b) {
  return (YaesuFt857MicEq)(b & 0x03);
}

uint8_t ft857MicEqByte(YaesuFt857MicEq eq, uint8_t old) {
  return withLowBits(old, (uint8_t)eq);
}

struct Ft857LevelField {
  YaesuFt857Level level;
  uint16_t addr;
  uint8_t mask;
  uint8_t shift;
};

static constexpr Ft857LevelField kFt857LevelFields[] = {
  {YaesuFt857Level::AmMicGain, 0x007B, 0x7F, 0},
  {YaesuFt857Level::DigGain, 0x007D, 0x7F, 0},    {YaesuFt857Level::DigVox, 0x0090, 0x7F, 0},
  {YaesuFt857Level::BpfWidth, 0x0093, 0x0C, 2},   {YaesuFt857Level::HpfCutoff, 0x0095, 0x0F, 0},
  {YaesuFt857Level::LpfCutoff, 0x0094, 0x1F, 0},  {YaesuFt857Level::NrLevel, 0x0093, 0xF0, 4},
  {YaesuFt857Level::FmMicGain, 0x007C, 0x7F, 0},  {YaesuFt857Level::NbLevel, 0x0099, 0x7F, 0},
  {YaesuFt857Level::Pkt1200, 0x007E, 0x7F, 0},    {YaesuFt857Level::Pkt9600, 0x007F, 0x7F, 0},
  {YaesuFt857Level::ProcLevel, 0x009A, 0x7F, 0},  {YaesuFt857Level::SsbMicGain, 0x007A, 0x7F, 0},
  {YaesuFt857Level::VoxDelay, 0x0077, 0xFF, 0},   {YaesuFt857Level::VoxGain, 0x0076, 0x7F, 0},
};

static const Ft857LevelField* ft857LevelField(YaesuFt857Level level) {
  for (const Ft857LevelField& f : kFt857LevelFields) {
    if (f.level == level) return &f;
  }
  return nullptr;
}

bool ft857LevelAddr(YaesuFt857Level level, uint16_t& addrOut) {
  const Ft857LevelField* f = ft857LevelField(level);
  if (!f) return false;
  addrOut = f->addr;
  return true;
}

uint16_t ft857LevelValue(YaesuFt857Level level, uint8_t b) {
  const Ft857LevelField* f = ft857LevelField(level);
  if (!f) return 0;
  const uint16_t raw = (uint16_t)((b & f->mask) >> f->shift);
  switch (level) {
    case YaesuFt857Level::BpfWidth: return (uint16_t)(60 << raw);        // 60 / 120 / 240 Hz
    case YaesuFt857Level::HpfCutoff: return (uint16_t)(100 + 60 * raw);  // 100..1000 Hz
    // 1000..6000 Hz in 32 steps; measured at the ends and at 11 = 2770 Hz (12 = 2940 Hz on
    // the radio), the radio's other steps may differ from this straight line by a few Hz.
    case YaesuFt857Level::LpfCutoff: return (uint16_t)(((1000 + raw * 5000 / 31) + 5) / 10 * 10);
    case YaesuFt857Level::NrLevel: return (uint16_t)(raw + 1);           // 1..16
    case YaesuFt857Level::VoxDelay: return (uint16_t)(raw * 100);        // 100..3000 ms
    default: return raw;                                                 // 0..100
  }
}

bool ft857LevelRange(YaesuFt857Level level, uint16_t& minOut, uint16_t& maxOut) {
  switch (level) {
    case YaesuFt857Level::NrLevel: minOut = 1; maxOut = 16; return true;
    case YaesuFt857Level::BpfWidth: minOut = 60; maxOut = 240; return true;
    case YaesuFt857Level::NbLevel:
    case YaesuFt857Level::ProcLevel:
    case YaesuFt857Level::SsbMicGain:
    case YaesuFt857Level::AmMicGain:
    case YaesuFt857Level::FmMicGain:
    case YaesuFt857Level::DigGain: minOut = 0; maxOut = 100; return true;
    case YaesuFt857Level::HpfCutoff: minOut = 100; maxOut = 1000; return true;
    case YaesuFt857Level::LpfCutoff: minOut = 1000; maxOut = 6000; return true;
    case YaesuFt857Level::VoxGain: minOut = 1; maxOut = 100; return true;
    case YaesuFt857Level::VoxDelay: minOut = 100; maxOut = 3000; return true;
    default: return false;
  }
}

// The stored field (before ft857LevelValue's scaling) whose value is nearest value, among those
// within the range; the lower one on a tie.
static uint8_t ft857NearestRaw(const Ft857LevelField& f, uint16_t value, uint16_t lo, uint16_t hi) {
  const uint8_t rawMax = (uint8_t)(f.mask >> f.shift);
  uint8_t best = 0;
  int bestDistance = -1;
  for (int raw = 0; raw <= rawMax; ++raw) {
    const uint16_t v = ft857LevelValue(f.level, (uint8_t)(raw << f.shift));
    if (v < lo || v > hi) continue;
    const int distance = v > value ? v - value : value - v;
    if (bestDistance >= 0 && distance >= bestDistance) continue;
    best = (uint8_t)raw;
    bestDistance = distance;
  }
  return best;
}

bool ft857LevelByte(YaesuFt857Level level, uint16_t value, uint8_t old, uint8_t& out) {
  const Ft857LevelField* f = ft857LevelField(level);
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!f || !ft857LevelRange(level, lo, hi)) return false;
  if (value < lo || value > hi) return false;
  const uint16_t stored = ft857LevelValue(level, old);
  if (stored < lo || stored > hi) return false;
  const uint8_t raw = ft857NearestRaw(*f, value, lo, hi);
  out = (uint8_t)((old & ~f->mask) | ((raw << f->shift) & f->mask));
  return true;
}

uint16_t ft857LevelStep(YaesuFt857Level level, uint16_t value, int steps) {
  const Ft857LevelField* f = ft857LevelField(level);
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!f || !ft857LevelRange(level, lo, hi)) return value;
  // More steps than the field has go to its end; the bound also keeps the sum from overflowing.
  const int rawMax = f->mask >> f->shift;
  if (steps > rawMax) steps = rawMax;
  if (steps < -rawMax) steps = -rawMax;
  int raw = ft857NearestRaw(*f, value, lo, hi) + steps;
  if (raw < 0) raw = 0;
  if (raw > rawMax) raw = rawMax;
  uint16_t stepped = ft857LevelValue(level, (uint8_t)(raw << f->shift));
  if (stepped < lo) stepped = lo;
  if (stepped > hi) stepped = hi;
  return stepped;
}

bool ft8x7MicGainForMode(uint8_t modeByte, Ft8x7MicGain& out) {
  switch (modeByte & 0x7F) {
    case 0x00:
    case 0x01: out = Ft8x7MicGain::Ssb; return true;
    case 0x04: out = Ft8x7MicGain::Am; return true;
    case 0x08: out = Ft8x7MicGain::Fm; return true;
    case 0x0A: out = Ft8x7MicGain::Dig; return true;
    case 0x0C: out = Ft8x7MicGain::Pkt; return true;
    default: return false;
  }
}

bool ft857MicGainLevel(Ft8x7MicGain gain, YaesuFt857Level& out) {
  switch (gain) {
    case Ft8x7MicGain::Ssb: out = YaesuFt857Level::SsbMicGain; return true;
    case Ft8x7MicGain::Am: out = YaesuFt857Level::AmMicGain; return true;
    case Ft8x7MicGain::Fm: out = YaesuFt857Level::FmMicGain; return true;
    case Ft8x7MicGain::Dig: out = YaesuFt857Level::DigGain; return true;
    default: return false;
  }
}

uint16_t ft817RfPowerTenths(uint8_t b, bool ft818) {
  static constexpr uint16_t kFt817Tenths[] = {50, 25, 10, 5};
  static constexpr uint16_t kFt818Tenths[] = {60, 50, 25, 10};
  return (ft818 ? kFt818Tenths : kFt817Tenths)[b & 0x03];
}

Ft817PowerLevel ft817PowerLevel(uint8_t b) { return (Ft817PowerLevel)(b & 0x03); }

uint8_t ft817PowerLevelByte(Ft817PowerLevel level, uint8_t old) {
  return withLowBits(old, (uint8_t)level);
}

YaesuAgc ft817AgcFromByte(uint8_t b) {
  static constexpr YaesuAgc kAgc[] = {YaesuAgc::Auto, YaesuAgc::Fast, YaesuAgc::Slow, YaesuAgc::Off};
  return kAgc[b & 0x03];
}

uint8_t ft817AgcByte(YaesuAgc agc, uint8_t old) {
  uint8_t bits = 0;
  switch (agc) {
    case YaesuAgc::Auto: bits = 0x00; break;
    case YaesuAgc::Fast: bits = 0x01; break;
    case YaesuAgc::Slow: bits = 0x02; break;
    case YaesuAgc::Off: bits = 0x03; break;
  }
  return withLowBits(old, bits);
}

bool ft8x7MenuRowMap(Ft8x7Model model, Ft8x7MenuRowMap& out) {
  if (model == Ft8x7Model::Ft857) {
    out = {0x0088, 0xFF, 0xFF, 17, 88, 91, 0x0091, 0x80};
    return true;
  }
  if (!ft8x7IsFt817Family(model)) return false;
  out = {0x0075, 0x3F, 0x0F, 12, 52, 57, 0x006B, 0x80};
  return true;
}

uint8_t ft8x7MenuFromPair(const Ft8x7MenuRowMap& map, const uint8_t pair[2]) {
  return (uint8_t)((pair[0] & map.menuMask) + 1);
}

uint8_t ft8x7RowFromPair(const Ft8x7MenuRowMap& map, const uint8_t pair[2]) {
  return (uint8_t)((pair[1] & map.rowMask) + 1);
}

uint8_t ft8x7MenuRowLast(const Ft8x7MenuRowMap& map, bool row, bool extOn) {
  if (row) return map.rows;
  return extOn ? map.extMenus : map.menus;
}

bool ft8x7MenuRowPair(const Ft8x7MenuRowMap& map, bool row, uint8_t number, bool extOn,
                      const uint8_t old[2], uint8_t out[2]) {
  // The fields as stored, from 0: adding 1 would wrap a whole-byte 0xFF to 0.
  if ((old[0] & map.menuMask) >= map.extMenus || (old[1] & map.rowMask) >= map.rows) return false;
  if (number < 1 || number > ft8x7MenuRowLast(map, row, extOn)) return false;
  const uint8_t i = row ? 1 : 0;
  const uint8_t mask = row ? map.rowMask : map.menuMask;
  out[0] = old[0];
  out[1] = old[1];
  out[i] = (uint8_t)((old[i] & ~mask) | ((number - 1) & mask));
  return true;
}

uint16_t ft817MicGainAddr(Ft8x7MicGain gain, bool pkt9600) {
  switch (gain) {
    case Ft8x7MicGain::Ssb: return 0x0067;
    case Ft8x7MicGain::Am: return 0x0068;
    case Ft8x7MicGain::Fm: return 0x0069;
    case Ft8x7MicGain::Dig: return 0x006A;
    case Ft8x7MicGain::Pkt: return pkt9600 ? 0x006C : 0x006B;
  }
  return 0x0067;
}

uint8_t ft817LevelFromByte(uint8_t b) {
  const uint8_t value = (uint8_t)(b & 0x7F);
  return value > 100 ? 100 : value;
}

uint8_t percentStep(uint8_t value, int steps) {
  if (steps > 100) steps = 100;
  if (steps < -100) steps = -100;
  const int stepped = (value > 100 ? 100 : value) + steps;
  return (uint8_t)(stepped < 0 ? 0 : stepped > 100 ? 100 : stepped);
}

bool ft817LevelByte(uint8_t value, uint8_t old, uint8_t& out) {
  if (value > 100 || (old & 0x7F) > 100) return false;
  out = (uint8_t)((old & 0x80) | value);
  return true;
}

uint16_t ft817VoxDelayMs(uint8_t b) {
  const uint8_t steps = (uint8_t)(b & 0x1F);
  if (steps < 1) return 100;
  return (uint16_t)((steps > 25 ? 25 : steps) * 100);
}

bool ft817VoxGainByte(uint8_t value, uint8_t old, uint8_t& out) {
  const uint8_t stored = (uint8_t)(old & 0x7F);
  if (value < 1 || stored < 1) return false;
  return ft817LevelByte(value, old, out);
}

bool ft817VoxDelayByte(uint16_t ms, uint8_t old, uint8_t& out) {
  const uint8_t stored = (uint8_t)(old & 0x1F);
  if (ms < 100 || ms > 2500 || stored < 1 || stored > 25) return false;
  out = (uint8_t)((old & 0xE0) | ((ms + 50) / 100));
  return true;
}

uint16_t ft817VoxDelayStep(uint16_t ms, int steps) {
  if (steps > 25) steps = 25;
  if (steps < -25) steps = -25;
  const int stepped = (ms + 50) / 100 + steps;
  return (uint16_t)((stepped < 1 ? 1 : stepped > 25 ? 25 : stepped) * 100);
}

uint8_t ft817RearAntennaMask(Ft8x7BandGroup group) {
  switch (group) {
    case Ft8x7BandGroup::Hf: return 0x01;
    case Ft8x7BandGroup::SixM: return 0x02;
    case Ft8x7BandGroup::FmBroadcast: return 0x04;
    case Ft8x7BandGroup::Air: return 0x08;
    case Ft8x7BandGroup::Vhf: return 0x10;
    case Ft8x7BandGroup::Uhf: return 0x20;
  }
  return 0x01;
}

uint8_t ft817RearAntennaByte(Ft8x7BandGroup group, bool rear, uint8_t old) {
  const uint8_t mask = ft817RearAntennaMask(group);
  return rear ? (uint8_t)(old | mask) : (uint8_t)(old & ~mask);
}

// A CW value: the stored field is (pair[0] & mask) >> shift, plus 256 when highMask is set in
// pair[1]; it holds rawMin..rawMax, and the value is base + step * field. fullRaw, when not 0,
// is the field that means FULL (value 0).
struct Ft8x7CwField {
  Ft8x7CwLevel level;
  uint16_t addr;
  uint8_t mask;
  uint8_t shift;
  uint8_t highMask;
  uint16_t rawMin;
  uint16_t rawMax;
  uint16_t base;
  uint16_t step;
  uint16_t fullRaw;
};

static constexpr Ft8x7CwField kFt857CwFields[] = {
  {Ft8x7CwLevel::Speed, 0x0075, 0x3F, 0, 0, 0, 56, 4, 1, 0},
  {Ft8x7CwLevel::Pitch, 0x0071, 0x07, 0, 0, 0, 4, 400, 100, 0},
  {Ft8x7CwLevel::Delay, 0x0073, 0xFF, 0, 0x80, 2, 300, 0, 10, 2},
  {Ft8x7CwLevel::Weight, 0x0072, 0x1F, 0, 0, 0, 20, 25, 1, 0},
  {Ft8x7CwLevel::Sidetone, 0x0074, 0x7F, 0, 0, 0, 100, 0, 1, 0},
  {Ft8x7CwLevel::Qsk, 0x0092, 0x1C, 2, 0, 0, 4, 10, 5, 0},
};

static constexpr Ft8x7CwField kFt817CwFields[] = {
  {Ft8x7CwLevel::Speed, 0x0062, 0x3F, 0, 0, 0, 56, 4, 1, 0},
  {Ft8x7CwLevel::Pitch, 0x005E, 0x0F, 0, 0, 0, 14, 300, 50, 0},
  {Ft8x7CwLevel::Delay, 0x0060, 0xFF, 0, 0, 1, 250, 0, 10, 0},
  {Ft8x7CwLevel::Weight, 0x005F, 0x1F, 0, 0, 0, 20, 25, 1, 0},
  {Ft8x7CwLevel::Sidetone, 0x0061, 0x7F, 0, 0, 0, 100, 0, 1, 0},
};

template <size_t N>
static const Ft8x7CwField* findCwField(const Ft8x7CwField (&fields)[N], Ft8x7CwLevel level) {
  for (const Ft8x7CwField& f : fields) {
    if (f.level == level) return &f;
  }
  return nullptr;
}

static const Ft8x7CwField* ft8x7CwField(Ft8x7Model model, Ft8x7CwLevel level) {
  if (ft8x7IsFt817Family(model)) return findCwField(kFt817CwFields, level);
  if (model == Ft8x7Model::Ft857) return findCwField(kFt857CwFields, level);
  return nullptr;
}

static uint16_t cwRaw(const Ft8x7CwField& f, const uint8_t pair[2]) {
  uint16_t raw = (uint16_t)((pair[0] & f.mask) >> f.shift);
  if (f.highMask && (pair[1] & f.highMask)) raw = (uint16_t)(raw + 256);
  return raw;
}

static uint16_t cwRawValue(const Ft8x7CwField& f, uint16_t raw) {
  if (f.fullRaw && raw == f.fullRaw) return 0;
  return (uint16_t)(f.base + f.step * raw);
}

// The field whose value is nearest value; the lower one on a tie.
static uint16_t cwNearestRaw(const Ft8x7CwField& f, uint16_t value) {
  uint16_t best = f.rawMin;
  int bestDistance = -1;
  for (int raw = f.rawMin; raw <= f.rawMax; ++raw) {
    const uint16_t v = cwRawValue(f, (uint16_t)raw);
    const int distance = v > value ? v - value : value - v;
    if (bestDistance >= 0 && distance >= bestDistance) continue;
    best = (uint16_t)raw;
    bestDistance = distance;
  }
  return best;
}

bool ft8x7CwLevelAddr(Ft8x7Model model, Ft8x7CwLevel level, uint16_t& addrOut) {
  const Ft8x7CwField* f = ft8x7CwField(model, level);
  if (!f) return false;
  addrOut = f->addr;
  return true;
}

// The field stored in pair; false when it is not one the radio offers.
static bool cwStoredRaw(const Ft8x7CwField& f, const uint8_t pair[2], uint16_t& rawOut) {
  const uint16_t raw = cwRaw(f, pair);
  if (raw < f.rawMin || raw > f.rawMax) return false;
  rawOut = raw;
  return true;
}

bool ft8x7CwLevelValue(Ft8x7Model model, Ft8x7CwLevel level, const uint8_t pair[2], uint16_t& valueOut) {
  const Ft8x7CwField* f = ft8x7CwField(model, level);
  uint16_t raw = 0;
  if (!f || !cwStoredRaw(*f, pair, raw)) return false;
  valueOut = cwRawValue(*f, raw);
  return true;
}

bool ft8x7CwLevelRange(Ft8x7Model model, Ft8x7CwLevel level, uint16_t& minOut, uint16_t& maxOut) {
  const Ft8x7CwField* f = ft8x7CwField(model, level);
  if (!f) return false;
  // FULL is the lowest field and not a number of ms.
  minOut = cwRawValue(*f, f->fullRaw ? (uint16_t)(f->rawMin + 1) : f->rawMin);
  maxOut = cwRawValue(*f, f->rawMax);
  return true;
}

bool ft8x7CwDelayHasFull(Ft8x7Model model) {
  const Ft8x7CwField* f = ft8x7CwField(model, Ft8x7CwLevel::Delay);
  return f && f->fullRaw;
}

bool ft8x7CwLevelAccepts(Ft8x7Model model, Ft8x7CwLevel level, uint16_t value) {
  const Ft8x7CwField* f = ft8x7CwField(model, level);
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!f || !ft8x7CwLevelRange(model, level, lo, hi)) return false;
  return (value == 0 && f->fullRaw) || (value >= lo && value <= hi);
}

bool ft8x7CwLevelPair(Ft8x7Model model, Ft8x7CwLevel level, uint16_t value, const uint8_t old[2],
                      uint8_t out[2]) {
  const Ft8x7CwField* f = ft8x7CwField(model, level);
  uint16_t stored = 0;
  if (!f || !ft8x7CwLevelAccepts(model, level, value) || !cwStoredRaw(*f, old, stored)) return false;
  // FULL (0) is nearest its own field.
  const uint16_t raw = cwNearestRaw(*f, value);
  out[0] = (uint8_t)((old[0] & ~f->mask) | ((raw << f->shift) & f->mask));
  out[1] = old[1];
  if (f->highMask) out[1] = (uint8_t)(raw >= 256 ? (old[1] | f->highMask) : (old[1] & ~f->highMask));
  return true;
}

uint16_t ft8x7CwLevelStep(Ft8x7Model model, Ft8x7CwLevel level, uint16_t value, int steps) {
  const Ft8x7CwField* f = ft8x7CwField(model, level);
  if (!f) return value;
  // More steps than the field has go to its end; the bound also keeps the sum from overflowing.
  const int span = f->rawMax - f->rawMin;
  if (steps > span) steps = span;
  if (steps < -span) steps = -span;
  int raw = cwNearestRaw(*f, value) + steps;
  if (raw < f->rawMin) raw = f->rawMin;
  if (raw > f->rawMax) raw = f->rawMax;
  return cwRawValue(*f, (uint16_t)raw);
}

bool ft857CwBfoFromByte(uint8_t b, Ft857CwBfo& out) {
  if ((b & 0x03) == 0x03) return false;
  out = (Ft857CwBfo)(b & 0x03);
  return true;
}

uint8_t ft857CwBfoByte(Ft857CwBfo bfo, uint8_t old) {
  return withLowBits(old, (uint8_t)bfo);
}
