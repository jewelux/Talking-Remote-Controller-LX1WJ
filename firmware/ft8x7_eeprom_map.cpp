#include "ft8x7_eeprom_map.h"

Ft8x7BandGroup ft8x7BandGroupForHz(uint64_t hz) {
  if (hz >= 420000000ULL) return Ft8x7BandGroup::Uhf;
  if (hz >= 137000000ULL) return Ft8x7BandGroup::Vhf;
  if (hz >= 108000000ULL) return Ft8x7BandGroup::Air;
  if (hz >= 76000000ULL) return Ft8x7BandGroup::FmBroadcast;
  if (hz >= 33000000ULL) return Ft8x7BandGroup::SixM;
  return Ft8x7BandGroup::Hf;
}

static constexpr uint16_t kFt857VfoBBlockOffset = 0x01C0;

static constexpr Ft857BandSlot kFt857BandSlots[] = {
  {1800, 2000, 0x00BA, true},     {3500, 4000, 0x00D6, true},     {5250, 5450, 0x00F2, true},
  {7000, 7300, 0x010E, true},     {10100, 10150, 0x012A, true},   {14000, 14350, 0x0146, true},
  {18068, 18168, 0x0162, true},   {21000, 21450, 0x017E, true},   {24890, 24990, 0x019A, true},
  {28000, 29700, 0x01B6, true},   {50000, 54000, 0x01D2, true},   {144000, 148000, 0x0226, false},
  {430000, 450000, 0x0242, false},
};

const Ft857BandSlot* ft857BandSlotForHz(uint64_t hz) {
  const uint32_t khz = (uint32_t)(hz / 1000ULL);
  for (const Ft857BandSlot& slot : kFt857BandSlots) {
    if (khz >= slot.lowKhz && khz <= slot.highKhz) return &slot;
  }
  return nullptr;
}

uint16_t ft857BandBlock(const Ft857BandSlot& slot, bool vfoB) {
  return vfoB ? (uint16_t)(slot.block + kFt857VfoBBlockOffset) : slot.block;
}

int32_t ft857RitOffsetHz(const uint8_t word[2]) {
  return (int32_t)(int16_t)(((uint16_t)word[0] << 8) | word[1]) * 10;
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

YaesuAgc ft857AgcFromSpeed(uint8_t b) {
  switch (b & 0x03) {
    case 0x00: return YaesuAgc::Slow;
    case 0x02: return YaesuAgc::Fast;
    default: return YaesuAgc::Auto;
  }
}

YaesuFt857MicEq ft857MicEqFromByte(uint8_t b) {
  return (YaesuFt857MicEq)(b & 0x03);
}

struct Ft857LevelField {
  YaesuFt857Level level;
  uint16_t addr;
  uint8_t mask;
  uint8_t shift;
};

static constexpr Ft857LevelField kFt857LevelFields[] = {
  {YaesuFt857Level::CwSpeed, 0x0075, 0x3F, 0},    {YaesuFt857Level::AmMicGain, 0x007B, 0x7F, 0},
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
    case YaesuFt857Level::CwSpeed: return (uint16_t)(raw + 4);           // 4..60 WPM
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

uint16_t ft817RfPowerTenths(uint8_t b, bool ft818) {
  static constexpr uint16_t kFt817Tenths[] = {50, 25, 10, 5};
  static constexpr uint16_t kFt818Tenths[] = {60, 50, 25, 10};
  return (ft818 ? kFt818Tenths : kFt817Tenths)[b & 0x03];
}

YaesuAgc ft817AgcFromByte(uint8_t b) {
  static constexpr YaesuAgc kAgc[] = {YaesuAgc::Auto, YaesuAgc::Fast, YaesuAgc::Slow, YaesuAgc::Off};
  return kAgc[b & 0x03];
}

uint8_t ft817MenuFromByte(uint8_t b) {
  return (uint8_t)((b & 0x3F) + 1);
}

uint8_t ft817RowFromByte(uint8_t b) {
  return (uint8_t)((b & 0x0F) + 1);
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
