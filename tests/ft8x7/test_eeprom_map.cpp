// Tests for the FT-8x7 EEPROM map (firmware/ft8x7_eeprom_map.*). The addresses and values were
// measured on an FT-817 and an FT-897; these tests keep them from changing by accident.
#include "ft8x7_eeprom_map.h"
#include "test_runner.h"

TEST(band_group_edges) {
  CHECK_EQ(ft8x7BandGroupForHz(1800000ULL), Ft8x7BandGroup::Hf);
  CHECK_EQ(ft8x7BandGroupForHz(32999999ULL), Ft8x7BandGroup::Hf);
  CHECK_EQ(ft8x7BandGroupForHz(33000000ULL), Ft8x7BandGroup::SixM);
  CHECK_EQ(ft8x7BandGroupForHz(75999999ULL), Ft8x7BandGroup::SixM);
  CHECK_EQ(ft8x7BandGroupForHz(76000000ULL), Ft8x7BandGroup::FmBroadcast);
  CHECK_EQ(ft8x7BandGroupForHz(108000000ULL), Ft8x7BandGroup::Air);
  CHECK_EQ(ft8x7BandGroupForHz(136999999ULL), Ft8x7BandGroup::Air);
  CHECK_EQ(ft8x7BandGroupForHz(137000000ULL), Ft8x7BandGroup::Vhf);
  CHECK_EQ(ft8x7BandGroupForHz(419999999ULL), Ft8x7BandGroup::Vhf);
  CHECK_EQ(ft8x7BandGroupForHz(420000000ULL), Ft8x7BandGroup::Uhf);
}

TEST(ft857_band_slot_for_each_band) {
  struct Case {
    uint64_t hz;
    uint16_t block;
    bool hasIpoAtt;
  };
  const Case cases[] = {
    {1840000ULL, 0x00BA, true},    {3700000ULL, 0x00D6, true},    {5357000ULL, 0x00F2, true},
    {7100000ULL, 0x010E, true},    {10125000ULL, 0x012A, true},   {14250000ULL, 0x0146, true},
    {18100000ULL, 0x0162, true},   {21300000ULL, 0x017E, true},   {24940000ULL, 0x019A, true},
    {28500000ULL, 0x01B6, true},   {50313000ULL, 0x01D2, true},   {145500000ULL, 0x0226, false},
    {435000000ULL, 0x0242, false},
  };
  for (const Case &c : cases) {
    const Ft857BandSlot *slot = ft857BandSlotForHz(c.hz);
    CHECK(slot != nullptr);
    if (!slot) continue;
    CHECK_EQ(slot->block, c.block);
    CHECK_EQ(slot->hasIpoAtt, c.hasIpoAtt);
  }
}

TEST(ft857_band_slot_edges_are_inclusive_khz) {
  CHECK(ft857BandSlotForHz(1799999ULL) == nullptr);
  CHECK(ft857BandSlotForHz(1800000ULL) != nullptr);
  CHECK(ft857BandSlotForHz(2000999ULL) != nullptr);
  CHECK(ft857BandSlotForHz(2001000ULL) == nullptr);
}

TEST(ft857_band_slot_is_null_outside_the_amateur_bands) {
  CHECK(ft857BandSlotForHz(100000ULL) == nullptr);
  CHECK(ft857BandSlotForHz(9000000ULL) == nullptr);
  CHECK(ft857BandSlotForHz(100000000ULL) == nullptr);
  CHECK(ft857BandSlotForHz(460000000ULL) == nullptr);
}

TEST(ft857_vfo_b_block_follows_vfo_a) {
  const Ft857BandSlot *slot = ft857BandSlotForHz(14250000ULL);
  CHECK(slot != nullptr);
  if (!slot) return;
  CHECK_EQ(ft857BandBlock(*slot, false), 0x0146);
  CHECK_EQ(ft857BandBlock(*slot, true), 0x0306);
}

TEST(ft857_rit_offset_is_signed_10_hz_units) {
  const uint8_t minus310[2] = {0xFF, 0xE1};
  const uint8_t plus1000[2] = {0x00, 0x64};
  const uint8_t zero[2] = {0x00, 0x00};
  CHECK_EQ(ft857RitOffsetHz(minus310), -310);
  CHECK_EQ(ft857RitOffsetHz(plus1000), 1000);
  CHECK_EQ(ft857RitOffsetHz(zero), 0);
}

TEST(ft857_rf_power_address_per_band_group) {
  CHECK_EQ(ft857RfPowerAddr(Ft8x7BandGroup::Hf), 0x009B);
  CHECK_EQ(ft857RfPowerAddr(Ft8x7BandGroup::SixM), 0x00AA);
  CHECK_EQ(ft857RfPowerAddr(Ft8x7BandGroup::FmBroadcast), 0x00AB);
  CHECK_EQ(ft857RfPowerAddr(Ft8x7BandGroup::Air), 0x00AB);
  CHECK_EQ(ft857RfPowerAddr(Ft8x7BandGroup::Vhf), 0x00AB);
  CHECK_EQ(ft857RfPowerAddr(Ft8x7BandGroup::Uhf), 0x00AC);
}

TEST(ft857_rf_power_drops_bit_7) {
  CHECK_EQ(ft857RfPowerWatts(0xE4), 100);
  CHECK_EQ(ft857RfPowerWatts(0x94), 20);
  CHECK_EQ(ft857RfPowerWatts(0x05), 5);
}

TEST(ft857_agc_speed) {
  CHECK_EQ(ft857AgcFromSpeed(0x00), YaesuAgc::Slow);
  CHECK_EQ(ft857AgcFromSpeed(0x01), YaesuAgc::Auto);
  CHECK_EQ(ft857AgcFromSpeed(0x02), YaesuAgc::Fast);
  CHECK_EQ(ft857AgcFromSpeed(0x03), YaesuAgc::Auto);
  CHECK_EQ(ft857AgcFromSpeed(0xFC), YaesuAgc::Slow);
}

TEST(ft857_mic_eq) {
  CHECK_EQ(ft857MicEqFromByte(0x00), YaesuFt857MicEq::Off);
  CHECK_EQ(ft857MicEqFromByte(0xF1), YaesuFt857MicEq::Lpf);
  CHECK_EQ(ft857MicEqFromByte(0x02), YaesuFt857MicEq::Hpf);
  CHECK_EQ(ft857MicEqFromByte(0x03), YaesuFt857MicEq::Both);
}

TEST(ft857_every_level_has_an_address) {
  for (uint8_t i = 0; i <= (uint8_t)YaesuFt857Level::VoxGain; ++i) {
    uint16_t addr = 0;
    CHECK(ft857LevelAddr((YaesuFt857Level)i, addr));
    CHECK(addr >= 0x0075 && addr <= 0x009A);
  }
}

TEST(ft857_level_addresses) {
  uint16_t addr = 0;
  CHECK(ft857LevelAddr(YaesuFt857Level::CwSpeed, addr));
  CHECK_EQ(addr, 0x0075);
  CHECK(ft857LevelAddr(YaesuFt857Level::NrLevel, addr));
  CHECK_EQ(addr, 0x0093);
  CHECK(ft857LevelAddr(YaesuFt857Level::LpfCutoff, addr));
  CHECK_EQ(addr, 0x0094);
  CHECK(ft857LevelAddr(YaesuFt857Level::VoxGain, addr));
  CHECK_EQ(addr, 0x0076);
}

TEST(ft857_level_scaling) {
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::CwSpeed, 0x00), 4);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::CwSpeed, 0x38), 60);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::CwSpeed, 0xC8), 12);  // bits 7..6 are not the speed
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::HpfCutoff, 0x00), 100);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::HpfCutoff, 0x0F), 1000);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::LpfCutoff, 0x00), 1000);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::LpfCutoff, 0x0B), 2770);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::LpfCutoff, 0x1F), 6000);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::NbLevel, 0x64), 100);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::VoxDelay, 0x01), 100);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::VoxDelay, 0x1E), 3000);
}

// 0x93 holds the NR level (bits 7..4), the BPF width (3..2) and the mic EQ (1..0).
TEST(ft857_byte_0x93_splits_into_three_settings) {
  const uint8_t b = 0xFB;
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::NrLevel, b), 16);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::BpfWidth, b), 240);
  CHECK_EQ(ft857MicEqFromByte(b), YaesuFt857MicEq::Both);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::NrLevel, 0x00), 1);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::BpfWidth, 0x00), 60);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::BpfWidth, 0x04), 120);
}

TEST(mic_gain_follows_the_mode) {
  Ft8x7MicGain gain = Ft8x7MicGain::Pkt;
  CHECK(ft8x7MicGainForMode(0x00, gain));  // LSB
  CHECK_EQ(gain, Ft8x7MicGain::Ssb);
  CHECK(ft8x7MicGainForMode(0x01, gain));  // USB
  CHECK_EQ(gain, Ft8x7MicGain::Ssb);
  CHECK(ft8x7MicGainForMode(0x84, gain));  // AM narrow
  CHECK_EQ(gain, Ft8x7MicGain::Am);
  CHECK(ft8x7MicGainForMode(0x88, gain));  // FM narrow
  CHECK_EQ(gain, Ft8x7MicGain::Fm);
  CHECK(ft8x7MicGainForMode(0x0A, gain));  // DIG
  CHECK_EQ(gain, Ft8x7MicGain::Dig);
  CHECK(ft8x7MicGainForMode(0x0C, gain));  // PKT
  CHECK_EQ(gain, Ft8x7MicGain::Pkt);
  CHECK(!ft8x7MicGainForMode(0x02, gain));  // CW
  CHECK(!ft8x7MicGainForMode(0x03, gain));  // CW-R
  CHECK(!ft8x7MicGainForMode(0x06, gain));  // WFM
}

TEST(ft857_mic_gain_menus) {
  YaesuFt857Level level = YaesuFt857Level::CwSpeed;
  CHECK(ft857MicGainLevel(Ft8x7MicGain::Ssb, level));
  CHECK_EQ(level, YaesuFt857Level::SsbMicGain);
  CHECK(ft857MicGainLevel(Ft8x7MicGain::Am, level));
  CHECK_EQ(level, YaesuFt857Level::AmMicGain);
  CHECK(ft857MicGainLevel(Ft8x7MicGain::Fm, level));
  CHECK_EQ(level, YaesuFt857Level::FmMicGain);
  CHECK(ft857MicGainLevel(Ft8x7MicGain::Dig, level));
  CHECK_EQ(level, YaesuFt857Level::DigGain);
  CHECK(!ft857MicGainLevel(Ft8x7MicGain::Pkt, level));
}

TEST(ft817_mic_gain_addresses) {
  CHECK_EQ(ft817MicGainAddr(Ft8x7MicGain::Ssb, false), 0x0067);
  CHECK_EQ(ft817MicGainAddr(Ft8x7MicGain::Am, false), 0x0068);
  CHECK_EQ(ft817MicGainAddr(Ft8x7MicGain::Fm, false), 0x0069);
  CHECK_EQ(ft817MicGainAddr(Ft8x7MicGain::Dig, false), 0x006A);
  CHECK_EQ(ft817MicGainAddr(Ft8x7MicGain::Pkt, false), 0x006B);
  CHECK_EQ(ft817MicGainAddr(Ft8x7MicGain::Pkt, true), 0x006C);
  CHECK_EQ(ft817MicGainAddr(Ft8x7MicGain::Ssb, true), 0x0067);  // the rate only picks the PKT one
  CHECK_EQ(ft817MicGainFromByte(0xE4), 100);  // bit 7 is another setting (0x68: mic key)
  CHECK_EQ(ft817MicGainFromByte(0x32), 50);
}

TEST(ft817_rf_power_tenths) {
  CHECK_EQ(ft817RfPowerTenths(0x00, false), 50);
  CHECK_EQ(ft817RfPowerTenths(0x01, false), 25);
  CHECK_EQ(ft817RfPowerTenths(0x02, false), 10);
  CHECK_EQ(ft817RfPowerTenths(0x03, false), 5);
  CHECK_EQ(ft817RfPowerTenths(0xFC, false), 50);
}

TEST(ft818_rf_power_tenths) {
  CHECK_EQ(ft817RfPowerTenths(0x00, true), 60);
  CHECK_EQ(ft817RfPowerTenths(0x01, true), 50);
  CHECK_EQ(ft817RfPowerTenths(0x02, true), 25);
  CHECK_EQ(ft817RfPowerTenths(0x03, true), 10);
}

TEST(ft817_agc) {
  CHECK_EQ(ft817AgcFromByte(0x00), YaesuAgc::Auto);
  CHECK_EQ(ft817AgcFromByte(0x01), YaesuAgc::Fast);
  CHECK_EQ(ft817AgcFromByte(0x02), YaesuAgc::Slow);
  CHECK_EQ(ft817AgcFromByte(0xF3), YaesuAgc::Off);
}

TEST(ft817_menu_and_row_count_from_1) {
  CHECK_EQ(ft817MenuFromByte(0x00), 1);
  CHECK_EQ(ft817MenuFromByte(0xFF), 64);
  CHECK_EQ(ft817RowFromByte(0x00), 1);
  CHECK_EQ(ft817RowFromByte(0xFF), 16);
}

TEST(ft817_rear_antenna_bit_per_band_group) {
  CHECK_EQ(ft817RearAntennaMask(Ft8x7BandGroup::Hf), 0x01);
  CHECK_EQ(ft817RearAntennaMask(Ft8x7BandGroup::SixM), 0x02);
  CHECK_EQ(ft817RearAntennaMask(Ft8x7BandGroup::FmBroadcast), 0x04);
  CHECK_EQ(ft817RearAntennaMask(Ft8x7BandGroup::Air), 0x08);
  CHECK_EQ(ft817RearAntennaMask(Ft8x7BandGroup::Vhf), 0x10);
  CHECK_EQ(ft817RearAntennaMask(Ft8x7BandGroup::Uhf), 0x20);
}

namespace {

struct FlagCase {
  Ft8x7Flag flag;
  uint16_t addr;
  uint8_t mask;
  bool inverted;
};

constexpr Ft8x7Flag kAllFlags[] = {
  Ft8x7Flag::Nb,     Ft8x7Flag::BreakIn, Ft8x7Flag::Keyer,   Ft8x7Flag::Vox,
  Ft8x7Flag::Proc,   Ft8x7Flag::Lock,    Ft8x7Flag::FastTuning, Ft8x7Flag::IfShift,
  Ft8x7Flag::Dnr,    Ft8x7Flag::Dnf,     Ft8x7Flag::Dbf,     Ft8x7Flag::AgcOn,
  Ft8x7Flag::DspRow, Ft8x7Flag::VfoB,    Ft8x7Flag::Filter2, Ft8x7Flag::KnobIsSquelch,
  Ft8x7Flag::Split,
};

void checkFlags(Ft8x7Model model, const FlagCase *cases, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    Ft8x7FlagField field{};
    CHECK(ft8x7FlagField(model, cases[i].flag, field));
    CHECK_EQ(field.addr, cases[i].addr);
    CHECK_EQ(field.mask, cases[i].mask);
    CHECK_EQ(field.inverted, cases[i].inverted);
  }
  // Every other flag is not kept.
  size_t kept = 0;
  for (Ft8x7Flag flag : kAllFlags) {
    Ft8x7FlagField field{};
    if (ft8x7FlagField(model, flag, field)) ++kept;
  }
  CHECK_EQ(kept, count);
}

// Two flags in one byte must not share a bit.
void checkNoSharedBits(Ft8x7Model model) {
  for (Ft8x7Flag a : kAllFlags) {
    for (Ft8x7Flag b : kAllFlags) {
      if (a == b) continue;
      Ft8x7FlagField fa{};
      Ft8x7FlagField fb{};
      if (!ft8x7FlagField(model, a, fa) || !ft8x7FlagField(model, b, fb)) continue;
      CHECK(fa.addr != fb.addr || (fa.mask & fb.mask) == 0);
    }
  }
}

}  // namespace

TEST(ft857_flag_map) {
  const FlagCase cases[] = {
    {Ft8x7Flag::VfoB, 0x0068, 0x01, false},       {Ft8x7Flag::IfShift, 0x006A, 0x10, false},
    {Ft8x7Flag::Nb, 0x006A, 0x20, false},         {Ft8x7Flag::Lock, 0x006A, 0x40, true},
    {Ft8x7Flag::FastTuning, 0x006A, 0x80, true},  {Ft8x7Flag::Keyer, 0x006B, 0x10, false},
    {Ft8x7Flag::BreakIn, 0x006B, 0x20, false},    {Ft8x7Flag::Vox, 0x006B, 0x80, false},
    {Ft8x7Flag::KnobIsSquelch, 0x0072, 0x80, false}, {Ft8x7Flag::Split, 0x008D, 0x80, false},
    {Ft8x7Flag::Filter2, 0x00A7, 0x80, false},    {Ft8x7Flag::Dnf, 0x00A8, 0x01, false},
    {Ft8x7Flag::Dnr, 0x00A8, 0x02, false},        {Ft8x7Flag::Dbf, 0x00A8, 0x0C, false},
    {Ft8x7Flag::AgcOn, 0x00A8, 0x20, false},      {Ft8x7Flag::DspRow, 0x00A8, 0x80, false},
    {Ft8x7Flag::Proc, 0x00A9, 0x02, false},
  };
  checkFlags(Ft8x7Model::Ft857, cases, sizeof(cases) / sizeof(cases[0]));
  checkNoSharedBits(Ft8x7Model::Ft857);
}

TEST(ft817_flag_map) {
  const FlagCase cases[] = {
    {Ft8x7Flag::IfShift, 0x0057, 0x10, false},   {Ft8x7Flag::Nb, 0x0057, 0x20, false},
    {Ft8x7Flag::Lock, 0x0057, 0x40, true},       {Ft8x7Flag::FastTuning, 0x0057, 0x80, true},
    {Ft8x7Flag::Keyer, 0x0058, 0x10, false},     {Ft8x7Flag::BreakIn, 0x0058, 0x20, false},
    {Ft8x7Flag::Vox, 0x0058, 0x80, false},       {Ft8x7Flag::Split, 0x007A, 0x80, false},
  };
  checkFlags(Ft8x7Model::Ft817, cases, sizeof(cases) / sizeof(cases[0]));
  checkFlags(Ft8x7Model::Ft818, cases, sizeof(cases) / sizeof(cases[0]));
  checkNoSharedBits(Ft8x7Model::Ft817);
}

TEST(ft817_split_does_not_share_the_antenna_bits) {
  Ft8x7FlagField split{};
  CHECK(ft8x7FlagField(Ft8x7Model::Ft817, Ft8x7Flag::Split, split));
  CHECK_EQ(split.addr, FT817_ANTENNA_ADDR);
  for (uint8_t g = 0; g <= (uint8_t)Ft8x7BandGroup::Uhf; ++g) {
    CHECK_EQ(split.mask & ft817RearAntennaMask((Ft8x7BandGroup)g), 0);
  }
}

TEST(no_flags_without_a_known_model) {
  for (Ft8x7Flag flag : kAllFlags) {
    Ft8x7FlagField field{};
    CHECK(!ft8x7FlagField(Ft8x7Model::None, flag, field));
  }
}

TEST(flag_value_reads_any_bit_of_the_mask) {
  const Ft8x7FlagField dbf{0x00A8, 0x0C, false};
  CHECK(!ft8x7FlagValue(dbf, 0x00));
  CHECK(ft8x7FlagValue(dbf, 0x04));
  CHECK(ft8x7FlagValue(dbf, 0x08));
  CHECK(!ft8x7FlagValue(dbf, 0xF3));
}

TEST(flag_value_of_an_inverted_bit) {
  const Ft8x7FlagField lock{0x006A, 0x40, true};
  CHECK(ft8x7FlagValue(lock, 0x00));
  CHECK(!ft8x7FlagValue(lock, 0x40));
  CHECK(ft8x7FlagValue(lock, 0xBF));
}
