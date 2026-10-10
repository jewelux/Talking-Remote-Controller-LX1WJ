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

// The bytes the FT-897 stored for each CAT tone mode, with bit 7 set as it was there.
TEST(ft857_tone_mode_from_the_band_block) {
  using T = YaesuToneType;
  struct Case {
    uint8_t b;
    T tx;
    T rx;
  };
  const Case cases[] = {
    {0x80, T::None, T::None},  {0x85, T::Ctcss, T::Ctcss}, {0x84, T::Ctcss, T::None},
    {0x81, T::None, T::Ctcss}, {0x8A, T::Dcs, T::Dcs},     {0x88, T::Dcs, T::None},
    {0x82, T::None, T::Dcs},   {0x06, T::Ctcss, T::Dcs},   {0x00, T::None, T::None},
  };
  for (const Case& c : cases) {
    const YaesuToneMode mode = ft857ToneModeFromByte(c.b);
    CHECK_EQ(mode.tx, c.tx);
    CHECK_EQ(mode.rx, c.rx);
  }
}

// The bytes the FT-897 stored with split tone (menu 79) on, set with its own A (encode) and B
// (decode) keys, and a mixed mode left after split tone was switched off.
TEST(ft857_split_tone_bytes_from_the_band_block) {
  using T = YaesuToneType;
  struct Case {
    uint8_t b;
    bool split;
    T tx;
    T rx;
  };
  const Case cases[] = {
    {0x90, true, T::None, T::None},   {0x94, true, T::Ctcss, T::None}, {0x98, true, T::Dcs, T::None},
    {0x91, true, T::None, T::Ctcss},  {0x99, true, T::Dcs, T::Ctcss},  {0x9A, true, T::Dcs, T::Dcs},
    {0x96, true, T::Ctcss, T::Dcs},   {0x86, false, T::Ctcss, T::Dcs}, {0x80, false, T::None, T::None},
  };
  for (const Case& c : cases) {
    CHECK_EQ((c.b & FT857_BAND_SPLIT_TONE_MASK) != 0, c.split);
    const YaesuToneMode mode = ft857ToneModeFromByte(c.b);
    CHECK_EQ(mode.tx, c.tx);
    CHECK_EQ(mode.rx, c.rx);
  }
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
  CHECK_EQ(ft817LevelFromByte(0xE4), 100);  // bit 7 is another setting (0x68: mic key)
  CHECK_EQ(ft817LevelFromByte(0x32), 50);
}

TEST(vox_gain) {
  CHECK_EQ(FT817_VOX_GAIN_ADDR, 0x0063);
  uint16_t addr = 0;
  CHECK(ft857LevelAddr(YaesuFt857Level::VoxGain, addr));
  CHECK_EQ(addr, 0x0076);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::VoxGain, 0xB2), 50);  // bit 7 is not the gain
}

TEST(vox_delay) {
  uint16_t addr = 0;
  CHECK(ft857LevelAddr(YaesuFt857Level::VoxDelay, addr));
  CHECK_EQ(addr, 0x0077);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::VoxDelay, 0x05), 500);
  CHECK_EQ(FT817_VOX_DELAY_ADDR, 0x0064);
  CHECK_EQ(ft817VoxDelayMs(0x01), 100);
  CHECK_EQ(ft817VoxDelayMs(0x19), 2500);
  CHECK_EQ(ft817VoxDelayMs(0x83), 300);  // bits 7..5 are other settings
  CHECK_EQ(ft817VoxDelayMs(0x00), 100);
  CHECK_EQ(ft817VoxDelayMs(0x1F), 2500);
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

TEST(menu_row_map_per_model) {
  Ft8x7MenuRowMap map{};
  CHECK(!ft8x7MenuRowMap(Ft8x7Model::None, map));
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft857, map));
  CHECK_EQ(map.addr, 0x0088);
  CHECK_EQ(map.extAddr, 0x0091);
  CHECK_EQ(map.extMask, 0x80);
  CHECK_EQ(ft8x7MenuRowLast(map, true, false), 17);
  CHECK_EQ(ft8x7MenuRowLast(map, true, true), 17);
  CHECK_EQ(ft8x7MenuRowLast(map, false, false), 88);
  CHECK_EQ(ft8x7MenuRowLast(map, false, true), 91);
  const Ft8x7Model ft817Family[] = {Ft8x7Model::Ft817, Ft8x7Model::Ft818};
  for (Ft8x7Model model : ft817Family) {
    CHECK(ft8x7MenuRowMap(model, map));
    CHECK_EQ(map.addr, 0x0075);
    CHECK_EQ(map.extAddr, 0x006B);
    CHECK_EQ(map.extMask, 0x80);
    CHECK_EQ(ft8x7MenuRowLast(map, true, true), 12);
    CHECK_EQ(ft8x7MenuRowLast(map, false, false), 52);
    CHECK_EQ(ft8x7MenuRowLast(map, false, true), 57);
  }
}

// Every value up to the last fits in the field, so writing it cannot reach another bit.
TEST(menu_row_last_fits_the_field) {
  const Ft8x7Model models[] = {Ft8x7Model::Ft817, Ft8x7Model::Ft857};
  for (Ft8x7Model model : models) {
    Ft8x7MenuRowMap map{};
    CHECK(ft8x7MenuRowMap(model, map));
    CHECK((uint8_t)(map.extMenus - 1) <= map.menuMask);
    CHECK((uint8_t)(map.rows - 1) <= map.rowMask);
    CHECK(map.menus <= map.extMenus);
  }
}

TEST(menu_and_row_from_pair_count_from_1) {
  Ft8x7MenuRowMap map{};
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft817, map));
  const uint8_t ft817Pair[2] = {0xF1, 0xF3};  // menu 50, row 4, foreign bits set
  CHECK_EQ(ft8x7MenuFromPair(map, ft817Pair), 50);
  CHECK_EQ(ft8x7RowFromPair(map, ft817Pair), 4);
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft857, map));
  const uint8_t ft897Pair[2] = {0x4E, 0x04};  // measured: menu 79, row 5
  CHECK_EQ(ft8x7MenuFromPair(map, ft897Pair), 79);
  CHECK_EQ(ft8x7RowFromPair(map, ft897Pair), 5);
}

TEST(ft817_menu_row_pair_keeps_the_other_bits) {
  Ft8x7MenuRowMap map{};
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft817, map));
  const uint8_t old[2] = {0xC5, 0xF3};  // menu 6, row 4, every foreign bit set
  uint8_t out[2] = {0, 0};
  CHECK(ft8x7MenuRowPair(map, false, 57, true, old, out));
  CHECK_EQ(out[0], 0xF8);
  CHECK_EQ(out[1], 0xF3);
  CHECK(ft8x7MenuRowPair(map, false, 1, false, old, out));
  CHECK_EQ(out[0], 0xC0);
  CHECK_EQ(out[1], 0xF3);
  CHECK(ft8x7MenuRowPair(map, true, 12, false, old, out));
  CHECK_EQ(out[0], 0xC5);
  CHECK_EQ(out[1], 0xFB);
  CHECK(ft8x7MenuRowPair(map, true, 1, false, old, out));
  CHECK_EQ(out[0], 0xC5);
  CHECK_EQ(out[1], 0xF0);
  const uint8_t clear[2] = {0x31, 0x03};
  CHECK(ft8x7MenuRowPair(map, true, 8, false, clear, out));
  CHECK_EQ(out[0], 0x31);
  CHECK_EQ(out[1], 0x07);
}

TEST(ft857_menu_row_pair_changes_one_byte) {
  Ft8x7MenuRowMap map{};
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft857, map));
  const uint8_t old[2] = {0x4E, 0x04};
  uint8_t out[2] = {0, 0};
  CHECK(ft8x7MenuRowPair(map, false, 91, true, old, out));
  CHECK_EQ(out[0], 0x5A);
  CHECK_EQ(out[1], 0x04);
  CHECK(ft8x7MenuRowPair(map, true, 17, true, old, out));
  CHECK_EQ(out[0], 0x4E);
  CHECK_EQ(out[1], 0x10);
  CHECK(ft8x7MenuRowPair(map, true, 1, false, old, out));
  CHECK_EQ(out[0], 0x4E);
  CHECK_EQ(out[1], 0x00);
}

TEST(menu_row_pair_rejects_numbers_out_of_range) {
  Ft8x7MenuRowMap map{};
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft857, map));
  const uint8_t old[2] = {0x4E, 0x04};
  uint8_t out[2] = {0xAA, 0xBB};
  CHECK(!ft8x7MenuRowPair(map, false, 0, true, old, out));
  CHECK(!ft8x7MenuRowPair(map, false, 92, true, old, out));
  CHECK(!ft8x7MenuRowPair(map, false, 89, false, old, out));
  CHECK(ft8x7MenuRowPair(map, false, 88, false, old, out));
  out[0] = 0xAA;
  out[1] = 0xBB;
  CHECK(!ft8x7MenuRowPair(map, true, 0, true, old, out));
  CHECK(!ft8x7MenuRowPair(map, true, 18, true, old, out));
  CHECK_EQ(out[0], 0xAA);
  CHECK_EQ(out[1], 0xBB);
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft817, map));
  CHECK(!ft8x7MenuRowPair(map, false, 53, false, old, out));
  CHECK(!ft8x7MenuRowPair(map, false, 58, true, old, out));
  CHECK(!ft8x7MenuRowPair(map, true, 13, true, old, out));
  CHECK_EQ(out[0], 0xAA);
  CHECK_EQ(out[1], 0xBB);
}

// A stored pair out of range means the map does not fit the radio: nothing is written.
TEST(menu_row_pair_rejects_unexpected_stored_bytes) {
  Ft8x7MenuRowMap map{};
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft857, map));
  uint8_t out[2] = {0xAA, 0xBB};
  const uint8_t ft857Cases[][2] = {{0x5B, 0x04}, {0xFF, 0x04}, {0x4E, 0x11}, {0x4E, 0x80}};
  for (const auto& old : ft857Cases) {
    CHECK(!ft8x7MenuRowPair(map, false, 5, true, old, out));
    CHECK(!ft8x7MenuRowPair(map, true, 5, true, old, out));
  }
  // The last item with the extended menu on stays valid when it is off.
  const uint8_t ext[2] = {0x5A, 0x10};
  CHECK(ft8x7MenuRowPair(map, true, 2, false, ext, out));
  CHECK_EQ(out[0], 0x5A);
  CHECK_EQ(out[1], 0x01);
  out[0] = 0xAA;
  out[1] = 0xBB;
  CHECK(ft8x7MenuRowMap(Ft8x7Model::Ft817, map));
  const uint8_t ft817Cases[][2] = {{0x39, 0x03}, {0x3F, 0x03}, {0x31, 0x0C}, {0x31, 0x0F}};
  for (const auto& old : ft817Cases) {
    CHECK(!ft8x7MenuRowPair(map, false, 5, true, old, out));
    CHECK(!ft8x7MenuRowPair(map, true, 5, true, old, out));
  }
  CHECK_EQ(out[0], 0xAA);
  CHECK_EQ(out[1], 0xBB);
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
