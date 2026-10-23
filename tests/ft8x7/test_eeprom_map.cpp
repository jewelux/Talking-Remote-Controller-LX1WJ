// Tests for the FT-8x7 EEPROM map (firmware/ft8x7_eeprom_map.*). The addresses and values were
// measured on an FT-817 and an FT-897; these tests keep them from changing by accident.
#include "ft8x7_eeprom_map.h"
#include "test_runner.h"

#include <climits>

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

TEST(ft857_band_slot_for_each_amateur_band) {
  const uint64_t hz[] = {1840000ULL,  3700000ULL,  5357000ULL,  7100000ULL,   10125000ULL,
                         14250000ULL, 18100000ULL, 21300000ULL, 24940000ULL,  28500000ULL,
                         50313000ULL, 145500000ULL, 435000000ULL};
  const Ft857BandSlot *previous = nullptr;
  for (uint64_t h : hz) {
    const Ft857BandSlot *slot = ft857BandSlotForHz(h);
    CHECK(slot != nullptr);
    CHECK(slot != previous);
    previous = slot;
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

TEST(ft857_vfo_band_block_for_each_band) {
  // 160 m to 70 cm, then general coverage, as measured.
  const uint16_t blocks[16] = {0x00BA, 0x00D6, 0x00F2, 0x010E, 0x012A, 0x0146, 0x0162, 0x017E,
                               0x019A, 0x01B6, 0x01D2, 0x01EE, 0x020A, 0x0226, 0x0242, 0x025E};
  for (uint8_t band = 0; band < 16; ++band) {
    CHECK_EQ(ft857VfoBandBlock(band, false), blocks[band]);
    CHECK_EQ(ft857VfoBandBlock((uint8_t)(band << 4), true), (uint16_t)(blocks[band] + 0x01C0));
  }
}

TEST(ft857_vfo_band_block_comes_from_the_band_numbers) {
  // 0x6C as read: 2 m on A with general coverage on B, then 70 cm, then 26.8 MHz on both.
  CHECK_EQ(ft857VfoBandBlock(0xFD, false), 0x0226);
  CHECK_EQ(ft857VfoBandBlock(0xFE, false), 0x0242);
  CHECK_EQ(ft857VfoBandBlock(0xFF, false), 0x025E);
  CHECK_EQ(ft857VfoBandBlock(0xFD, true), 0x041E);
  CHECK_EQ(ft857VfoBandBlock(0x90, false), 0x00BA);
  CHECK_EQ(ft857VfoBandBlock(0x90, true), 0x0376);
}

TEST(ft857_ipo_att_exist_on_hf_and_6_m_only) {
  CHECK(ft857HasIpoAtt(1840000ULL));
  CHECK(ft857HasIpoAtt(26800000ULL));
  CHECK(ft857HasIpoAtt(50313000ULL));
  CHECK(!ft857HasIpoAtt(95000000ULL));
  CHECK(!ft857HasIpoAtt(145500000ULL));
  CHECK(!ft857HasIpoAtt(435000000ULL));
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
  CHECK_EQ(ft857AgcFromSpeed(0x00), YaesuAgc::Auto);
  CHECK_EQ(ft857AgcFromSpeed(0x01), YaesuAgc::Fast);
  CHECK_EQ(ft857AgcFromSpeed(0x02), YaesuAgc::Slow);
  CHECK_EQ(ft857AgcFromSpeed(0x03), YaesuAgc::Auto);
  CHECK_EQ(ft857AgcFromSpeed(0xFE), YaesuAgc::Slow);
}

TEST(ft857_agc_speed_byte_keeps_the_other_bits) {
  CHECK_EQ(ft857AgcSpeedByte(YaesuAgc::Auto, 0xF3), 0xF0);
  CHECK_EQ(ft857AgcSpeedByte(YaesuAgc::Fast, 0xF0), 0xF1);
  CHECK_EQ(ft857AgcSpeedByte(YaesuAgc::Slow, 0x31), 0x32);
  // Off is a bit of its own: the speed stays.
  CHECK_EQ(ft857AgcSpeedByte(YaesuAgc::Off, 0x32), 0x32);
  static constexpr YaesuAgc kSpeeds[] = {YaesuAgc::Slow, YaesuAgc::Auto, YaesuAgc::Fast};
  for (const YaesuAgc agc : kSpeeds) {
    CHECK_EQ(ft857AgcFromSpeed(ft857AgcSpeedByte(agc, 0xA7)), agc);
  }
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
  CHECK(ft857LevelAddr(YaesuFt857Level::AmMicGain, addr));
  CHECK_EQ(addr, 0x007B);
  CHECK(ft857LevelAddr(YaesuFt857Level::NrLevel, addr));
  CHECK_EQ(addr, 0x0093);
  CHECK(ft857LevelAddr(YaesuFt857Level::LpfCutoff, addr));
  CHECK_EQ(addr, 0x0094);
  CHECK(ft857LevelAddr(YaesuFt857Level::VoxGain, addr));
  CHECK_EQ(addr, 0x0076);
}

TEST(ft857_level_scaling) {
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
  YaesuFt857Level level = YaesuFt857Level::NrLevel;
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

TEST(ft817_agc_byte_keeps_the_other_bits) {
  CHECK_EQ(ft817AgcByte(YaesuAgc::Auto, 0xF3), 0xF0);
  CHECK_EQ(ft817AgcByte(YaesuAgc::Fast, 0x20), 0x21);
  CHECK_EQ(ft817AgcByte(YaesuAgc::Slow, 0x10), 0x12);
  CHECK_EQ(ft817AgcByte(YaesuAgc::Off, 0xC0), 0xC3);
  static constexpr YaesuAgc kAgcs[] = {YaesuAgc::Off, YaesuAgc::Fast, YaesuAgc::Slow, YaesuAgc::Auto};
  for (const YaesuAgc agc : kAgcs) {
    CHECK_EQ(ft817AgcFromByte(ft817AgcByte(agc, 0x5D)), agc);
  }
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
  Ft8x7Flag::Split,  Ft8x7Flag::KeyReverse, Ft8x7Flag::CwAutoMode, Ft8x7Flag::PaddleMicKey,
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
    {Ft8x7Flag::KeyReverse, 0x006B, 0x04, false}, {Ft8x7Flag::PaddleMicKey, 0x007B, 0x80, false},
    {Ft8x7Flag::CwAutoMode, 0x0092, 0x80, false},
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
    {Ft8x7Flag::KeyReverse, 0x0058, 0x04, false},
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

// Measured on an FT-897: its DBF key sets and clears both bits, in LSB and CW (0x20 <-> 0x2C).
TEST(dbf_byte_sets_both_bits) {
  Ft8x7FlagField dbf{};
  CHECK(ft8x7FlagField(Ft8x7Model::Ft857, Ft8x7Flag::Dbf, dbf));
  CHECK_EQ(ft8x7FlagByte(dbf, true, 0x20), 0x2C);
  CHECK_EQ(ft8x7FlagByte(dbf, false, 0x2C), 0x20);
  CHECK_EQ(ft8x7FlagByte(dbf, true, 0x24), 0x2C);
  CHECK_EQ(ft8x7FlagByte(dbf, false, 0xFF), 0xF3);
}

TEST(flag_byte_keeps_the_other_bits) {
  const Ft8x7FlagField nb{0x006A, 0x20, false};
  CHECK_EQ(ft8x7FlagByte(nb, true, 0x00), 0x20);
  CHECK_EQ(ft8x7FlagByte(nb, true, 0xDF), 0xFF);
  CHECK_EQ(ft8x7FlagByte(nb, false, 0xFF), 0xDF);
  CHECK_EQ(ft8x7FlagByte(nb, false, 0x00), 0x00);
  const Ft8x7FlagField lock{0x006A, 0x40, true};
  CHECK_EQ(ft8x7FlagByte(lock, true, 0xFF), 0xBF);
  CHECK_EQ(ft8x7FlagByte(lock, false, 0x00), 0x40);
}

TEST(ft857_settable_level_ranges) {
  uint16_t lo = 0xFFFF;
  uint16_t hi = 0;
  CHECK(ft857LevelRange(YaesuFt857Level::NrLevel, lo, hi));
  CHECK_EQ(lo, 1);
  CHECK_EQ(hi, 16);
  CHECK(ft857LevelRange(YaesuFt857Level::NbLevel, lo, hi));
  CHECK_EQ(lo, 0);
  CHECK_EQ(hi, 100);
  CHECK(ft857LevelRange(YaesuFt857Level::BpfWidth, lo, hi));
  CHECK_EQ(lo, 60);
  CHECK_EQ(hi, 240);
  CHECK(ft857LevelRange(YaesuFt857Level::HpfCutoff, lo, hi));
  CHECK_EQ(lo, 100);
  CHECK_EQ(hi, 1000);
  CHECK(ft857LevelRange(YaesuFt857Level::LpfCutoff, lo, hi));
  CHECK_EQ(lo, 1000);
  CHECK_EQ(hi, 6000);
  static constexpr YaesuFt857Level kPercent[] = {YaesuFt857Level::ProcLevel, YaesuFt857Level::SsbMicGain,
                                                 YaesuFt857Level::AmMicGain, YaesuFt857Level::FmMicGain,
                                                 YaesuFt857Level::DigGain};
  for (const YaesuFt857Level level : kPercent) {
    lo = 0xFFFF;
    hi = 0;
    CHECK(ft857LevelRange(level, lo, hi));
    CHECK_EQ(lo, 0);
    CHECK_EQ(hi, 100);
  }
  CHECK(!ft857LevelRange(YaesuFt857Level::DigVox, lo, hi));
}

// Bit 7 of the PROC level and mic gain bytes is not theirs and stays.
TEST(ft857_proc_level_and_mic_gain_bytes_keep_bit_7) {
  uint8_t out = 0;
  CHECK(ft857LevelByte(YaesuFt857Level::ProcLevel, 75, 0x80, out));
  CHECK_EQ(out, 0xCB);
  CHECK(ft857LevelByte(YaesuFt857Level::SsbMicGain, 0, 0xB2, out));
  CHECK_EQ(out, 0x80);
  CHECK(ft857LevelByte(YaesuFt857Level::FmMicGain, 100, 0x32, out));
  CHECK_EQ(out, 0x64);
  out = 0xAA;
  CHECK(!ft857LevelByte(YaesuFt857Level::AmMicGain, 101, 0x32, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::DigGain, 50, 0x7F, out));
  CHECK_EQ(out, 0xAA);
}

// 0x93 also holds the NR level and the BPF width, which stay.
TEST(ft857_mic_eq_byte_keeps_nr_level_and_bpf_width) {
  CHECK_EQ(ft857MicEqByte(YaesuFt857MicEq::Both, 0x58), 0x5B);
  CHECK_EQ(ft857MicEqByte(YaesuFt857MicEq::Off, 0xFB), 0xF8);
  CHECK_EQ(ft857MicEqByte(YaesuFt857MicEq::Lpf, 0x02), 0x01);
  CHECK_EQ(ft857MicEqByte(YaesuFt857MicEq::Hpf, 0x01), 0x02);
  CHECK_EQ(ft857MicEqFromByte(ft857MicEqByte(YaesuFt857MicEq::Hpf, 0xA4)), YaesuFt857MicEq::Hpf);
}

// FT-817 0x6B bit 7 is menu 52 EXTEND, 0x68 bit 7 another setting: kept.
TEST(ft817_level_byte_keeps_bit_7) {
  uint8_t out = 0;
  CHECK(ft817LevelByte(50, 0xE4, out));
  CHECK_EQ(out, 0xB2);
  CHECK(ft817LevelByte(0, 0x32, out));
  CHECK_EQ(out, 0x00);
  CHECK(ft817LevelByte(100, 0x80, out));
  CHECK_EQ(out, 0xE4);
  out = 0xAA;
  CHECK(!ft817LevelByte(101, 0x32, out));
  CHECK(!ft817LevelByte(50, 0x7F, out));  // stored 127: the map does not fit
  CHECK_EQ(out, 0xAA);
}

// 0x93 also holds the BPF width and the mic EQ, which stay.
TEST(ft857_nr_level_byte_keeps_bpf_and_mic_eq) {
  uint8_t out = 0;
  CHECK(ft857LevelByte(YaesuFt857Level::NrLevel, 16, 0x5B, out));
  CHECK_EQ(out, 0xFB);
  CHECK(ft857LevelByte(YaesuFt857Level::NrLevel, 1, 0x5B, out));
  CHECK_EQ(out, 0x0B);
  CHECK(ft857LevelByte(YaesuFt857Level::NrLevel, 8, 0x00, out));
  CHECK_EQ(out, 0x70);
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::NrLevel, out), 8);
}

// 0x93 bits 3..2: 60, 120 or 240 Hz; the NR level and the mic EQ stay.
TEST(ft857_bpf_width_byte_keeps_nr_level_and_mic_eq) {
  uint8_t out = 0;
  CHECK(ft857LevelByte(YaesuFt857Level::BpfWidth, 240, 0x53, out));
  CHECK_EQ(out, 0x5B);
  CHECK(ft857LevelByte(YaesuFt857Level::BpfWidth, 60, 0xFB, out));
  CHECK_EQ(out, 0xF3);
  CHECK(ft857LevelByte(YaesuFt857Level::BpfWidth, 120, 0x00, out));
  CHECK_EQ(out, 0x04);
  CHECK(ft857LevelByte(YaesuFt857Level::BpfWidth, 180, 0x00, out));  // a tie: the lower one
  CHECK_EQ(out, 0x04);
  CHECK(ft857LevelByte(YaesuFt857Level::BpfWidth, 200, 0x00, out));
  CHECK_EQ(out, 0x08);
  out = 0xAA;
  CHECK(!ft857LevelByte(YaesuFt857Level::BpfWidth, 59, 0x08, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::BpfWidth, 241, 0x08, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::BpfWidth, 120, 0x0C, out));  // stored 3: not a width
  CHECK_EQ(out, 0xAA);
}

TEST(ft857_nb_level_byte_keeps_bit_7) {
  uint8_t out = 0;
  CHECK(ft857LevelByte(YaesuFt857Level::NbLevel, 0, 0xB2, out));
  CHECK_EQ(out, 0x80);
  CHECK(ft857LevelByte(YaesuFt857Level::NbLevel, 100, 0xB2, out));
  CHECK_EQ(out, 0xE4);
  CHECK(ft857LevelByte(YaesuFt857Level::NbLevel, 50, 0x00, out));
  CHECK_EQ(out, 0x32);
}

TEST(ft857_level_byte_rejects_values_out_of_range) {
  uint8_t out = 0xAA;
  CHECK(!ft857LevelByte(YaesuFt857Level::NrLevel, 0, 0x5B, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::NrLevel, 17, 0x5B, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::NbLevel, 101, 0x32, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::DigVox, 20, 0x10, out));
  CHECK_EQ(out, 0xAA);
}

// A stored NB level above 100 means the map does not fit the radio: nothing is written.
TEST(ft857_level_byte_rejects_unexpected_stored_bytes) {
  uint8_t out = 0xAA;
  CHECK(!ft857LevelByte(YaesuFt857Level::NbLevel, 50, 0x65, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::NbLevel, 50, 0xFF, out));
  CHECK_EQ(out, 0xAA);
}

// 0x95 bits 3..0, 60 Hz steps from 100 Hz; the radio's value nearest the one asked for.
TEST(ft857_hpf_byte_rounds_to_the_nearest_step) {
  uint8_t out = 0;
  CHECK(ft857LevelByte(YaesuFt857Level::HpfCutoff, 100, 0x6F, out));
  CHECK_EQ(out, 0x60);
  CHECK(ft857LevelByte(YaesuFt857Level::HpfCutoff, 1000, 0x60, out));
  CHECK_EQ(out, 0x6F);
  CHECK(ft857LevelByte(YaesuFt857Level::HpfCutoff, 300, 0x60, out));  // 280 and 340: 280
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::HpfCutoff, out), 280);
  CHECK(ft857LevelByte(YaesuFt857Level::HpfCutoff, 310, 0x60, out));  // a tie: the lower one
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::HpfCutoff, out), 280);
  CHECK_EQ(out & 0xF0, 0x60);
}

// 0x94 bits 4..0, 32 steps from 1000 to 6000 Hz.
TEST(ft857_lpf_byte_rounds_to_the_nearest_step) {
  uint8_t out = 0;
  CHECK(ft857LevelByte(YaesuFt857Level::LpfCutoff, 6000, 0x80, out));
  CHECK_EQ(out, 0x9F);
  CHECK(ft857LevelByte(YaesuFt857Level::LpfCutoff, 1000, 0x9F, out));
  CHECK_EQ(out, 0x80);
  CHECK(ft857LevelByte(YaesuFt857Level::LpfCutoff, 2700, 0x9F, out));
  CHECK_EQ(ft857LevelValue(YaesuFt857Level::LpfCutoff, out), 2770);
  CHECK(ft857LevelByte(YaesuFt857Level::LpfCutoff, 2770, 0x9F, out));
  CHECK_EQ(out, 0x8B);
}

TEST(ft857_cut_bytes_reject_values_out_of_range) {
  uint8_t out = 0xAA;
  CHECK(!ft857LevelByte(YaesuFt857Level::HpfCutoff, 99, 0x60, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::HpfCutoff, 1001, 0x60, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::LpfCutoff, 999, 0x9F, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::LpfCutoff, 6001, 0x9F, out));
  CHECK_EQ(out, 0xAA);
}

TEST(ft857_level_step_moves_by_the_radios_steps) {
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NrLevel, 8, -3), 5);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NrLevel, 15, 5), 16);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NbLevel, 80, 10), 90);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NbLevel, 95, 10), 100);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NbLevel, 5, -10), 0);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::BpfWidth, 60, 1), 120);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::BpfWidth, 120, 1), 240);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::BpfWidth, 240, 1), 240);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::BpfWidth, 240, -1), 120);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::BpfWidth, 60, -1), 60);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::HpfCutoff, 100, 1), 160);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::HpfCutoff, 100, -1), 100);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::HpfCutoff, 1000, 1), 1000);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::LpfCutoff, 6000, -1), ft857LevelValue(YaesuFt857Level::LpfCutoff, 30));
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::LpfCutoff, 2770, 1), ft857LevelValue(YaesuFt857Level::LpfCutoff, 12));
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::DigVox, 20, 1), 20);
}

// The console passes any step count; more steps than the radio has go to the end of the range.
TEST(ft857_level_step_takes_any_number_of_steps) {
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::LpfCutoff, 2770, INT_MAX), 6000);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::LpfCutoff, 2770, INT_MIN), 1000);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NbLevel, 50, INT_MAX), 100);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NbLevel, 50, INT_MIN), 0);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NrLevel, 8, 1000), 16);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::NrLevel, 8, -1000), 1);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::BpfWidth, 60, INT_MAX), 240);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::BpfWidth, 240, INT_MIN), 60);
}

TEST(percent_step_stays_within_0_to_100) {
  CHECK_EQ(percentStep(50, 5), 55);
  CHECK_EQ(percentStep(50, -5), 45);
  CHECK_EQ(percentStep(98, 5), 100);
  CHECK_EQ(percentStep(3, -5), 0);
  CHECK_EQ(percentStep(50, INT_MAX), 100);
  CHECK_EQ(percentStep(50, INT_MIN), 0);
  CHECK_EQ(percentStep(127, 0), 100);  // above 100 counts as 100
}

TEST(ft857_rf_power_range_per_band_group) {
  uint8_t lo = 0;
  uint8_t hi = 0;
  ft857RfPowerRange(Ft8x7BandGroup::Hf, lo, hi);
  CHECK_EQ(lo, 5);
  CHECK_EQ(hi, 100);
  ft857RfPowerRange(Ft8x7BandGroup::SixM, lo, hi);
  CHECK_EQ(lo, 5);
  CHECK_EQ(hi, 100);
  ft857RfPowerRange(Ft8x7BandGroup::Vhf, lo, hi);
  CHECK_EQ(lo, 5);
  CHECK_EQ(hi, 50);
  ft857RfPowerRange(Ft8x7BandGroup::Uhf, lo, hi);
  CHECK_EQ(lo, 2);
  CHECK_EQ(hi, 20);
}

// Stored on an FT-897: 5, 10 and 19 W without bit 7, 20, 50 and 100 W with it; 19 and 20 W
// measured on HF and UHF.
TEST(ft857_rf_power_byte_like_the_radio_stores_it) {
  uint8_t out = 0;
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Hf, 10, 0xE4, out));
  CHECK_EQ(out, 0x0A);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Hf, 100, 0x0A, out));
  CHECK_EQ(out, 0xE4);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Hf, 5, 0x0A, out));
  CHECK_EQ(out, 0x05);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Vhf, 50, 0x94, out));
  CHECK_EQ(out, 0xB2);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Uhf, 20, 0x05, out));
  CHECK_EQ(out, 0x94);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Uhf, 2, 0x94, out));
  CHECK_EQ(out, 0x02);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Hf, 19, 0x94, out));
  CHECK_EQ(out, 0x13);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Uhf, 19, 0x94, out));
  CHECK_EQ(out, 0x13);
  CHECK(ft857RfPowerByte(Ft8x7BandGroup::Uhf, 10, 0x94, out));
  CHECK_EQ(out, 0x0A);
}

TEST(ft857_rf_power_byte_rejects_values_out_of_range) {
  uint8_t out = 0xAA;
  CHECK(!ft857RfPowerByte(Ft8x7BandGroup::Hf, 4, 0x0A, out));
  CHECK(!ft857RfPowerByte(Ft8x7BandGroup::Hf, 101, 0x0A, out));
  CHECK(!ft857RfPowerByte(Ft8x7BandGroup::Vhf, 51, 0xB2, out));
  CHECK(!ft857RfPowerByte(Ft8x7BandGroup::Uhf, 21, 0x94, out));
  CHECK(!ft857RfPowerByte(Ft8x7BandGroup::Uhf, 1, 0x94, out));
  // Stored watts outside the range: the map does not fit.
  CHECK(!ft857RfPowerByte(Ft8x7BandGroup::Hf, 50, 0x7F, out));
  CHECK(!ft857RfPowerByte(Ft8x7BandGroup::Uhf, 10, 0xB2, out));
  CHECK_EQ(out, 0xAA);
}

TEST(ft817_power_level_is_bits_1_0) {
  CHECK(ft817PowerLevel(0xFC) == Ft817PowerLevel::High);
  CHECK(ft817PowerLevel(0x01) == Ft817PowerLevel::L3);
  CHECK(ft817PowerLevel(0x02) == Ft817PowerLevel::L2);
  CHECK(ft817PowerLevel(0xF3) == Ft817PowerLevel::L1);
}

TEST(ft817_power_level_byte_keeps_the_other_bits) {
  CHECK_EQ(ft817PowerLevelByte(Ft817PowerLevel::High, 0xF3), 0xF0);
  CHECK_EQ(ft817PowerLevelByte(Ft817PowerLevel::L1, 0xF0), 0xF3);
  CHECK_EQ(ft817PowerLevelByte(Ft817PowerLevel::L3, 0x00), 0x01);
  CHECK_EQ(ft817PowerLevelByte(Ft817PowerLevel::L2, 0x03), 0x02);
}

TEST(ft857_vox_levels_can_be_set) {
  uint16_t lo = 0;
  uint16_t hi = 0;
  CHECK(ft857LevelRange(YaesuFt857Level::VoxGain, lo, hi));
  CHECK_EQ(lo, 1);
  CHECK_EQ(hi, 100);
  CHECK(ft857LevelRange(YaesuFt857Level::VoxDelay, lo, hi));
  CHECK_EQ(lo, 100);
  CHECK_EQ(hi, 3000);
  uint8_t out = 0;
  CHECK(ft857LevelByte(YaesuFt857Level::VoxGain, 50, 0x80 | 10, out));
  CHECK_EQ(out, 0x80 | 50);  // bit 7 kept
  CHECK(ft857LevelByte(YaesuFt857Level::VoxDelay, 500, 3, out));
  CHECK_EQ(out, 5);
  CHECK(ft857LevelByte(YaesuFt857Level::VoxDelay, 549, 3, out));  // to the nearest 100 ms
  CHECK_EQ(out, 5);
  out = 0xAA;
  CHECK(!ft857LevelByte(YaesuFt857Level::VoxGain, 0, 10, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::VoxDelay, 3100, 3, out));
  CHECK(!ft857LevelByte(YaesuFt857Level::VoxDelay, 500, 0, out));  // stored 0: the map does not fit
  CHECK_EQ(out, 0xAA);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::VoxDelay, 500, -2), 300);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::VoxDelay, 2900, INT_MAX), 3000);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::VoxDelay, 200, INT_MIN), 100);
  CHECK_EQ(ft857LevelStep(YaesuFt857Level::VoxGain, 3, -10), 1);
}

TEST(ft817_vox_gain_byte_is_1_to_100) {
  uint8_t out = 0;
  CHECK(ft817VoxGainByte(50, 0x80 | 10, out));
  CHECK_EQ(out, 0x80 | 50);
  out = 0xAA;
  CHECK(!ft817VoxGainByte(0, 10, out));
  CHECK(!ft817VoxGainByte(101, 10, out));
  CHECK(!ft817VoxGainByte(50, 0, out));  // stored 0: the map does not fit
  CHECK_EQ(out, 0xAA);
}

TEST(ft817_vox_delay_byte_keeps_bits_7_5) {
  uint8_t out = 0;
  CHECK(ft817VoxDelayByte(300, 0xE5, out));
  CHECK_EQ(out, 0xE3);
  CHECK(ft817VoxDelayByte(2500, 0x05, out));
  CHECK_EQ(out, 0x19);
  CHECK(ft817VoxDelayByte(149, 0x05, out));  // to the nearest 100 ms
  CHECK_EQ(out, 0x01);
  out = 0xAA;
  CHECK(!ft817VoxDelayByte(99, 0x05, out));
  CHECK(!ft817VoxDelayByte(2600, 0x05, out));
  CHECK(!ft817VoxDelayByte(500, 0x00, out));  // stored 0 or above 25: the map does not fit
  CHECK(!ft817VoxDelayByte(500, 0x1A, out));
  CHECK_EQ(out, 0xAA);
  CHECK_EQ(ft817VoxDelayStep(500, 2), 700);
  CHECK_EQ(ft817VoxDelayStep(500, INT_MAX), 2500);
  CHECK_EQ(ft817VoxDelayStep(500, INT_MIN), 100);
}

TEST(ft817_rear_antenna_byte_keeps_the_other_bits) {
  CHECK_EQ(ft817RearAntennaByte(Ft8x7BandGroup::Hf, true, 0x80), 0x81);  // split kept
  CHECK_EQ(ft817RearAntennaByte(Ft8x7BandGroup::Vhf, false, 0xBF), 0xAF);
  CHECK_EQ(ft817RearAntennaByte(Ft8x7BandGroup::Uhf, true, 0x20), 0x20);
}

// The CW value read from pair, or 0xFFFF where ft8x7CwLevelValue refuses it.
uint16_t cwValue(Ft8x7Model model, Ft8x7CwLevel level, const uint8_t pair[2]) {
  uint16_t value = 0;
  return ft8x7CwLevelValue(model, level, pair, value) ? value : 0xFFFF;
}

TEST(ft857_cw_levels_as_measured) {
  // An FT-897 as found: 0x71..0x75 = 13 E5 19 05 53, 0x92 = C2.
  const uint8_t pitch[2] = {0x13, 0xE5};
  const uint8_t weight[2] = {0xE5, 0x19};
  const uint8_t delay[2] = {0x19, 0x05};
  const uint8_t sidetone[2] = {0x05, 0x53};
  const uint8_t speed[2] = {0x53, 0xB2};
  const uint8_t qsk[2] = {0xC2, 0x78};
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, pitch), 700);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Weight, weight), 30);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, delay), 250);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Sidetone, sidetone), 5);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Speed, speed), 23);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Qsk, qsk), 10);
  // The ends set on the radio.
  const uint8_t pitchMin[2] = {0x10, 0xE5};
  const uint8_t pitchMax[2] = {0x14, 0xE5};
  const uint8_t weightMax[2] = {0xF4, 0x19};
  const uint8_t delayFull[2] = {0x02, 0x05};
  const uint8_t delay30[2] = {0x03, 0x05};
  const uint8_t delayMax[2] = {0x2C, 0x85};
  const uint8_t sidetoneMax[2] = {0x64, 0x53};
  const uint8_t sidetoneWithDelayBit[2] = {0x85, 0x53};
  const uint8_t qskMax[2] = {0xD2, 0x78};
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, pitchMin), 400);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, pitchMax), 800);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Weight, weightMax), 45);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, delayFull), 0);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, delay30), 30);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, delayMax), 3000);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Sidetone, sidetoneMax), 100);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Sidetone, sidetoneWithDelayBit), 5);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Qsk, qskMax), 30);
}

// A stored value outside the radio's steps means the map does not fit: no value is read.
TEST(cw_level_reads_refuse_values_the_radio_does_not_offer) {
  const uint8_t speedMin[2] = {0x00, 0x00};
  const uint8_t speedMax[2] = {0xF8, 0x00};  // bits 7..6 are not the speed
  const uint8_t speed61[2] = {0x39, 0x00};
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Speed, speedMin), 4);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Speed, speedMax), 60);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Speed, speed61), 0xFFFF);
  const uint8_t delay0[2] = {0x00, 0x05};
  const uint8_t delay1[2] = {0x01, 0x05};  // not FULL, which is 2
  const uint8_t pitch5[2] = {0x15, 0xE5};
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, delay0), 0xFFFF);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, delay1), 0xFFFF);
  CHECK_EQ(cwValue(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, pitch5), 0xFFFF);
  const uint8_t ft817Pitch15[2] = {0x0F, 0x00};
  const uint8_t ft817Delay0[2] = {0x00, 0x00};
  CHECK_EQ(cwValue(Ft8x7Model::Ft817, Ft8x7CwLevel::Pitch, ft817Pitch15), 0xFFFF);
  CHECK_EQ(cwValue(Ft8x7Model::Ft817, Ft8x7CwLevel::Delay, ft817Delay0), 0xFFFF);
  CHECK_EQ(cwValue(Ft8x7Model::Ft817, Ft8x7CwLevel::Qsk, ft817Delay0), 0xFFFF);  // not kept
}

TEST(ft857_cw_level_pair_keeps_the_other_bits) {
  uint8_t out[2] = {0, 0};
  const uint8_t delay[2] = {0x19, 0x05};
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 3000, delay, out));
  CHECK_EQ(out[0], 0x2C);
  CHECK_EQ(out[1], 0x85);  // bit 8 set, side tone 5 kept
  const uint8_t delayMax[2] = {0x2C, 0x85};
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 250, delayMax, out));
  CHECK_EQ(out[0], 0x19);
  CHECK_EQ(out[1], 0x05);
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 0, delay, out));  // FULL
  CHECK_EQ(out[0], 0x02);
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 254, delay, out));  // nearest 10 ms
  CHECK_EQ(out[0], 0x19);
  const uint8_t pitch[2] = {0x13, 0xE5};
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, 400, pitch, out));
  CHECK_EQ(out[0], 0x10);
  CHECK_EQ(out[1], 0xE5);  // the weight's byte untouched
  const uint8_t qsk[2] = {0xC2, 0x78};
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Qsk, 15, qsk, out));
  CHECK_EQ(out[0], 0xC6);  // auto mode, the TRAINING bit and the BFO kept
  const uint8_t weight[2] = {0xE5, 0x19};
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Weight, 31, weight, out));
  CHECK_EQ(out[0], 0xE6);  // the SQL/RF knob bit kept
  const uint8_t sidetone[2] = {0x85, 0x53};
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Sidetone, 100, sidetone, out));
  CHECK_EQ(out[0], 0xE4);  // the delay's bit 8 kept
}

TEST(ft857_cw_level_pair_refuses_what_the_radio_does_not_offer) {
  const uint8_t delay[2] = {0x19, 0x05};
  uint8_t out[2] = {0xAA, 0xBB};
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 20, delay, out));
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 3010, delay, out));
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, 900, delay, out));
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Speed, 3, delay, out));
  // Stored values the map does not fit: delay 1, delay above 300, pitch 5, QSK 5.
  const uint8_t delay1[2] = {0x01, 0x05};
  const uint8_t delay301[2] = {0x2D, 0x85};
  const uint8_t pitch5[2] = {0x15, 0xE5};
  const uint8_t qsk5[2] = {0xD6, 0x78};
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 250, delay1, out));
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 250, delay301, out));
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, 600, pitch5, out));
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft857, Ft8x7CwLevel::Qsk, 20, qsk5, out));
  CHECK_EQ(out[0], 0xAA);
  CHECK_EQ(out[1], 0xBB);
}

TEST(ft857_cw_level_ranges_and_steps) {
  uint16_t lo = 0;
  uint16_t hi = 0;
  CHECK(ft8x7CwLevelRange(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, lo, hi));
  CHECK_EQ(lo, 30);
  CHECK_EQ(hi, 3000);
  CHECK(ft8x7CwDelayHasFull(Ft8x7Model::Ft857));
  CHECK(ft8x7CwLevelRange(Ft8x7Model::Ft857, Ft8x7CwLevel::Speed, lo, hi));
  CHECK_EQ(lo, 4);
  CHECK_EQ(hi, 60);
  CHECK(ft8x7CwLevelRange(Ft8x7Model::Ft857, Ft8x7CwLevel::Qsk, lo, hi));
  CHECK_EQ(lo, 10);
  CHECK_EQ(hi, 30);
  // FULL is the step below 30 ms.
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 30, -1), 0);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 0, 1), 30);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 250, -100), 0);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Delay, 250, INT_MAX), 3000);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, 700, 1), 800);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Pitch, 700, 5), 800);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Weight, 30, INT_MIN), 25);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft857, Ft8x7CwLevel::Qsk, 10, 2), 20);
  CHECK_EQ(ft8x7CwLevelStep(Ft8x7Model::Ft817, Ft8x7CwLevel::Qsk, 10, 2), 10);  // not kept
}

TEST(ft817_cw_levels_from_the_ka7oei_map) {
  const uint8_t pitch[2] = {0x08, 0x00};  // 700 Hz
  const uint8_t delay[2] = {0x19, 0x00};
  const uint8_t speed[2] = {0x93, 0x00};  // bits 7..6 the charge time
  CHECK_EQ(cwValue(Ft8x7Model::Ft817, Ft8x7CwLevel::Pitch, pitch), 700);
  CHECK_EQ(cwValue(Ft8x7Model::Ft818, Ft8x7CwLevel::Delay, delay), 250);
  CHECK_EQ(cwValue(Ft8x7Model::Ft817, Ft8x7CwLevel::Speed, speed), 23);
  uint16_t addr = 0;
  CHECK(ft8x7CwLevelAddr(Ft8x7Model::Ft817, Ft8x7CwLevel::Sidetone, addr));
  CHECK_EQ(addr, 0x0061);
  CHECK(ft8x7CwLevelAddr(Ft8x7Model::Ft817, Ft8x7CwLevel::Weight, addr));
  CHECK_EQ(addr, 0x005F);
  CHECK(!ft8x7CwLevelAddr(Ft8x7Model::Ft817, Ft8x7CwLevel::Qsk, addr));
  CHECK(!ft8x7CwLevelAddr(Ft8x7Model::None, Ft8x7CwLevel::Speed, addr));
  CHECK(!ft8x7CwDelayHasFull(Ft8x7Model::Ft817));
  uint16_t lo = 0;
  uint16_t hi = 0;
  CHECK(ft8x7CwLevelRange(Ft8x7Model::Ft817, Ft8x7CwLevel::Pitch, lo, hi));
  CHECK_EQ(lo, 300);
  CHECK_EQ(hi, 1000);
  CHECK(ft8x7CwLevelRange(Ft8x7Model::Ft817, Ft8x7CwLevel::Delay, lo, hi));
  CHECK_EQ(lo, 10);
  CHECK_EQ(hi, 2500);
  uint8_t out[2] = {0, 0};
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft817, Ft8x7CwLevel::Pitch, 1000, pitch, out));
  CHECK_EQ(out[0], 0x0E);
  CHECK(ft8x7CwLevelPair(Ft8x7Model::Ft817, Ft8x7CwLevel::Speed, 60, speed, out));
  CHECK_EQ(out[0], 0xB8);  // the charge time kept
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft817, Ft8x7CwLevel::Delay, 0, delay, out));  // no FULL
  const uint8_t delay0[2] = {0x00, 0x00};
  CHECK(!ft8x7CwLevelPair(Ft8x7Model::Ft817, Ft8x7CwLevel::Delay, 100, delay0, out));
}

TEST(ft857_cw_bfo) {
  Ft857CwBfo bfo = Ft857CwBfo::Usb;
  CHECK(ft857CwBfoFromByte(0xC2, bfo));
  CHECK_EQ(bfo, Ft857CwBfo::Auto);
  CHECK(ft857CwBfoFromByte(0xC1, bfo));
  CHECK_EQ(bfo, Ft857CwBfo::Lsb);
  CHECK(ft857CwBfoFromByte(0xC0, bfo));
  CHECK_EQ(bfo, Ft857CwBfo::Usb);
  CHECK(!ft857CwBfoFromByte(0xC3, bfo));
  CHECK_EQ(ft857CwBfoByte(Ft857CwBfo::Lsb, 0xD2), 0xD1);  // QSK and auto mode kept
}
