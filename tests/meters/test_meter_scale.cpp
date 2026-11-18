// Tests for the meter scales (firmware/meter_scale.*) and the tables the built-in profiles
// carry (firmware/radio_profile_table.cpp). S-meter: FTDX10 and FT-891 S9 = 130, FTDX101 S9 =
// 160, all S9+60 dB = 255; TS-480 S9 = 11, S9+60 dB = 19; KX2 S9 = 6, S9+60 dB = 15. SWR: each
// radio's own scale (Icom, KX2 tenths, TS-480 0..20, FTDX, FT-891, FT-8x7 bars).
#include "radio_profile_table.h"
#include "meter_scale.h"
#include "test_runner.h"

namespace {

struct Reading {
  int s = -1;
  int db = -1;
};

Reading decode(const MeterScale& scale, int32_t raw) {
  uint8_t s = 99, db = 99;
  smeterScaleDecode(scale, raw, s, db);
  return {s, db};
}

const MeterScale& scaleOfSlot(uint8_t slot) {
  static const MeterScale kNone{};
  const RadioProfile* p = profileForSlot(slot);
  CHECK(p != nullptr);
  return p ? p->smeterScale : kNone;
}

// S0 at 0, S9 at 90, S9+60 dB at 150: 10 raw steps per 6 dB below S9, per 10 dB above.
constexpr MeterCalPoint kLinearPoints[] = {{0, -54}, {90, 0}, {150, 60}};
const MeterScale kLinear{kLinearPoints};

}  // namespace

// ---- The scale itself ----

TEST(interpolates_between_points) {
  CHECK_EQ(smeterScaleDbRelS9(kLinear, 45), -27);
  CHECK_EQ(smeterScaleDbRelS9(kLinear, 120), 30);
}

TEST(s_units_below_s9_and_tens_above) {
  CHECK_EQ(decode(kLinear, 0).s, 0);
  CHECK_EQ(decode(kLinear, 50).s, 5);
  CHECK_EQ(decode(kLinear, 50).db, 0);
  CHECK_EQ(decode(kLinear, 90).s, 9);
  CHECK_EQ(decode(kLinear, 90).db, 0);
  CHECK_EQ(decode(kLinear, 110).s, 9);
  CHECK_EQ(decode(kLinear, 110).db, 20);
}

TEST(rounds_once_from_the_exact_value) {
  // 0.7 dB per raw step: -2.8 dB is still S9 and +4.9 dB not yet S9+10, though both would
  // round to a whole 3 or 5 dB first.
  constexpr MeterCalPoint kPoints[] = {{0, -7}, {10, 0}, {20, 7}};
  const MeterScale scale{kPoints};
  CHECK_EQ(decode(scale, 6).s, 9);
  CHECK_EQ(decode(scale, 17).s, 9);
  CHECK_EQ(decode(scale, 17).db, 0);
}

TEST(out_of_range_is_clamped) {
  CHECK_EQ(decode(kLinear, 300).s, 9);
  CHECK_EQ(decode(kLinear, 300).db, 60);
  CHECK_EQ(decode(kLinear, -5).s, 0);
  CHECK_EQ(decode(kLinear, -5).db, 0);
}

// ---- The profiles' tables ----

TEST(profiles_carry_their_scale) {
  CHECK(!scaleOfSlot(6).empty());   // KX2
  CHECK(!scaleOfSlot(7).empty());   // TS-480
  CHECK(!scaleOfSlot(11).empty());  // FTDX-10
  CHECK(!scaleOfSlot(12).empty());  // FTDX-101D
  CHECK(!scaleOfSlot(13).empty());  // FTDX-101MP
  CHECK(!scaleOfSlot(15).empty());  // FT-891
  CHECK(scaleOfSlot(1).empty());    // IC-7300: Icom decoder
  CHECK(scaleOfSlot(8).empty());    // FT-817: FT-8x7 decoder
}

TEST(every_profile_scale_ascends) {
  for (size_t i = 0; i < profileCount(); ++i) {
    CHECK(profileAt(i).smeterScale.ascends());
    CHECK(profileAt(i).swrScale.ascends());
  }
}

TEST(ftdx10_reference_points) {
  const MeterScale& scale = scaleOfSlot(11);
  CHECK_EQ(decode(scale, 0).s, 0);
  CHECK_EQ(decode(scale, 26).s, 2);
  CHECK_EQ(decode(scale, 51).s, 4);
  CHECK_EQ(decode(scale, 81).s, 6);
  CHECK_EQ(decode(scale, 105).s, 7);  // S7.5: a half unit is not yet the next one
  CHECK_EQ(decode(scale, 130).s, 9);  // the old generic scale (S9 = 297) said S4 here
  CHECK_EQ(decode(scale, 130).db, 0);
  CHECK_EQ(decode(scale, 157).db, 10);  // +12 dB
  CHECK_EQ(decode(scale, 186).db, 30);  // +25 dB rounds up
  CHECK_EQ(decode(scale, 237).db, 50);
  CHECK_EQ(decode(scale, 255).db, 60);
  CHECK_EQ(smeterScaleDbRelS9(scale, 143), 6);  // halfway 130..157
}

TEST(ftdx101_s9_is_160) {
  constexpr uint8_t kSlots[] = {12, 13};  // FTDX-101D, FTDX-101MP
  for (uint8_t slot : kSlots) {
    const MeterScale& scale = scaleOfSlot(slot);
    CHECK_EQ(decode(scale, 0).s, 0);    // -60 dB, below S0
    CHECK_EQ(decode(scale, 136).s, 8);  // -6 dB
    CHECK_EQ(decode(scale, 160).s, 9);
    CHECK_EQ(decode(scale, 160).db, 0);
    CHECK_EQ(decode(scale, 255).db, 60);
  }
}

TEST(ft891_points) {
  const MeterScale& scale = scaleOfSlot(15);
  CHECK_EQ(decode(scale, 65).s, 5);
  CHECK_EQ(decode(scale, 130).s, 9);
  CHECK_EQ(decode(scale, 172).s, 9);
  CHECK_EQ(decode(scale, 172).db, 20);
}

TEST(ts480_matches_its_voice_guide) {
  // The manual's VOICE2 table for meter levels 0..20, as S units and dB over S9. The old generic
  // scale (S0 = 21) said S0 to all of them.
  constexpr Reading kVoiceGuide[] = {
      {0, 0}, {1, 0}, {1, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0},
      {9, 0}, {9, 10}, {9, 20}, {9, 20}, {9, 30}, {9, 40}, {9, 40}, {9, 50}, {9, 60}, {9, 60},
  };
  const MeterScale& scale = scaleOfSlot(7);
  for (int32_t raw = 0; raw <= 20; ++raw) {
    CHECK_EQ(decode(scale, raw).s, kVoiceGuide[raw].s);
    CHECK_EQ(decode(scale, raw).db, kVoiceGuide[raw].db);
  }
  CHECK_EQ(decode(scale, 30).db, 60);  // past the end
}

TEST(kx2_points) {
  const MeterScale& scale = scaleOfSlot(6);
  CHECK_EQ(decode(scale, 0).s, 0);
  CHECK_EQ(decode(scale, 1).s, 2);
  CHECK_EQ(decode(scale, 4).s, 7);
  CHECK_EQ(decode(scale, 5).s, 8);
  CHECK_EQ(decode(scale, 6).s, 9);
  CHECK_EQ(decode(scale, 6).db, 0);
  CHECK_EQ(decode(scale, 7).db, 10);
  CHECK_EQ(decode(scale, 8).db, 20);  // +15 dB, a half rounds up
  CHECK_EQ(decode(scale, 12).db, 40);
  CHECK_EQ(decode(scale, 15).db, 60);
}

// ---- SWR ----

namespace {

const MeterScale& swrScaleOfSlot(uint8_t slot) {
  static const MeterScale kNone{};
  const RadioProfile* p = profileForSlot(slot);
  CHECK(p != nullptr);
  return p ? p->swrScale : kNone;
}

uint16_t swrOfSlot(uint8_t slot, int32_t raw) { return swrTenthsFromRaw(swrScaleOfSlot(slot), raw); }

}  // namespace

TEST(swr_rounds_once_to_tenths) {
  // 1.0 at 0, 1.1 at 10: raw 5 is exactly 1.05 and rounds up; raw 4 (1.04) does not.
  constexpr MeterCalPoint kPoints[] = {{0, 100}, {10, 110}};
  const MeterScale scale{kPoints};
  CHECK_EQ(swrTenthsFromRaw(scale, 5), 11);
  CHECK_EQ(swrTenthsFromRaw(scale, 4), 10);
  CHECK_EQ(swrTenthsFromRaw(scale, -3), 10);
  CHECK_EQ(swrTenthsFromRaw(MeterScale{}, 50), 10);
}

TEST(every_swr_radio_has_a_scale) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (p.caps.getSwr) CHECK(!p.swrScale.empty());
  }
  CHECK(swrScaleOfSlot(2).empty());  // IC-706: no SWR meter
}

TEST(icom_swr_points) {
  constexpr uint8_t kSlots[] = {1, 3, 17, 18};  // IC-7300, its RS-232 slot, IC-705, IC-7760
  for (uint8_t slot : kSlots) {
    CHECK_EQ(swrOfSlot(slot, 0), 10);
    CHECK_EQ(swrOfSlot(slot, 48), 15);
    CHECK_EQ(swrOfSlot(slot, 100), 25);
    CHECK_EQ(swrOfSlot(slot, 120), 30);
    CHECK_EQ(swrOfSlot(slot, 255), 60);
  }
}

TEST(kx2_swr_is_in_tenths) {
  CHECK_EQ(swrOfSlot(6, 0), 10);  // nothing transmitted yet
  CHECK_EQ(swrOfSlot(6, 15), 15);
  CHECK_EQ(swrOfSlot(6, 30), 30);  // the old Icom curve said 1.3
  CHECK_EQ(swrOfSlot(6, 999), 999);
}

TEST(ts480_swr_points) {
  CHECK_EQ(swrOfSlot(7, 0), 10);
  CHECK_EQ(swrOfSlot(7, 4), 15);
  CHECK_EQ(swrOfSlot(7, 12), 30);  // the old Icom curve said 1.1
  CHECK_EQ(swrOfSlot(7, 16), 65);
  CHECK_EQ(swrOfSlot(7, 20), 100);
}

TEST(ftdx_swr_points) {
  constexpr uint8_t kSlots[] = {11, 12, 13};  // FTDX-10, FTDX-101D, FTDX-101MP
  for (uint8_t slot : kSlots) {
    CHECK_EQ(swrOfSlot(slot, 26), 12);
    CHECK_EQ(swrOfSlot(slot, 89), 20);
    CHECK_EQ(swrOfSlot(slot, 236), 50);
    CHECK_EQ(swrOfSlot(slot, 255), 250);
  }
}

TEST(ft891_swr_points) {
  CHECK_EQ(swrOfSlot(15, 0), 10);
  CHECK_EQ(swrOfSlot(15, 39), 14);  // 1.35 rounds up
  CHECK_EQ(swrOfSlot(15, 65), 15);
  CHECK_EQ(swrOfSlot(15, 242), 50);
}

TEST(ft8x7_swr_per_bar) {
  constexpr uint8_t kSlots[] = {8, 9, 10, 14};  // FT-817, FT-857, FT-897, FT-818
  for (uint8_t slot : kSlots) {
    CHECK_EQ(swrOfSlot(slot, 0), 10);
    CHECK_EQ(swrOfSlot(slot, 3), 21);  // 2.13
    CHECK_EQ(swrOfSlot(slot, 5), 37);
    CHECK_EQ(swrOfSlot(slot, 9), 90);
    CHECK_EQ(swrOfSlot(slot, 15), 100);
  }
}
