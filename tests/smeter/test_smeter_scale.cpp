// Tests for the S-meter scales (firmware/smeter_scale.*) and the tables the built-in profiles
// carry (firmware/radio_profile_table.cpp): FTDX10 and FT-891 S9 = 130, FTDX101 S9 = 160,
// all S9+60 dB = 255, as in Hamlib.
#include "radio_profile_table.h"
#include "smeter_scale.h"
#include "test_runner.h"

namespace {

struct Reading {
  int s = -1;
  int db = -1;
};

Reading decode(const SMeterScale& scale, int32_t raw) {
  uint8_t s = 99, db = 99;
  smeterScaleDecode(scale, raw, s, db);
  return {s, db};
}

const SMeterScale& scaleOfSlot(uint8_t slot) {
  static const SMeterScale kNone{};
  const RadioProfile* p = profileForSlot(slot);
  CHECK(p != nullptr);
  return p ? p->smeterScale : kNone;
}

// S0 at 0, S9 at 90, S9+60 dB at 150: 10 raw steps per 6 dB below S9, per 10 dB above.
constexpr SMeterCalPoint kLinearPoints[] = {{0, -54}, {90, 0}, {150, 60}};
const SMeterScale kLinear{kLinearPoints};

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

TEST(out_of_range_is_clamped) {
  CHECK_EQ(decode(kLinear, 300).s, 9);
  CHECK_EQ(decode(kLinear, 300).db, 60);
  CHECK_EQ(decode(kLinear, -5).s, 0);
  CHECK_EQ(decode(kLinear, -5).db, 0);
}

// ---- The profiles' tables ----

TEST(ftdx_family_profiles_carry_their_scale) {
  CHECK(!scaleOfSlot(11).empty());  // FTDX-10
  CHECK(!scaleOfSlot(12).empty());  // FTDX-101D
  CHECK(!scaleOfSlot(13).empty());  // FTDX-101MP
  CHECK(!scaleOfSlot(15).empty());  // FT-891
  CHECK(scaleOfSlot(7).empty());    // TS-480: generic scale
  CHECK(scaleOfSlot(1).empty());    // IC-7300: Icom decoder
}

TEST(every_profile_scale_ascends) {
  for (size_t i = 0; i < profileCount(); ++i) CHECK(profileAt(i).smeterScale.ascends());
}

TEST(ftdx10_reference_points) {
  const SMeterScale& scale = scaleOfSlot(11);
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
    const SMeterScale& scale = scaleOfSlot(slot);
    CHECK_EQ(decode(scale, 136).s, 8);  // -6 dB
    CHECK_EQ(decode(scale, 160).s, 9);
    CHECK_EQ(decode(scale, 160).db, 0);
    CHECK_EQ(decode(scale, 255).db, 60);
  }
}

TEST(ft891_points) {
  const SMeterScale& scale = scaleOfSlot(15);
  CHECK_EQ(decode(scale, 65).s, 5);
  CHECK_EQ(decode(scale, 130).s, 9);
  CHECK_EQ(decode(scale, 172).s, 9);
  CHECK_EQ(decode(scale, 172).db, 20);
}
