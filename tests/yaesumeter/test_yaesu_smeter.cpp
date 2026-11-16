// Tests for the Yaesu FTDX S-meter scales (firmware/yaesu_smeter.*), Hamlib's str_cal tables:
// FTDX10 and FT-891 S9 = 130, FTDX101 S9 = 160, all S9+60 dB = 255.
#include "yaesu_smeter.h"
#include "test_runner.h"

namespace {

void decode(const SMeterScale& scale, int32_t raw, uint8_t& s, uint8_t& db) {
  smeterScaleDecode(scale, raw, s, db);
}

}  // namespace

TEST(ftdx10_reference_points) {
  uint8_t s = 99, db = 99;
  decode(kYaesuSMeterScaleFtdx10, 0, s, db);
  CHECK(s == 0);
  CHECK(db == 0);
  decode(kYaesuSMeterScaleFtdx10, 130, s, db);
  CHECK(s == 9);
  CHECK(db == 0);
  decode(kYaesuSMeterScaleFtdx10, 255, s, db);
  CHECK(s == 9);
  CHECK(db == 60);
}

TEST(ftdx10_s_units_below_s9) {
  uint8_t s, db;
  decode(kYaesuSMeterScaleFtdx10, 26, s, db);  // S2
  CHECK(s == 2);
  decode(kYaesuSMeterScaleFtdx10, 51, s, db);  // S4
  CHECK(s == 4);
  decode(kYaesuSMeterScaleFtdx10, 81, s, db);  // S6
  CHECK(s == 6);
  decode(kYaesuSMeterScaleFtdx10, 105, s, db);  // S7.5: a half unit is not yet the next one
  CHECK(s == 7);
  CHECK(db == 0);
}

TEST(ftdx10_db_over_s9_in_tens) {
  uint8_t s, db;
  decode(kYaesuSMeterScaleFtdx10, 157, s, db);  // +12 dB
  CHECK(s == 9);
  CHECK(db == 10);
  decode(kYaesuSMeterScaleFtdx10, 186, s, db);  // +25 dB rounds to 30
  CHECK(db == 30);
  decode(kYaesuSMeterScaleFtdx10, 237, s, db);  // +50 dB
  CHECK(db == 50);
}

TEST(ftdx10_was_too_low_on_the_old_linear_scale) {
  // The generic scale (S0 = 21, S9 = 297) said S3 here; the radio shows S9.
  uint8_t s, db;
  decode(kYaesuSMeterScaleFtdx10, 130, s, db);
  CHECK(s == 9);
}

TEST(ftdx101_s9_is_160) {
  uint8_t s, db;
  decode(kYaesuSMeterScaleFtdx101, 136, s, db);  // -6 dB
  CHECK(s == 8);
  decode(kYaesuSMeterScaleFtdx101, 160, s, db);
  CHECK(s == 9);
  CHECK(db == 0);
  decode(kYaesuSMeterScaleFtdx101, 255, s, db);
  CHECK(db == 60);
}

TEST(ft891_points) {
  uint8_t s, db;
  decode(kYaesuSMeterScaleFt891, 65, s, db);  // S5
  CHECK(s == 5);
  decode(kYaesuSMeterScaleFt891, 172, s, db);  // +20
  CHECK(s == 9);
  CHECK(db == 20);
}

TEST(interpolates_between_points) {
  CHECK(smeterScaleDbRelS9(kYaesuSMeterScaleFtdx10, 143) == 6);  // halfway 130..157
  CHECK(smeterScaleDbRelS9(kYaesuSMeterScaleFtdx10, 13) == -48);  // halfway 0..26
}

TEST(out_of_range_is_clamped) {
  uint8_t s, db;
  decode(kYaesuSMeterScaleFtdx10, 300, s, db);
  CHECK(s == 9);
  CHECK(db == 60);
  decode(kYaesuSMeterScaleFtdx10, -5, s, db);
  CHECK(s == 0);
  CHECK(db == 0);
}
