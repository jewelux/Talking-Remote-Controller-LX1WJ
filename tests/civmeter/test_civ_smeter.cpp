// Tests for the Icom CI-V S-meter scale (firmware/civ_smeter.*): 0 = S0, 120 = S9,
// 241 = S9+60 dB, as in the IC-7300 CI-V reference.
#include "civ_smeter.h"
#include "test_runner.h"

namespace {

void decode(int32_t raw, uint8_t& s, uint8_t& db) { civDecodeSMeter(raw, s, db); }

}  // namespace

TEST(zero_is_s0) {
  uint8_t s = 99, db = 99;
  decode(0, s, db);
  CHECK(s == 0);
  CHECK(db == 0);
}

TEST(reference_points) {
  uint8_t s, db;
  decode(120, s, db);
  CHECK(s == 9);
  CHECK(db == 0);
  decode(241, s, db);
  CHECK(s == 9);
  CHECK(db == 60);
}

TEST(s_units_below_s9) {
  uint8_t s, db;
  decode(13, s, db);  // 13.3 raw per S unit
  CHECK(s == 1);
  decode(67, s, db);  // S5 = 66.7
  CHECK(s == 5);
  decode(107, s, db);  // S8 = 106.7
  CHECK(s == 8);
  CHECK(db == 0);
}

TEST(db_over_s9_in_tens) {
  uint8_t s, db;
  decode(140, s, db);  // about S9+10 dB: was spoken "4" before this scale
  CHECK(s == 9);
  CHECK(db == 10);
  decode(160, s, db);  // S9+20
  CHECK(db == 20);
  decode(201, s, db);  // S9+40
  CHECK(db == 40);
  decode(125, s, db);  // just over S9 rounds to S9
  CHECK(db == 0);
}

TEST(out_of_range_is_clamped) {
  uint8_t s, db;
  decode(255, s, db);
  CHECK(s == 9);
  CHECK(db == 60);
  decode(-5, s, db);
  CHECK(s == 0);
  CHECK(db == 0);
}
