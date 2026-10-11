// Tests for the QMX S-meter scale (firmware/qmx_codec.*): 6 dB per S unit, S0 at -127 dBm,
// S9 at -73 dBm, as in the QMX operating manual's S-meter display sub-menu.
#include "qmx_codec.h"
#include "test_runner.h"

namespace {

struct Reading {
  int sUnits;
  int dbOverS9;
};

Reading decode(int32_t db) {
  uint8_t s = 0xFF;
  uint8_t over = 0xFF;
  qmxDecodeSMeter(db, s, over);
  return {s, over};
}

}  // namespace

TEST(dbm_reading_below_s9_is_whole_s_units) {
  CHECK_EQ(decode(-127).sUnits, 0);
  CHECK_EQ(decode(-91).sUnits, 6);
  CHECK_EQ(decode(-87).sUnits, 6);
  CHECK_EQ(decode(-74).sUnits, 8);
  CHECK_EQ(decode(-74).dbOverS9, 0);
}

TEST(dbm_reading_from_s9_is_tens_over_s9) {
  CHECK_EQ(decode(-73).sUnits, 9);
  CHECK_EQ(decode(-73).dbOverS9, 0);
  CHECK_EQ(decode(-64).dbOverS9, 0);
  CHECK_EQ(decode(-63).dbOverS9, 10);
  // The top of the QMX meter, S9+36.
  CHECK_EQ(decode(-37).sUnits, 9);
  CHECK_EQ(decode(-37).dbOverS9, 30);
}

TEST(dbm_reading_below_s0_is_s0) {
  CHECK_EQ(decode(-140).sUnits, 0);
  CHECK_EQ(decode(-140).dbOverS9, 0);
}

TEST(reading_from_zero_up_is_db_over_s0) {
  CHECK_EQ(decode(0).sUnits, 0);
  CHECK_EQ(decode(5).sUnits, 0);
  CHECK_EQ(decode(6).sUnits, 1);
  CHECK_EQ(decode(53).sUnits, 8);
  CHECK_EQ(decode(54).sUnits, 9);
  CHECK_EQ(decode(54).dbOverS9, 0);
  CHECK_EQ(decode(90).sUnits, 9);
  CHECK_EQ(decode(90).dbOverS9, 30);
}

TEST(db_over_s9_stops_at_sixty) {
  CHECK_EQ(decode(500).sUnits, 9);
  CHECK_EQ(decode(500).dbOverS9, 60);
}
