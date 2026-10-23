// Tests for the CAT setting value encodings (firmware/cat_setting_codec.*).
#include "cat_setting_codec.h"
#include "test_runner.h"

// ---- CI-V levels ----

TEST(civ_level_percent_ends_and_middle) {
  CHECK_EQ(levelRawToPercent(0), 0);
  CHECK_EQ(levelRawToPercent(128), 50);
  CHECK_EQ(levelRawToPercent(255), 100);
  CHECK_EQ(levelRawToPercent(300), 100);
  CHECK_EQ(levelPercentToRaw(0), 0);
  CHECK_EQ(levelPercentToRaw(50), 128);
  CHECK_EQ(levelPercentToRaw(100), 255);
}

TEST(civ_level_percent_clamps) {
  CHECK_EQ(levelPercentToRaw(-5), 0);
  CHECK_EQ(levelPercentToRaw(150), 255);
}

TEST(civ_level_percent_round_trip) {
  for (int percent = 0; percent <= 100; ++percent) {
    CHECK_EQ(levelRawToPercent(levelPercentToRaw(percent)), percent);
  }
}

// ---- RF power ----

TEST(rf_power_scales_to_the_radio_maximum) {
  CHECK_EQ(rfPowerRawToWatts(255, 100), 100);
  CHECK_EQ(rfPowerRawToWatts(0, 100), 0);
  CHECK_EQ(rfPowerRawToWatts(255, 10), 10);
  CHECK_EQ(rfPowerRawToWatts(128, 10), 5);
  CHECK_EQ(rfPowerRawToWatts(255, 200), 200);
  CHECK_EQ(rfPowerWattsToRaw(10, 10), 255);
  CHECK_EQ(rfPowerWattsToRaw(5, 10), 128);
}

TEST(rf_power_zero_maximum_counts_as_100_watts) {
  CHECK_EQ(rfPowerRawToWatts(255, 0), 100);
  CHECK_EQ(rfPowerWattsToRaw(100, 0), 255);
}

TEST(rf_power_watts_clamp_to_the_maximum) {
  CHECK_EQ(rfPowerWattsToRaw(-1, 10), 0);
  CHECK_EQ(rfPowerWattsToRaw(50, 10), 255);
}

TEST(rf_power_watts_round_trip) {
  const uint16_t maxima[] = {10, 100, 200};
  for (uint16_t maxWatts : maxima) {
    for (int watts = 0; watts <= maxWatts; ++watts) {
      CHECK_EQ(rfPowerRawToWatts(rfPowerWattsToRaw(watts, maxWatts), maxWatts), watts);
    }
  }
}

// ---- Yaesu ASCII VOX delay ----

TEST(yaesu_ascii_vox_delay_steps) {
  CHECK_EQ(yaesuAsciiVoxDelayStepMs(0), 30);
  CHECK_EQ(yaesuAsciiVoxDelayStepMs(5), 250);
  CHECK_EQ(yaesuAsciiVoxDelayStepMs(6), 300);
  CHECK_EQ(yaesuAsciiVoxDelayStepMs(33), 3000);
  CHECK_EQ(yaesuAsciiVoxDelayStepMs(40), 3000);
}

TEST(yaesu_ascii_vox_delay_nearest_step) {
  CHECK_EQ(yaesuAsciiVoxDelayNearestStep(0), 0);
  CHECK_EQ(yaesuAsciiVoxDelayNearestStep(30), 0);
  CHECK_EQ(yaesuAsciiVoxDelayNearestStep(75), 1);  // tie between 50 and 100: the lower
  CHECK_EQ(yaesuAsciiVoxDelayNearestStep(260), 5);
  CHECK_EQ(yaesuAsciiVoxDelayNearestStep(1000), 13);
  CHECK_EQ(yaesuAsciiVoxDelayNearestStep(9999), 33);
}

TEST(yaesu_ascii_vox_delay_step_by) {
  CHECK_EQ(yaesuAsciiVoxDelayStepBy(250, 1), 300);
  CHECK_EQ(yaesuAsciiVoxDelayStepBy(300, -1), 250);
  CHECK_EQ(yaesuAsciiVoxDelayStepBy(30, -1), 30);
  CHECK_EQ(yaesuAsciiVoxDelayStepBy(3000, 1), 3000);
  CHECK_EQ(yaesuAsciiVoxDelayStepBy(30, 1000), 3000);
  CHECK_EQ(yaesuAsciiVoxDelayStepBy(3000, -1000), 30);
}

// ---- ASCII lock ----

TEST(lock_digit_decodes_with_the_profile_off_digits) {
  bool on = false;
  CHECK(asciiLockDigitDecode('1', "0", on) && on);
  CHECK(asciiLockDigitDecode('0', "0", on) && !on);
  // FTDX101: LK7 locks main and sub; 0 and 4 are unlocked.
  CHECK(asciiLockDigitDecode('7', "04", on) && on);
  CHECK(asciiLockDigitDecode('4', "04", on) && !on);
  CHECK(asciiLockDigitDecode('0', "04", on) && !on);
  CHECK(!asciiLockDigitDecode(';', "0", on));
}
