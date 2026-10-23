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

// ---- ASCII VOX delay in ms ----

TEST(vox_delay_ms_rounds_to_the_radio_step) {
  // TS-480: 0..3000 in 150 ms steps.
  CHECK_EQ(asciiVoxDelayMsRound(0, 0, 3000, 150), 0);
  CHECK_EQ(asciiVoxDelayMsRound(500, 0, 3000, 150), 450);
  CHECK_EQ(asciiVoxDelayMsRound(525, 0, 3000, 150), 600);
  CHECK_EQ(asciiVoxDelayMsRound(3100, 0, 3000, 150), 3000);
  // FT-891: 30..3000 in 10 ms steps.
  CHECK_EQ(asciiVoxDelayMsRound(0, 30, 3000, 10), 30);
  CHECK_EQ(asciiVoxDelayMsRound(505, 30, 3000, 10), 510);
  CHECK_EQ(asciiVoxDelayMsRound(9999, 30, 3000, 10), 3000);
}

TEST(vox_delay_ms_steps_by_at_least_100_ms) {
  CHECK_EQ(asciiVoxDelayMsStepBy(450, 1, 0, 3000, 150), 600);
  CHECK_EQ(asciiVoxDelayMsStepBy(450, -4, 0, 3000, 150), 0);
  CHECK_EQ(asciiVoxDelayMsStepBy(500, 1, 30, 3000, 10), 600);
  CHECK_EQ(asciiVoxDelayMsStepBy(30, -1, 30, 3000, 10), 30);
  CHECK_EQ(asciiVoxDelayMsStepBy(2950, 1, 30, 3000, 10), 3000);
  CHECK_EQ(asciiVoxDelayMsStepBy(100, 1000, 30, 3000, 10), 3000);
}

// ---- TS-480 processor levels ----

TEST(kenwood_proc_levels_decode) {
  uint8_t in = 0;
  uint8_t out = 0;
  CHECK(kenwoodProcLevelsDecode("050080;", in, out));
  CHECK_EQ(in, 50);
  CHECK_EQ(out, 80);
  CHECK(kenwoodProcLevelsDecode("100000", in, out));
  CHECK_EQ(in, 100);
  CHECK_EQ(out, 0);
  CHECK(!kenwoodProcLevelsDecode("101000;", in, out));
  CHECK(!kenwoodProcLevelsDecode("05008;", in, out));
  CHECK(!kenwoodProcLevelsDecode("0500800;", in, out));
  CHECK(!kenwoodProcLevelsDecode("05a080;", in, out));
}

// ---- AGC ----

TEST(ft8x7_agc_maps_both_ways_and_has_no_mid) {
  const YaesuAgc all[] = {YaesuAgc::Off, YaesuAgc::Fast, YaesuAgc::Slow, YaesuAgc::Auto};
  for (YaesuAgc agc : all) {
    YaesuAgc back = YaesuAgc::Off;
    CHECK(ft8x7AgcFromMode(ft8x7AgcToMode(agc), back));
    CHECK(back == agc);
  }
  CHECK(ft8x7AgcToMode(YaesuAgc::Auto) == AgcMode::Auto);
  YaesuAgc unused = YaesuAgc::Off;
  CHECK(!ft8x7AgcFromMode(AgcMode::Mid, unused));
}

TEST(civ_agc_is_fast_mid_slow) {
  AgcMode mode = AgcMode::Off;
  CHECK(civAgcDecode(0x01, mode) && mode == AgcMode::Fast);
  CHECK(civAgcDecode(0x02, mode) && mode == AgcMode::Mid);
  CHECK(civAgcDecode(0x03, mode) && mode == AgcMode::Slow);
  CHECK(!civAgcDecode(0x00, mode));
  CHECK(!civAgcDecode(0x04, mode));
  uint8_t raw = 0;
  CHECK(civAgcEncode(AgcMode::Mid, raw) && raw == 0x02);
  CHECK(!civAgcEncode(AgcMode::Off, raw));
  CHECK(!civAgcEncode(AgcMode::Auto, raw));
}

TEST(yaesu_agc_gt0) {
  CHECK_EQ(asciiAgcReadCommand(AgcEncoding::YaesuGt0), "GT0;");
  char cmd[8];
  CHECK(asciiAgcEncode(AgcEncoding::YaesuGt0, AgcMode::Off, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "GT00;");
  CHECK(asciiAgcEncode(AgcEncoding::YaesuGt0, AgcMode::Mid, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "GT02;");
  CHECK(asciiAgcEncode(AgcEncoding::YaesuGt0, AgcMode::Auto, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "GT04;");
  AgcMode mode = AgcMode::Off;
  CHECK(asciiAgcDecode(AgcEncoding::YaesuGt0, "03;", mode) && mode == AgcMode::Slow);
  // Auto answers 4, 5 or 6: auto-fast, auto-mid, auto-slow.
  CHECK(asciiAgcDecode(AgcEncoding::YaesuGt0, "05;", mode) && mode == AgcMode::Auto);
  CHECK(asciiAgcDecode(AgcEncoding::YaesuGt0, "06", mode) && mode == AgcMode::Auto);
  CHECK(!asciiAgcDecode(AgcEncoding::YaesuGt0, "07;", mode));
  CHECK(!asciiAgcDecode(AgcEncoding::YaesuGt0, "13;", mode));
  CHECK(!asciiAgcDecode(AgcEncoding::YaesuGt0, "0;", mode));
}

TEST(kenwood_agc_gt) {
  CHECK_EQ(asciiAgcReadCommand(AgcEncoding::KenwoodGt), "GT;");
  char cmd[8];
  CHECK(asciiAgcEncode(AgcEncoding::KenwoodGt, AgcMode::Slow, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "GT002;");
  CHECK(!asciiAgcEncode(AgcEncoding::KenwoodGt, AgcMode::Mid, cmd, sizeof(cmd)));
  CHECK(!asciiAgcEncode(AgcEncoding::KenwoodGt, AgcMode::Auto, cmd, sizeof(cmd)));
  AgcMode mode = AgcMode::Slow;
  CHECK(asciiAgcDecode(AgcEncoding::KenwoodGt, "000;", mode) && mode == AgcMode::Off);
  CHECK(asciiAgcDecode(AgcEncoding::KenwoodGt, "001;", mode) && mode == AgcMode::Fast);
  CHECK(!asciiAgcDecode(AgcEncoding::KenwoodGt, "003;", mode));
  // In FM the TS-480 answers three spaces.
  CHECK(!asciiAgcDecode(AgcEncoding::KenwoodGt, "   ;", mode));
}

TEST(elecraft_agc_gt) {
  char cmd[8];
  CHECK(asciiAgcEncode(AgcEncoding::ElecraftGt, AgcMode::Fast, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "GT002;");
  CHECK(asciiAgcEncode(AgcEncoding::ElecraftGt, AgcMode::Slow, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "GT004;");
  CHECK(!asciiAgcEncode(AgcEncoding::ElecraftGt, AgcMode::Off, cmd, sizeof(cmd)));
  AgcMode mode = AgcMode::Off;
  CHECK(asciiAgcDecode(AgcEncoding::ElecraftGt, "004;", mode) && mode == AgcMode::Slow);
  CHECK(!asciiAgcDecode(AgcEncoding::ElecraftGt, "000;", mode));
}

TEST(no_agc_encoding_does_nothing) {
  char cmd[8];
  AgcMode mode = AgcMode::Off;
  CHECK(asciiAgcReadCommand(AgcEncoding::None) == nullptr);
  CHECK(!asciiAgcEncode(AgcEncoding::None, AgcMode::Fast, cmd, sizeof(cmd)));
  CHECK(!asciiAgcDecode(AgcEncoding::None, "001;", mode));
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
