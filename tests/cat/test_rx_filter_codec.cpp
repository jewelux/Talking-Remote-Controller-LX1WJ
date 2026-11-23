// Tests for the receive filter encodings (firmware/rx_filter_codec.*).
#include "rx_filter_codec.h"
#include "test_runner.h"

namespace {

RxFilterScale scaleOf(RxFilterEncoding e, RxFilterLevel level, FilterModeGroup group) {
  RxFilterScale scale;
  CHECK(rxFilterScaleFor(e, level, group, scale));
  return scale;
}

bool hasScale(RxFilterEncoding e, RxFilterLevel level, FilterModeGroup group) {
  RxFilterScale scale;
  return rxFilterScaleFor(e, level, group, scale);
}

// The code a step reaches; kNoStep where there is no step.
constexpr int kNoStep = -1;
int stepOf(const RxFilterScale& scale, uint8_t code, int delta) {
  uint8_t out = 0;
  return rxFilterScaleStep(scale, code, delta, out) ? out : kNoStep;
}

}  // namespace

// ---- Mode groups ----

TEST(yaesu_mode_groups) {
  CHECK(yaesuFilterModeGroup('1') == FilterModeGroup::Ssb);
  CHECK(yaesuFilterModeGroup('2') == FilterModeGroup::Ssb);
  CHECK(yaesuFilterModeGroup('3') == FilterModeGroup::Cw);
  CHECK(yaesuFilterModeGroup('7') == FilterModeGroup::Cw);
  CHECK(yaesuFilterModeGroup('6') == FilterModeGroup::Rtty);
  CHECK(yaesuFilterModeGroup('9') == FilterModeGroup::Rtty);
  CHECK(yaesuFilterModeGroup('8') == FilterModeGroup::Data);
  CHECK(yaesuFilterModeGroup('C') == FilterModeGroup::Data);
  CHECK(yaesuFilterModeGroup('E') == FilterModeGroup::Data);
  CHECK(yaesuFilterModeGroup('5') == FilterModeGroup::Am);
  CHECK(yaesuFilterModeGroup('D') == FilterModeGroup::Am);
  CHECK(yaesuFilterModeGroup('4') == FilterModeGroup::Fm);
  CHECK(yaesuFilterModeGroup('B') == FilterModeGroup::Fm);
  // DATA-FM and DATA-FM-N have no menu of their own.
  CHECK(yaesuFilterModeGroup('A') == FilterModeGroup::Unknown);
  CHECK(yaesuFilterModeGroup('F') == FilterModeGroup::Unknown);
}

TEST(kenwood_and_icom_mode_groups) {
  CHECK(kenwoodFilterModeGroup('1') == FilterModeGroup::Ssb);
  CHECK(kenwoodFilterModeGroup('3') == FilterModeGroup::Cw);
  CHECK(kenwoodFilterModeGroup('4') == FilterModeGroup::Fm);
  CHECK(kenwoodFilterModeGroup('5') == FilterModeGroup::Am);
  CHECK(kenwoodFilterModeGroup('9') == FilterModeGroup::Rtty);
  CHECK(kenwoodFilterModeGroup('8') == FilterModeGroup::Unknown);
  CHECK(civFilterModeGroup(0x01) == FilterModeGroup::Ssb);
  CHECK(civFilterModeGroup(0x02) == FilterModeGroup::Am);
  CHECK(civFilterModeGroup(0x07) == FilterModeGroup::Cw);
  CHECK(civFilterModeGroup(0x08) == FilterModeGroup::Rtty);
  CHECK(civFilterModeGroup(0x05) == FilterModeGroup::Fm);
  CHECK(civFilterModeGroup(0x06) == FilterModeGroup::Unknown);
}

// ---- Widths ----

// The FTDX10 manual's Table 3: SSB 01..23, 2250/2400/2450 at 12..14; CW, RTTY, PSK 01..18.
TEST(ftdx10_width_table) {
  const RxFilterScale ssb = scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Ssb);
  CHECK(ssb.hasZero());
  CHECK_EQ(ssb.last, 23);
  CHECK_EQ(ssb.valueOf(1), 300);
  CHECK_EQ(ssb.valueOf(12), 2250);
  CHECK_EQ(ssb.valueOf(13), 2400);
  CHECK_EQ(ssb.valueOf(14), 2450);
  CHECK_EQ(ssb.valueOf(23), 4000);
  const RxFilterScale cw = scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Cw);
  CHECK_EQ(cw.last, 18);
  CHECK_EQ(cw.valueOf(1), 50);
  CHECK_EQ(cw.valueOf(11), 600);
  CHECK_EQ(cw.valueOf(18), 3000);
  // RTTY and the data modes use the CW column; AM and FM have no SH width.
  CHECK_EQ(scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Rtty).valueOf(10), 500);
  CHECK_EQ(scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Data).valueOf(10), 500);
  CHECK(!hasScale(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Am));
  CHECK(!hasScale(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Fm));
}

// The FTDX101MP manual (1912-D): SSB 12..14 = 2200/2300/2400, up to 21 = 3200.
TEST(ftdx101_width_table) {
  const RxFilterScale ssb = scaleOf(RxFilterEncoding::YaesuFtdx101, RxFilterLevel::Width, FilterModeGroup::Ssb);
  CHECK_EQ(ssb.last, 21);
  CHECK_EQ(ssb.valueOf(12), 2200);
  CHECK_EQ(ssb.valueOf(13), 2300);
  CHECK_EQ(ssb.valueOf(14), 2400);
  CHECK_EQ(ssb.valueOf(21), 3200);
  CHECK_EQ(scaleOf(RxFilterEncoding::YaesuFtdx101, RxFilterLevel::Width, FilterModeGroup::Cw).last, 18);
  // The FT-891 has none.
  CHECK(!hasScale(RxFilterEncoding::YaesuFt891, RxFilterLevel::Width, FilterModeGroup::Ssb));
}

// Icom 1A 03: 00..09 = 50..500 Hz, 10..40 = 600..3600 Hz; RTTY to 31; AM 00..49 = 200..10000 Hz.
TEST(icom_width_table) {
  const RxFilterScale ssb = scaleOf(RxFilterEncoding::Icom, RxFilterLevel::Width, FilterModeGroup::Ssb);
  CHECK(!ssb.hasZero());
  CHECK_EQ(ssb.first, 0);
  CHECK_EQ(ssb.last, 40);
  CHECK_EQ(ssb.valueOf(0), 50);
  CHECK_EQ(ssb.valueOf(9), 500);
  CHECK_EQ(ssb.valueOf(10), 600);
  CHECK_EQ(ssb.valueOf(31), 2700);
  CHECK_EQ(ssb.valueOf(40), 3600);
  CHECK_EQ(scaleOf(RxFilterEncoding::Icom, RxFilterLevel::Width, FilterModeGroup::Rtty).last, 31);
  const RxFilterScale am = scaleOf(RxFilterEncoding::Icom, RxFilterLevel::Width, FilterModeGroup::Am);
  CHECK_EQ(am.last, 49);
  CHECK_EQ(am.valueOf(0), 200);
  CHECK_EQ(am.valueOf(49), 10000);
  CHECK(!hasScale(RxFilterEncoding::Icom, RxFilterLevel::Width, FilterModeGroup::Fm));
  // Hamlib's formula agrees at every code.
  for (uint8_t code = 0; code <= 40; ++code) {
    const uint16_t hz = ssb.valueOf(code);
    CHECK_EQ(hz <= 500 ? (hz + 49) / 50 - 1 : (hz + 99) / 100 + 4, code);
  }
}

TEST(scale_nearest_and_zero) {
  const RxFilterScale ssb = scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Ssb);
  CHECK_EQ(rxFilterScaleCodeOf(ssb, 0), 0);        // NORMAL
  CHECK_EQ(rxFilterScaleCodeOf(ssb, 100), 1);      // not NORMAL: the narrowest width
  CHECK_EQ(rxFilterScaleCodeOf(ssb, 2400), 13);
  CHECK_EQ(rxFilterScaleCodeOf(ssb, 2425), 13);    // a tie goes to the lower width
  CHECK_EQ(rxFilterScaleCodeOf(ssb, 2430), 14);
  CHECK_EQ(rxFilterScaleCodeOf(ssb, 9000), 23);
  const RxFilterScale icom = scaleOf(RxFilterEncoding::Icom, RxFilterLevel::Width, FilterModeGroup::Ssb);
  CHECK_EQ(rxFilterScaleCodeOf(icom, 0), 0);       // no zero code: 50 Hz
  CHECK_EQ(rxFilterScaleCodeOf(icom, 2400), 28);
}

TEST(scale_steps) {
  const RxFilterScale ssb = scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Ssb);
  CHECK_EQ(stepOf(ssb, 13, 1), 14);
  CHECK_EQ(stepOf(ssb, 13, -2), 11);
  CHECK_EQ(stepOf(ssb, 1, -1), 1);   // down stops at the narrowest, not NORMAL
  CHECK_EQ(stepOf(ssb, 23, 5), 23);
  CHECK_EQ(stepOf(ssb, 0, 1), kNoStep);  // no step from NORMAL, it has no place among the widths
  CHECK_EQ(stepOf(ssb, 0, -1), kNoStep);
  CHECK(ssb.zero == RxFilterZero::Default);
  const RxFilterScale icom = scaleOf(RxFilterEncoding::Icom, RxFilterLevel::Width, FilterModeGroup::Ssb);
  CHECK_EQ(stepOf(icom, 0, -1), 0);
  CHECK_EQ(stepOf(icom, 40, 1), 40);
}

// ---- Cuts ----

// LCUT 00 off, 01..19 = 100..1000 Hz; HCUT 00 off, 01..67 = 700..4000 Hz; slopes 6 and 18 dB/oct.
TEST(yaesu_cut_scales) {
  const RxFilterEncoding radios[] = {RxFilterEncoding::YaesuFtdx10, RxFilterEncoding::YaesuFtdx101,
                                     RxFilterEncoding::YaesuFt891};
  for (RxFilterEncoding e : radios) {
    const RxFilterScale low = scaleOf(e, RxFilterLevel::LowCut, FilterModeGroup::Ssb);
    CHECK(low.hasZero());
    CHECK_EQ(low.last, 19);
    CHECK_EQ(low.valueOf(0), 0);
    CHECK_EQ(low.valueOf(1), 100);
    CHECK_EQ(low.valueOf(5), 300);
    CHECK_EQ(low.valueOf(19), 1000);
    const RxFilterScale high = scaleOf(e, RxFilterLevel::HighCut, FilterModeGroup::Cw);
    CHECK_EQ(high.last, 67);
    CHECK_EQ(high.valueOf(1), 700);
    CHECK_EQ(high.valueOf(67), 4000);
    const RxFilterScale slope = scaleOf(e, RxFilterLevel::LowCutSlope, FilterModeGroup::Am);
    CHECK(!slope.hasZero());
    CHECK_EQ(slope.valueOf(0), 6);
    CHECK_EQ(slope.valueOf(1), 18);
    CHECK_EQ(rxFilterScaleCodeOf(slope, 18), 1);
  }
  // A low cut steps up from off to its lowest frequency, a high cut down from off to its highest.
  const RxFilterScale low = scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::LowCut, FilterModeGroup::Ssb);
  CHECK_EQ(stepOf(low, 0, 1), 1);
  CHECK_EQ(stepOf(low, 0, 3), 3);
  CHECK_EQ(stepOf(low, 0, -1), 0);
  CHECK_EQ(stepOf(low, 1, -1), 1);   // down to off only by setting it
  const RxFilterScale high = scaleOf(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::HighCut, FilterModeGroup::Ssb);
  CHECK_EQ(stepOf(high, 0, -1), 67);
  CHECK_EQ(stepOf(high, 0, -3), 65);
  CHECK_EQ(stepOf(high, 0, -100), 1);
  CHECK_EQ(stepOf(high, 0, 1), 0);
  CHECK_EQ(stepOf(high, 67, 1), 67);  // up to off only by setting it
  CHECK_EQ(rxFilterScaleCodeOf(low, 320), 5);
  // The FT-891 has no FM cut, the FTDX radios do.
  CHECK(!hasScale(RxFilterEncoding::YaesuFt891, RxFilterLevel::LowCut, FilterModeGroup::Fm));
  CHECK(hasScale(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::LowCut, FilterModeGroup::Fm));
  CHECK(!hasScale(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::LowCut, FilterModeGroup::Unknown));
}

// The EX items of each manual.
TEST(yaesu_cut_menus) {
  char cmd[16];
  struct Row {
    RxFilterEncoding e;
    RxFilterLevel level;
    FilterModeGroup group;
    const char* menu;
  };
  const Row rows[] = {
      {RxFilterEncoding::YaesuFtdx10, RxFilterLevel::LowCut, FilterModeGroup::Ssb, "EX010107"},
      {RxFilterEncoding::YaesuFtdx10, RxFilterLevel::LowCutSlope, FilterModeGroup::Ssb, "EX010108"},
      {RxFilterEncoding::YaesuFtdx10, RxFilterLevel::HighCut, FilterModeGroup::Am, "EX010209"},
      {RxFilterEncoding::YaesuFtdx10, RxFilterLevel::HighCutSlope, FilterModeGroup::Fm, "EX010310"},
      {RxFilterEncoding::YaesuFtdx10, RxFilterLevel::LowCut, FilterModeGroup::Data, "EX010409"},
      {RxFilterEncoding::YaesuFtdx10, RxFilterLevel::HighCutSlope, FilterModeGroup::Rtty, "EX010512"},
      {RxFilterEncoding::YaesuFtdx10, RxFilterLevel::HighCut, FilterModeGroup::Cw, "EX020109"},
      {RxFilterEncoding::YaesuFtdx101, RxFilterLevel::LowCut, FilterModeGroup::Ssb, "EX010104"},
      {RxFilterEncoding::YaesuFtdx101, RxFilterLevel::HighCut, FilterModeGroup::Ssb, "EX010106"},
      {RxFilterEncoding::YaesuFtdx101, RxFilterLevel::LowCut, FilterModeGroup::Data, "EX010406"},
      {RxFilterEncoding::YaesuFtdx101, RxFilterLevel::HighCutSlope, FilterModeGroup::Rtty, "EX010509"},
      {RxFilterEncoding::YaesuFtdx101, RxFilterLevel::LowCutSlope, FilterModeGroup::Cw, "EX020105"},
      {RxFilterEncoding::YaesuFt891, RxFilterLevel::LowCut, FilterModeGroup::Am, "EX0601"},
      {RxFilterEncoding::YaesuFt891, RxFilterLevel::HighCutSlope, FilterModeGroup::Cw, "EX0704"},
      {RxFilterEncoding::YaesuFt891, RxFilterLevel::LowCut, FilterModeGroup::Data, "EX0805"},
      {RxFilterEncoding::YaesuFt891, RxFilterLevel::HighCut, FilterModeGroup::Data, "EX0807"},
      {RxFilterEncoding::YaesuFt891, RxFilterLevel::HighCut, FilterModeGroup::Rtty, "EX1003"},
      {RxFilterEncoding::YaesuFt891, RxFilterLevel::LowCutSlope, FilterModeGroup::Ssb, "EX1102"},
  };
  for (const Row& r : rows) {
    CHECK(yaesuCutMenu(r.e, r.level, r.group, cmd, sizeof(cmd)));
    CHECK_EQ(cmd, r.menu);
  }
  CHECK(!yaesuCutMenu(RxFilterEncoding::YaesuFt891, RxFilterLevel::LowCut, FilterModeGroup::Fm, cmd, sizeof(cmd)));
  CHECK(!yaesuCutMenu(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::Width, FilterModeGroup::Ssb, cmd, sizeof(cmd)));
  CHECK_EQ(yaesuCutDigits(RxFilterLevel::LowCut), 2);
  CHECK_EQ(yaesuCutDigits(RxFilterLevel::HighCutSlope), 1);
}

// TS-480 SL and SH (the PC command reference, Hamlib): SSB and FM, AM; not CW or FSK.
TEST(ts480_cut_scales) {
  const RxFilterScale low = scaleOf(RxFilterEncoding::KenwoodTs480, RxFilterLevel::LowCut, FilterModeGroup::Ssb);
  CHECK_EQ(low.last, 11);
  CHECK_EQ(low.valueOf(1), 50);
  CHECK_EQ(low.valueOf(11), 1000);
  const RxFilterScale high = scaleOf(RxFilterEncoding::KenwoodTs480, RxFilterLevel::HighCut, FilterModeGroup::Fm);
  CHECK(!high.hasZero());
  CHECK_EQ(high.valueOf(0), 1000);
  CHECK_EQ(high.valueOf(11), 3400);
  CHECK_EQ(high.valueOf(13), 5000);
  CHECK_EQ(scaleOf(RxFilterEncoding::KenwoodTs480, RxFilterLevel::LowCut, FilterModeGroup::Am).valueOf(3), 500);
  CHECK_EQ(scaleOf(RxFilterEncoding::KenwoodTs480, RxFilterLevel::HighCut, FilterModeGroup::Am).valueOf(0), 2500);
  CHECK(!hasScale(RxFilterEncoding::KenwoodTs480, RxFilterLevel::LowCut, FilterModeGroup::Cw));
  CHECK(!hasScale(RxFilterEncoding::KenwoodTs480, RxFilterLevel::HighCut, FilterModeGroup::Rtty));
  CHECK(!hasScale(RxFilterEncoding::KenwoodTs480, RxFilterLevel::LowCutSlope, FilterModeGroup::Ssb));
  uint8_t code = 0;
  CHECK(kenwoodCutDecode("07;", code) && code == 7);
  char cmd[8];
  CHECK(kenwoodCutEncode(RxFilterLevel::LowCut, 4, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "SL04;");
  CHECK(kenwoodCutEncode(RxFilterLevel::HighCut, 13, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "SH13;");
}

// ---- Spans ----

TEST(yaesu_spans) {
  RxFilterSpan span;
  CHECK(rxFilterSpanFor(RxFilterEncoding::YaesuFtdx10, RxFilterLevel::IfShift, span));
  CHECK(span.min == -1200 && span.max == 1200 && span.step == 20);
  CHECK(rxFilterSpanFor(RxFilterEncoding::YaesuFt891, RxFilterLevel::NotchFreq, span));
  CHECK(span.min == 10 && span.max == 3200 && span.step == 10);
  CHECK(rxFilterSpanFor(RxFilterEncoding::YaesuFtdx101, RxFilterLevel::ApfFreq, span));
  CHECK(span.min == -250 && span.max == 250 && span.step == 10);
  CHECK(!rxFilterSpanFor(RxFilterEncoding::Icom, RxFilterLevel::IfShift, span));
  CHECK(!rxFilterSpanFor(RxFilterEncoding::KenwoodTs480, RxFilterLevel::NotchFreq, span));
}

TEST(span_rounding) {
  const RxFilterSpan shift = {-1200, 1200, 20};
  CHECK_EQ(rxFilterSpanRound(shift, 405), 400);
  CHECK_EQ(rxFilterSpanRound(shift, 410), 420);
  CHECK_EQ(rxFilterSpanRound(shift, -410), -420);
  CHECK_EQ(rxFilterSpanRound(shift, -1300), -1200);
  CHECK_EQ(rxFilterSpanRound(shift, 1300), 1200);
  const RxFilterSpan notch = {10, 3200, 10};
  CHECK_EQ(rxFilterSpanRound(notch, 3), 10);
  CHECK_EQ(rxFilterSpanRound(notch, 1504), 1500);
}

// ---- Yaesu newcat strings ----

TEST(yaesu_if_shift) {
  int16_t hz = 0;
  CHECK(yaesuIfShiftDecode(RxFilterEncoding::YaesuFtdx10, "0+0400;", hz) && hz == 400);
  CHECK(yaesuIfShiftDecode(RxFilterEncoding::YaesuFtdx101, "0-1200", hz) && hz == -1200);
  CHECK(!yaesuIfShiftDecode(RxFilterEncoding::YaesuFtdx10, "0+1300;", hz));
  CHECK(!yaesuIfShiftDecode(RxFilterEncoding::YaesuFtdx10, "+0400;", hz));
  // FT-891: the on digit, off reads as 0; the older manual has none.
  CHECK(yaesuIfShiftDecode(RxFilterEncoding::YaesuFt891, "1-0200;", hz) && hz == -200);
  CHECK(yaesuIfShiftDecode(RxFilterEncoding::YaesuFt891, "0+0200;", hz) && hz == 0);
  CHECK(yaesuIfShiftDecode(RxFilterEncoding::YaesuFt891, "+0100;", hz) && hz == 100);
  char cmd[16];
  CHECK(yaesuIfShiftEncode(RxFilterEncoding::YaesuFtdx10, 400, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "IS00+0400;");
  CHECK(yaesuIfShiftEncode(RxFilterEncoding::YaesuFtdx101, -20, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "IS00-0020;");
  CHECK(yaesuIfShiftEncode(RxFilterEncoding::YaesuFtdx10, 0, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "IS00+0000;");
  CHECK(yaesuIfShiftEncode(RxFilterEncoding::YaesuFt891, -200, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "IS01-0200;");
  CHECK(yaesuIfShiftEncode(RxFilterEncoding::YaesuFt891, 0, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "IS00+0000;");
  CHECK(!yaesuIfShiftEncode(RxFilterEncoding::YaesuFtdx10, 410, cmd, sizeof(cmd)));
  CHECK(!yaesuIfShiftEncode(RxFilterEncoding::YaesuFtdx10, 1220, cmd, sizeof(cmd)));
  CHECK(!yaesuIfShiftEncode(RxFilterEncoding::Icom, 0, cmd, sizeof(cmd)));
}

TEST(yaesu_width_strings) {
  uint8_t code = 0;
  CHECK(yaesuWidthDecode("013;", code) && code == 13);
  CHECK(yaesuWidthDecode("000", code) && code == 0);
  CHECK(!yaesuWidthDecode("113;", code));
  char cmd[10];
  CHECK(yaesuWidthEncode(13, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "SH0013;");
  CHECK(yaesuWidthEncode(0, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "SH0000;");
}

TEST(yaesu_switch_strings) {
  CHECK_EQ(yaesuSwitchReadCommand(RxFilterSwitch::Narrow), "NA0;");
  CHECK_EQ(yaesuSwitchReadCommand(RxFilterSwitch::Contour), "CO00;");
  CHECK_EQ(yaesuSwitchReadCommand(RxFilterSwitch::Apf), "CO02;");
  bool on = false;
  CHECK(yaesuSwitchDecode(RxFilterSwitch::Narrow, "1;", on) && on);
  CHECK(yaesuSwitchDecode(RxFilterSwitch::Contour, "0000;", on) && !on);
  CHECK(yaesuSwitchDecode(RxFilterSwitch::Apf, "0001", on) && on);
  CHECK(!yaesuSwitchDecode(RxFilterSwitch::Apf, "0002;", on));
  CHECK(!yaesuSwitchDecode(RxFilterSwitch::Contour, "1;", on));
  char cmd[12];
  CHECK(yaesuSwitchEncode(RxFilterSwitch::Narrow, true, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "NA01;");
  CHECK(yaesuSwitchEncode(RxFilterSwitch::Contour, false, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "CO000000;");
  CHECK(yaesuSwitchEncode(RxFilterSwitch::Apf, true, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "CO020001;");
}

TEST(yaesu_frequency_strings) {
  int16_t hz = 0;
  CHECK(yaesuFreqDecode(RxFilterLevel::NotchFreq, "150;", hz) && hz == 1500);
  CHECK(!yaesuFreqDecode(RxFilterLevel::NotchFreq, "000;", hz));
  CHECK(!yaesuFreqDecode(RxFilterLevel::NotchFreq, "321;", hz));
  CHECK(yaesuFreqDecode(RxFilterLevel::ContourFreq, "1000;", hz) && hz == 1000);
  CHECK(yaesuFreqDecode(RxFilterLevel::ApfFreq, "0025;", hz) && hz == 0);
  CHECK(yaesuFreqDecode(RxFilterLevel::ApfFreq, "0000;", hz) && hz == -250);
  CHECK(yaesuFreqDecode(RxFilterLevel::ApfFreq, "0050;", hz) && hz == 250);
  CHECK(!yaesuFreqDecode(RxFilterLevel::ApfFreq, "0051;", hz));
  char cmd[12];
  CHECK(yaesuFreqEncode(RxFilterLevel::NotchFreq, 1500, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "BP01150;");
  CHECK(yaesuFreqEncode(RxFilterLevel::ContourFreq, 10, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "CO010010;");
  CHECK(yaesuFreqEncode(RxFilterLevel::ApfFreq, -100, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "CO030015;");
  CHECK(!yaesuFreqEncode(RxFilterLevel::NotchFreq, 1505, cmd, sizeof(cmd)));
  CHECK(!yaesuFreqEncode(RxFilterLevel::ApfFreq, 260, cmd, sizeof(cmd)));
  CHECK_EQ(yaesuFreqReadCommand(RxFilterLevel::NotchFreq), "BP01;");
  CHECK_EQ(yaesuFreqReplyPrefix(RxFilterLevel::ApfFreq), "CO03");
}

TEST(icom_width_bcd) {
  uint8_t code = 0;
  CHECK(civWidthCodeDecode(0x40, code) && code == 40);
  CHECK(civWidthCodeDecode(0x09, code) && code == 9);
  CHECK(!civWidthCodeDecode(0x4A, code));
  CHECK_EQ(civWidthCodeEncode(28), 0x28);
  CHECK_EQ(civWidthCodeEncode(49), 0x49);
}
