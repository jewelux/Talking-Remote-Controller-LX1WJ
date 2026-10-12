// Tests for the NanoVNA shell codec (firmware/nanovna_codec.*): the reply reader, scan lines,
// SWR, the sweep summary, the band presets and the command lines.
#include <cmath>
#include <string>
#include <vector>

#include "nanovna_codec.h"
#include "test_runner.h"

namespace {

// What the reader makes of text: each line as itself, each prompt as "<prompt>".
std::vector<std::string> readAll(NanoVnaReplyReader& reader, const char* text) {
  std::vector<std::string> events;
  for (const char* p = text; *p; ++p) {
    switch (reader.feed(*p)) {
      case NanoVnaReplyReader::Event::Line: events.push_back(reader.line()); break;
      case NanoVnaReplyReader::Event::Prompt: events.push_back("<prompt>"); break;
      case NanoVnaReplyReader::Event::None: break;
    }
  }
  return events;
}

std::string swrText(float swr, bool* over = nullptr) {
  char text[16];
  const bool isOver = nanoVnaFormatSwr(swr, text, sizeof(text));
  if (over) *over = isOver;
  return text;
}

}  // namespace

TEST(reader_splits_echo_reply_and_prompt) {
  NanoVnaReplyReader reader;
  const auto events = readAll(reader, "version\r\n1.0.3\r\nch> ");
  CHECK_EQ(events.size(), 3u);
  CHECK_EQ(events[0], std::string("version"));
  CHECK_EQ(events[1], std::string("1.0.3"));
  CHECK_EQ(events[2], std::string("<prompt>"));
}

TEST(reader_answers_a_bare_cr_with_an_empty_line_and_the_prompt) {
  NanoVnaReplyReader reader;
  const auto events = readAll(reader, "\r\nch> ");
  CHECK_EQ(events.size(), 2u);
  CHECK_EQ(events[0], std::string(""));
  CHECK_EQ(events[1], std::string("<prompt>"));
}

TEST(reader_keeps_its_place_across_feeds) {
  NanoVnaReplyReader reader;
  auto events = readAll(reader, "3500000 0.1 ");
  CHECK(events.empty());
  events = readAll(reader, "-0.2\r\nch");
  CHECK_EQ(events.size(), 1u);
  CHECK_EQ(events[0], std::string("3500000 0.1 -0.2"));
  events = readAll(reader, "> ");
  CHECK_EQ(events.size(), 1u);
  CHECK_EQ(events[0], std::string("<prompt>"));
}

TEST(reader_finds_no_prompt_inside_a_line) {
  NanoVnaReplyReader reader;
  const auto events = readAll(reader, "xch> \r\n");
  CHECK_EQ(events.size(), 1u);
  CHECK_EQ(events[0], std::string("xch> "));
}

TEST(reader_cuts_long_lines_and_says_so) {
  NanoVnaReplyReader reader;
  const std::string longLine(NanoVnaReplyReader::kMaxLine + 10, 'a');
  const auto events = readAll(reader, (longLine + "\r\n").c_str());
  CHECK_EQ(events.size(), 1u);
  CHECK_EQ(events[0].size(), NanoVnaReplyReader::kMaxLine);
  CHECK(reader.lineTruncated());
  (void)readAll(reader, "ok\r\n");
  CHECK(!reader.lineTruncated());
}

TEST(echo_matches_the_command_with_or_without_trailing_spaces) {
  CHECK(nanoVnaIsEcho("scan 1 2 101 3", "scan 1 2 101 3"));
  CHECK(nanoVnaIsEcho("scan 1 2 101 3 ", "scan 1 2 101 3"));
  CHECK(!nanoVnaIsEcho("scan 1 2 101", "scan 1 2 101 3"));
  CHECK(!nanoVnaIsEcho("1.0.3", "version"));
}

TEST(error_replies_are_recognised) {
  CHECK(nanoVnaReplyIsError("usage: recall 0..6", "recall 9"));
  CHECK(nanoVnaReplyIsError("Err, default load", "recall 3"));
  CHECK(nanoVnaReplyIsError("frequency range is invalid", "scan 2 1 101 3"));
  CHECK(nanoVnaReplyIsError("sweep points exceeds range 101", "scan 1 2 401 3"));
  CHECK(nanoVnaReplyIsError("foo?", "foo"));
  CHECK(nanoVnaReplyIsError("foo bar?", "foo bar"));
  CHECK(!nanoVnaReplyIsError("1.0.3", "version"));
  CHECK(!nanoVnaReplyIsError("3500000 0.1 -0.2", "scan 3500000 3800000 101 3"));
  CHECK(!nanoVnaReplyIsError("is it?", "version"));
}

TEST(scan_lines_parse_with_or_without_a_trailing_space) {
  NanoVnaPoint p{};
  CHECK(nanoVnaParseScanLine("3500000 0.123456 -0.654321", p));
  CHECK_EQ(p.hz, 3500000u);
  CHECK(std::fabs(p.re - 0.123456f) < 1e-6f);
  CHECK(std::fabs(p.im + 0.654321f) < 1e-6f);
  CHECK(nanoVnaParseScanLine("2999000000 -1.5e-02 0.000000 ", p));
  CHECK_EQ(p.hz, 2999000000u);
  CHECK(std::fabs(p.re + 0.015f) < 1e-7f);
}

TEST(other_lines_are_not_scan_lines) {
  NanoVnaPoint p{};
  CHECK(!nanoVnaParseScanLine("", p));
  CHECK(!nanoVnaParseScanLine("scan 3500000 3800000 101 3", p));
  CHECK(!nanoVnaParseScanLine("3500000 0.1", p));
  CHECK(!nanoVnaParseScanLine("3500000 0.1 0.2 0.3", p));
  CHECK(!nanoVnaParseScanLine("3500000 0.1 nan", p));
  CHECK(!nanoVnaParseScanLine("-3500000 0.1 0.2", p));
  CHECK(!nanoVnaParseScanLine("99999999999 0.1 0.2", p));
}

TEST(swr_from_s11) {
  CHECK(std::fabs(nanoVnaSwr(0.0f, 0.0f) - 1.0f) < 1e-6f);
  CHECK(std::fabs(nanoVnaSwr(1.0f / 3.0f, 0.0f) - 2.0f) < 1e-5f);
  CHECK(std::fabs(nanoVnaSwr(0.0f, -0.5f) - 3.0f) < 1e-5f);
  CHECK(std::isinf(nanoVnaSwr(1.0f, 0.0f)));
  CHECK(std::isinf(nanoVnaSwr(0.8f, 0.8f)));
}

TEST(summary_keeps_best_and_worst_and_the_first_on_a_tie) {
  NanoVnaSweepSummary s;
  s.add(3500000, 2.5f);
  s.add(3600000, 1.2f);
  s.add(3700000, 1.2f);
  s.add(3800000, 4.0f);
  s.add(3900000, 4.0f);
  CHECK_EQ((unsigned)s.points, 5u);
  CHECK_EQ(s.bestHz, 3600000u);
  CHECK(s.bestSwr == 1.2f);
  CHECK_EQ(s.worstHz, 3800000u);
  CHECK(s.worstSwr == 4.0f);
}

TEST(summary_takes_infinity_as_the_worst) {
  NanoVnaSweepSummary s;
  s.add(1000000, INFINITY);
  s.add(2000000, 3.0f);
  CHECK_EQ(s.bestHz, 2000000u);
  CHECK_EQ(s.worstHz, 1000000u);
  CHECK(std::isinf(s.worstSwr));
}

TEST(swr_text_has_two_decimals_below_ten_one_below_a_hundred) {
  bool over = true;
  CHECK_EQ(swrText(1.0f, &over), std::string("1.00"));
  CHECK(!over);
  CHECK_EQ(swrText(1.214f), std::string("1.21"));
  CHECK_EQ(swrText(9.994f), std::string("9.99"));
  CHECK_EQ(swrText(9.996f), std::string("10.0"));
  CHECK_EQ(swrText(12.46f), std::string("12.5"));
  CHECK_EQ(swrText(99.94f, &over), std::string("99.9"));
  CHECK(!over);
}

TEST(swr_from_99_95_and_infinity_is_99_plus) {
  bool over = false;
  CHECK_EQ(swrText(99.95f, &over), std::string("99"));
  CHECK(over);
  over = false;
  CHECK_EQ(swrText(INFINITY, &over), std::string("99"));
  CHECK(over);
}

TEST(bands_run_from_160_m_to_70_cm_in_order) {
  CHECK_EQ(nanoVnaBandCount(), 13u);
  CHECK_EQ((unsigned)nanoVnaBandAt(0).wavelength, 160u);
  CHECK_EQ(nanoVnaBandAt(1).startHz, 3500000u);
  CHECK_EQ(nanoVnaBandAt(1).stopHz, 3800000u);
  const NanoVnaBand& last = nanoVnaBandAt(nanoVnaBandCount() - 1);
  CHECK_EQ((unsigned)last.wavelength, 70u);
  CHECK(last.unit == NanoVnaBandUnit::Centimeters);
  for (size_t i = 0; i < nanoVnaBandCount(); ++i) {
    const NanoVnaBand& b = nanoVnaBandAt(i);
    CHECK(nanoVnaRangeValid(b.startHz, b.stopHz));
    if (i > 0) CHECK(b.startHz > nanoVnaBandAt(i - 1).stopHz);
  }
}

TEST(band_matching_needs_both_edges) {
  CHECK_EQ(nanoVnaBandMatching(3500000, 3800000), 1);
  CHECK_EQ(nanoVnaBandMatching(3500000, 3700000), -1);
  CHECK_EQ(nanoVnaBandMatching(430000000, 440000000), 12);
}

TEST(adjacent_band_steps_and_wraps) {
  CHECK_EQ(nanoVnaAdjacentBand(3500000, 3800000, 1), 2u);
  CHECK_EQ(nanoVnaAdjacentBand(3500000, 3800000, -1), 0u);
  CHECK_EQ(nanoVnaAdjacentBand(430000000, 440000000, 1), 0u);
  CHECK_EQ(nanoVnaAdjacentBand(1810000, 2000000, -1), 12u);
}

TEST(adjacent_band_from_a_range_of_its_own_goes_by_its_start) {
  // 3.6-3.7 MHz lies in 80 m: next is 60 m, previous is 80 m itself.
  CHECK_EQ(nanoVnaAdjacentBand(3600000, 3700000, 1), 2u);
  CHECK_EQ(nanoVnaAdjacentBand(3600000, 3700000, -1), 1u);
  CHECK_EQ(nanoVnaAdjacentBand(100000, 200000, 1), 0u);
  CHECK_EQ(nanoVnaAdjacentBand(100000, 200000, -1), 12u);
  CHECK_EQ(nanoVnaAdjacentBand(1000000000, 1100000000, 1), 0u);
  CHECK_EQ(nanoVnaAdjacentBand(1000000000, 1100000000, -1), 12u);
}

TEST(range_check) {
  CHECK(nanoVnaRangeValid(3500000, 3800000));
  CHECK(!nanoVnaRangeValid(3800000, 3500000));
  CHECK(!nanoVnaRangeValid(3500000, 3500000));
  CHECK(!nanoVnaRangeValid(9999, 3500000));
  CHECK(!nanoVnaRangeValid(3500000, 3000000001ull));
  CHECK(nanoVnaFrequencyValid(kNanoVnaMaxHz));
}

TEST(command_lines) {
  char line[48];
  CHECK(nanoVnaScanCommand(3500000, 3800000, line, sizeof(line)));
  CHECK_EQ(line, "scan 3500000 3800000 101 3");
  CHECK(nanoVnaSweepCommand(430000000, 440000000, line, sizeof(line)));
  CHECK_EQ(line, "sweep 430000000 440000000");
  CHECK(nanoVnaRecallCommand(6, line, sizeof(line)));
  CHECK_EQ(line, "recall 6");
  char tiny[8];
  CHECK(!nanoVnaScanCommand(3500000, 3800000, tiny, sizeof(tiny)));
}

TEST(version_number_prefers_a_dotted_one) {
  char v[16];
  CHECK(nanoVnaVersionNumber("1.2.52", v, sizeof(v)));
  CHECK_EQ(v, "1.2.52");
  CHECK(nanoVnaVersionNumber("NanoVNA-F_V2 v1.0.3", v, sizeof(v)));
  CHECK_EQ(v, "1.0.3");
  CHECK(nanoVnaVersionNumber("Version: 0.3.0.", v, sizeof(v)));
  CHECK_EQ(v, "0.3.0");
  CHECK(nanoVnaVersionNumber("build 42", v, sizeof(v)));
  CHECK_EQ(v, "42");
  CHECK(!nanoVnaVersionNumber("NanoVNA", v, sizeof(v)));
  CHECK(!nanoVnaVersionNumber("x1 y2", v, sizeof(v)));
}

TEST(battery_reply_in_millivolts) {
  uint32_t mv = 0;
  CHECK(nanoVnaParseMillivolts("4256 mV", mv));
  CHECK_EQ(mv, 4256u);
  CHECK(nanoVnaParseMillivolts(" 3700mV ", mv));
  CHECK_EQ(mv, 3700u);
  CHECK(nanoVnaParseMillivolts("4100", mv));
  CHECK_EQ(mv, 4100u);
  CHECK(!nanoVnaParseMillivolts("", mv));
  CHECK(!nanoVnaParseMillivolts("vbat", mv));
  CHECK(!nanoVnaParseMillivolts("4256 V", mv));
  CHECK(!nanoVnaParseMillivolts("4256 mV low", mv));
  CHECK(!nanoVnaParseMillivolts("1234567 mV", mv));
}

TEST(volts_to_10_mv) {
  char v[12];
  nanoVnaFormatVolts(4256, v, sizeof(v));
  CHECK_EQ(v, "4.26");
  nanoVnaFormatVolts(4000, v, sizeof(v));
  CHECK_EQ(v, "4.00");
  nanoVnaFormatVolts(3994, v, sizeof(v));
  CHECK_EQ(v, "3.99");
  nanoVnaFormatVolts(3995, v, sizeof(v));
  CHECK_EQ(v, "4.00");
  nanoVnaFormatVolts(50, v, sizeof(v));
  CHECK_EQ(v, "0.05");
}

TEST(serial_number_is_letters_and_digits_in_upper_case) {
  char sn[24];
  CHECK(nanoVnaSerialNumber("5736383845107C54", sn, sizeof(sn)));
  CHECK_EQ(sn, "5736383845107C54");
  CHECK(nanoVnaSerialNumber(" 5736383845107c54 ", sn, sizeof(sn)));
  CHECK_EQ(sn, "5736383845107C54");
  CHECK(!nanoVnaSerialNumber("", sn, sizeof(sn)));
  CHECK(!nanoVnaSerialNumber("SN?", sn, sizeof(sn)));
  CHECK(!nanoVnaSerialNumber("57 36", sn, sizeof(sn)));
  char tiny[8];
  CHECK(!nanoVnaSerialNumber("5736383845107C54", tiny, sizeof(tiny)));
}
