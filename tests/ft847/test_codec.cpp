// Tests for the FT-847 CAT frame fields (firmware/ft847_codec.*). Expected frames are the
// examples from the FT-847 operating manual, pp. 92-93.
#include "ft847_codec.h"
#include "test_runner.h"

#include <string.h>

namespace {

bool sameBytes(const uint8_t *a, const uint8_t *b, size_t n) { return memcmp(a, b, n) == 0; }

}  // namespace

TEST(cat_on_and_off_frames) {
  uint8_t frame[5];
  ft847BuildFrame(FT847_OP_CAT_ON, frame);
  const uint8_t on[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(sameBytes(frame, on, 5));
  ft847BuildFrame(FT847_OP_CAT_OFF, frame);
  const uint8_t off[5] = {0x00, 0x00, 0x00, 0x00, 0x80};
  CHECK(sameBytes(frame, off, 5));
}

TEST(set_freq_frame_matches_the_manual_example) {
  // Manual p. 93: main VFO to 439.70 MHz = 43 97 00 00 01.
  uint8_t frame[5];
  ft847BuildSetFreqFrame(439700000ULL, frame);
  const uint8_t expected[5] = {0x43, 0x97, 0x00, 0x00, 0x01};
  CHECK(sameBytes(frame, expected, 5));
}

TEST(set_freq_frame_on_hf) {
  uint8_t frame[5];
  ft847BuildSetFreqFrame(14250000ULL, frame);
  const uint8_t expected[5] = {0x01, 0x42, 0x50, 0x00, 0x01};
  CHECK(sameBytes(frame, expected, 5));
}

TEST(freq_drops_the_1_hz_digit) {
  uint8_t out[4];
  ft847EncodeFreqHz(7100009ULL, out);
  CHECK_EQ(ft847DecodeFreqHz(out), 7100000ULL);
}

TEST(freq_decodes_the_manual_status_example) {
  // Manual p. 92, note 3: 43 97 00 00 = 439.700 MHz.
  const uint8_t data[4] = {0x43, 0x97, 0x00, 0x00};
  CHECK_EQ(ft847DecodeFreqHz(data), 439700000ULL);
  CHECK(ft847FreqFieldValid(data));
}

TEST(freq_round_trip_across_the_bands) {
  const uint64_t samples[] = {100000ULL, 1838010ULL, 50313000ULL, 145500000ULL, 435000000ULL, 512000000ULL};
  for (uint64_t hz : samples) {
    uint8_t out[4];
    ft847EncodeFreqHz(hz, out);
    CHECK_EQ(ft847DecodeFreqHz(out), hz);
    CHECK(ft847FreqFieldValid(out));
  }
}

TEST(freq_field_rejects_non_bcd_and_out_of_range) {
  const uint8_t notBcd[4] = {0x01, 0x4A, 0x00, 0x00};
  CHECK(!ft847FreqFieldValid(notBcd));
  const uint8_t tooLow[4] = {0x00, 0x00, 0x99, 0x99};  // 99.99 kHz
  CHECK(!ft847FreqFieldValid(tooLow));
  const uint8_t tooHigh[4] = {0x51, 0x20, 0x00, 0x01};  // 512.0001 MHz
  CHECK(!ft847FreqFieldValid(tooHigh));
}

TEST(set_mode_frames) {
  uint8_t frame[5];
  ft847BuildSetModeFrame(FT847_MODE_USB, frame);
  const uint8_t usb[5] = {0x01, 0x00, 0x00, 0x00, 0x07};
  CHECK(sameBytes(frame, usb, 5));
  ft847BuildSetModeFrame(0x88, frame);
  const uint8_t fmn[5] = {0x88, 0x00, 0x00, 0x00, 0x07};
  CHECK(sameBytes(frame, fmn, 5));
}

TEST(mode_narrow_flag) {
  CHECK_EQ(ft847ModeBase(0x82), FT847_MODE_CW);
  CHECK_EQ(ft847ModeBase(0x83), FT847_MODE_CWR);
  CHECK_EQ(ft847ModeBase(0x84), FT847_MODE_AM);
  CHECK_EQ(ft847ModeBase(0x88), FT847_MODE_FM);
  CHECK(ft847ModeNarrow(0x84));
  CHECK(!ft847ModeNarrow(0x04));
}

TEST(mode_bases_the_radio_reports) {
  const uint8_t known[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x08};
  for (uint8_t m : known) CHECK(ft847ModeBaseKnown(m));
  const uint8_t unknown[] = {0x05, 0x06, 0x07, 0x09, 0x0A, 0x0C, 0x7F};
  for (uint8_t m : unknown) CHECK(!ft847ModeBaseKnown(m));
}

TEST(narrow_only_for_cw_cwr_and_am) {
  CHECK(ft847ModeCanBeNarrow(FT847_MODE_CW));
  CHECK(ft847ModeCanBeNarrow(FT847_MODE_CWR));
  CHECK(ft847ModeCanBeNarrow(FT847_MODE_AM));
  CHECK(!ft847ModeCanBeNarrow(FT847_MODE_LSB));
  CHECK(!ft847ModeCanBeNarrow(FT847_MODE_USB));
  CHECK(!ft847ModeCanBeNarrow(FT847_MODE_FM));
}

TEST(mode_byte_sets_and_clears_the_narrow_flag) {
  // Manual p. 92: 82 CW(N), 83 CW-R(N), 84 AM(N).
  CHECK_EQ(ft847ModeByte(FT847_MODE_CW, true), 0x82);
  CHECK_EQ(ft847ModeByte(FT847_MODE_CWR, true), 0x83);
  CHECK_EQ(ft847ModeByte(FT847_MODE_AM, true), 0x84);
  CHECK_EQ(ft847ModeByte(FT847_MODE_AM, false), 0x04);
  CHECK_EQ(ft847ModeByte(0x82, false), 0x02);
}

namespace {

void decode(uint8_t rx, uint8_t &s, uint8_t &over) { ft847DecodeSMeter(rx, s, over); }

}  // namespace

TEST(smeter_uses_five_bits) {
  CHECK_EQ(ft847SMeterDots(0x1F), 31);
  CHECK_EQ(ft847SMeterDots(0xFF), 31);
  CHECK_EQ(ft847SMeterDots(0x93), 19);
}

TEST(smeter_dots_to_s_units) {
  uint8_t s = 0, over = 0;
  decode(0, s, over);
  CHECK_EQ(s, 0);
  CHECK_EQ(over, 0);
  decode(3, s, over);  // -48 dB
  CHECK_EQ(s, 1);
  decode(9, s, over);  // -30 dB
  CHECK_EQ(s, 4);
  decode(19, s, over);  // 0 dB = S9
  CHECK_EQ(s, 9);
  CHECK_EQ(over, 0);
}

TEST(smeter_over_s9) {
  uint8_t s = 0, over = 0;
  decode(21, s, over);  // +10 dB
  CHECK_EQ(s, 9);
  CHECK_EQ(over, 10);
  decode(31, s, over);  // +60 dB
  CHECK_EQ(s, 9);
  CHECK_EQ(over, 60);
}

TEST(smeter_ignores_the_flag_bits) {
  uint8_t s1 = 0, o1 = 0, s2 = 0, o2 = 0;
  decode(0x0C, s1, o1);
  decode(0xEC, s2, o2);
  CHECK_EQ(s1, s2);
  CHECK_EQ(o1, o2);
}

TEST(smeter_never_falls_with_more_dots) {
  uint8_t prevSteps = 0;
  for (uint8_t dots = 0; dots <= 31; ++dots) {
    uint8_t s = 0, over = 0;
    decode(dots, s, over);
    const uint8_t steps = (uint8_t)(s + over / 10);
    CHECK(steps >= prevSteps);
    prevSteps = steps;
  }
}

TEST(squelch_bit) {
  CHECK(ft847SquelchOpen(0x05));
  CHECK(!ft847SquelchOpen(0x85));
}

TEST(tx_status) {
  CHECK(ft847TxStatusTransmitting(0x0A));
  CHECK(!ft847TxStatusTransmitting(0x80));
  CHECK_EQ(ft847TxMeter(0x9F), 31);
  CHECK_EQ(ft847TxMeter(0x0A), 10);
}
