// Tests for the FT-8x7 CAT frame fields (firmware/ft8x7_codec.*).
#include "ft8x7_codec.h"
#include "test_runner.h"

namespace {

bool sameBytes(const uint8_t *a, const uint8_t *b, size_t n) { return memcmp(a, b, n) == 0; }

}  // namespace

TEST(freq_decodes_bcd_in_10_hz_units) {
  const uint8_t f20m[4] = {0x01, 0x42, 0x50, 0x00};
  CHECK_EQ(yaesuCatDecodeFreqHz(f20m), 14250000ULL);
  const uint8_t f70cm[4] = {0x43, 0x99, 0x87, 0x65};
  CHECK_EQ(yaesuCatDecodeFreqHz(f70cm), 439987650ULL);
}

TEST(freq_encodes_bcd_in_10_hz_units) {
  uint8_t out[4] = {0};
  yaesuCatEncodeFreqHz(14250000ULL, out);
  const uint8_t f20m[4] = {0x01, 0x42, 0x50, 0x00};
  CHECK(sameBytes(out, f20m, 4));
  yaesuCatEncodeFreqHz(439987650ULL, out);
  const uint8_t f70cm[4] = {0x43, 0x99, 0x87, 0x65};
  CHECK(sameBytes(out, f70cm, 4));
}

TEST(freq_encode_drops_the_1_hz_digit) {
  uint8_t out[4] = {0};
  yaesuCatEncodeFreqHz(7100009ULL, out);
  CHECK_EQ(yaesuCatDecodeFreqHz(out), 7100000ULL);
}

TEST(freq_encode_decode_round_trip) {
  const uint64_t samples[] = {100000ULL, 1838010ULL, 50313000ULL, 145500000ULL, 470000000ULL};
  for (uint64_t hz : samples) {
    uint8_t out[4] = {0};
    yaesuCatEncodeFreqHz(hz, out);
    CHECK_EQ(yaesuCatDecodeFreqHz(out), hz);
  }
}

TEST(repeater_offset_uses_the_frequency_encoding) {
  uint8_t offset[4] = {0};
  uint8_t freq[4] = {0};
  yaesuCatEncodeRepeaterOffsetHz(600000ULL, offset);
  yaesuCatEncodeFreqHz(600000ULL, freq);
  CHECK(sameBytes(offset, freq, 4));
}

TEST(freq_field_rejects_non_bcd_digits) {
  const uint8_t highNibble[4] = {0xA1, 0x42, 0x50, 0x00};
  const uint8_t lowNibble[4] = {0x01, 0x4F, 0x50, 0x00};
  CHECK(!yaesuCatFreqFieldValid(highNibble));
  CHECK(!yaesuCatFreqFieldValid(lowNibble));
}

TEST(freq_field_checks_the_coverage_bounds) {
  const uint8_t lowest[4] = {0x00, 0x01, 0x00, 0x00};   // 100 kHz
  const uint8_t below[4] = {0x00, 0x00, 0x99, 0x99};    // 99.99 kHz
  const uint8_t highest[4] = {0x47, 0x00, 0x00, 0x00};  // 470 MHz
  const uint8_t above[4] = {0x47, 0x00, 0x00, 0x01};
  CHECK(yaesuCatFreqFieldValid(lowest));
  CHECK(!yaesuCatFreqFieldValid(below));
  CHECK(yaesuCatFreqFieldValid(highest));
  CHECK(!yaesuCatFreqFieldValid(above));
}

TEST(smeter_decodes_s_units_and_db_over_s9) {
  uint8_t s = 0xFF;
  uint8_t db = 0xFF;
  yaesuCatDecodeSMeterLevel(0x00, s, db);
  CHECK_EQ(s, 0);
  CHECK_EQ(db, 0);
  yaesuCatDecodeSMeterLevel(0x09, s, db);
  CHECK_EQ(s, 9);
  CHECK_EQ(db, 0);
  yaesuCatDecodeSMeterLevel(0x0A, s, db);
  CHECK_EQ(s, 9);
  CHECK_EQ(db, 10);
  yaesuCatDecodeSMeterLevel(0x0F, s, db);
  CHECK_EQ(s, 9);
  CHECK_EQ(db, 60);
}

TEST(smeter_ignores_the_squelch_and_tone_flags) {
  uint8_t s = 0;
  uint8_t db = 0;
  yaesuCatDecodeSMeterLevel(0xF5, s, db);
  CHECK_EQ(s, 5);
  CHECK_EQ(db, 0);
}

TEST(tx_status_bit_7_low_means_transmitting) {
  CHECK(yaesuCatTxStatusTransmitting(0x00));
  CHECK(yaesuCatTxStatusTransmitting(0x7F));
  CHECK(!yaesuCatTxStatusTransmitting(0x80));
  CHECK(!yaesuCatTxStatusTransmitting(0xFF));
}

TEST(swr_follows_the_meter_calibration) {
  CHECK_EQ(yaesuSwrFromMeter(0), 1.0f);
  CHECK_EQ(yaesuSwrFromMeter(3), 2.13f);
  CHECK_EQ(yaesuSwrFromMeter(9), 9.0f);
  CHECK_EQ(yaesuSwrFromMeter(10), 10.0f);
  CHECK_EQ(yaesuSwrFromMeter(15), 10.0f);
}

TEST(ctcss_accepts_only_standard_tones) {
  CHECK(yaesuCtcssTenthsValid(670));
  CHECK(yaesuCtcssTenthsValid(885));
  CHECK(yaesuCtcssTenthsValid(2541));
  CHECK(!yaesuCtcssTenthsValid(0));
  CHECK(!yaesuCtcssTenthsValid(886));
  CHECK(!yaesuCtcssTenthsValid(2542));
}

TEST(dcs_accepts_only_standard_codes) {
  CHECK(yaesuDcsCodeValid(23));
  CHECK(yaesuDcsCodeValid(754));
  CHECK(!yaesuDcsCodeValid(0));
  CHECK(!yaesuDcsCodeValid(24));
  CHECK(!yaesuDcsCodeValid(755));
}

TEST(tone_data_repeats_the_value_for_separate_tx_and_rx) {
  uint8_t data[4] = {0xEE, 0xEE, 0xEE, 0xEE};
  yaesuEncodeToneData(885, true, data);
  const uint8_t both[4] = {0x08, 0x85, 0x08, 0x85};
  CHECK(sameBytes(data, both, 4));
}

TEST(tone_data_zeroes_the_second_pair_for_one_value) {
  uint8_t data[4] = {0xEE, 0xEE, 0xEE, 0xEE};
  yaesuEncodeToneData(885, false, data);
  const uint8_t one[4] = {0x08, 0x85, 0x00, 0x00};
  CHECK(sameBytes(data, one, 4));
}

TEST(tone_data_encodes_four_bcd_digits) {
  uint8_t data[4] = {0};
  yaesuEncodeToneData(2541, false, data);
  CHECK_EQ(data[0], 0x25);
  CHECK_EQ(data[1], 0x41);
  yaesuEncodeToneData(23, false, data);
  CHECK_EQ(data[0], 0x00);
  CHECK_EQ(data[1], 0x23);
}
