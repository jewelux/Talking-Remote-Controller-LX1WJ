#include "radio_utils.h"

#include "radio_frequency.h"
#include "radio_types.h"

uint64_t decodeBcdFrequencyHz(const uint8_t* bcd, size_t len) {
  uint64_t hz = 0, place = 1;
  for (size_t i = 0; i < len; ++i) {
    uint8_t lo = bcd[i] & 0x0F;
    uint8_t hi = (bcd[i] >> 4) & 0x0F;
    hz += (uint64_t)lo * place;
    place *= 10;
    hz += (uint64_t)hi * place;
    place *= 10;
  }
  return hz;
}

String hzToMHzString3(uint64_t hz) {
  // Delegates to RadioFrequency so display and speech share one formatter.
  // Now renders full 10 Hz resolution with trailing zeros stripped
  // (e.g. 14123450 -> "14.12345", 14250000 -> "14.25").
  return RadioFrequency::fromHz(hz).toString();
}

int32_t bcdDigitsToInt(const uint8_t* b, size_t n) {
  int32_t v = 0;
  for (size_t i = 0; i < n; ++i) {
    uint8_t hi = (b[i] >> 4) & 0x0F;
    uint8_t lo = b[i] & 0x0F;
    if (hi <= 9) v = v * 10 + hi;
    if (lo <= 9) v = v * 10 + lo;
  }
  return v;
}

uint8_t smRawToS(int32_t raw) {
  if (SMETER_RAW_AT_S9 <= SMETER_RAW_AT_S0) return 0;
  if (raw <= SMETER_RAW_AT_S0) return 0;
  if (raw >= SMETER_RAW_AT_S9) return 9;
  float t = float(raw - SMETER_RAW_AT_S0) / float(SMETER_RAW_AT_S9 - SMETER_RAW_AT_S0);
  int s = (int)(t * 9.0f + 0.5f);
  if (s < 0) s = 0;
  if (s > 9) s = 9;
  return (uint8_t)s;
}

float swrRawToValue(int32_t raw) {
  if (raw <= 0) return 1.0f;
  if (raw <= 48) return 1.0f + (raw / 48.0f) * 0.5f;
  if (raw <= 80) return 1.5f + ((raw - 48) / 32.0f) * 0.5f;
  if (raw <= 120) return 2.0f + ((raw - 80) / 40.0f) * 1.0f;
  return 3.0f + ((raw - 120) / 135.0f) * 3.0f;
}
