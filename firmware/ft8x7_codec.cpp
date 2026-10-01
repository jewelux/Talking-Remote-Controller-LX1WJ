#include "ft8x7_codec.h"

uint64_t yaesuCatDecodeFreqHz(const uint8_t data[4]) {
  uint64_t digits = 0;
  for (int i = 0; i < 4; ++i) {
    digits = digits * 100ULL + (uint64_t)(((data[i] >> 4) & 0x0F) * 10 + (data[i] & 0x0F));
  }
  return digits * 10ULL;
}

bool yaesuCatFreqFieldValid(const uint8_t data[4]) {
  for (int i = 0; i < 4; ++i) {
    if (((data[i] >> 4) & 0x0F) > 9 || (data[i] & 0x0F) > 9) return false;
  }
  const uint64_t hz = yaesuCatDecodeFreqHz(data);
  return hz >= YAESU_CAT_MIN_FREQ_HZ && hz <= YAESU_CAT_MAX_FREQ_HZ;
}

void yaesuCatEncodeFreqHz(uint64_t hz, uint8_t out[4]) {
  uint32_t units10 = (uint32_t)((hz / 10ULL) % 100000000ULL);
  for (int i = 3; i >= 0; --i) {
    const uint8_t pair = (uint8_t)(units10 % 100);
    units10 /= 100;
    out[i] = (uint8_t)(((pair / 10) << 4) | (pair % 10));
  }
}

void yaesuCatEncodeRepeaterOffsetHz(uint64_t hz, uint8_t out[4]) {
  // FT-817 practical testing shows repeater offset uses the same 10 Hz BCD
  // scaling as the standard Yaesu frequency write path.
  yaesuCatEncodeFreqHz(hz, out);
}

void yaesuCatDecodeSMeterLevel(uint8_t rxStatus, uint8_t& sUnitsOut, uint8_t& dbOverS9Out) {
  const uint8_t level = rxStatus & 0x0F;
  if (level <= 9) {
    sUnitsOut = level;
    dbOverS9Out = 0;
  } else {
    sUnitsOut = 9;
    dbOverS9Out = (uint8_t)((level - 9) * 10);
  }
}

bool yaesuCatTxStatusTransmitting(uint8_t txStatus) {
  return (txStatus & 0x80) == 0;
}

// SWR per meter bar, measured on an FT-817 by WA4YA/DL4YA (Hamlib's FT817_SWR_CAL).
float yaesuSwrFromMeter(uint8_t bars) {
  static constexpr float kSwr[] = {1.0f, 1.4f, 1.8f, 2.13f, 2.25f, 3.7f, 6.0f, 7.0f, 8.0f, 9.0f};
  static constexpr size_t kCount = sizeof(kSwr) / sizeof(kSwr[0]);
  return bars < kCount ? kSwr[bars] : 10.0f;
}

static constexpr uint16_t kValidCtcssTenths[] = {
  670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
  948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
  1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
  1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
  2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541
};

static constexpr uint16_t kValidDcsCodes[] = {
  23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72, 73,
  74, 114, 115, 116, 122, 125, 131, 132, 134, 143, 145, 152, 155, 156, 162,
  165, 172, 174, 205, 212, 223, 225, 226, 243, 244, 245, 246, 251, 252, 255,
  261, 263, 265, 266, 271, 274, 306, 311, 315, 325, 331, 332, 343, 346, 351,
  356, 364, 365, 371, 411, 412, 413, 423, 431, 432, 445, 446, 452, 454, 455,
  462, 464, 465, 466, 503, 506, 516, 523, 526, 532, 546, 565, 606, 612, 624,
  627, 631, 632, 654, 662, 664, 703, 712, 723, 731, 732, 734, 743, 754
};

template <size_t N>
static bool containsU16(const uint16_t (&values)[N], uint16_t needle) {
  for (size_t i = 0; i < N; ++i) {
    if (values[i] == needle) return true;
  }
  return false;
}

bool yaesuCtcssTenthsValid(uint16_t toneTenths) {
  return containsU16(kValidCtcssTenths, toneTenths);
}

bool yaesuDcsCodeValid(uint16_t dcsCode) {
  return containsU16(kValidDcsCodes, dcsCode);
}

void yaesuEncodeToneData(uint16_t value, bool separateTxRx, uint8_t data[4]) {
  const uint8_t d1 = (uint8_t)(value % 10); value /= 10;
  const uint8_t d10 = (uint8_t)(value % 10); value /= 10;
  const uint8_t d100 = (uint8_t)(value % 10); value /= 10;
  const uint8_t d1000 = (uint8_t)(value % 10);
  data[0] = (uint8_t)((d1000 << 4) | d100);
  data[1] = (uint8_t)((d10 << 4) | d1);
  data[2] = separateTxRx ? data[0] : 0x00;
  data[3] = separateTxRx ? data[1] : 0x00;
}
