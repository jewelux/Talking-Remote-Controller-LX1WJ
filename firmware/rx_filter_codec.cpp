#include "rx_filter_codec.h"

#include <array>
#include <stdio.h>

#include "cat_setting_codec.h"

namespace {

// count values, start, start + step, ...; with zero first a 0 before them.
template <size_t N>
constexpr std::array<uint16_t, N> arithmetic(uint16_t start, uint16_t step, bool zeroFirst) {
  std::array<uint16_t, N> out{};
  for (size_t i = 0; i < N; ++i) {
    if (zeroFirst) out[i] = i == 0 ? 0 : (uint16_t)(start + (i - 1) * step);
    else out[i] = (uint16_t)(start + i * step);
  }
  return out;
}

// --- Yaesu SH widths, code 00 the default (the CAT manuals' Table 3) ---

// FTDX10 (2204-D; flrig FTdx10 agrees; Hamlib gives it the FTDX101's 2200/2300/2400 at 12..14).
constexpr uint16_t kFtdx10SsbWidths[] = {
    0,    300,  400,  600,  850,  1100, 1200, 1500, 1650, 1800, 1950, 2100,
    2250, 2400, 2450, 2500, 2600, 2700, 2800, 2900, 3000, 3200, 3500, 4000,
};

// FTDX101D and FTDX101MP (FTDX101MP 1912-D; Hamlib ftdx101_ssb_widths agrees up to 21).
constexpr uint16_t kFtdx101SsbWidths[] = {
    0,    300,  400,  600,  850,  1100, 1200, 1500, 1650, 1800, 1950,
    2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3200,
};

// CW, RTTY and PSK on both (the manuals; Hamlib and flrig use them for DATA-L/U too).
constexpr uint16_t kFtdxCwWidths[] = {
    0,   50,  100, 150,  200,  250,  300,  350,  400,  450,
    500, 600, 800, 1200, 1400, 1700, 2000, 2400, 3000,
};

// --- Yaesu EX cuts (the FTDX10, FTDX101 and FT-891 manuals agree) ---
// LCUT FREQ 00 off, 01..19 = 100..1000 Hz; HCUT FREQ 00 off, 01..67 = 700..4000 Hz; 50 Hz steps.
constexpr auto kYaesuLowCuts = arithmetic<20>(100, 50, true);
constexpr auto kYaesuHighCuts = arithmetic<68>(700, 50, true);
// LCUT SLOPE, HCUT SLOPE: 0 = 6 dB/oct, 1 = 18 dB/oct.
constexpr uint16_t kYaesuCutSlopes[] = {6, 18};

// --- TS-480 SL and SH (the PC command reference; Hamlib ts480_slope_filter_low/high agree) ---
constexpr uint16_t kTs480LowCutsSsbFm[] = {0, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
constexpr uint16_t kTs480LowCutsAm[] = {0, 100, 200, 500};
constexpr uint16_t kTs480HighCutsSsbFm[] = {1000, 1200, 1400, 1600, 1800, 2000, 2200,
                                            2400, 2600, 2800, 3000, 3400, 4000, 5000};
constexpr uint16_t kTs480HighCutsAm[] = {2500, 3000, 4000, 5000};

// --- Icom 1A 03 (the IC-7300 / IC-9700 / IC-7760 CI-V guides; Hamlib icom.c) ---
// SSB, CW and RTTY: 00..09 = 50..500 Hz in 50 Hz steps, 10..40 = 600..3600 Hz in 100 Hz steps
// (RTTY to 31, 2700 Hz). AM: 00..49 = 200..10000 Hz in 200 Hz steps.
constexpr std::array<uint16_t, 41> icomNarrowWidths() {
  std::array<uint16_t, 41> out{};
  for (size_t i = 0; i < out.size(); ++i) out[i] = (uint16_t)(i <= 9 ? (i + 1) * 50 : (i - 4) * 100);
  return out;
}
constexpr auto kIcomWidths = icomNarrowWidths();
constexpr auto kIcomAmWidths = arithmetic<50>(200, 200, false);

template <size_t N>
bool scaleOf(const uint16_t (&values)[N], RxFilterZero zero, RxFilterScale& out) {
  out.first = 0;
  out.last = (uint8_t)(N - 1);
  out.values = values;
  out.zero = zero;
  return true;
}

template <size_t N>
bool scaleOf(const std::array<uint16_t, N>& values, RxFilterZero zero, RxFilterScale& out, uint8_t last = N - 1) {
  out.first = 0;
  out.last = last;
  out.values = values.data();
  out.zero = zero;
  return true;
}

bool isCutLevel(RxFilterLevel level) {
  return level == RxFilterLevel::LowCut || level == RxFilterLevel::HighCut || rxFilterLevelIsSlope(level);
}

// The EX menu of a cut: its group numbers and the item of the low cut frequency; the low cut
// slope, high cut and high cut slope follow it.
struct CutMenu {
  uint8_t p1 = 0;
  uint8_t p2 = 0;  // FTDX: the mode; the FT-891 has no second number
  uint8_t lowCutItem = 0;
};

bool yaesuCutMenuOf(RxFilterEncoding encoding, FilterModeGroup group, CutMenu& out) {
  if (encoding == RxFilterEncoding::YaesuFt891) {
    // 0601 AM, 0701 CW, 0805 DATA, 1001 RTTY, 1101 SSB; no FM.
    switch (group) {
      case FilterModeGroup::Am: out = {6, 0, 1}; return true;
      case FilterModeGroup::Cw: out = {7, 0, 1}; return true;
      case FilterModeGroup::Data: out = {8, 0, 5}; return true;
      case FilterModeGroup::Rtty: out = {10, 0, 1}; return true;
      case FilterModeGroup::Ssb: out = {11, 0, 1}; return true;
      default: return false;
    }
  }
  if (encoding != RxFilterEncoding::YaesuFtdx10 && encoding != RxFilterEncoding::YaesuFtdx101) return false;
  // RADIO SETTING 01: 01 SSB, 02 AM, 03 FM, 04 PSK/DATA, 05 RTTY; CW SETTING 02: 01 CW. The FTDX10
  // has AF TREBLE, MIDDLE and BASS first, three items the FTDX101 does not.
  const uint8_t shift = encoding == RxFilterEncoding::YaesuFtdx10 ? 3 : 0;
  switch (group) {
    case FilterModeGroup::Ssb: out = {1, 1, (uint8_t)(4 + shift)}; return true;
    case FilterModeGroup::Am: out = {1, 2, (uint8_t)(4 + shift)}; return true;
    case FilterModeGroup::Fm: out = {1, 3, (uint8_t)(4 + shift)}; return true;
    case FilterModeGroup::Data: out = {1, 4, (uint8_t)(6 + shift)}; return true;
    case FilterModeGroup::Rtty: out = {1, 5, (uint8_t)(6 + shift)}; return true;
    case FilterModeGroup::Cw: out = {2, 1, (uint8_t)(4 + shift)}; return true;
    default: return false;
  }
}

uint8_t cutItemOffset(RxFilterLevel level) {
  switch (level) {
    case RxFilterLevel::LowCutSlope: return 1;
    case RxFilterLevel::HighCut: return 2;
    case RxFilterLevel::HighCutSlope: return 3;
    default: return 0;
  }
}

bool fits(int written, size_t size) {
  return written > 0 && (size_t)written < size;
}

// A sign and four digits, the value within -max..max.
bool signedDecode(const char* text, int16_t max, int16_t& out) {
  if (text[0] != '+' && text[0] != '-') return false;
  uint16_t magnitude = 0;
  if (!asciiDigitsDecode(text + 1, 4, magnitude) || magnitude > (uint16_t)max) return false;
  out = (int16_t)(text[0] == '-' ? -(int)magnitude : (int)magnitude);
  return true;
}

}  // namespace

FilterModeGroup yaesuFilterModeGroup(char code) {
  switch (code) {
    case '1': case '2': return FilterModeGroup::Ssb;
    case '3': case '7': return FilterModeGroup::Cw;
    case '6': case '9': return FilterModeGroup::Rtty;
    case '8': case 'C': case 'E': return FilterModeGroup::Data;
    case '5': case 'D': return FilterModeGroup::Am;
    case '4': case 'B': return FilterModeGroup::Fm;
    default: return FilterModeGroup::Unknown;  // A DATA-FM, F DATA-FM-N
  }
}

FilterModeGroup kenwoodFilterModeGroup(char code) {
  switch (code) {
    case '1': case '2': return FilterModeGroup::Ssb;
    case '3': case '7': return FilterModeGroup::Cw;
    case '4': return FilterModeGroup::Fm;
    case '5': return FilterModeGroup::Am;
    case '6': case '9': return FilterModeGroup::Rtty;
    default: return FilterModeGroup::Unknown;
  }
}

FilterModeGroup civFilterModeGroup(uint8_t mode) {
  switch (mode) {
    case 0x00: case 0x01: return FilterModeGroup::Ssb;
    case 0x02: return FilterModeGroup::Am;
    case 0x03: case 0x07: return FilterModeGroup::Cw;
    case 0x04: case 0x08: return FilterModeGroup::Rtty;
    case 0x05: return FilterModeGroup::Fm;
    default: return FilterModeGroup::Unknown;
  }
}

bool rxFilterEncodingIsYaesu(RxFilterEncoding encoding) {
  return encoding == RxFilterEncoding::YaesuFtdx10 || encoding == RxFilterEncoding::YaesuFtdx101 ||
         encoding == RxFilterEncoding::YaesuFt891;
}

bool rxFilterLevelByMode(RxFilterLevel level) {
  return level == RxFilterLevel::Width || isCutLevel(level);
}

bool rxFilterLevelIsSlope(RxFilterLevel level) {
  return level == RxFilterLevel::LowCutSlope || level == RxFilterLevel::HighCutSlope;
}

bool rxFilterScaleFor(RxFilterEncoding encoding, RxFilterLevel level, FilterModeGroup group,
                      RxFilterScale& out) {
  if (group == FilterModeGroup::Unknown) return false;
  if (level == RxFilterLevel::Width) {
    switch (encoding) {
      case RxFilterEncoding::YaesuFtdx10:
      case RxFilterEncoding::YaesuFtdx101:
        if (group == FilterModeGroup::Ssb) {
          return encoding == RxFilterEncoding::YaesuFtdx10 ? scaleOf(kFtdx10SsbWidths, RxFilterZero::Default, out)
                                                           : scaleOf(kFtdx101SsbWidths, RxFilterZero::Default, out);
        }
        if (group == FilterModeGroup::Cw || group == FilterModeGroup::Rtty || group == FilterModeGroup::Data) {
          return scaleOf(kFtdxCwWidths, RxFilterZero::Default, out);
        }
        return false;
      case RxFilterEncoding::Icom:
        if (group == FilterModeGroup::Am) return scaleOf(kIcomAmWidths, RxFilterZero::None, out);
        if (group == FilterModeGroup::Rtty) return scaleOf(kIcomWidths, RxFilterZero::None, out, 31);
        if (group == FilterModeGroup::Ssb || group == FilterModeGroup::Cw) {
          return scaleOf(kIcomWidths, RxFilterZero::None, out);
        }
        return false;
      default: return false;
    }
  }
  if (!isCutLevel(level)) return false;
  if (rxFilterEncodingIsYaesu(encoding)) {
    CutMenu menu;
    if (!yaesuCutMenuOf(encoding, group, menu)) return false;
    switch (level) {
      case RxFilterLevel::LowCut: return scaleOf(kYaesuLowCuts, RxFilterZero::Below, out);
      case RxFilterLevel::HighCut: return scaleOf(kYaesuHighCuts, RxFilterZero::Above, out);
      default: return scaleOf(kYaesuCutSlopes, RxFilterZero::None, out);
    }
  }
  if (encoding == RxFilterEncoding::KenwoodTs480) {
    const bool am = group == FilterModeGroup::Am;
    if (!am && group != FilterModeGroup::Ssb && group != FilterModeGroup::Fm) return false;
    // A low cut of 0 Hz is no cut, below the other values.
    if (level == RxFilterLevel::LowCut) {
      return am ? scaleOf(kTs480LowCutsAm, RxFilterZero::Below, out)
                : scaleOf(kTs480LowCutsSsbFm, RxFilterZero::Below, out);
    }
    if (level == RxFilterLevel::HighCut) {
      return am ? scaleOf(kTs480HighCutsAm, RxFilterZero::None, out)
                : scaleOf(kTs480HighCutsSsbFm, RxFilterZero::None, out);
    }
  }
  return false;
}

uint8_t rxFilterScaleCodeOf(const RxFilterScale& scale, uint16_t value) {
  if (value == 0 && scale.hasZero()) return scale.first;
  uint8_t best = scale.firstValueCode();
  uint16_t bestDistance = 0xFFFF;
  for (uint16_t code = scale.firstValueCode(); code <= scale.last; ++code) {
    const uint16_t v = scale.valueOf((uint8_t)code);
    const uint16_t distance = v > value ? (uint16_t)(v - value) : (uint16_t)(value - v);
    if (distance < bestDistance) {
      best = (uint8_t)code;
      bestDistance = distance;
    }
  }
  return best;
}

bool rxFilterScaleStep(const RxFilterScale& scale, uint8_t code, int delta, uint8_t& out) {
  const int lo = scale.firstValueCode();
  const int hi = scale.last;
  int next = (int)code + delta;
  if (scale.hasZero() && code == scale.first) {
    switch (scale.zero) {
      case RxFilterZero::Below:
        if (delta <= 0) {
          out = code;
          return true;
        }
        next = lo + delta - 1;
        break;
      case RxFilterZero::Above:
        if (delta >= 0) {
          out = code;
          return true;
        }
        next = hi + delta + 1;
        break;
      default: return false;
    }
  }
  if (next < lo) next = lo;
  if (next > hi) next = hi;
  out = (uint8_t)next;
  return true;
}

bool rxFilterScaleNext(const RxFilterScale& scale, uint16_t value, int delta, uint16_t& out) {
  uint8_t code = 0;
  if (!rxFilterScaleStep(scale, rxFilterScaleCodeOf(scale, value), delta, code)) return false;
  out = code == scale.first && scale.hasZero() ? 0 : scale.valueOf(code);
  return true;
}

bool rxFilterSpanFor(RxFilterEncoding encoding, RxFilterLevel level, RxFilterSpan& out) {
  if (!rxFilterEncodingIsYaesu(encoding)) return false;
  switch (level) {
    case RxFilterLevel::IfShift: out = {-1200, 1200, 20}; return true;
    case RxFilterLevel::NotchFreq: out = {10, 3200, 10}; return true;
    case RxFilterLevel::ContourFreq: out = {10, 3200, 10}; return true;
    case RxFilterLevel::ApfFreq: out = {-250, 250, 10}; return true;
    default: return false;
  }
}

int16_t rxFilterSpanRound(const RxFilterSpan& span, int32_t value) {
  const int32_t step = span.step > 0 ? span.step : 1;
  const int32_t half = step / 2;
  int32_t rounded = value >= 0 ? (value + half) / step * step : -((-value + half) / step * step);
  // Every span's ends are steps.
  if (rounded < span.min) rounded = span.min;
  if (rounded > span.max) rounded = span.max;
  return (int16_t)rounded;
}

bool yaesuIfShiftDecode(RxFilterEncoding encoding, const char* digits, int16_t& hzOut) {
  if (!digits) return false;
  if (encoding == RxFilterEncoding::YaesuFt891) {
    // The older manual: no on digit.
    if (digits[0] == '+' || digits[0] == '-') return signedDecode(digits, 1200, hzOut);
    if (digits[0] != '0' && digits[0] != '1') return false;
    int16_t hz = 0;
    if (!signedDecode(digits + 1, 1200, hz)) return false;
    hzOut = digits[0] == '1' ? hz : 0;
    return true;
  }
  if (encoding != RxFilterEncoding::YaesuFtdx10 && encoding != RxFilterEncoding::YaesuFtdx101) return false;
  return digits[0] == '0' && signedDecode(digits + 1, 1200, hzOut);
}

bool yaesuIfShiftEncode(RxFilterEncoding encoding, int16_t hz, char* out, size_t size) {
  if (!rxFilterEncodingIsYaesu(encoding) || hz < -1200 || hz > 1200 || hz % 20) return false;
  const char sign = hz < 0 ? '-' : '+';
  const int magnitude = hz < 0 ? -hz : hz;
  if (encoding == RxFilterEncoding::YaesuFt891) {
    return fits(snprintf(out, size, "IS0%c%c%04d;", hz ? '1' : '0', sign, magnitude), size);
  }
  return fits(snprintf(out, size, "IS00%c%04d;", sign, magnitude), size);
}

bool yaesuWidthDecode(const char* digits, uint8_t& codeOut) {
  uint16_t value = 0;
  if (!digits || digits[0] != '0' || !asciiDigitsDecode(digits + 1, 2, value)) return false;
  codeOut = (uint8_t)value;
  return true;
}

bool yaesuWidthEncode(uint8_t code, char* out, size_t size) {
  return code <= 99 && fits(snprintf(out, size, "SH00%02u;", (unsigned)code), size);
}

const char* yaesuSwitchReadCommand(RxFilterSwitch sw) {
  switch (sw) {
    case RxFilterSwitch::Narrow: return "NA0;";
    case RxFilterSwitch::Contour: return "CO00;";
    case RxFilterSwitch::Apf: return "CO02;";
  }
  return "";
}

const char* yaesuSwitchReplyPrefix(RxFilterSwitch sw) {
  switch (sw) {
    case RxFilterSwitch::Narrow: return "NA0";
    case RxFilterSwitch::Contour: return "CO00";
    case RxFilterSwitch::Apf: return "CO02";
  }
  return "";
}

bool yaesuSwitchDecode(RxFilterSwitch sw, const char* digits, bool& onOut) {
  uint16_t value = 0;
  if (!digits || !asciiDigitsDecode(digits, sw == RxFilterSwitch::Narrow ? 1 : 4, value) || value > 1) return false;
  onOut = value == 1;
  return true;
}

bool yaesuSwitchEncode(RxFilterSwitch sw, bool on, char* out, size_t size) {
  if (sw == RxFilterSwitch::Narrow) return fits(snprintf(out, size, "NA0%c;", on ? '1' : '0'), size);
  return fits(snprintf(out, size, "%s%04d;", yaesuSwitchReplyPrefix(sw), on ? 1 : 0), size);
}

const char* yaesuFreqReadCommand(RxFilterLevel level) {
  switch (level) {
    case RxFilterLevel::NotchFreq: return "BP01;";
    case RxFilterLevel::ContourFreq: return "CO01;";
    case RxFilterLevel::ApfFreq: return "CO03;";
    default: return nullptr;
  }
}

const char* yaesuFreqReplyPrefix(RxFilterLevel level) {
  switch (level) {
    case RxFilterLevel::NotchFreq: return "BP01";
    case RxFilterLevel::ContourFreq: return "CO01";
    case RxFilterLevel::ApfFreq: return "CO03";
    default: return nullptr;
  }
}

bool yaesuFreqDecode(RxFilterLevel level, const char* digits, int16_t& hzOut) {
  if (!digits) return false;
  uint16_t value = 0;
  switch (level) {
    case RxFilterLevel::NotchFreq:
      if (!asciiDigitsDecode(digits, 3, value) || value < 1 || value > 320) return false;
      hzOut = (int16_t)(value * 10);
      return true;
    case RxFilterLevel::ContourFreq:
      if (!asciiDigitsDecode(digits, 4, value) || value < 10 || value > 3200) return false;
      hzOut = (int16_t)value;
      return true;
    case RxFilterLevel::ApfFreq:
      if (!asciiDigitsDecode(digits, 4, value) || value > 50) return false;
      hzOut = (int16_t)(value * 10 - 250);
      return true;
    default: return false;
  }
}

bool yaesuFreqEncode(RxFilterLevel level, int16_t hz, char* out, size_t size) {
  switch (level) {
    case RxFilterLevel::NotchFreq:
      if (hz < 10 || hz > 3200 || hz % 10) return false;
      return fits(snprintf(out, size, "BP01%03d;", hz / 10), size);
    case RxFilterLevel::ContourFreq:
      if (hz < 10 || hz > 3200) return false;
      return fits(snprintf(out, size, "CO01%04d;", hz), size);
    case RxFilterLevel::ApfFreq:
      if (hz < -250 || hz > 250 || hz % 10) return false;
      return fits(snprintf(out, size, "CO03%04d;", (hz + 250) / 10), size);
    default: return false;
  }
}

bool yaesuCutMenu(RxFilterEncoding encoding, RxFilterLevel level, FilterModeGroup group, char* out,
                  size_t size) {
  CutMenu menu;
  if (!isCutLevel(level) || !yaesuCutMenuOf(encoding, group, menu)) return false;
  const unsigned item = (unsigned)(menu.lowCutItem + cutItemOffset(level));
  if (encoding == RxFilterEncoding::YaesuFt891) {
    return fits(snprintf(out, size, "EX%02u%02u", (unsigned)menu.p1, item), size);
  }
  return fits(snprintf(out, size, "EX%02u%02u%02u", (unsigned)menu.p1, (unsigned)menu.p2, item), size);
}

uint8_t yaesuCutDigits(RxFilterLevel level) {
  return level == RxFilterLevel::LowCutSlope || level == RxFilterLevel::HighCutSlope ? 1 : 2;
}

bool kenwoodCutDecode(const char* digits, uint8_t& codeOut) {
  uint16_t value = 0;
  if (!digits || !asciiDigitsDecode(digits, 2, value)) return false;
  codeOut = (uint8_t)value;
  return true;
}

bool kenwoodCutEncode(RxFilterLevel level, uint8_t code, char* out, size_t size) {
  if (code > 99) return false;
  if (level == RxFilterLevel::LowCut) return fits(snprintf(out, size, "SL%02u;", (unsigned)code), size);
  if (level == RxFilterLevel::HighCut) return fits(snprintf(out, size, "SH%02u;", (unsigned)code), size);
  return false;
}

bool civWidthCodeDecode(uint8_t raw, uint8_t& codeOut) {
  const uint8_t hi = raw >> 4;
  const uint8_t lo = raw & 0x0F;
  if (hi > 9 || lo > 9) return false;
  codeOut = (uint8_t)(hi * 10 + lo);
  return true;
}

uint8_t civWidthCodeEncode(uint8_t code) {
  return (uint8_t)(((code / 10) << 4) | (code % 10));
}
