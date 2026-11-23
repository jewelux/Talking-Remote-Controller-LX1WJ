#pragma once

// How CAT radios encode the receive filter settings (radio_setting_types.h RxFilterLevel,
// RxFilterSwitch): the values each takes in each mode, and the Yaesu newcat command strings. No
// Arduino.h, so the host tests (tests/cat) can build it.
//
// Sources: the FTDX10 (2204-D), FTDX101MP (1912-D), FTDX101D (1904-A) and FT-891 (1710-B, 1609-A0)
// CAT manuals, Hamlib newcat.c, ftdx10.c, ftdx101.c, ft891.c, icom.c and ts480.c, flrig FTdx10,
// FTdx101D and FT891, the IC-7300, IC-705 and IC-7760 CI-V guides, the TS-480 PC command reference.

#include <stddef.h>
#include <stdint.h>

#include "radio_setting_types.h"

// The modes the filter settings differ by. Data is the Yaesu DATA-L, DATA-U and PSK; Unknown a mode
// with none (Yaesu DATA-FM, or a code not known).
enum class FilterModeGroup : uint8_t { Unknown, Ssb, Cw, Rtty, Data, Am, Fm };

// The group of a Yaesu newcat MD code (1 LSB ... F DATA-FM-N), of a TS-480 MD code (1 LSB ... 9
// FSK-R), of an Icom mode byte (00 LSB ... 08 RTTY-R).
FilterModeGroup yaesuFilterModeGroup(char code);
FilterModeGroup kenwoodFilterModeGroup(char code);
FilterModeGroup civFilterModeGroup(uint8_t mode);

// Whether encoding is one of the Yaesu newcat radios (FTDX10, FTDX101, FT-891).
bool rxFilterEncodingIsYaesu(RxFilterEncoding encoding);

// Whether level's values depend on the mode (the width, the cuts and their slopes).
bool rxFilterLevelByMode(RxFilterLevel level);
// Whether level is a cut slope (6 or 18 dB/oct), which takes only its exact values.
bool rxFilterLevelIsSlope(RxFilterLevel level);

// A scale's first code when it stands for value 0 rather than for one of its values, and where it
// sits for a step from it.
enum class RxFilterZero : uint8_t {
  None,     // no such code
  Below,    // a low cut off: below the first value
  Above,    // a high cut off: above the last value
  Default,  // the radio's default width: no place among the values, so no step from it
};

// A setting the radio takes as a code, first..last, each standing for a value: a width, a cut
// frequency, a slope. With a zero code the first code is the radio's default width or the cut
// off, value 0; it is not one of the values to pick from, only set by asking for 0.
struct RxFilterScale {
  uint8_t first = 0;
  uint8_t last = 0;
  const uint16_t* values = nullptr;  // values[code - first]
  RxFilterZero zero = RxFilterZero::None;

  bool hasZero() const { return zero != RxFilterZero::None; }
  bool has(uint8_t code) const { return values && code >= first && code <= last; }
  uint16_t valueOf(uint8_t code) const { return values[code - first]; }
  // The first code that is a value to pick from.
  uint8_t firstValueCode() const { return hasZero() ? (uint8_t)(first + 1) : first; }
};

// A read of a setting on a scale: Unknown when the radio answered with a code the scale does not
// have.
enum class RxFilterRead : uint8_t { Ok, NoReply, Unknown };

// The scale of level (Width, LowCut, HighCut and the slopes) in group on a radio of encoding;
// false where it has none (a Yaesu width in AM or FM, a TS-480 cut in CW, FM on the FT-891).
bool rxFilterScaleFor(RxFilterEncoding encoding, RxFilterLevel level, FilterModeGroup group,
                      RxFilterScale& out);
// The code of value: 0 is the zero code where the scale has one; any other value goes to the
// nearest code that is a value, the lower one on a tie.
uint8_t rxFilterScaleCodeOf(const RxFilterScale& scale, uint16_t value);
// delta codes on from code into out, kept among the codes that are values. From a zero code below
// the values a step up goes to the first value, from one above them a step down to the last; a
// step the other way stays at the zero code. False from a Default zero code.
bool rxFilterScaleStep(const RxFilterScale& scale, uint8_t code, int delta, uint8_t& out);

// A setting the radio takes as a number within min..max in steps of step: the IF shift, the notch,
// contour and APF frequencies.
struct RxFilterSpan {
  int16_t min = 0;
  int16_t max = 0;
  int16_t step = 1;
};

// The span of level on a radio of encoding; false where it has none.
bool rxFilterSpanFor(RxFilterEncoding encoding, RxFilterLevel level, RxFilterSpan& out);
// value rounded to the nearest step of span (half away from zero), kept within it.
int16_t rxFilterSpanRound(const RxFilterSpan& span, int32_t value);

// --- Yaesu newcat (FTDX10, FTDX101 main band, FT-891) ---
// Decoders take what follows the reply prefix the matching ...Read command gives; the ';' may be
// left out. Encoders write the whole set command, ';' included; false where it does not fit or
// the value is not one the radio takes.

// IS: the IF shift in Hz, -1200..1200 in 20 Hz steps. Read "IS0;", reply prefix "IS0". FTDX10 and
// FTDX101 IS00+nnnn; FT-891 IS0 then 0 off or 1 on, then +nnnn (off reads as 0), and its older
// manual IS0+nnnn, which is read too.
bool yaesuIfShiftDecode(RxFilterEncoding encoding, const char* digits, int16_t& hzOut);
bool yaesuIfShiftEncode(RxFilterEncoding encoding, int16_t hz, char* out, size_t size);

// SH: the width code, 00 the default. Read "SH0;", reply prefix "SH0", then a fixed 0 and the code;
// set SH00nn.
bool yaesuWidthDecode(const char* digits, uint8_t& codeOut);
bool yaesuWidthEncode(uint8_t code, char* out, size_t size);

// The switches: NA0n (narrow), CO00nnnn (contour), CO02nnnn (APF); 0 off, 1 on. The read
// command, the reply prefix, the digits after it, and the set command.
const char* yaesuSwitchReadCommand(RxFilterSwitch sw);
const char* yaesuSwitchReplyPrefix(RxFilterSwitch sw);
bool yaesuSwitchDecode(RxFilterSwitch sw, const char* digits, bool& onOut);
bool yaesuSwitchEncode(RxFilterSwitch sw, bool on, char* out, size_t size);

// BP01nnn: the manual notch, nnn x 10 Hz, 001..320. CO01nnnn: the contour, 0010..3200 Hz.
// CO03nnnn: the APF, 0000..0050 = -250..+250 Hz in 10 Hz steps. Read "BP01;", "CO01;", "CO03;",
// the reply prefix the same without ';'.
const char* yaesuFreqReadCommand(RxFilterLevel level);
const char* yaesuFreqReplyPrefix(RxFilterLevel level);
bool yaesuFreqDecode(RxFilterLevel level, const char* digits, int16_t& hzOut);
bool yaesuFreqEncode(RxFilterLevel level, int16_t hz, char* out, size_t size);

// The EX menu item that holds level (LowCut, HighCut and their slopes) in group, as its read
// command without the ';' ("EX010107" on the FTDX10, "EX1101" on the FT-891); the reply has the
// same prefix, then the code in yaesuCutDigits digits (2 for a frequency, 1 for a slope).
bool yaesuCutMenu(RxFilterEncoding encoding, RxFilterLevel level, FilterModeGroup group, char* out,
                  size_t size);
uint8_t yaesuCutDigits(RxFilterLevel level);

// --- Kenwood TS-480 ---
// SL and SH, the slope tune: the low cut and the high cut as a code of two digits. Read "SL;" and
// "SH;", the reply prefix without ';'. Codes as kenwoodTs480 scales give them.
bool kenwoodCutDecode(const char* digits, uint8_t& codeOut);
bool kenwoodCutEncode(RxFilterLevel level, uint8_t code, char* out, size_t size);

// --- Icom CI-V ---
// 1A 03: the width of the current filter as one BCD byte, 00..40 (SSB, CW), 00..31 (RTTY), 00..49
// (AM). False on a byte that is not BCD.
bool civWidthCodeDecode(uint8_t raw, uint8_t& codeOut);
uint8_t civWidthCodeEncode(uint8_t code);
