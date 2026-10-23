#pragma once

#include <stdint.h>

#include "keypad_digit_buffer.h"
#include "keypad_settings.h"

// A number typed or stepped on the keypad: RF power, a CTCSS tone, a DCS code.
// Digits type it, '*' is its point or minus, B and C step it and Enter sets it.
// Pure C++ with no Arduino.h, so it runs in the host unit tests (tests/keypad).
//
// A value is a whole number in the units of its last decimal: with scale 1,
// 45 W is 450.

// How one value is typed and stepped. The firmware fills it per radio
// (keypad_setting_ops.cpp), the range of the current band and all.
struct ValueSpec {
  const char* name = "";  // label of its beeps and trace: "RFPOWER"
  Setting setting = Setting::None;
  int32_t min = 0;
  int32_t max = 0;
  uint8_t scale = 0;        // decimals of the value: 1 = tenths
  uint8_t pointDigits = 0;  // digits typed after the '*' point, at most scale; 0 = no point
  // Digits typed without a point are in the value's units: the CTCSS "885" is 88.5 Hz, as
  // "88*5" is.
  bool bareInUnits = false;
  bool sign = false;      // '*' as the first key is a minus
  uint8_t maxDigits = 0;  // digits typed, the point and minus not counted
  // One B or C step: the next value up (dir 1) or down (dir -1), or nullptr for step.
  // The result is kept within min..max.
  int32_t (*stepFn)(int32_t value, int8_t dir) = nullptr;
  int32_t step = 1;
  const char* unit = "";  // shown after the digits in the trace
};

// What the entry starts from: the radio's value when it was read, and whether
// it is already staged, so that Enter sets it with nothing typed.
struct EntrySeed {
  bool known = false;
  int32_t value = 0;
  bool staged = false;
};

class ValueEntry {
 public:
  enum class Step : uint8_t {
    Moved,    // value() now holds the new value
    AtLimit,  // already at max (B) or min (C): nothing changed
    NoValue,  // nothing typed and no value known to step from
  };
  enum class Result : uint8_t {
    Ok,
    None,      // nothing typed or staged
    AboveMax,
    BelowMin,
  };

  void begin(const ValueSpec& spec, const EntrySeed& seed);
  const ValueSpec& spec() const { return spec_; }
  // As typed, with '.' for the point and '-' for the minus.
  const char* digits() const { return digits_.c_str(); }

  // '0'-'9' or '*'. False when the key is not taken (too many digits, a
  // second point, a point the value has none of).
  bool type(char key);
  // dir 1 (B) or -1 (C), from the value typed or else the value so far.
  Step step(int8_t dir);
  // What Enter sets: the value typed, or the value staged by a step or the seed.
  Result result(int32_t& out) const;
  // A value typed past min..max becomes that end, staged: true when it was past.
  bool snapToRange();
  // The value so far, for A: typed (kept within min..max), staged or read. False when there
  // is none.
  bool current(int32_t& out) const;

 private:
  // The digits as a value; false while they hold none (nothing, or a lone minus).
  bool parse(int32_t& out) const;
  int32_t clamp(int32_t v) const;

  ValueSpec spec_;
  DigitBuffer<12> digits_;
  bool known_ = false;
  bool staged_ = false;
  int32_t value_ = 0;
};
