#pragma once

#include <Arduino.h>

// Owns all frequency parsing / formatting / rounding in one place.
//
// Internally the value is stored in Hz. Announcement and display are limited to
// 10 Hz resolution: a decimal MHz string with trailing zeros stripped
// (e.g. 14123450 -> "14.12345", 14100000 -> "14.1", 14000000 -> "14").
class RadioFrequency {
public:
  RadioFrequency() : hz_(0) {}

  static RadioFrequency fromHz(uint64_t hz) { return RadioFrequency(hz); }

  // Parse a keypad entry string where the integer part is MHz and '*' is the
  // decimal point, with up to 5 fractional digits (10 Hz step):
  //   "14"        -> 14000000 Hz
  //   "14*1"      -> 14100000 Hz
  //   "14*12345"  -> 14123450 Hz
  // Returns false on an empty string, a non-digit character, or more than one '*'.
  static bool parseEntry(const String& entry, RadioFrequency& out);

  uint64_t hz() const { return hz_; }

  // Decimal MHz with trailing zeros stripped. Fed straight to
  // speakDigitsAndPoint(), which speaks each digit and '.' as "point".
  String toString() const;

  // Nearest multiple of stepHz (e.g. 500 for the Bank 1 rounding feature).
  RadioFrequency roundedTo(uint32_t stepHz) const;

private:
  explicit RadioFrequency(uint64_t hz) : hz_(hz) {}
  uint64_t hz_;
};
