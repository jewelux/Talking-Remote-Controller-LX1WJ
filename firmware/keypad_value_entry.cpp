#include "keypad_value_entry.h"

#include <stddef.h>

namespace {

int32_t pow10(uint8_t n) {
  int32_t p = 1;
  while (n--) p *= 10;
  return p;
}

}  // namespace

void ValueEntry::begin(const ValueSpec& spec, const EntrySeed& seed) {
  spec_ = spec;
  digits_.clear();
  known_ = seed.known;
  staged_ = seed.known && seed.staged;
  value_ = seed.known ? clamp(seed.value) : 0;
}

bool ValueEntry::type(char key) {
  if (key == '*') {
    if (spec_.sign && digits_.empty()) return digits_.push('-');
    // One point, after a digit.
    if (spec_.pointDigits == 0 || digits_.indexOf('.') >= 0) return false;
    const size_t len = digits_.length();
    if (len == 0 || digits_.c_str()[len - 1] == '-') return false;
    return digits_.push('.');
  }
  if (key < '0' || key > '9') return false;
  uint8_t count = 0;
  for (const char* p = digits_.c_str(); *p; ++p) {
    if (*p >= '0' && *p <= '9') ++count;
  }
  if (count >= spec_.maxDigits) return false;
  const int point = digits_.indexOf('.');
  if (point >= 0 && (int)digits_.length() - point - 1 >= spec_.pointDigits) return false;
  return digits_.push(key);
}

bool ValueEntry::parse(int32_t& out) const {
  const char* p = digits_.c_str();
  const bool negative = *p == '-';
  if (negative) ++p;
  int32_t whole = 0;
  int32_t fraction = 0;
  uint8_t fractionDigits = 0;
  bool point = false;
  bool any = false;
  for (; *p; ++p) {
    if (*p == '.') {
      point = true;
      continue;
    }
    any = true;
    if (point) {
      fraction = fraction * 10 + (*p - '0');
      ++fractionDigits;
    } else {
      whole = whole * 10 + (*p - '0');
    }
  }
  if (!any) return false;
  int32_t v;
  if (!point && spec_.bareInUnits) {
    v = whole;
  } else {
    v = whole * pow10(spec_.scale) + fraction * pow10((uint8_t)(spec_.scale - fractionDigits));
  }
  out = negative ? -v : v;
  return true;
}

int32_t ValueEntry::clamp(int32_t v) const {
  if (v < spec_.min) return spec_.min;
  if (v > spec_.max) return spec_.max;
  return v;
}

ValueEntry::Step ValueEntry::step(int8_t dir) {
  int32_t base = 0;
  if (!parse(base)) {
    if (!known_) return Step::NoValue;
    base = value_;
  }
  // A value typed out of range steps to the end it is past.
  int32_t next = clamp(base);
  if (next == base) next = clamp(spec_.stepFn ? spec_.stepFn(base, dir) : base + dir * spec_.step);
  if (next == base) return Step::AtLimit;
  digits_.clear();
  known_ = staged_ = true;
  value_ = next;
  return Step::Moved;
}

ValueEntry::Step ValueEntry::step(int8_t dir, uint8_t times) {
  const Step first = step(dir);
  if (first != Step::Moved) return first;
  for (uint8_t i = 1; i < times; ++i) {
    if (step(dir) != Step::Moved) break;
  }
  return Step::Moved;
}

ValueEntry::Result ValueEntry::result(int32_t& out) const {
  int32_t v = 0;
  if (parse(v)) {
    if (v > spec_.max) return Result::AboveMax;
    if (v < spec_.min) return Result::BelowMin;
    out = v;
    return Result::Ok;
  }
  if (!staged_) return Result::None;
  out = value_;
  return Result::Ok;
}

bool ValueEntry::snapToRange() {
  int32_t v = 0;
  if (!parse(v) || clamp(v) == v) return false;
  digits_.clear();
  known_ = staged_ = true;
  value_ = clamp(v);
  return true;
}

bool ValueEntry::current(int32_t& out) const {
  if (parse(out)) {
    out = clamp(out);
    return true;
  }
  if (!known_) return false;
  out = value_;
  return true;
}
