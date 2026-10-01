#include "radio_frequency.h"

#include "formatted_line.h"

namespace {
bool isDigitRun(const String& s) {
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c < '0' || c > '9') return false;
  }
  return true;
}
}  // namespace

bool RadioFrequency::parseEntry(const String& entry, RadioFrequency& out) {
  String e = entry;
  e.trim();
  if (e.length() == 0) return false;

  int star = e.indexOf('*');
  String mhzPart, fracPart;
  if (star < 0) {
    mhzPart = e;
  } else {
    mhzPart = e.substring(0, star);
    fracPart = e.substring(star + 1);
    if (fracPart.indexOf('*') >= 0) return false;  // more than one '*'
  }

  if (mhzPart.length() == 0 && fracPart.length() == 0) return false;
  if (!isDigitRun(mhzPart) || !isDigitRun(fracPart)) return false;

  // Keep up to 5 fractional digits (10 Hz step), right-padded with zeros.
  if (fracPart.length() > 5) fracPart = fracPart.substring(0, 5);
  while (fracPart.length() < 5) fracPart += '0';

  uint64_t mhz = mhzPart.length() ? strtoull(mhzPart.c_str(), nullptr, 10) : 0ULL;
  uint64_t frac = strtoull(fracPart.c_str(), nullptr, 10);  // 0..99999
  out.hz_ = mhz * 1000000ULL + frac * 10ULL;
  return true;
}

String RadioFrequency::toString() const {
  return String(FormattedLine("{}", *this).c_str());
}

void FormattedLine::append(const RadioFrequency& freq) {
  uint32_t frac = (uint32_t)((freq.hz() % 1000000ULL) / 10ULL);  // 0..99999 (10 Hz units)
  char decimals[5];
  for (int i = 4; i >= 0; --i, frac /= 10) decimals[i] = (char)('0' + frac % 10);
  size_t len = 5;
  while (len > 1 && decimals[len - 1] == '0') --len;  // strip trailing zeros, keep >=1 decimal

  append((uint32_t)(freq.hz() / 1000000ULL));
  append('.');
  append(std::string_view(decimals, len));  // always at least one decimal digit (7000000 -> "7.0")
}

RadioFrequency RadioFrequency::roundedTo(uint32_t stepHz) const {
  if (stepHz == 0) return *this;
  uint64_t step = (uint64_t)stepHz;
  uint64_t rounded = ((hz_ + step / 2) / step) * step;
  return RadioFrequency(rounded);
}
