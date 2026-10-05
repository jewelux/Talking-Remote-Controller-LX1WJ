#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string_view>
#include <type_traits>

class RadioFrequency;

// Never defined and not constexpr: LineFormat calls it only when the {} count is wrong,
// which makes the mismatch a compile error naming this function.
void lineFormatPlaceholderCountDoesNotMatchArguments();

// A format string whose "{}" placeholders match the argument count; checked at compile time.
template <size_t ArgCount>
struct LineFormat {
  consteval LineFormat(const char* text) : text(text) {
    std::string_view rest = text;
    size_t placeholders = 0;
    for (size_t at; (at = rest.find("{}")) != rest.npos; rest.remove_prefix(at + 2)) ++placeholders;
    if (placeholders != ArgCount) lineFormatPlaceholderCountDoesNotMatchArguments();
  }

  const char* text;
};

// A short console line formatted on the stack, with no heap use. Longer text is cut off.
//   FormattedLine("VFO{}: {} MHz", which, RadioFrequency::fromHz(hz)).c_str()
// Arguments: integers, char, text and RadioFrequency (MHz, as RadioFrequency::toString()).
class FormattedLine {
public:
  template <class... Args>
  explicit FormattedLine(LineFormat<sizeof...(Args)> format, const Args&... args) {
    appendFormat(format, args...);
  }

  template <class... Args>
  void appendFormat(LineFormat<sizeof...(Args)> format, const Args&... args) {
    std::string_view rest = format.text;
    ((appendTextBeforePlaceholder(rest), append(args)), ...);
    append(rest);
  }

  const char* c_str() const { return buf_; }

  void append(std::string_view text) {
    const size_t n = std::min(text.size(), sizeof(buf_) - 1 - len_);
    text.copy(buf_ + len_, n);
    len_ += n;
    buf_[len_] = '\0';
  }
  void append(const char* text) { if (text) append(std::string_view(text)); }
  // char is a character, other integers are numbers. No implicit conversions: a bool,
  // float or enum argument does not compile.
  template <std::integral T>
    requires(!std::same_as<T, bool>)
  void append(T value) {
    if constexpr (std::same_as<T, char>) {
      append(std::string_view(&value, 1));
    } else {
      uint64_t magnitude = static_cast<uint64_t>(value);
      if constexpr (std::is_signed_v<T>) {
        if (value < 0) {
          append('-');
          magnitude = 0 - magnitude;
        }
      }
      char digits[20];
      char* first = std::end(digits);
      do {
        *--first = static_cast<char>('0' + magnitude % 10);
        magnitude /= 10;
      } while (magnitude != 0);
      append(std::string_view(first, std::end(digits) - first));
    }
  }
  void append(const RadioFrequency& freq);  // radio_frequency.cpp

private:
  // Appends the text before the next {} and drops both from rest.
  void appendTextBeforePlaceholder(std::string_view& rest) {
    const size_t at = rest.find("{}");
    append(rest.substr(0, at));
    rest.remove_prefix(at + 2);
  }

  char buf_[128] = "";
  size_t len_ = 0;
};
