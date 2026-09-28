#pragma once

#include <stddef.h>

// Fixed-size, always NUL-terminated buffer for the digits typed into a keypad
// entry. Pure C++, so the host unit tests can use it.
template <size_t N>
class DigitBuffer {
 public:
  static constexpr size_t capacity() { return N; }

  void clear() {
    len_ = 0;
    buf_[0] = '\0';
  }

  // Appends c. Returns false and leaves the buffer as it was when it is full.
  bool push(char c) {
    if (len_ >= N) return false;
    buf_[len_++] = c;
    buf_[len_] = '\0';
    return true;
  }

  size_t length() const { return len_; }
  bool empty() const { return len_ == 0; }
  const char* c_str() const { return buf_; }

  // Position of the first c, or -1.
  int indexOf(char c) const {
    for (size_t i = 0; i < len_; ++i) {
      if (buf_[i] == c) return (int)i;
    }
    return -1;
  }

 private:
  char buf_[N + 1] = {};
  size_t len_ = 0;
};
