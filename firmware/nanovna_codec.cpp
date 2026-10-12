#include "nanovna_codec.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr char kPrompt[] = "ch> ";
constexpr size_t kPromptLen = sizeof(kPrompt) - 1;

// IARU Region 1 band plan edges.
constexpr NanoVnaBand kBands[] = {
    {160, NanoVnaBandUnit::Meters, 1810000, 2000000},
    {80, NanoVnaBandUnit::Meters, 3500000, 3800000},
    {60, NanoVnaBandUnit::Meters, 5351500, 5366500},
    {40, NanoVnaBandUnit::Meters, 7000000, 7200000},
    {30, NanoVnaBandUnit::Meters, 10100000, 10150000},
    {20, NanoVnaBandUnit::Meters, 14000000, 14350000},
    {17, NanoVnaBandUnit::Meters, 18068000, 18168000},
    {15, NanoVnaBandUnit::Meters, 21000000, 21450000},
    {12, NanoVnaBandUnit::Meters, 24890000, 24990000},
    {10, NanoVnaBandUnit::Meters, 28000000, 29700000},
    {6, NanoVnaBandUnit::Meters, 50000000, 52000000},
    {2, NanoVnaBandUnit::Meters, 144000000, 146000000},
    {70, NanoVnaBandUnit::Centimeters, 430000000, 440000000},
};
constexpr size_t kBandCount = sizeof(kBands) / sizeof(kBands[0]);

bool isDigit(char c) { return c >= '0' && c <= '9'; }

bool isLetterOrDigit(char c) {
  return isDigit(c) || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

bool startsWith(const char* s, const char* prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

// strstr, ignoring case; needle in lower case.
bool containsLower(const char* s, const char* needle) {
  const size_t n = strlen(needle);
  for (; *s; ++s) {
    size_t i = 0;
    while (i < n && s[i] && (s[i] | 0x20) == needle[i]) ++i;
    if (i == n) return true;
  }
  return false;
}

// Length of s without the spaces at its end.
size_t trimmedLength(const char* s) {
  size_t n = strlen(s);
  while (n && s[n - 1] == ' ') --n;
  return n;
}

const char* skipSpaces(const char* s) {
  while (*s == ' ' || *s == '\t') ++s;
  return s;
}

bool fits(int written, size_t outLen) {
  return written >= 0 && (size_t)written < outLen;
}

// The run of digits and points at line[i], if a version number may start there: a digit at the
// start of a word, or after a 'v' that starts one.
size_t versionRunAt(const char* line, size_t i) {
  if (!isDigit(line[i])) return 0;
  if (i > 0 && isLetterOrDigit(line[i - 1])) {
    const bool afterV = line[i - 1] == 'v' || line[i - 1] == 'V';
    if (!afterV || (i > 1 && isLetterOrDigit(line[i - 2]))) return 0;
  }
  size_t n = 0;
  while (isDigit(line[i + n]) || line[i + n] == '.') ++n;
  while (n && line[i + n - 1] == '.') --n;
  return n;
}

}  // namespace

void NanoVnaReplyReader::reset() {
  buf_[0] = '\0';
  len_ = 0;
  truncated_ = false;
  lineDone_ = false;
}

NanoVnaReplyReader::Event NanoVnaReplyReader::feed(char c) {
  if (lineDone_) reset();
  if (c == '\r') return Event::None;
  if (c == '\n') {
    lineDone_ = true;
    return Event::Line;
  }
  if (len_ < kMaxLine) {
    buf_[len_++] = c;
    buf_[len_] = '\0';
  } else {
    truncated_ = true;
  }
  if (len_ == kPromptLen && !truncated_ && memcmp(buf_, kPrompt, kPromptLen) == 0) {
    reset();
    return Event::Prompt;
  }
  return Event::None;
}

bool nanoVnaIsEcho(const char* line, const char* command) {
  const size_t n = trimmedLength(command);
  return trimmedLength(line) == n && strncmp(line, command, n) == 0;
}

bool nanoVnaReplyIsError(const char* line, const char* command) {
  if (startsWith(line, "usage:") || startsWith(line, "Err")) return true;
  if (containsLower(line, "invalid") || containsLower(line, "exceeds") || containsLower(line, "error")) {
    return true;
  }
  // An unknown command comes back as itself and '?'.
  const size_t n = trimmedLength(line);
  if (n < 2 || line[n - 1] != '?') return false;
  size_t word = 0;
  while (command[word] && command[word] != ' ') ++word;
  return word > 0 && strncmp(line, command, word) == 0;
}

bool nanoVnaParseScanLine(const char* line, NanoVnaPoint& out) {
  const char* p = skipSpaces(line);
  if (!isDigit(*p)) return false;
  char* end = nullptr;
  const unsigned long long hz = strtoull(p, &end, 10);
  if (end == p || hz > UINT32_MAX || (*end != ' ' && *end != '\t')) return false;
  float values[2];
  for (float& v : values) {
    p = skipSpaces(end);
    v = strtof(p, &end);
    if (end == p || !std::isfinite(v)) return false;
  }
  if (*skipSpaces(end)) return false;
  out = {(uint32_t)hz, values[0], values[1]};
  return true;
}

float nanoVnaSwr(float re, float im) {
  const float gamma = std::sqrt(re * re + im * im);
  if (gamma >= 1.0f) return INFINITY;
  return (1.0f + gamma) / (1.0f - gamma);
}

void NanoVnaSweepSummary::add(uint32_t hz, float swr) {
  if (points == 0 || swr < bestSwr) {
    bestSwr = swr;
    bestHz = hz;
  }
  if (points == 0 || swr > worstSwr) {
    worstSwr = swr;
    worstHz = hz;
  }
  if (points < UINT16_MAX) ++points;
}

bool nanoVnaFormatSwr(float swr, char* out, size_t outLen) {
  if (!out || !outLen) return false;
  if (!(swr < 99.95f)) {
    snprintf(out, outLen, "99");
    return true;
  }
  snprintf(out, outLen, swr < 9.995f ? "%.2f" : "%.1f", (double)swr);
  return false;
}

size_t nanoVnaBandCount() { return kBandCount; }

const NanoVnaBand& nanoVnaBandAt(size_t index) {
  return kBands[index < kBandCount ? index : 0];
}

int nanoVnaBandMatching(uint32_t startHz, uint32_t stopHz) {
  for (size_t i = 0; i < kBandCount; ++i) {
    if (kBands[i].startHz == startHz && kBands[i].stopHz == stopHz) return (int)i;
  }
  return -1;
}

size_t nanoVnaAdjacentBand(uint32_t startHz, uint32_t stopHz, int direction) {
  const int band = nanoVnaBandMatching(startHz, stopHz);
  if (direction > 0) {
    if (band >= 0) return ((size_t)band + 1) % kBandCount;
    for (size_t i = 0; i < kBandCount; ++i) {
      if (kBands[i].startHz > startHz) return i;
    }
    return 0;
  }
  if (band >= 0) return ((size_t)band + kBandCount - 1) % kBandCount;
  for (size_t i = kBandCount; i-- > 0;) {
    if (kBands[i].startHz < startHz) return i;
  }
  return kBandCount - 1;
}

bool nanoVnaFrequencyValid(uint64_t hz) {
  return hz >= kNanoVnaMinHz && hz <= kNanoVnaMaxHz;
}

bool nanoVnaRangeValid(uint64_t startHz, uint64_t stopHz) {
  return nanoVnaFrequencyValid(startHz) && nanoVnaFrequencyValid(stopHz) && startHz < stopHz;
}

bool nanoVnaScanCommand(uint32_t startHz, uint32_t stopHz, char* out, size_t outLen) {
  // Outmask 3: the frequency and S11 of each point.
  return fits(snprintf(out, outLen, "scan %lu %lu %u 3", (unsigned long)startHz, (unsigned long)stopHz,
                       (unsigned)kNanoVnaScanPoints),
              outLen);
}

bool nanoVnaSweepCommand(uint32_t startHz, uint32_t stopHz, char* out, size_t outLen) {
  return fits(snprintf(out, outLen, "sweep %lu %lu", (unsigned long)startHz, (unsigned long)stopHz), outLen);
}

bool nanoVnaRecallCommand(uint8_t slot, char* out, size_t outLen) {
  return fits(snprintf(out, outLen, "recall %u", (unsigned)slot), outLen);
}

bool nanoVnaParseMillivolts(const char* line, uint32_t& millivoltsOut) {
  const char* p = skipSpaces(line);
  uint32_t mv = 0;
  size_t digits = 0;
  for (; isDigit(*p); ++p, ++digits) {
    if (digits == 6) return false;
    mv = mv * 10 + (uint32_t)(*p - '0');
  }
  if (!digits) return false;
  p = skipSpaces(p);
  if ((p[0] | 0x20) == 'm' && (p[1] | 0x20) == 'v') p = skipSpaces(p + 2);
  if (*p) return false;
  millivoltsOut = mv;
  return true;
}

void nanoVnaFormatVolts(uint32_t millivolts, char* out, size_t outLen) {
  if (!out || !outLen) return;
  const uint32_t centivolts = (millivolts + 5) / 10;
  snprintf(out, outLen, "%lu.%02lu", (unsigned long)(centivolts / 100), (unsigned long)(centivolts % 100));
}

bool nanoVnaVersionNumber(const char* line, char* out, size_t outLen) {
  if (!out || !outLen) return false;
  // A dotted number first, so that the "2" of "F_V2" is not taken for the version.
  constexpr bool kDottedFirst[] = {true, false};
  for (const bool dotted : kDottedFirst) {
    for (size_t i = 0; line[i]; ++i) {
      const size_t n = versionRunAt(line, i);
      if (!n || (dotted && !memchr(line + i, '.', n))) continue;
      if (n >= outLen) return false;
      memcpy(out, line + i, n);
      out[n] = '\0';
      return true;
    }
  }
  return false;
}
