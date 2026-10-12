#pragma once

// The NanoVNA text shell, without Arduino so the host tests can check it (tests/nanovna): its
// replies, scan lines, the SWR they give, and the band presets.
//
// The shell is the edy555 one of the NanoVNA, -H, -H4, -F and -F V2 (NanoVNA-F V2 user guide,
// chapter 7; DiSlord NanoVNA-D main.c). The NanoVNA V2 and V2Plus4 speak a binary protocol
// instead. A command line ends with CR. The shell echoes it, ends each line with CR LF, and
// prints the prompt "ch> ", with no line end, once it is ready for the next command.
// "scan {start} {stop} [points] [outmask]" with outmask 3 prints one line per point:
// "<Hz> <S11 re> <S11 im>", DiSlord with a space at the end.

#include <stddef.h>
#include <stdint.h>

// 101 points: the most every NanoVNA takes (the F V2 takes up to 201).
constexpr uint16_t kNanoVnaScanPoints = 101;
// What HamTRC lets a sweep start and stop at. The F V2 covers 50 kHz to 3 GHz, older models
// less; the device refuses what it cannot do.
constexpr uint32_t kNanoVnaMinHz = 10000;
constexpr uint32_t kNanoVnaMaxHz = 3000000000u;
// Calibration slots 0-6 on the F V2; older models take 0-4 and refuse the others.
constexpr uint8_t kNanoVnaCalSlots = 7;

// Splits the shell's output into lines and the prompt, a character at a time.
class NanoVnaReplyReader {
 public:
  enum class Event : uint8_t { None, Line, Prompt };
  // Longer lines are cut; lineTruncated() says so. A scan line is about 35 characters.
  static constexpr size_t kMaxLine = 95;

  void reset();
  Event feed(char c);
  // After Event::Line: the line without its CR LF, until the next feed().
  const char* line() const { return buf_; }
  bool lineTruncated() const { return truncated_; }

 private:
  char buf_[kMaxLine + 1] = "";
  size_t len_ = 0;
  bool truncated_ = false;
  bool lineDone_ = false;
};

// line is the shell's echo of command (a device that does not echo has none).
bool nanoVnaIsEcho(const char* line, const char* command);
// line says command failed: "usage: ...", "Err, default load", "frequency range is invalid",
// "sweep points exceeds range", or "<command>?" for a command the device does not know.
bool nanoVnaReplyIsError(const char* line, const char* command);

struct NanoVnaPoint {
  uint32_t hz;
  float re;  // S11
  float im;
};
// A scan line with outmask 3: "<Hz> <re> <im>". False for anything else.
bool nanoVnaParseScanLine(const char* line, NanoVnaPoint& out);

// SWR from S11: (1 + |G|) / (1 - |G|). Infinity when |G| is 1 or more, which an open port or an
// uncalibrated device can give.
float nanoVnaSwr(float re, float im);

// The best (lowest) and worst (highest) SWR of a sweep and where they are. On a tie, the first
// point keeps it.
struct NanoVnaSweepSummary {
  uint16_t points = 0;
  uint32_t bestHz = 0;
  float bestSwr = 0;
  uint32_t worstHz = 0;
  float worstSwr = 0;

  void add(uint32_t hz, float swr);
};

// "1.21" below 10, "12.5" below 100. From 99.95 up, and for infinity, "99" and true: it is
// said as "99 plus".
bool nanoVnaFormatSwr(float swr, char* out, size_t outLen);

// The amateur bands the keys step through: IARU Region 1, 160 m to 70 cm.
enum class NanoVnaBandUnit : uint8_t { Meters, Centimeters };
struct NanoVnaBand {
  uint16_t wavelength;  // 160 for 160 m, 70 for 70 cm
  NanoVnaBandUnit unit;
  uint32_t startHz;
  uint32_t stopHz;
};
size_t nanoVnaBandCount();
const NanoVnaBand& nanoVnaBandAt(size_t index);
// The band the range is exactly, or -1 for a range of its own.
int nanoVnaBandMatching(uint32_t startHz, uint32_t stopHz);
// The band after (direction > 0) or before (direction < 0) the range, by its start; it wraps
// around at either end, like the profiles.
size_t nanoVnaAdjacentBand(uint32_t startHz, uint32_t stopHz, int direction);

bool nanoVnaFrequencyValid(uint64_t hz);
// Both valid, start below stop.
bool nanoVnaRangeValid(uint64_t startHz, uint64_t stopHz);

// The command lines, without the CR. False when out is too small.
bool nanoVnaScanCommand(uint32_t startHz, uint32_t stopHz, char* out, size_t outLen);
bool nanoVnaSweepCommand(uint32_t startHz, uint32_t stopHz, char* out, size_t outLen);
bool nanoVnaRecallCommand(uint8_t slot, char* out, size_t outLen);

// The battery in a "vbat" reply, "4256 mV" (the unit may be missing). False for anything else.
bool nanoVnaParseMillivolts(const char* line, uint32_t& millivoltsOut);
// Volts to 10 mV: "4.26" for 4256.
void nanoVnaFormatVolts(uint32_t millivolts, char* out, size_t outLen);

// The version number in a "version" reply: the first word of digits and points, a leading 'v'
// dropped ("1.0.3" in "NanoVNA-F V2 v1.0.3"). False when there is none.
bool nanoVnaVersionNumber(const char* line, char* out, size_t outLen);
