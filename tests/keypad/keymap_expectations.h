// The characterization matrix (keymap_expectations.inc) as a table.
#pragma once

#include <cstdint>
#include <cstring>

namespace keymap_expect {

// Radio families, one bit each. See keymap_expectations.inc for their traits.
constexpr uint16_t IC7300 = 1u << 0;
constexpr uint16_t IC706 = 1u << 1;
constexpr uint16_t LIGHT = 1u << 2;
constexpr uint16_t TS480 = 1u << 3;
constexpr uint16_t FTDX10 = 1u << 4;
constexpr uint16_t FT891 = 1u << 5;
constexpr uint16_t FT817 = 1u << 6;
constexpr uint16_t FT857 = 1u << 7;
constexpr uint16_t FT8X7_OTHER = 1u << 8;

constexpr uint16_t ALL = IC7300 | IC706 | LIGHT | TS480 | FTDX10 | FT891 | FT817 | FT857 | FT8X7_OTHER;
constexpr uint16_t CIV = IC7300 | IC706 | LIGHT;
constexpr uint16_t FT8X7 = FT817 | FT857 | FT8X7_OTHER;

constexpr uint16_t NOT(uint16_t families) { return ALL & ~families; }

struct Family {
  uint16_t bit;
  const char *name;
};

inline constexpr Family kFamilies[] = {
    {IC7300, "IC-7300"}, {IC706, "IC-706"}, {LIGHT, "light-Icom"}, {TS480, "TS-480"},
    {FTDX10, "FTDX10"},  {FT891, "FT-891"}, {FT817, "FT-817"},     {FT857, "FT-857/897"},
    {FT8X7_OTHER, "FT-8x7 other"},
};

// Bank keys the keymap handles; '*', '#' and 'D' belong to the state machine.
inline constexpr char kBankKeys[] = "0123456789ABC";

struct Row {
  uint16_t families;
  uint8_t bank;
  char key;
  const char *shortAction;
  const char *longAction;
  const char *doubleAction;
};

inline constexpr Row kRows[] = {
#define KEYMAP_ROW(families, bank, key, shortAction, longAction, doubleAction) \
  {families, bank, key, shortAction, longAction, doubleAction},
#include "keymap_expectations.inc"
#undef KEYMAP_ROW
};

// Expectation for a key that no row covers.
inline constexpr Row kUnassigned = {ALL, 0, 0, "unassigned", "none", "-"};

// The row covering (family, bank, key), or kUnassigned.
inline const Row &lookup(uint16_t family, uint8_t bank, char key) {
  for (const Row &r : kRows) {
    if ((r.families & family) && r.bank == bank && r.key == key) {
      return r;
    }
  }
  return kUnassigned;
}

}  // namespace keymap_expect
