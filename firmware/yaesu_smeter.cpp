#include "yaesu_smeter.h"

namespace {

// Hamlib newcat.c yaesu_default_str_cal, used by Hamlib for the FTDX10.
constexpr SMeterCalPoint kFtdx10Points[] = {
    {0, -54},  {26, -42}, {51, -30}, {81, -18}, {105, -9}, {130, 0},
    {157, 12}, {186, 25}, {203, 35}, {237, 50}, {255, 60},
};

// Hamlib ftdx101.h FTDX101D_STR_CAL (also used for the FTDX101MP).
constexpr SMeterCalPoint kFtdx101Points[] = {
    {0, -60},  {17, -54}, {25, -48},  {34, -42},  {51, -36}, {68, -30},
    {85, -24}, {102, -18}, {119, -12}, {136, -6}, {160, 0},  {255, 60},
};

// Hamlib ft891.h FT891_STR_CAL ("TBC" in Hamlib).
constexpr SMeterCalPoint kFt891Points[] = {
    {0, -54},  {12, -48}, {27, -42}, {40, -36},  {55, -30},  {65, -24},
    {80, -18}, {95, -12}, {112, -6}, {130, 0},   {150, 10},  {172, 20},
    {190, 30}, {220, 40}, {240, 50}, {255, 60},
};

int32_t roundedDiv(int32_t num, int32_t den) {
  // den > 0
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

}  // namespace

const SMeterScale kYaesuSMeterScaleFtdx10{kFtdx10Points};
const SMeterScale kYaesuSMeterScaleFtdx101{kFtdx101Points};
const SMeterScale kYaesuSMeterScaleFt891{kFt891Points};

int16_t smeterScaleDbRelS9(const SMeterScale& scale, int32_t raw) {
  if (scale.count == 0) return -54;
  const SMeterCalPoint* p = scale.points;
  if (raw <= p[0].raw) return p[0].dbRelS9;
  for (uint8_t i = 1; i < scale.count; ++i) {
    if (raw <= p[i].raw) {
      const int32_t span = p[i].raw - p[i - 1].raw;
      const int32_t rise = p[i].dbRelS9 - p[i - 1].dbRelS9;
      return (int16_t)(p[i - 1].dbRelS9 + roundedDiv((raw - p[i - 1].raw) * rise, span));
    }
  }
  return p[scale.count - 1].dbRelS9;
}

void smeterScaleDecode(const SMeterScale& scale, int32_t raw, uint8_t& sUnitsOut,
                       uint8_t& dbOverS9Out) {
  const int16_t db = smeterScaleDbRelS9(scale, raw);
  if (db < 0) {
    // 6 dB per S unit, to the nearest unit; S9 minus less than 3 dB is still S9.
    int32_t s = 9 + roundedDiv(db, 6);
    if (s < 0) s = 0;
    sUnitsOut = (uint8_t)s;
    dbOverS9Out = 0;
    return;
  }
  sUnitsOut = 9;
  int32_t tens = roundedDiv(db, 10);
  if (tens > 6) tens = 6;
  dbOverS9Out = (uint8_t)(tens * 10);
}
