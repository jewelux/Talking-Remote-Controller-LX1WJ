#include "smeter_scale.h"

namespace {

// Rounds to the nearest integer, halves away from zero. den > 0.
int32_t roundedDiv(int32_t num, int32_t den) {
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

}  // namespace

int16_t smeterScaleDbRelS9(const SMeterScale& scale, int32_t raw) {
  if (scale.empty()) return kSMeterDbRelS9AtS0;
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
