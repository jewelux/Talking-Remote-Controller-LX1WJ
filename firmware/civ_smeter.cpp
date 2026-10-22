#include "civ_smeter.h"

void civDecodeSMeter(int32_t raw, uint8_t& sUnitsOut, uint8_t& dbOverS9Out) {
  if (raw < 0) raw = 0;
  if (raw < CIV_SMETER_RAW_S9) {
    // Nearest S unit: S9 is 120, so each unit is 120 / 9 raw steps.
    sUnitsOut = (uint8_t)((raw * 9 + CIV_SMETER_RAW_S9 / 2) / CIV_SMETER_RAW_S9);
    dbOverS9Out = 0;
    return;
  }
  sUnitsOut = 9;
  if (raw > CIV_SMETER_RAW_S9_PLUS_60) raw = CIV_SMETER_RAW_S9_PLUS_60;
  const int32_t span = CIV_SMETER_RAW_S9_PLUS_60 - CIV_SMETER_RAW_S9;  // 121 steps for 60 dB
  // dB over S9, rounded to the nearest 10 dB like the radio's scale marks.
  const int32_t tens = ((raw - CIV_SMETER_RAW_S9) * 6 + span / 2) / span;
  dbOverS9Out = (uint8_t)(tens * 10);
}
