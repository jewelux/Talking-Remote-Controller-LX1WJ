#include "cat_setting_codec.h"

#include <stdlib.h>
#include <string.h>

uint8_t levelRawToPercent(uint16_t raw) {
  if (raw >= 255) return 100;
  return (uint8_t)((raw * 100U + 127U) / 255U);
}

uint16_t levelPercentToRaw(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return (uint16_t)((percent * 255 + 50) / 100);
}

uint16_t rfPowerRawToWatts(uint16_t raw, uint16_t maxWatts) {
  if (!maxWatts) maxWatts = 100;
  if (raw >= 255) return maxWatts;
  return (uint16_t)((raw * (uint32_t)maxWatts + 127U) / 255U);
}

uint16_t rfPowerWattsToRaw(int watts, uint16_t maxWatts) {
  if (!maxWatts) maxWatts = 100;
  if (watts < 0) watts = 0;
  if (watts > maxWatts) watts = maxWatts;
  return (uint16_t)((watts * 255UL + (maxWatts / 2U)) / maxWatts);
}

uint16_t yaesuAsciiVoxDelayStepMs(uint8_t step) {
  static constexpr uint16_t kFirstSteps[] = {30, 50, 100, 150, 200, 250};
  if (step >= kYaesuAsciiVoxDelaySteps) step = kYaesuAsciiVoxDelaySteps - 1;
  return step < 6 ? kFirstSteps[step] : (uint16_t)(300 + (step - 6) * 100);
}

uint8_t yaesuAsciiVoxDelayNearestStep(uint16_t ms) {
  uint8_t best = 0;
  for (uint8_t step = 1; step < kYaesuAsciiVoxDelaySteps; ++step) {
    const int distance = abs((int)yaesuAsciiVoxDelayStepMs(step) - (int)ms);
    if (distance < abs((int)yaesuAsciiVoxDelayStepMs(best) - (int)ms)) best = step;
  }
  return best;
}

uint16_t yaesuAsciiVoxDelayStepBy(uint16_t ms, int steps) {
  const int limit = kYaesuAsciiVoxDelaySteps - 1;
  if (steps > limit) steps = limit;
  if (steps < -limit) steps = -limit;
  const int step = yaesuAsciiVoxDelayNearestStep(ms) + steps;
  return yaesuAsciiVoxDelayStepMs((uint8_t)(step < 0 ? 0 : step > limit ? limit : step));
}

bool asciiLockDigitDecode(char digit, const char* offDigits, bool& onOut) {
  if (digit < '0' || digit > '9') return false;
  onOut = strchr(offDigits, digit) == nullptr;
  return true;
}
