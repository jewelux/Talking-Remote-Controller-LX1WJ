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

uint16_t asciiVoxDelayMsRound(uint16_t ms, uint16_t minMs, uint16_t maxMs, uint16_t stepMs) {
  if (!stepMs) stepMs = 1;
  uint32_t rounded = ((uint32_t)ms + stepMs / 2) / stepMs * stepMs;
  if (rounded < minMs) rounded = minMs;
  if (rounded > maxMs) rounded = maxMs;
  return (uint16_t)rounded;
}

uint16_t asciiVoxDelayMsStepBy(uint16_t ms, int steps, uint16_t minMs, uint16_t maxMs, uint16_t stepMs) {
  const int stride = stepMs < 100 ? 100 : stepMs;
  if (steps > 100) steps = 100;
  if (steps < -100) steps = -100;
  const int stepped = (int)ms + steps * stride;
  return asciiVoxDelayMsRound((uint16_t)(stepped < 0 ? 0 : stepped > 60000 ? 60000 : stepped), minMs, maxMs, stepMs);
}

static bool threeDigits(const char* s, uint8_t& out) {
  int value = 0;
  for (int i = 0; i < 3; ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    value = value * 10 + (s[i] - '0');
  }
  if (value > 100) return false;
  out = (uint8_t)value;
  return true;
}

bool kenwoodProcLevelsDecode(const char* digits, uint8_t& inputOut, uint8_t& outputOut) {
  if (!digits || strlen(digits) < 6 || (digits[6] != '\0' && digits[6] != ';')) return false;
  uint8_t input = 0;
  uint8_t output = 0;
  if (!threeDigits(digits, input) || !threeDigits(digits + 3, output)) return false;
  inputOut = input;
  outputOut = output;
  return true;
}

AgcMode ft8x7AgcToMode(YaesuAgc agc) {
  switch (agc) {
    case YaesuAgc::Fast: return AgcMode::Fast;
    case YaesuAgc::Slow: return AgcMode::Slow;
    case YaesuAgc::Auto: return AgcMode::Auto;
    case YaesuAgc::Off: break;
  }
  return AgcMode::Off;
}

bool ft8x7AgcFromMode(AgcMode mode, YaesuAgc& out) {
  switch (mode) {
    case AgcMode::Off: out = YaesuAgc::Off; return true;
    case AgcMode::Fast: out = YaesuAgc::Fast; return true;
    case AgcMode::Slow: out = YaesuAgc::Slow; return true;
    case AgcMode::Auto: out = YaesuAgc::Auto; return true;
    case AgcMode::Mid: break;
  }
  return false;
}

bool asciiLockDigitDecode(char digit, const char* offDigits, bool& onOut) {
  if (digit < '0' || digit > '9') return false;
  onOut = strchr(offDigits, digit) == nullptr;
  return true;
}
