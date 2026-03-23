#include "radio_state.h"

#include "radio_globals.h"

void resetLiveRadioState() {
  live.freqValid = false;
  live.lastFreqPollMs = 0;
  live.tuning = false;
  live.pendingHz = 0;
  live.tuningStartSpokenHz = 0;
  live.modeValid = false;
  live.smValid = false;
  live.lastSmPollMs = 0;
  live.powerValid = false;
  live.swrValid = false;
  live.nrValid = false;
  live.nbValid = false;
  live.notchValid = false;
  live.notchWidthValid = false;
  live.notchWidth = NOTCH_WIDTH_UNKNOWN;
}

void rememberLiveFrequency(uint64_t hz, uint32_t nowMs) {
  live.freqHz = hz;
  live.freqValid = true;
  live.lastFreqMs = nowMs;
}

void rememberLiveMode(uint8_t mode, uint32_t nowMs) {
  live.mode = mode;
  live.modeValid = true;
  live.lastModeMs = nowMs;
}

void rememberLiveSmeter(int32_t raw, uint32_t nowMs) {
  live.smRaw = raw;
  live.smValid = true;
  live.lastSmMs = nowMs;
}

void rememberLivePower(int32_t raw, uint32_t nowMs) {
  live.powerRaw = raw;
  live.powerValid = true;
  live.lastPowerMs = nowMs;
}

void rememberLiveSwr(int32_t raw, uint32_t nowMs) {
  live.swrRaw = raw;
  live.swrValid = true;
  live.lastSwrMs = nowMs;
}

void rememberLiveNr(bool on, uint32_t nowMs) {
  live.nrOn = on;
  live.nrValid = true;
  live.lastNrMs = nowMs;
}

void rememberLiveNb(bool on, uint32_t nowMs) {
  live.nbOn = on;
  live.nbValid = true;
  live.lastNbMs = nowMs;
}

void rememberLiveNotch(bool on, uint32_t nowMs, bool widthValid, NotchWidth width) {
  live.notchOn = on;
  live.notchValid = true;
  live.notchWidthValid = on && widthValid;
  live.notchWidth = (on && widthValid) ? width : NOTCH_WIDTH_UNKNOWN;
  live.lastNotchMs = nowMs;
}
