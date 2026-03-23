#include "radio_runtime.h"

#include "radio_protocol.h"
#include "radio_state.h"
#include "radio_utils.h"

bool refreshLiveFrequency() {
  uint64_t hz = 0;
  if (!queryFrequency(hz)) return false;
  rememberLiveFrequency(hz, millis());
  return true;
}

bool refreshLiveMode() {
  uint8_t mode = 0xFF;
  if (!queryMode(mode)) return false;
  rememberLiveMode(mode, millis());
  return true;
}

bool refreshLiveSmeter() {
  int32_t raw = 0;
  if (!querySMeterRaw(raw)) return false;
  rememberLiveSmeter(raw, millis());
  live.smS = smRawToS(raw);
  return true;
}

bool refreshLivePower() {
  int32_t raw = 0;
  if (!queryPoMeterRaw(raw)) return false;
  rememberLivePower(raw, millis());
  return true;
}

bool refreshLiveSwr() {
  int32_t raw = 0;
  if (!querySWRRaw(raw)) return false;
  rememberLiveSwr(raw, millis());
  return true;
}

bool refreshLiveNr() {
  bool on = false;
  if (!queryNr(on)) return false;
  rememberLiveNr(on, millis());
  return true;
}

bool refreshLiveNb() {
  bool on = false;
  if (!queryNb(on)) return false;
  rememberLiveNb(on, millis());
  return true;
}

bool refreshLiveNotch() {
  bool on = false;
  if (!queryNotch(on)) return false;
  bool widthValid = false;
  NotchWidth width = NOTCH_WIDTH_UNKNOWN;
  if (on) widthValid = queryNotchWidth(width);
  rememberLiveNotch(on, millis(), widthValid, width);
  return true;
}

bool refreshLiveNotchWidth() {
  if (!live.notchOn) return false;
  NotchWidth width = NOTCH_WIDTH_UNKNOWN;
  if (!queryNotchWidth(width)) return false;
  rememberLiveNotch(true, millis(), true, width);
  return true;
}

bool applyFrequencyAndTrack(uint64_t hz, bool suppressSpeechWindow) {
  if (!setFrequency(hz)) return false;
  if (suppressSpeechWindow) g_suppressFreqSpeakUntilMs = millis() + 1500;
  rememberLiveFrequency(hz, millis());
  return true;
}

bool applyModeAndTrack(uint8_t mode, uint8_t filter) {
  if (!setMode(mode, filter)) return false;
  rememberLiveMode(mode, millis());
  return true;
}

bool applyNrAndTrack(bool on) {
  if (!setNr(on)) return false;
  rememberLiveNr(on, millis());
  return true;
}

bool applyNbAndTrack(bool on) {
  if (!setNb(on)) return false;
  rememberLiveNb(on, millis());
  return true;
}

bool applyNotchAndTrack(bool on) {
  if (!setNotch(on)) return false;
  bool widthValid = false;
  NotchWidth width = NOTCH_WIDTH_UNKNOWN;
  if (on) widthValid = queryNotchWidth(width);
  rememberLiveNotch(on, millis(), widthValid, width);
  return true;
}

bool applyNotchWidthAndTrack(NotchWidth width) {
  if (width == NOTCH_WIDTH_UNKNOWN) return false;
  if (!setNotchWidth(width)) return false;
  rememberLiveNotch(true, millis(), true, width);
  return true;
}
