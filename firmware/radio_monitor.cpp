#include "radio_monitor.h"

#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_protocol.h"
#include "radio_state.h"
#include "ui_speech.h"
#include "radio_utils.h"
#include "debug_log.h"

void updateFreqSpeechDebounce(uint64_t newHz) {
  const uint32_t now = millis();
  if (!g_tuningSpeakEnabled) return;
  if ((int32_t)(now - g_suppressFreqSpeakUntilMs) < 0) return;
  if (live.pendingHz != 0) {
    uint64_t diff = (newHz > live.pendingHz) ? (newHz - live.pendingHz) : (live.pendingHz - newHz);
    if (diff < FREQ_SPEAK_MIN_STEP_HZ) return;
  }
  if (!live.tuning) {
    live.tuning = true;
    live.tuningStartSpokenHz = 0;
  }
  live.pendingHz = newHz;
  live.lastChangeMs = now;
  if (g_speechEnabled && FREQ_SPEAK_START_IMMEDIATELY && live.tuningStartSpokenHz == 0 && now - live.lastSpokenMs >= FREQ_SPEAK_MIN_INTERVAL_MS) {
    live.tuningStartSpokenHz = newHz;
    live.lastSpokenHz = newHz;
    live.lastSpokenMs = now;
    speakDigitsAndPoint(hzToMHzString3(newHz));
  }
}

void speakPendingFreqIfIdle() {
  if (!g_speechEnabled || !g_tuningSpeakEnabled) return;
  const uint32_t now0 = millis();
  if ((int32_t)(now0 - g_suppressFreqSpeakUntilMs) < 0) return;
  if (!live.tuning || live.pendingHz == 0) return;
  const uint32_t now = millis();
  if (now - live.lastChangeMs < FREQ_SPEAK_IDLE_MS || now - live.lastSpokenMs < FREQ_SPEAK_MIN_INTERVAL_MS) return;
  if (live.pendingHz != live.lastSpokenHz) {
    live.lastSpokenHz = live.pendingHz;
    live.lastSpokenMs = now;
    speakDigitsAndPoint(hzToMHzString3(live.pendingHz));
  }
  live.tuning = false;
}

void pollFrequencyIfDue() {
  if (!FREQ_POLL_ENABLE) return;
  if (!currentStoredProfile().caps.getFreq) return;
  if (currentProtocolType() == PROTO_YAESU_FT8X7) return;

  const uint32_t now = millis();
  if ((int32_t)(now - g_suspendPollingUntilMs) < 0) return;
  if (now - live.lastFreqPollMs < FREQ_POLL_MS) return;
  live.lastFreqPollMs = now;

  uint64_t hz = 0;
  if (!queryFrequency(hz, FREQ_POLL_TIMEOUT_MS)) return;
  handleObservedFrequency(hz, false);
}

void handleSMeterRaw(int32_t raw) {
  rememberLiveSmeter(raw, millis());
  live.smS = smRawToS(raw);
  if (!g_quiet) {
    DBG_PRINT("SM: raw=");
    DBG_PRINT(raw);
    DBG_PRINT("  est=S");
    DBG_PRINTLN((int)live.smS);
  }
  if (!g_speechEnabled || !SMETER_SPEAK_ENABLE) return;
  const uint32_t now = millis();
  if (now - live.lastSmSpokenMs < SMETER_SPEAK_MIN_INTERVAL_MS) return;
  if (live.lastSpokenS == 0xFF) {
    live.lastSpokenS = live.smS;
    live.lastSmSpokenMs = now;
    speakSValue(live.smS);
    return;
  }
  uint8_t diff = (live.smS > live.lastSpokenS) ? (live.smS - live.lastSpokenS) : (live.lastSpokenS - live.smS);
  if (diff >= SMETER_SPEAK_MIN_DELTA_S) {
    live.lastSpokenS = live.smS;
    live.lastSmSpokenMs = now;
    speakSValue(live.smS);
  }
}

void pollSMeterIfDue() {
  if (!SMETER_POLL_ENABLE) return;
  const uint32_t now = millis();
  if ((int32_t)(now - g_suspendPollingUntilMs) < 0) return;
  if (now - live.lastSmPollMs < SMETER_POLL_MS) return;
  live.lastSmPollMs = now;
  int32_t raw = 0;
  if (querySMeterRaw(raw, SMETER_POLL_TIMEOUT_MS)) handleSMeterRaw(raw);
}

void handleObservedFrequency(uint64_t hz, bool verbose) {
  const bool changed = !live.freqValid || hz != live.freqHz;
  rememberLiveFrequency(hz, millis());
  if (verbose && !g_quiet) {
    DBG_PRINT("FREQ: ");
    DBG_PRINT(hzToMHzString3(hz));
    DBG_PRINT(" MHz (");
    DBG_PRINT(hz);
    DBG_PRINTLN(" Hz)");
  }
  if (changed) updateFreqSpeechDebounce(hz);
}

void handleObservedMode(uint8_t mode, bool verbose) {
  if (live.modeValid && mode == live.mode) return;
  rememberLiveMode(mode, millis());
  if (verbose) {
    DBG_PRINT("MODE: ");
    DBG_PRINTLN(modeToString(mode));
  }
  speakMode(mode);
}
