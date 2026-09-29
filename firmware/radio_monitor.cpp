#include "radio_monitor.h"

#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_protocol.h"
#include "radio_state.h"
#include "ui_speech.h"
#include "radio_utils.h"
#include "debug_log.h"

static uint64_t freqDiffHz(uint64_t a, uint64_t b) {
  return (a > b) ? (a - b) : (b - a);
}

// Small moves relative to the last announced frequency are not worth announcing.
static bool freqDiffersEnoughToSpeak(uint64_t hz) {
  return live.lastSpokenHz == 0 || freqDiffHz(hz, live.lastSpokenHz) >= FREQ_SPEAK_MIN_STEP_HZ;
}

// Queues hz as a tuning announcement, replacing any tuning announcement still playing.
static void speakTuningFrequency(uint64_t hz) {
  live.lastSpokenHz = hz;
  beginTuningSpeech();
  speakDigitsAndPoint(hzToMHzString3(hz));
  endTuningSpeech();
}

static bool tuningSpeechGapElapsed(uint32_t now) {
  return !tuningSpeechActive() && now - tuningSpeechEndedMs() >= FREQ_SPEAK_MIN_GAP_MS;
}

void updateFreqSpeechDebounce(uint64_t newHz) {
  const uint32_t now = millis();
  // The dial moved away from the frequency being read out: stop the stale
  // announcement. Part of it was heard, so the user no longer knows where the
  // radio is: forget the last announced frequency and announce the next stop.
  if (tuningSpeechActive() && freqDiffersEnoughToSpeak(newHz)) {
    cancelTuningSpeech();
    live.lastSpokenHz = 0;
  }
  if (!g_tuningSpeakEnabled) return;
  if ((int32_t)(now - g_suppressFreqSpeakUntilMs) < 0) return;
  if (!live.tuning) {
    live.tuning = true;
    live.tuningStartSpokenHz = 0;
  }
  live.pendingHz = newHz;
  live.lastChangeMs = now;
  if (g_speechEnabled && FREQ_SPEAK_START_IMMEDIATELY && live.tuningStartSpokenHz == 0 && tuningSpeechGapElapsed(now) && freqDiffersEnoughToSpeak(newHz)) {
    live.tuningStartSpokenHz = newHz;
    speakTuningFrequency(newHz);
  }
}

void speakPendingFreqIfIdle() {
  if (!g_speechEnabled || !g_tuningSpeakEnabled) return;
  const uint32_t now = millis();
  if ((int32_t)(now - g_suppressFreqSpeakUntilMs) < 0) return;
  if (!live.tuning || live.pendingHz == 0) return;
  if (now - live.lastChangeMs < FREQ_SPEAK_IDLE_MS || !tuningSpeechGapElapsed(now)) return;
  if (freqDiffersEnoughToSpeak(live.pendingHz)) speakTuningFrequency(live.pendingHz);
  live.tuning = false;
}

struct FreqPollPolicy {
  uint32_t intervalMs;
  uint32_t timeoutMs;
  // Accept a changed frequency only after two identical readings; guards
  // protocols without framing against a single misaligned reply.
  bool confirmChanges;
};

static FreqPollPolicy freqPollPolicyFor(ProtocolType pt) {
  // The frame checks (BCD, range, quiet line) are enough on their own; set to
  // true to require two identical readings if misread frequencies show up.
  if (pt == PROTO_YAESU_FT8X7) return {FREQ_POLL_MS_FT8X7, FREQ_POLL_TIMEOUT_MS_FT8X7, false};
  return {FREQ_POLL_MS, FREQ_POLL_TIMEOUT_MS, false};
}

// Returns true when hz should be accepted as the observed frequency.
static bool confirmPolledFrequency(uint64_t hz) {
  if (live.freqValid && hz == live.freqHz) {
    live.freqPollCandidateValid = false;
    return true;
  }
  if (live.freqPollCandidateValid && hz == live.freqPollCandidateHz) {
    live.freqPollCandidateValid = false;
    return true;
  }
  live.freqPollCandidateValid = true;
  live.freqPollCandidateHz = hz;
  return false;
}

void cancelPendingFreqAnnouncement() {
  live.tuning = false;
  live.pendingHz = 0;
  live.tuningStartSpokenHz = 0;
}

void pollFrequencyIfDue() {
  if (!FREQ_POLL_ENABLE) return;
  if (!currentStoredProfile().caps.getFreq) return;

  const FreqPollPolicy policy = freqPollPolicyFor(currentProtocolType());
  const uint32_t intervalMs = (live.freqPollFailures >= FREQ_POLL_BACKOFF_AFTER_FAILURES) ? FREQ_POLL_BACKOFF_MS : policy.intervalMs;

  const uint32_t now = millis();
  if ((int32_t)(now - g_suspendPollingUntilMs) < 0) return;
  if (now - live.lastFreqPollMs < intervalMs) return;
  live.lastFreqPollMs = now;

  uint64_t hz = 0;
  if (!queryFrequency(hz, policy.timeoutMs)) {
    if (live.freqPollFailures < 0xFF) ++live.freqPollFailures;
    return;
  }
  live.freqPollFailures = 0;
  if (policy.confirmChanges && !confirmPolledFrequency(hz)) return;
  handleObservedFrequency(hz, false);
}

void handleSMeterRaw(int32_t raw) {
  rememberLiveSmeter(raw, sMeterFromRaw(raw), millis());
  if (!g_quiet) {
    DBG_PRINT("SM: raw=");
    DBG_PRINT(raw);
    DBG_PRINT("  ");
    DBG_PRINTLN(live.sm.toString());
  }
  if (!g_speechEnabled || !SMETER_SPEAK_ENABLE) return;
  const uint32_t now = millis();
  if (now - live.lastSmSpokenMs < SMETER_SPEAK_MIN_INTERVAL_MS) return;
  const uint8_t steps = live.sm.steps();
  const bool firstReading = live.lastSpokenSmSteps == 0xFF;
  const uint8_t diff = (steps > live.lastSpokenSmSteps) ? (steps - live.lastSpokenSmSteps) : (live.lastSpokenSmSteps - steps);
  if (!firstReading && diff < SMETER_SPEAK_MIN_DELTA_S) return;
  live.lastSpokenSmSteps = steps;
  live.lastSmSpokenMs = now;
  speakSValue(live.sm);
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
