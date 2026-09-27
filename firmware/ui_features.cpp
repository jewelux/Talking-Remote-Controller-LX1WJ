#include "ui_features.h"
#include "ui_keypad_common.h"  // speakNotchCycleState
#include "ui_speech.h"

const char* featureStatusText(FeatureStatus status) {
  switch (status) {
    case FeatureStatus::Unsupported: return "unsupported";
    case FeatureStatus::Timeout: return "timeout";
    case FeatureStatus::NoReply: return "no reply";
    case FeatureStatus::Failed: return "failed";
    default: return "";
  }
}

String nrStateText(const NrState& state) {
  if (!state.on) return "NR OFF";
  if (state.level > 0) return String("NR ") + String((int)state.level);
  return "NR ON";
}

// Speech by token, not by clip: a clip named in a new file is another copy of
// it in flash (voice_data.h).
void speakNrState(const NrState& state) {
  if (!g_speechEnabled) return;
  if (state.level == 0) {
    speakTokenState("noisereduction", state.on);
    return;
  }
  speakToken("noisereduction");
  playSilenceMs(60);
  playDigit(state.level);
}

String nbStateText(bool on) {
  return on ? "NB ON" : "NB OFF";
}

void speakNbState(bool on) {
  speakTokenState("noiseblanker", on);
}

String notchStateText(const NotchState& state) {
  if (!state.on) return "NOTCH OFF";
  switch (state.width) {
    case NOTCH_WIDTH_NAR: return "NOTCH NAR";
    case NOTCH_WIDTH_MID: return "NOTCH MID";
    case NOTCH_WIDTH_WIDE: return "NOTCH WIDE";
    default: return "NOTCH ON";
  }
}

void speakNotchState(const NotchState& state) {
  speakNotchCycleState(state.on, state.width);
}
