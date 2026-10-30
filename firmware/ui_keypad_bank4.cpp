// Bank 4 keypad actions: the tuner.
#include "ui_keypad_bank.h"

void queryBank4Tuner() {
  printKeypadAction("TUNER?");
  KeypadAnswer answer("TUNER?");
  if (answer.unavailable(protocolSupportsTuner())) return;
  bool on = false;
  if (!queryTuner(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("TUNER {}", on ? "ON" : "OFF");
  speakTokenState("tuner", on);
}

void toggleBank4Tuner() {
  printKeypadAction("TUNER");
  KeypadAnswer answer("TUNER");
  if (answer.unavailable(protocolSupportsTuner())) return;
  bool on = false;
  if (!queryTuner(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  if (!setTuner(!on)) { answer.radioFailed(); return; }
  printKeypadStatus("TUNER {}", !on ? "ON" : "OFF");
  speakTokenState("tuner", !on);
}

void triggerBank4Tune() {
  printKeypadAction("TUNE");
  KeypadAnswer answer("TUNE");
  if (answer.unavailable(protocolSupportsTuner())) return;
  if (!startTune()) { answer.radioFailed(); return; }
  printKeypadStatus("TUNE");
  answer.done();
}
