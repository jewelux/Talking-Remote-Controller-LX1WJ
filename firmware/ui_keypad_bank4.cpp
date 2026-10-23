// Bank 4 keypad actions: the tuner.
#include "ui_keypad_bank.h"

void queryBank4Tuner() {
  printKeypadAction("TUNER?");
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNER?")) return;
  bool on = false;
  if (!queryTuner(on, kCatReplyTimeoutMs)) { keypadReportFailure("TUNER?"); return; }
  printKeypadStatus("TUNER {}", on ? "ON" : "OFF");
  speakTokenState("tuner", on);
}

void toggleBank4Tuner() {
  printKeypadAction("TUNER");
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNER")) return;
  bool on = false;
  if (!queryTuner(on, kCatReplyTimeoutMs)) { keypadReportFailure("TUNER"); return; }
  if (!setTuner(!on)) { keypadReportFailure("TUNER"); return; }
  printKeypadStatus("TUNER {}", !on ? "ON" : "OFF");
  speakTokenState("tuner", !on);
}

void triggerBank4Tune() {
  printKeypadAction("TUNE");
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNE")) return;
  if (!startTune()) { keypadReportFailure("TUNE"); return; }
  printKeypadStatus("TUNE");
  if (g_speechEnabled) speakToken("tune");
}
