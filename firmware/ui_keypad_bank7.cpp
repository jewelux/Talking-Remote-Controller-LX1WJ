// Bank 7 keypad actions: CW break-in, keyer, speed, pitch and delay.
#include "ui_keypad_bank.h"
#include "ui_features.h"

void queryBank7BreakIn() {
  printKeypadAction("BK?");
  prepareKeypadSpeechResponse();
  BreakInMode mode = BreakInMode::Off;
  if (keypadReportFeatureFailure(breakInQuery(mode), "BK?")) return;
  printKeypadStatus("{}", breakInText(mode).c_str());
  speakBreakIn(mode);
}

void toggleBank7BreakIn() {
  printKeypadAction("BK");
  prepareKeypadSpeechResponse();
  BreakInMode mode = BreakInMode::Off;
  if (keypadReportFeatureFailure(breakInToggle(mode), "BK")) return;
  printKeypadStatus("{}", breakInText(mode).c_str());
  speakBreakIn(mode);
}

void queryBank7CwLevel(CwLevel level) {
  const String label = String(cwLevelName(level)) + "?";
  printKeypadAction("{}", label.c_str());
  prepareKeypadSpeechResponse();
  uint16_t value = 0;
  if (keypadReportFeatureFailure(cwLevelQuery(level, value), label.c_str())) return;
  printKeypadStatus("{}", cwLevelText(level, value).c_str());
  speakCwLevel(level, value);
}

void stepBank7CwLevel(CwLevel level, int delta) {
  const char* name = cwLevelName(level);
  printKeypadAction("{} STEP {}", name, delta);
  prepareKeypadSpeechResponse();
  uint16_t value = 0;
  if (keypadReportFeatureFailure(cwLevelStep(level, delta, value), name)) return;
  printKeypadStatus("{}", cwLevelText(level, value).c_str());
  speakCwLevel(level, value);
}

void queryBank7Keyer() {
  printKeypadAction("KYR?");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(keyerQuery(on), "KYR?")) return;
  printKeypadStatus("{}", keyerText(on).c_str());
  speakKeyer(on);
}

void toggleBank7Keyer() {
  printKeypadAction("KYR");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(keyerToggle(on), "KYR")) return;
  printKeypadStatus("{}", keyerText(on).c_str());
  speakKeyer(on);
}
