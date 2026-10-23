// Bank 7 keypad actions: CW break-in and keyer.
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
