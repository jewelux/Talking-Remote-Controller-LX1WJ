// Bank 6 keypad actions: FT-8x7 repeater shift, offset and tones.
#include "ui_keypad_bank.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_utils.h"

static uint32_t currentFt8x7RepeaterOffsetHz(uint8_t index) {
  if (index >= 2) return 0;
  return currentStoredProfile().ft8x7Bank6.repeaterOffsetsHz[index];
}

static uint16_t currentFt8x7DefaultCtcssTenths() {
  return currentStoredProfile().ft8x7Bank6.ctcssDefaultTenths;
}

static uint16_t currentFt8x7DefaultDcsCode() {
  return currentStoredProfile().ft8x7Bank6.dcsDefaultCode;
}

static void setBank6Ft8x7RepeaterShift(uint8_t shiftByte, const char* label) {
  printKeypadAction(String("RPT ") + label);
  yaesuCatSetRepeaterShiftRaw(shiftByte);
  printKeypadStatus(String("RPT ") + label);
  if (!g_speechEnabled) return;
  speakLabel("repeater");
  if (String(label) == "MINUS") speakToken("minus");
  else if (String(label) == "PLUS") speakToken("plus");
  else speakToken("off");
}

static void setBank6Ft8x7RepeaterOffsetHz(uint64_t hz) {
  printKeypadAction(String("RPTSHIFT ") + hzToMHzString3(hz));
  yaesuCatSetRepeaterOffsetHzRaw(hz);
  printKeypadStatus(String("RPTSHIFT ") + hzToMHzString3(hz) + " MHz");
  if (!g_speechEnabled) return;
  speakLabel("repeater frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

static void setBank6Ft8x7ToneMode(uint8_t modeByte, const char* label) {
  printKeypadAction(String("TONE ") + label);
  yaesuCatSetToneDcsModeRaw(modeByte);
  printKeypadStatus(String("TONE ") + label);
  if (!g_speechEnabled) return;
  speakLabel("tone");
  if (String(label) == "OFF") {
    speakToken("off");
  } else if (String(label) == "CTCSS") {
    speakToken("ctcss");
    playSilenceMs(60);
    speakToken("on");
  } else {
    speakToken("dcs");
    playSilenceMs(60);
    speakToken("on");
  }
}

void beginBank6RepeaterOffsetEntry() {
  printKeypadAction("RPTSHIFT ENTRY");
  keypadBeginEntry(InputMode::RptOffsetEntry);
  printKeypadStatus("RPTSHIFT KHZ PLEASE");
  if (g_speechEnabled) {
    speakToken("repeater");
    playSilenceMs(60);
    speakFrequencyWord();
    speakPlease();
  }
}

void beginBank6CtcssEntry() {
  printKeypadAction("CTCSS ENTRY");
  keypadBeginEntry(InputMode::CtcssEntry);
  printKeypadStatus("CTCSS PLEASE");
  speakPrompt("ctcss");
}

void beginBank6DcsEntry() {
  printKeypadAction("DCS ENTRY");
  keypadBeginEntry(InputMode::DcsEntry);
  printKeypadStatus("DCS PLEASE");
  speakPrompt("dcs");
}

void setBank6RepeaterOff() {
  setBank6Ft8x7RepeaterShift(0x89, "OFF");
}

void setBank6RepeaterMinus() {
  setBank6Ft8x7RepeaterShift(0x09, "MINUS");
}

void setBank6RepeaterPlus() {
  setBank6Ft8x7RepeaterShift(0x49, "PLUS");
}

void setBank6RepeaterOffsetPreset(uint8_t preset) {
  setBank6Ft8x7RepeaterOffsetHz(currentFt8x7RepeaterOffsetHz(preset - 1));
}

void setBank6ToneOff() {
  setBank6Ft8x7ToneMode(0x8A, "OFF");
}

void setBank6ToneModeCtcss() {
  setBank6Ft8x7ToneMode(0x2A, "CTCSS");
}

void setBank6ToneModeDcs() {
  setBank6Ft8x7ToneMode(0x0A, "DCS");
}

void queryBank6CtcssDefault() {
  char label[12] = "";
  uint16_t toneTenths = live.ctcssValid ? live.ctcssTenths : currentFt8x7DefaultCtcssTenths();
  formatCtcssTenthsLabel(toneTenths, label, sizeof(label));
  printKeypadAction(String("CTCSS ") + label);
  printKeypadStatus(String("CTCSS ") + label);
  if (!g_speechEnabled) return;
  speakLabel("ctcss");
  speakDigitsAndPoint(label);
}

void queryBank6DcsDefault() {
  char label[8] = "";
  const uint16_t dcsCode = live.dcsValid ? live.dcsCode : currentFt8x7DefaultDcsCode();
  snprintf(label, sizeof(label), "%03u", (unsigned)dcsCode);
  printKeypadAction(String("DCS ") + label);
  printKeypadStatus(String("DCS ") + label);
  if (!g_speechEnabled) return;
  speakLabel("dcs");
  speakDigitsAndPoint(label);
}
