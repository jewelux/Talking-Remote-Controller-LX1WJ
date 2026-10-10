// Bank 6 keypad actions: FT-8x7 repeater shift, offset and tones.
#include "ui_keypad_bank.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_utils.h"
#include "ui_features.h"

static uint32_t currentFt8x7RepeaterOffsetHz(uint8_t index) {
  if (index >= 2) return 0;
  return currentProfile().ft8x7Bank6.repeaterOffsetsHz[index];
}

static uint16_t currentFt8x7DefaultCtcssTenths() {
  return currentProfile().ft8x7Bank6.ctcssDefaultTenths;
}

static uint16_t currentFt8x7DefaultDcsCode() {
  return currentProfile().ft8x7Bank6.dcsDefaultCode;
}

static void setBank6Ft8x7RepeaterShift(uint8_t shiftByte, const char* label) {
  printKeypadAction("RPT {}", label);
  yaesuCatSetRepeaterShiftRaw(shiftByte);
  printKeypadStatus("RPT {}", label);
  if (!g_speechEnabled) return;
  speakLabel("repeater");
  if (String(label) == "MINUS") speakToken("minus");
  else if (String(label) == "PLUS") speakToken("plus");
  else speakToken("off");
}

static void setBank6Ft8x7RepeaterOffsetHz(uint64_t hz) {
  printKeypadAction("RPTSHIFT {}", RadioFrequency::fromHz(hz));
  yaesuCatSetRepeaterOffsetHzRaw(hz);
  printKeypadStatus("RPTSHIFT {} MHz", RadioFrequency::fromHz(hz));
  if (!g_speechEnabled) return;
  speakLabel("repeater frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

static void setBank6ToneType(YaesuToneType type) {
  printKeypadAction("TONE {}", type == YaesuToneType::Ctcss ? "CTCSS" : "DCS");
  prepareKeypadSpeechResponse();
  YaesuToneMode mode;
  if (keypadReportFeatureFailure(toneTypeSet(type, mode), "TONE")) return;
  printKeypadStatus("{}", toneModeText(mode).c_str());
  speakToneMode(mode);
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

void queryBank6Tone() {
  printKeypadAction("TONE?");
  prepareKeypadSpeechResponse();
  YaesuToneMode mode;
  if (keypadReportFeatureFailure(toneModeQuery(mode), "TONE?")) return;
  printKeypadStatus("{}", toneTypeText(mode).c_str());
  speakToneType(mode);
}

void setBank6ToneOff() {
  printKeypadAction("TONE OFF");
  prepareKeypadSpeechResponse();
  if (keypadReportFeatureFailure(toneOff(), "TONE")) return;
  const YaesuToneMode off;
  printKeypadStatus("{}", toneModeText(off).c_str());
  speakToneMode(off);
}

void setBank6ToneModeCtcss() {
  setBank6ToneType(YaesuToneType::Ctcss);
}

void setBank6ToneModeDcs() {
  setBank6ToneType(YaesuToneType::Dcs);
}

void queryBank6ToneDir() {
  printKeypadAction("TONEDIR?");
  prepareKeypadSpeechResponse();
  YaesuToneMode mode;
  if (keypadReportFeatureFailure(toneModeQuery(mode), "TONEDIR?")) return;
  printKeypadStatus("{}", toneDirText(mode).c_str());
  speakToneDir(mode);
}

void toggleBank6ToneDir() {
  printKeypadAction("TONEDIR");
  prepareKeypadSpeechResponse();
  YaesuToneMode mode;
  if (keypadReportFeatureFailure(toneEncodeToggle(mode), "TONEDIR")) return;
  printKeypadStatus("{}", toneModeText(mode).c_str());
  speakToneMode(mode);
}

void queryBank6CtcssDefault() {
  char label[12] = "";
  uint16_t toneTenths = live.ctcssValid ? live.ctcssTenths : currentFt8x7DefaultCtcssTenths();
  formatCtcssTenthsLabel(toneTenths, label, sizeof(label));
  printKeypadAction("CTCSS {}", label);
  printKeypadStatus("CTCSS {}", label);
  if (!g_speechEnabled) return;
  speakLabel("ctcss");
  speakDigitsAndPoint(label);
}

void queryBank6DcsDefault() {
  char label[8] = "";
  const uint16_t dcsCode = live.dcsValid ? live.dcsCode : currentFt8x7DefaultDcsCode();
  snprintf(label, sizeof(label), "%03u", (unsigned)dcsCode);
  printKeypadAction("DCS {}", label);
  printKeypadStatus("DCS {}", label);
  if (!g_speechEnabled) return;
  speakLabel("dcs");
  speakDigitsAndPoint(label);
}
