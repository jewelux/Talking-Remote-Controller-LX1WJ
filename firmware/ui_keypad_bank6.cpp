// Bank 6 keypad actions: repeater shift, the FT-8x7 repeater offset, and tone squelch.
#include "ui_keypad_bank.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_utils.h"
#include "ui_features.h"

static uint32_t currentFt8x7RepeaterOffsetHz(uint8_t index) {
  if (index >= 2) return 0;
  return currentProfile().ft8x7Bank6.repeaterOffsetsHz[index];
}

// The FT-8x7 sends its own shift byte and cannot read it back. A CAT radio sets the shift and
// says what it reads back: the Yaesu OS0 works in FM only.
static void setBank6RepeaterShift(RptShift shift, uint8_t ft8x7Byte) {
  printKeypadAction("{}", rptShiftText(shift).c_str());
  KeypadAnswer answer("RPT");
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    yaesuCatSetRepeaterShiftRaw(ft8x7Byte);
    printKeypadStatus("{}", rptShiftText(shift).c_str());
    speakRptShift(shift);
    return;
  }
  prepareKeypadRadioWrite();
  if (answer.failed(rptShiftSet(shift))) return;
  RptShift readBack = shift;
  if (rptShiftQuery(readBack) != FeatureStatus::Ok) readBack = shift;
  printKeypadStatus("{}", rptShiftText(readBack).c_str());
  speakRptShift(readBack);
}

static void setBank6Ft8x7RepeaterOffsetHz(uint64_t hz) {
  printKeypadAction("RPTSHIFT {}", RadioFrequency::fromHz(hz));
  KeypadAnswer answer("RPTSHIFT");
  yaesuCatSetRepeaterOffsetHzRaw(hz);
  printKeypadStatus("RPTSHIFT {} MHz", RadioFrequency::fromHz(hz));
  if (!g_speechEnabled) return;
  speakLabel("repeater frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

static void setBank6ToneType(YaesuToneType type) {
  printKeypadAction("TONE {}", type == YaesuToneType::Ctcss ? "CTCSS" : "DCS");
  KeypadAnswer answer("TONE");
  YaesuToneMode mode;
  if (answer.failed(toneTypeSet(type, mode))) return;
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

void setBank6RepeaterOff() {
  setBank6RepeaterShift(RptShift::Simplex, 0x89);
}

void setBank6RepeaterMinus() {
  setBank6RepeaterShift(RptShift::Minus, 0x09);
}

void setBank6RepeaterPlus() {
  setBank6RepeaterShift(RptShift::Plus, 0x49);
}

void setBank6RepeaterOffsetPreset(uint8_t preset) {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) {
    printKeypadAction("RPTSHIFT");
    keypadReportNotAvailable("RPTSHIFT");
    return;
  }
  setBank6Ft8x7RepeaterOffsetHz(currentFt8x7RepeaterOffsetHz(preset - 1));
}

void queryBank6Tone() {
  printKeypadAction("TONE?");
  KeypadAnswer answer("TONE?");
  YaesuToneMode mode;
  if (answer.failed(toneModeQuery(mode))) return;
  printKeypadStatus("{}", toneTypeText(mode).c_str());
  speakToneType(mode);
}

void setBank6ToneOff() {
  printKeypadAction("TONE OFF");
  KeypadAnswer answer("TONE");
  if (answer.failed(toneOff())) return;
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
  KeypadAnswer answer("TONEDIR?");
  YaesuToneMode mode;
  if (answer.failed(toneModeQuery(mode))) return;
  printKeypadStatus("{}", toneDirText(mode).c_str());
  speakToneDir(mode);
}

void stepBank6ToneDir() {
  printKeypadAction("TONEDIR");
  KeypadAnswer answer("TONEDIR");
  YaesuToneMode mode;
  if (answer.failed(toneDirStep(mode))) return;
  printKeypadStatus("{}", toneModeText(mode).c_str());
  speakToneMode(mode);
}

void queryBank6SplitTone() {
  printKeypadAction("SPLITTONE?");
  KeypadAnswer answer("SPLITTONE?");
  bool on = false;
  if (answer.failed(splitToneQuery(on))) return;
  printKeypadStatus("{}", splitToneText(on).c_str());
  speakSplitTone(on);
}

