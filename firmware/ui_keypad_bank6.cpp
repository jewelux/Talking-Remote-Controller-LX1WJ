// Bank 6 keypad actions: FT-8x7 repeater shift, offset and tones.
#include "keypad_actions.h"
#include "debug_log.h"
#include "engine_civ.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_monitor.h"
#include "radio_profile.h"
#include "radio_prefs.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "sd_slots.h"
#include "ui_console_support.h"
#include "ui_keypad.h"
#include "ui_keypad_common.h"
#include "ui_speech.h"

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
  printKeypadCommand(String("BANK6 FT8X7 -> RPT ") + label);
  if (!yaesuCatSetRepeaterShiftRaw(shiftByte)) { keypadReportIfTimedOut("RPT"); return; }
  printKeypadStatus(String("RPT ") + label);
  if (!g_speechEnabled) return;
  speakToken("repeater");
  playSilenceMs(60);
  if (String(label) == "MINUS") speakToken("minus");
  else if (String(label) == "PLUS") speakToken("plus");
  else speakToken("off");
}

static void setBank6Ft8x7RepeaterOffsetHz(uint64_t hz) {
  printKeypadCommand(String("BANK6 FT8X7 -> RPTSHIFT ") + hzToMHzString3(hz));
  if (!yaesuCatSetRepeaterOffsetHzRaw(hz)) { keypadReportIfTimedOut("RPTSHIFT"); return; }
  printKeypadStatus(String("RPTSHIFT ") + hzToMHzString3(hz) + " MHz");
  if (!g_speechEnabled) return;
  speakToken("repeater");
  playSilenceMs(60);
  speakFrequencyWord();
  playSilenceMs(60);
  speakDigitsAndPoint(hzToMHzString3(hz));
}

static void setBank6Ft8x7ToneMode(uint8_t modeByte, const char* label) {
  printKeypadCommand(String("BANK6 FT8X7 -> TONE ") + label);
  if (!yaesuCatSetToneDcsModeRaw(modeByte)) { keypadReportIfTimedOut("TONE"); return; }
  printKeypadStatus(String("TONE ") + label);
  if (!g_speechEnabled) return;
  speakToken("tone");
  playSilenceMs(60);
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
  printKeypadCommand("BANK6 1 DOUBLE -> RPTSHIFT ENTRY");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  keypadInput().beginEntry(InputMode::RptOffsetEntry);
  printKeypadStatus("RPTSHIFT KHZ PLEASE");
  if (g_speechEnabled) {
    speakToken("repeater");
    playSilenceMs(60);
    speakFrequencyWord();
    playSilenceMs(80);
    speakToken("please");
  }
}

void beginBank6CtcssEntry() {
  printKeypadCommand("BANK6 3 LONG -> CTCSS ENTRY");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  keypadInput().beginEntry(InputMode::CtcssEntry);
  printKeypadStatus("CTCSS PLEASE");
  if (g_speechEnabled) {
    speakToken("ctcss");
    playSilenceMs(80);
    speakToken("please");
  }
}

void beginBank6DcsEntry() {
  printKeypadCommand("BANK6 4 LONG -> DCS ENTRY");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  keypadInput().beginEntry(InputMode::DcsEntry);
  printKeypadStatus("DCS PLEASE");
  if (g_speechEnabled) {
    speakToken("dcs");
    playSilenceMs(80);
    speakToken("please");
  }
}

void setBank6RepeaterOff() {
  printKeypadCommand("BANK6 0 SHORT -> RPT OFF");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7RepeaterShift(0x89, "OFF");
}

void setBank6RepeaterMinus() {
  printKeypadCommand("BANK6 0 LONG -> RPT MINUS");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7RepeaterShift(0x09, "MINUS");
}

void setBank6RepeaterPlus() {
  printKeypadCommand("BANK6 0 DOUBLE -> RPT PLUS");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7RepeaterShift(0x49, "PLUS");
}

void setBank6RepeaterOffset1() {
  const uint32_t hz = currentFt8x7RepeaterOffsetHz(0);
  printKeypadCommand(String("BANK6 1 SHORT -> RPTSHIFT ") + hzToMHzString3(hz));
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7RepeaterOffsetHz(hz);
}

void setBank6RepeaterOffset2() {
  const uint32_t hz = currentFt8x7RepeaterOffsetHz(1);
  printKeypadCommand(String("BANK6 1 LONG -> RPTSHIFT ") + hzToMHzString3(hz));
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7RepeaterOffsetHz(hz);
}

void setBank6ToneOff() {
  printKeypadCommand("BANK6 2 SHORT -> TONE OFF");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7ToneMode(0x8A, "OFF");
}

void setBank6ToneModeCtcss() {
  printKeypadCommand("BANK6 2 LONG -> TONE CTCSS");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7ToneMode(0x2A, "CTCSS");
}

void setBank6ToneModeDcs() {
  printKeypadCommand("BANK6 2 DOUBLE -> TONE DCS");
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  setBank6Ft8x7ToneMode(0x0A, "DCS");
}

void queryBank6CtcssDefault() {
  char label[12] = "";
  uint16_t toneTenths = live.ctcssValid ? live.ctcssTenths : currentFt8x7DefaultCtcssTenths();
  formatCtcssTenthsLabel(toneTenths, label, sizeof(label));
  printKeypadCommand(String("BANK6 3 SHORT -> CTCSS ") + label);
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  printKeypadStatus(String("CTCSS ") + label);
  if (!g_speechEnabled) return;
  speakToken("ctcss");
  playSilenceMs(60);
  speakDigitsAndPoint(label);
}

void queryBank6DcsDefault() {
  char label[8] = "";
  const uint16_t dcsCode = live.dcsValid ? live.dcsCode : currentFt8x7DefaultDcsCode();
  snprintf(label, sizeof(label), "%03u", (unsigned)dcsCode);
  printKeypadCommand(String("BANK6 4 SHORT -> DCS ") + label);
  if (!isFt8x7Keypad()) {
    printKeypadStatus("BANK6 reserved");
    playBeep();
    return;
  }
  printKeypadStatus(String("DCS ") + label);
  if (!g_speechEnabled) return;
  speakToken("dcs");
  playSilenceMs(60);
  speakDigitsAndPoint(label);
}
