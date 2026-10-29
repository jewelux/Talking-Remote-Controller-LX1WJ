// Bank 5 keypad actions: RIT.
#include "ui_keypad_bank.h"
#include "keypad_setting_ops.h"
#include "radio_frequency.h"

static void speakRitOffsetValue(int32_t hz) { settingOps(Setting::RitOffset)->speak(hz); }

void queryBank5Rit() {
  printKeypadAction("RIT?");
  KeypadAnswer answer("RIT?");
  if (answer.unavailable(protocolSupportsRit())) return;
  bool on = false;
  int32_t offset = 0;
  if (!queryRitEnabled(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  if (!queryRitOffsetHz(offset, kCatReplyTimeoutMs)) offset = 0;
  printKeypadStatus("RIT {} {} Hz", on ? "ON" : "OFF", offset);
  if (!g_speechEnabled) return;
  speakLabel("rit");
  speakToken(on ? "on" : "off");
  if (offset != 0) {
    playSilenceMs(60);
    speakRitOffsetValue(offset);
  }
}

void toggleBank5Rit() {
  printKeypadAction("RIT");
  KeypadAnswer answer("RIT");
  if (answer.unavailable(protocolSupportsRit())) return;
  bool on = false;
  if (!queryRitEnabled(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  if (!setRitEnabled(!on)) { answer.radioFailed(); return; }
  printKeypadStatus("RIT {}", !on ? "ON" : "OFF");
  speakTokenState("rit", !on);
}

void setBank5RitOffset(int32_t hz) {
  printKeypadAction("RIT {} Hz", hz);
  KeypadAnswer answer("RIT");
  if (answer.unavailable(protocolSupportsRit())) return;
  if (!setRitOffsetHz(hz)) { answer.radioFailed(); return; }
  printKeypadStatus("RIT {} Hz", hz);
  speakRitOffsetValue(hz);
}

void adjustBank5Rit(int32_t deltaHz) {
  printKeypadAction("RIT STEP {}{} Hz", deltaHz >= 0 ? "+" : "", deltaHz);
  KeypadAnswer answer("RIT");
  if (answer.unavailable(protocolSupportsRit())) return;
  int32_t offset = 0;
  if (!queryRitOffsetHz(offset, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  int32_t next = offset + deltaHz;
  if (next < -9999) next = -9999;
  if (next > 9999) next = 9999;
  setBank5RitOffset(next);
}

void setBank5RitOff() {
  printKeypadAction("RIT OFF");
  KeypadAnswer answer("RIT");
  if (answer.unavailable(protocolSupportsRit())) return;
  if (!setRitEnabled(false)) { answer.radioFailed(); return; }
  printKeypadStatus("RIT OFF");
  if (g_speechEnabled) speakToken("off");
}
