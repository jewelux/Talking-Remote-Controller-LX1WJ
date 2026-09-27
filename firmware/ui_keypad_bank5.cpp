// Bank 5 keypad actions: RIT.
#include "ui_keypad_bank.h"
#include "radio_frequency.h"

static void speakRitLabel() {
  if (!g_speechEnabled) return;
  speakToken("rit");
}

static void speakRitOffsetValue(int32_t hz) {
  if (!g_speechEnabled) return;
  speakRitLabel();
  playSilenceMs(60);
  if (hz > 0) {
    speakToken("plus");
    playSilenceMs(60);
  }
  if (hz < 0) {
    speakToken("minus");
    playSilenceMs(60);
  }
  speakDigitsAndPoint(String(hz < 0 ? -hz : hz));
  playSilenceMs(60);
  speakToken("hertz");
}

void queryBank5Rit() {
  printKeypadAction("RIT?");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT?")) return;
  bool on = false;
  int32_t offset = 0;
  if (!queryRitEnabled(on, 800)) { keypadReportIfTimedOut("RIT?"); return; }
  if (!queryRitOffsetHz(offset, 800)) offset = 0;
  printKeypadStatus(String(on ? "RIT ON " : "RIT OFF ") + String(offset) + " Hz");
  if (!g_speechEnabled) return;
  speakRitLabel();
  playSilenceMs(60);
  speakToken(on ? "on" : "off");
  if (offset != 0) {
    playSilenceMs(60);
    speakRitOffsetValue(offset);
  }
}

void toggleBank5Rit() {
  printKeypadAction("RIT");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  bool on = false;
  if (!queryRitEnabled(on, 800)) { keypadReportIfTimedOut("RIT"); return; }
  if (!setRitEnabled(!on)) { keypadReportIfTimedOut("RIT"); return; }
  printKeypadStatus(!on ? "RIT ON" : "RIT OFF");
  speakRitLabel();
  playSilenceMs(60);
  speakToken(!on ? "on" : "off");
}

void setBank5RitOffset(int32_t hz) {
  printKeypadAction(String("RIT ") + String(hz) + " Hz");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  if (!setRitOffsetHz(hz)) { keypadReportIfTimedOut("RIT"); return; }
  printKeypadStatus(String("RIT ") + String(hz) + " Hz");
  speakRitOffsetValue(hz);
}

void adjustBank5Rit(int32_t deltaHz) {
  printKeypadAction(String("RIT STEP ") + (deltaHz >= 0 ? "+" : "") + String(deltaHz) + " Hz");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  int32_t offset = 0;
  if (!queryRitOffsetHz(offset, 800)) { keypadReportIfTimedOut("RIT"); return; }
  int32_t next = offset + deltaHz;
  if (next < -9999) next = -9999;
  if (next > 9999) next = 9999;
  setBank5RitOffset(next);
}

void setBank5RitOff() {
  printKeypadAction("RIT OFF");
  if (!keypadReportIfUnsupported(protocolSupportsRit(), "RIT") && setRitEnabled(false)) {
    printKeypadStatus("RIT OFF");
    if (g_speechEnabled) speakToken("off");
  }
}
