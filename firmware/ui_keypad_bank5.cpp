// Bank 5 keypad actions: RIT.
#include "ui_keypad_bank.h"
#include "radio_frequency.h"

static void speakRitOffsetValue(int32_t hz) {
  if (!g_speechEnabled) return;
  speakLabel("rit");
  if (hz > 0) {
    speakToken("plus");
    playSilenceMs(60);
  }
  if (hz < 0) {
    speakToken("minus");
    playSilenceMs(60);
  }
  speakNumber(String(hz < 0 ? -hz : hz));
  playSilenceMs(60);
  speakToken("hertz");
}

void queryBank5Rit() {
  printKeypadAction("RIT?");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT?")) return;
  bool on = false;
  int32_t offset = 0;
  if (!queryRitEnabled(on, 800)) { keypadReportFailure("RIT?"); return; }
  if (!queryRitOffsetHz(offset, 800)) offset = 0;
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
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  bool on = false;
  if (!queryRitEnabled(on, 800)) { keypadReportFailure("RIT"); return; }
  if (!setRitEnabled(!on)) { keypadReportFailure("RIT"); return; }
  printKeypadStatus("RIT {}", !on ? "ON" : "OFF");
  speakTokenState("rit", !on);
}

void setBank5RitOffset(int32_t hz) {
  printKeypadAction("RIT {} Hz", hz);
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  if (!setRitOffsetHz(hz)) { keypadReportFailure("RIT"); return; }
  printKeypadStatus("RIT {} Hz", hz);
  speakRitOffsetValue(hz);
}

void adjustBank5Rit(int32_t deltaHz) {
  printKeypadAction("RIT STEP {}{} Hz", deltaHz >= 0 ? "+" : "", deltaHz);
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  int32_t offset = 0;
  if (!queryRitOffsetHz(offset, 800)) { keypadReportFailure("RIT"); return; }
  int32_t next = offset + deltaHz;
  if (next < -9999) next = -9999;
  if (next > 9999) next = 9999;
  setBank5RitOffset(next);
}

void setBank5RitOff() {
  printKeypadAction("RIT OFF");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  if (!setRitEnabled(false)) { keypadReportFailure("RIT"); return; }
  printKeypadStatus("RIT OFF");
  if (g_speechEnabled) speakToken("off");
}
