// Bank 5 keypad actions: RIT.
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
  printKeypadCommand("BANK5 0 SHORT -> RIT?");
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
  printKeypadCommand("BANK5 0 LONG -> RIT");
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
  printKeypadCommand(hz == 0 ? "BANK5 0 DOUBLE / 3 SHORT -> RIT 0" : String("BANK5 RIT -> ") + String(hz) + " Hz");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  if (!setRitOffsetHz(hz)) { keypadReportIfTimedOut("RIT"); return; }
  printKeypadStatus(String("RIT ") + String(hz) + " Hz");
  speakRitOffsetValue(hz);
}

void adjustBank5Rit(int32_t deltaHz) {
  printKeypadCommand(String("BANK5 STEP -> ") + (deltaHz >= 0 ? "+" : "") + String(deltaHz) + " Hz");
  if (keypadReportIfUnsupported(protocolSupportsRit(), "RIT")) return;
  int32_t offset = 0;
  if (!queryRitOffsetHz(offset, 800)) { keypadReportIfTimedOut("RIT"); return; }
  int32_t next = offset + deltaHz;
  if (next < -9999) next = -9999;
  if (next > 9999) next = 9999;
  setBank5RitOffset(next);
}

void setBank5RitOff() {
  printKeypadCommand("BANK5 3 LONG -> RIT OFF");
  if (!keypadReportIfUnsupported(protocolSupportsRit(), "RIT") && setRitEnabled(false)) {
    printKeypadStatus("RIT OFF");
    if (g_speechEnabled) speakToken("off");
  }
}
