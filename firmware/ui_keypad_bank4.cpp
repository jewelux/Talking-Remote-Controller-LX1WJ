// Bank 4 keypad actions: tuner, monitor and transceive.
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

void queryBank4Tuner() {
  printKeypadCommand("BANK4 0 SHORT -> TUNER?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("TUNER?");
    return;
  }
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNER?")) return;
  bool on = false;
  if (!queryTuner(on, 800)) { keypadReportIfTimedOut("TUNER?"); return; }
  printKeypadStatus(on ? "TUNER ON" : "TUNER OFF");
  speakTokenState("tuner", on);
}

void toggleBank4Tuner() {
  printKeypadCommand("BANK4 0 LONG -> TUNER");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("TUNER TOGGLE");
    return;
  }
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNER")) return;
  bool on = false;
  if (!queryTuner(on, 800)) { keypadReportIfTimedOut("TUNER"); return; }
  if (!setTuner(!on)) { keypadReportIfTimedOut("TUNER"); return; }
  printKeypadStatus(!on ? "TUNER ON" : "TUNER OFF");
  speakTokenState("tuner", !on);
}

void triggerBank4Tune() {
  printKeypadCommand("BANK4 0 DOUBLE -> TUNE");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("TUNE");
    return;
  }
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNE")) return;
  if (!startTune()) { keypadReportIfTimedOut("TUNE"); return; }
  printKeypadStatus("TUNE");
  if (g_speechEnabled) speakToken("tune");
}

void sendBank4MonitorQuery() { sendKeypadCommand("BANK4 1 SHORT -> MONITOR?", "MONITOR?"); }

void toggleBank4Monitor() {
  printKeypadCommand("BANK4 1 LONG -> MONITOR");
  if (keypadReportIfUnsupported(protocolSupportsMonitor(), "MONITOR")) return;
  bool on = false;
  if (!queryMonitorEnabled(on, 800)) { keypadReportIfTimedOut("MONITOR"); return; }
  if (!setMonitorEnabled(!on)) { keypadReportIfTimedOut("MONITOR"); return; }
  printKeypadStatus(!on ? "MONITOR ON" : "MONITOR OFF");
  speakTokenState("monitor", !on);
}

void queryBank4MonitorLevel() {
  printKeypadCommand("BANK4 2 SHORT -> MONLEVEL?");
  if (keypadReportIfUnsupported(protocolSupportsMonitor(), "MONLEVEL?")) return;
  uint16_t raw = 0;
  if (!queryMonitorLevel(raw, 800)) { keypadReportIfTimedOut("MONLEVEL?"); return; }
  const uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus(String("MONLEVEL ") + String((int)percent) + "%");
  speakFeatureValue(voice_monitor, voice_monitor_len, percent);
}

void adjustBank4MonitorLevel(int deltaPercent) {
  printKeypadCommand(String("BANK4 2 ") + (deltaPercent > 0 ? "LONG" : "DOUBLE") + " -> MONLEVEL");
  if (keypadReportIfUnsupported(protocolSupportsMonitor(), "MONLEVEL")) return;
  uint16_t raw = 0;
  if (!queryMonitorLevel(raw, 800)) { keypadReportIfTimedOut("MONLEVEL"); return; }
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  if (!setMonitorLevel(levelPercentToRaw(percent))) { keypadReportIfTimedOut("MONLEVEL"); return; }
  queryBank4MonitorLevel();
}

void sendBank4TransceiveQuery() { sendKeypadCommand("BANK4 3 SHORT -> TRANSCEIVE?", "TRANSCEIVE?"); }

void toggleBank4Transceive() {
  printKeypadCommand("BANK4 3 LONG -> TRANSCEIVE");
  if (keypadReportIfUnsupported(protocolSupportsTransceive(), "TRANSCEIVE")) return;
  bool on = false;
  if (!queryTransceiveEnabled(on, 800)) { keypadReportIfTimedOut("TRANSCEIVE"); return; }
  if (!setTransceiveEnabled(!on)) { keypadReportIfTimedOut("TRANSCEIVE"); return; }
  printKeypadStatus(!on ? "TRANSCEIVE ON" : "TRANSCEIVE OFF");
  speakTokenState("transceiver", !on);
}
