// Bank 4 keypad actions: tuner, monitor and transceive.
#include "ui_keypad_bank.h"

void queryBank4Tuner() {
  printKeypadAction("TUNER?");
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNER?")) return;
  bool on = false;
  if (!queryTuner(on, 800)) { keypadReportFailure("TUNER?"); return; }
  printKeypadStatus("TUNER {}", on ? "ON" : "OFF");
  speakTokenState("tuner", on);
}

void toggleBank4Tuner() {
  printKeypadAction("TUNER");
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNER")) return;
  bool on = false;
  if (!queryTuner(on, 800)) { keypadReportFailure("TUNER"); return; }
  if (!setTuner(!on)) { keypadReportFailure("TUNER"); return; }
  printKeypadStatus("TUNER {}", !on ? "ON" : "OFF");
  speakTokenState("tuner", !on);
}

void triggerBank4Tune() {
  printKeypadAction("TUNE");
  if (keypadReportIfUnsupported(protocolSupportsTuner(), "TUNE")) return;
  if (!startTune()) { keypadReportFailure("TUNE"); return; }
  printKeypadStatus("TUNE");
  if (g_speechEnabled) speakToken("tune");
}

void toggleBank4Monitor() {
  printKeypadAction("MONITOR");
  if (keypadReportIfUnsupported(protocolSupportsMonitor(), "MONITOR")) return;
  bool on = false;
  if (!queryMonitorEnabled(on, 800)) { keypadReportFailure("MONITOR"); return; }
  if (!setMonitorEnabled(!on)) { keypadReportFailure("MONITOR"); return; }
  printKeypadStatus("MONITOR {}", !on ? "ON" : "OFF");
  speakTokenState("monitor", !on);
}

void queryBank4MonitorLevel() {
  printKeypadAction("MONLEVEL?");
  if (keypadReportIfUnsupported(protocolSupportsMonitor(), "MONLEVEL?")) return;
  uint16_t raw = 0;
  if (!queryMonitorLevel(raw, 800)) { keypadReportFailure("MONLEVEL?"); return; }
  const uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus("MONLEVEL {}%", percent);
  speakFeatureValue(voice_monitor, voice_monitor_len, percent);
}

void adjustBank4MonitorLevel(int deltaPercent) {
  printKeypadAction("MONLEVEL");
  if (keypadReportIfUnsupported(protocolSupportsMonitor(), "MONLEVEL")) return;
  uint16_t raw = 0;
  if (!queryMonitorLevel(raw, 800)) { keypadReportFailure("MONLEVEL"); return; }
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  if (!setMonitorLevel(levelPercentToRaw(percent))) { keypadReportFailure("MONLEVEL"); return; }
  queryBank4MonitorLevel();
}

void toggleBank4Transceive() {
  printKeypadAction("TRANSCEIVE");
  if (keypadReportIfUnsupported(protocolSupportsTransceive(), "TRANSCEIVE")) return;
  bool on = false;
  if (!queryTransceiveEnabled(on, 800)) { keypadReportFailure("TRANSCEIVE"); return; }
  if (!setTransceiveEnabled(!on)) { keypadReportFailure("TRANSCEIVE"); return; }
  printKeypadStatus("TRANSCEIVE {}", !on ? "ON" : "OFF");
  speakTokenState("transceiver", !on);
}
