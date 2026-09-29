// Bank 3 keypad actions: split, VFO A/B, VFO mode, RX/TX and band stack.
#include "ui_keypad_bank.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_state.h"
#include "radio_utils.h"

void setBank3Ft857Split(bool on) {
  printKeypadAction(String("SPLIT ") + (on ? "ON" : "OFF"));
  if (!setSplit(on)) { keypadReportIfTimedOut("SPLIT"); return; }
  printKeypadStatus(on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", on);
}

void calibrateBank3Ft857Split() {
  printKeypadAction("SPLIT CAL");
  if (!yaesuCatSetSplit(true)) { keypadReportIfTimedOut("SPLIT CAL"); return; }
  delay(120);
  if (!yaesuCatSetSplit(false)) { keypadReportIfTimedOut("SPLIT CAL"); return; }
  rememberSplitState(false);
  printKeypadStatus("SPLIT OFF");
  speakTokenState("split", false);
  if (g_speechEnabled) {
    playSilenceMs(60);
    speakToken("ok");
  }
}

void setBank3Ft857Clar(bool on) {
  printKeypadAction(String("CLAR ") + (on ? "ON" : "OFF"));
  if (!yaesuCatSetClarifier(on)) { keypadReportIfTimedOut("CLAR"); return; }
  printKeypadStatus(on ? "CLAR ON" : "CLAR OFF");
  if (g_speechEnabled) {
    speakToken("clarifier");
    playSilenceMs(60);
    speakSimpleBinaryState(on);
  }
}

void setBank3Ft857Ptt(bool on) {
  printKeypadAction(String("PTT ") + (on ? "ON" : "OFF"));
  if (!yaesuCatSetPtt(on)) { keypadReportIfTimedOut("PTT"); return; }
  printKeypadStatus(on ? "PTT ON" : "PTT OFF");
  speakToken("ptt");
  playSilenceMs(60);
  speakSimpleBinaryState(on);
}

void queryBank3Split() {
  printKeypadAction("SPLIT?");
  bool on = false;
  if (!querySplit(on, 800)) { keypadReportIfTimedOut("SPLIT?"); return; }
  printKeypadStatus(on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", on);
}

void toggleBank3Split() {
  printKeypadAction("SPLIT");
  bool on = false;
  if (!querySplit(on, 800)) { keypadReportIfTimedOut("SPLIT"); return; }
  if (!setSplit(!on)) { keypadReportIfTimedOut("SPLIT"); return; }
  printKeypadStatus(!on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", !on);
}

void queryBank3TxFrequency() {
  printKeypadAction("TXFREQ?");
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, 800)) {
    if (currentProtocolType() == PROTO_YAESU_FT8X7) {
      bool splitOn = false;
      if (querySplit(splitOn, 800) && !splitOn && queryFrequency(hz, 800)) {
        printKeypadStatus(String("TXFREQ: ") + hzToMHzString3(hz) + " MHz");
        speakQueriedFrequencyHz(hz);
      } else {
        printKeypadStatus("TXFREQ -> unavailable");
        if (g_speechEnabled) speakNotAvailable();
      }
    }
    return;
  }
  printKeypadStatus(String("TXFREQ: ") + hzToMHzString3(hz) + " MHz");
  speakQueriedFrequencyHz(hz);
}

void queryBank3VfoA() {
  printKeypadAction("VFOA?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(true, hz, 800)) { keypadReportIfTimedOut("VFOA?"); return; }
  printKeypadStatus(String("VFOA: ") + hzToMHzString3(hz) + " MHz");
  speakVfoFrequency('A', hz);
}

void queryBank3Ft8x7CurrentVfo() {
  ensureFt8x7VfoTrackingInitialized();
  const char which = ft8x7CurrentVfoLabel();
  const String label = String("VFO") + which + "?";
  printKeypadAction(label);
  uint64_t hz = 0;
  if (!queryFrequency(hz, 800)) { keypadReportIfTimedOut(label.c_str()); return; }
  printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
  speakVfoFrequency(which, hz);
}

void selectBank3VfoA() {
  printKeypadAction("VFO A");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  if (!selectVfoA()) { keypadReportIfTimedOut("VFO A"); return; }
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    printKeypadStatus("VFO A");
    speakVfoLabel('A');
    return;
  }
  queryBank3VfoA();
}

void toggleBank3Ft8x7Vfo() {
  printKeypadAction("A/B");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  ensureFt8x7VfoTrackingInitialized();
  if (!guardFt8x7VfoToggleLock()) return;
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFO A"); return; }
  rememberActiveVfo(!live.activeVfoA);
  const char which = ft8x7CurrentVfoLabel();
  printKeypadStatus(String("VFO") + which);
  speakVfoLabel(which);
}

void beginBank3VfoAFrequencySet() {
  printKeypadAction("VFOA FREQ");
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::A);
  speakPrompt("frequency");
}

void beginBank3Ft8x7CurrentVfoFrequencySet() {
  ensureFt8x7VfoTrackingInitialized();
  const char which = ft8x7CurrentVfoLabel();
  printKeypadAction(String("VFO") + which + " FREQ");
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::Current);
  speakVfoFrequencyLabel(which);
  speakPlease();
}

void queryBank3VfoB() {
  printKeypadAction("VFOB?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(false, hz, 800)) { keypadReportIfTimedOut("VFOB?"); return; }
  printKeypadStatus(String("VFOB: ") + hzToMHzString3(hz) + " MHz");
  speakVfoFrequency('B', hz);
}

// Switches to the other VFO, reads it and switches back.
void queryBank3Ft857OtherVfo() {
  ensureFt8x7VfoTrackingInitialized();
  const char other = ft8x7OtherVfoLabel();
  const String label = String("VFO") + other + "?";
  printKeypadAction(label);
  if (!guardFt8x7VfoToggleLock()) return;
  const bool priorVfoA = live.activeVfoA;
  uint64_t hz = 0;
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut(label.c_str()); return; }
  rememberActiveVfo(!priorVfoA);
  delay(120);
  bool ok = queryFrequency(hz, 800);
  delay(180);
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut(label.c_str()); return; }
  rememberActiveVfo(priorVfoA);
  delay(180);
  if (!ok) { keypadReportIfTimedOut(label.c_str()); return; }
  printKeypadStatus(String("VFO") + other + ": " + hzToMHzString3(hz) + " MHz");
  speakVfoFrequency(other, hz);
}

// Switches to the other VFO, reads it (one retry) and switches back.
void queryBank3Ft817OtherVfo() {
  ensureFt8x7VfoTrackingInitialized();
  const char other = ft8x7OtherVfoLabel();
  const String label = String("VFO") + other + "?";
  printKeypadAction(label);
  if (!guardFt8x7VfoToggleLock()) return;
  uint64_t hz = 0;
  bool ok = false;
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut(label.c_str()); return; }
  delay(120);
  ok = queryFrequency(hz, 800);
  if (!ok) {
    delay(120);
    ok = queryFrequency(hz, 800);
  }
  delay(40);
  yaesuCatToggleVfo();
  delay(120);
  if (!ok) { keypadReportIfTimedOut(label.c_str()); return; }
  printKeypadStatus(String("VFO") + other + ": " + hzToMHzString3(hz) + " MHz");
  speakVfoFrequency(other, hz);
}

void selectBank3VfoB() {
  printKeypadAction("VFO B");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  if (!selectVfoB()) { keypadReportIfTimedOut("VFO B"); return; }
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    printKeypadStatus("VFO B");
    speakVfoLabel('B');
    return;
  }
  queryBank3VfoB();
}

void copyBank3Ft817VfoToOther() {
  printKeypadAction("A=B");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  ensureFt8x7VfoTrackingInitialized();
  if (!guardFt8x7VfoToggleLock()) return;
  if (!ft8x7CopyActiveVfoToOther()) { keypadReportIfTimedOut("A=B"); return; }
  printKeypadStatus("A=B");
  if (g_speechEnabled) {
    speakToken("a");
    playSilenceMs(60);
    speakToken("equals");
    playSilenceMs(60);
    speakToken("b");
  }
}

void reportBank3Ft857VfoBUnsupported() {
  printKeypadAction("VFO B");
  printKeypadStatus("VFO B unsupported");
  if (g_speechEnabled) speakNotAvailable();
}

void beginBank3VfoBFrequencySet() {
  printKeypadAction("VFOB FREQ");
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::B);
  speakPrompt("frequency");
}

void beginBank3Ft8x7OtherVfoFrequencySet() {
  ensureFt8x7VfoTrackingInitialized();
  const char which = ft8x7OtherVfoLabel();
  printKeypadAction(String("VFO") + which + " FREQ");
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::Other);
  speakVfoFrequencyLabel(which);
  speakPlease();
}

void selectBank3Ft817ActiveVfoA() {
  printKeypadAction("VFO A ACTIVE");
  ensureFt8x7VfoTrackingInitialized();
  if (!live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFO A ACTIVE"); return; }
    rememberActiveVfo(true);
    delay(120);
  }
  printKeypadStatus("VFO A");
  speakVfoLabel('A');
}

void selectBank3Ft817ActiveVfoB() {
  printKeypadAction("VFO B ACTIVE");
  ensureFt8x7VfoTrackingInitialized();
  if (live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFO B ACTIVE"); return; }
    rememberActiveVfo(false);
    delay(120);
  }
  printKeypadStatus("VFO B");
  speakVfoLabel('B');
}

void syncBank3VfoA() {
  printKeypadAction("SYNC VFO A");
  rememberActiveVfo(true);
  printKeypadStatus("SYNC VFOA");
  if (g_speechEnabled) {
    speakToken("sync");
    playSilenceMs(60);
    speakVfoLabel('A');
  }
}

void syncBank3VfoB() {
  printKeypadAction("SYNC VFO B");
  rememberActiveVfo(false);
  printKeypadStatus("SYNC VFOB");
  if (g_speechEnabled) {
    speakToken("sync");
    playSilenceMs(60);
    speakVfoLabel('B');
  }
}

void queryBank3VfoAMode() {
  printKeypadAction("VFOA MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(true, mode, filter, 800)) { keypadReportIfTimedOut("VFOA MODE?"); return; }
  printKeypadStatus(String("VFOA MODE: ") + modeToString(mode));
  g_suppressModePrefixOnce = true;
  speakMode(mode);
}

void beginBank3VfoAModeSet() {
  printKeypadAction("VFOA MODE");
  keypadBeginModeSelect(TargetVfo::A);
  speakPrompt("mode");
}

void queryBank3VfoBMode() {
  printKeypadAction("VFOB MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(false, mode, filter, 800)) { keypadReportIfTimedOut("VFOB MODE?"); return; }
  printKeypadStatus(String("VFOB MODE: ") + modeToString(mode));
  g_suppressModePrefixOnce = true;
  speakMode(mode);
}

void beginBank3VfoBModeSet() {
  printKeypadAction("VFOB MODE");
  keypadBeginModeSelect(TargetVfo::B);
  speakPrompt("mode");
}

void queryBank3RxTx() {
  printKeypadAction("RXTX?");
  bool tx = false;
  if (!queryRxTxStatus(tx, 800)) { keypadReportIfTimedOut("RXTX?"); return; }
  printKeypadStatus(tx ? "TX" : "RX");
  if (!g_speechEnabled) return;
  speakToken("transceiver");
  playSilenceMs(60);
  speakSimpleBinaryState(tx);
}

void queryBank3BandStack(uint8_t reg) {
  printKeypadAction(String("BSTACK? ") + String(reg));
  if (keypadReportIfUnsupported(protocolSupportsBandStack(), "BSTACK?")) return;
  keypadSendNow(String("BSTACK? ") + String(reg));
}

void recallBank3BandStack(uint8_t reg) {
  printKeypadAction(String("BSTACK ") + String(reg));
  if (keypadReportIfUnsupported(protocolSupportsBandStack(), "BSTACK")) return;
  keypadSendNow(String("BSTACK ") + String(reg));
}
