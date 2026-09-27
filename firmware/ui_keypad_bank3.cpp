// Bank 3 keypad actions: split, VFO A/B, VFO mode, RX/TX and band stack.
#include "ui_keypad_bank.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_state.h"
#include "radio_utils.h"

static void speakVfoFrequencyLabel(char which) {
  if (!g_speechEnabled) return;
  speakToken("vfo");
  playSilenceMs(60);
  if (which == 'A') speakToken("a");
  else if (which == 'B') speakToken("b");
  playSilenceMs(60);
  speakFrequencyWord();
}

void setBank3Ft857Split(bool on) {
  printKeypadCommand(String("BANK3 FT857 -> SPLIT ") + (on ? "ON" : "OFF"));
  if (!setSplit(on)) { keypadReportIfTimedOut("SPLIT"); return; }
  printKeypadStatus(on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", on);
}

void calibrateBank3Ft857Split() {
  printKeypadCommand("BANK3 0 DOUBLE -> SPLIT CAL");
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
  printKeypadCommand(String("BANK3 FT857 -> CLAR ") + (on ? "ON" : "OFF"));
  if (!yaesuCatSetClarifier(on)) { keypadReportIfTimedOut("CLAR"); return; }
  printKeypadStatus(on ? "CLAR ON" : "CLAR OFF");
  if (g_speechEnabled) {
    speakToken("clarifier");
    playSilenceMs(60);
    speakSimpleBinaryState(on);
  }
}

void setBank3Ft857Ptt(bool on) {
  printKeypadCommand(String("BANK3 FT857 -> PTT ") + (on ? "ON" : "OFF"));
  if (!yaesuCatSetPtt(on)) { keypadReportIfTimedOut("PTT"); return; }
  printKeypadStatus(on ? "PTT ON" : "PTT OFF");
  speakToken("ptt");
  playSilenceMs(60);
  speakSimpleBinaryState(on);
}

void queryBank3Split() {
  printKeypadCommand("BANK3 0 SHORT -> SPLIT?");
  bool on = false;
  if (!querySplit(on, 800)) { keypadReportIfTimedOut("SPLIT?"); return; }
  printKeypadStatus(on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", on);
}

void toggleBank3Split() {
  printKeypadCommand("BANK3 0 LONG -> SPLIT");
  bool on = false;
  if (!querySplit(on, 800)) { keypadReportIfTimedOut("SPLIT"); return; }
  if (!setSplit(!on)) { keypadReportIfTimedOut("SPLIT"); return; }
  printKeypadStatus(!on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", !on);
}

void queryBank3TxFrequency() {
  printKeypadCommand("BANK3 0 DOUBLE -> TXFREQ?");
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
  printKeypadCommand("BANK3 1 SHORT -> VFOA?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(true, hz, 800)) { keypadReportIfTimedOut("VFOA?"); return; }
  printKeypadStatus(String("VFOA: ") + hzToMHzString3(hz) + " MHz");
  if (g_speechEnabled) {
    speakVfoFrequencyLabel('A');
    playSilenceMs(60);
    speakDigitsAndPoint(hzToMHzString3(hz));
  }
}

void queryBank3Ft8x7CurrentVfo() {
  ensureFt8x7VfoTrackingInitialized();
  const char which = ft8x7CurrentVfoLabel();
  printKeypadCommand(String("BANK3 1 SHORT -> VFO") + which + "?");
  uint64_t hz = 0;
  if (!queryFrequency(hz, 800)) { keypadReportIfTimedOut("VFOA?"); return; }
  printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
  if (g_speechEnabled) {
    speakVfoFrequencyLabel(which);
    playSilenceMs(60);
    speakDigitsAndPoint(hzToMHzString3(hz));
  }
}

void selectBank3VfoA() {
  printKeypadCommand("BANK3 1 LONG -> VFO A");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  if (!selectVfoA()) { keypadReportIfTimedOut("VFO A"); return; }
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    printKeypadStatus("VFO A");
    if (g_speechEnabled) {
      speakToken("vfo");
      playSilenceMs(60);
      speakToken("a");
    }
    return;
  }
  queryBank3VfoA();
}

void toggleBank3Ft857Vfo() {
  printKeypadCommand("BANK3 1 LONG -> A/B");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  ensureFt8x7VfoTrackingInitialized();
  if (!guardFt8x7VfoToggleLock()) return;
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFO A"); return; }
  rememberActiveVfo(!live.activeVfoA);
  const char which = ft8x7CurrentVfoLabel();
  printKeypadStatus(String("VFO") + which);
  if (g_speechEnabled) {
    speakToken("vfo");
    playSilenceMs(60);
    speakToken(which == 'A' ? "a" : "b");
  }
}

void beginBank3VfoAFrequencySet() {
  printKeypadCommand("BANK3 1 DOUBLE -> VFOA FREQ");
  keypadInput().beginEntry(InputMode::FreqEntry, KEYPAD_VFO_A);
  if (g_speechEnabled) {
    speakFrequencyWord();
    playSilenceMs(80);
    speakToken("please");
  }
}

void beginBank3Ft8x7CurrentVfoFrequencySet() {
  ensureFt8x7VfoTrackingInitialized();
  const char which = ft8x7CurrentVfoLabel();
  printKeypadCommand(String("BANK3 1 DOUBLE -> VFO") + which + " FREQ");
  keypadInput().beginEntry(InputMode::FreqEntry, KEYPAD_VFO_CURRENT);
  if (g_speechEnabled) {
    speakVfoFrequencyLabel(which);
    playSilenceMs(80);
    speakToken("please");
  }
}

void queryBank3VfoB() {
  printKeypadCommand("BANK3 2 SHORT -> VFOB?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(false, hz, 800)) { keypadReportIfTimedOut("VFOB?"); return; }
  printKeypadStatus(String("VFOB: ") + hzToMHzString3(hz) + " MHz");
  if (g_speechEnabled) {
    speakVfoFrequencyLabel('B');
    playSilenceMs(60);
    speakDigitsAndPoint(hzToMHzString3(hz));
  }
}

// Switches to the other VFO, reads it and switches back.
void queryBank3Ft857OtherVfo() {
  ensureFt8x7VfoTrackingInitialized();
  printKeypadCommand(String("BANK3 2 SHORT -> VFO") + ft8x7OtherVfoLabel() + "?");
  if (!guardFt8x7VfoToggleLock()) return;
  const bool priorVfoA = live.activeVfoA;
  const char other = priorVfoA ? 'B' : 'A';
  uint64_t hz = 0;
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFOB?"); return; }
  rememberActiveVfo(!priorVfoA);
  delay(120);
  bool ok = queryFrequency(hz, 800);
  delay(180);
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFOB?"); return; }
  rememberActiveVfo(priorVfoA);
  delay(180);
  if (!ok) { keypadReportIfTimedOut("VFOB?"); return; }
  printKeypadStatus(String("VFO") + other + ": " + hzToMHzString3(hz) + " MHz");
  if (g_speechEnabled) {
    speakVfoFrequencyLabel(other);
    playSilenceMs(60);
    speakDigitsAndPoint(hzToMHzString3(hz));
  }
}

// Switches to the other VFO, reads it (one retry) and switches back.
void queryBank3Ft817OtherVfo() {
  ensureFt8x7VfoTrackingInitialized();
  printKeypadCommand(String("BANK3 2 SHORT -> VFO") + ft8x7OtherVfoLabel() + "?");
  if (!guardFt8x7VfoToggleLock()) return;
  uint64_t hz = 0;
  bool ok = false;
  if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFOB?"); return; }
  delay(120);
  ok = queryFrequency(hz, 800);
  if (!ok) {
    delay(120);
    ok = queryFrequency(hz, 800);
  }
  delay(40);
  yaesuCatToggleVfo();
  delay(120);
  if (!ok) { keypadReportIfTimedOut("VFOB?"); return; }
  const char which = ft8x7OtherVfoLabel();
  printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
  if (g_speechEnabled) {
    speakVfoFrequencyLabel(which);
    playSilenceMs(60);
    speakDigitsAndPoint(hzToMHzString3(hz));
  }
}

void selectBank3VfoB() {
  printKeypadCommand("BANK3 2 LONG -> VFO B");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  if (!selectVfoB()) { keypadReportIfTimedOut("VFO B"); return; }
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    printKeypadStatus("VFO B");
    if (g_speechEnabled) {
      speakToken("vfo");
      playSilenceMs(60);
      speakToken("b");
    }
    return;
  }
  queryBank3VfoB();
}

void reportBank3Ft857VfoBUnsupported() {
  printKeypadCommand("BANK3 2 LONG -> VFO B");
  printKeypadStatus("VFO B unsupported");
  if (g_speechEnabled) speakNotAvailable();
}

void beginBank3VfoBFrequencySet() {
  printKeypadCommand("BANK3 2 DOUBLE -> VFOB FREQ");
  keypadInput().beginEntry(InputMode::FreqEntry, KEYPAD_VFO_B);
  if (g_speechEnabled) {
    speakFrequencyWord();
    playSilenceMs(80);
    speakToken("please");
  }
}

void beginBank3Ft8x7OtherVfoFrequencySet() {
  ensureFt8x7VfoTrackingInitialized();
  const char which = ft8x7OtherVfoLabel();
  printKeypadCommand(String("BANK3 2 DOUBLE -> VFO") + which + " FREQ");
  keypadInput().beginEntry(InputMode::FreqEntry, KEYPAD_VFO_OTHER);
  if (g_speechEnabled) {
    speakVfoFrequencyLabel(which);
    playSilenceMs(80);
    speakToken("please");
  }
}

void selectBank3Ft817ActiveVfoA() {
  printKeypadCommand("BANK3 6 SHORT -> VFO A ACTIVE");
  ensureFt8x7VfoTrackingInitialized();
  if (!live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFO A ACTIVE"); return; }
    rememberActiveVfo(true);
    delay(120);
  }
  printKeypadStatus("VFO A");
  if (g_speechEnabled) {
    speakToken("vfo");
    playSilenceMs(60);
    speakToken("a");
  }
}

void selectBank3Ft817ActiveVfoB() {
  printKeypadCommand("BANK3 6 LONG -> VFO B ACTIVE");
  ensureFt8x7VfoTrackingInitialized();
  if (live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    if (!yaesuCatToggleVfo()) { keypadReportIfTimedOut("VFO B ACTIVE"); return; }
    rememberActiveVfo(false);
    delay(120);
  }
  printKeypadStatus("VFO B");
  if (g_speechEnabled) {
    speakToken("vfo");
    playSilenceMs(60);
    speakToken("b");
  }
}

void syncBank3VfoA() {
  printKeypadCommand("BANK3 4 SHORT -> SYNC VFO A");
  rememberActiveVfo(true);
  printKeypadStatus("SYNC VFOA");
  if (g_speechEnabled) {
    speakToken("sync");
    playSilenceMs(60);
    speakToken("vfo");
    playSilenceMs(60);
    speakToken("a");
  }
}

void syncBank3VfoB() {
  printKeypadCommand("BANK3 4 LONG -> SYNC VFO B");
  rememberActiveVfo(false);
  printKeypadStatus("SYNC VFOB");
  if (g_speechEnabled) {
    speakToken("sync");
    playSilenceMs(60);
    speakToken("vfo");
    playSilenceMs(60);
    speakToken("b");
  }
}

void queryBank3VfoAMode(char key) {
  printKeypadCommand(String("BANK3 ") + key + " SHORT -> VFOA MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(true, mode, filter, 800)) { keypadReportIfTimedOut("VFOA MODE?"); return; }
  printKeypadStatus(String("VFOA MODE: ") + modeToString(mode));
  g_suppressModePrefixOnce = true;
  speakMode(mode);
}

void beginBank3VfoAModeSet(char key) {
  printKeypadCommand(String("BANK3 ") + key + " LONG -> VFOA MODE");
  keypadInput().beginModeSelect(KEYPAD_VFO_A);
  if (g_speechEnabled) {
    speakToken("mode");
    playSilenceMs(80);
    speakToken("please");
  }
}

void queryBank3VfoBMode() {
  printKeypadCommand("BANK3 5 SHORT -> VFOB MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(false, mode, filter, 800)) { keypadReportIfTimedOut("VFOB MODE?"); return; }
  printKeypadStatus(String("VFOB MODE: ") + modeToString(mode));
  g_suppressModePrefixOnce = true;
  speakMode(mode);
}

void beginBank3VfoBModeSet() {
  printKeypadCommand("BANK3 5 LONG -> VFOB MODE");
  keypadInput().beginModeSelect(KEYPAD_VFO_B);
  if (g_speechEnabled) {
    speakToken("mode");
    playSilenceMs(80);
    speakToken("please");
  }
}

void queryBank3RxTx() {
  printKeypadCommand("BANK3 6 SHORT -> RXTX?");
  bool tx = false;
  if (!queryRxTxStatus(tx, 800)) { keypadReportIfTimedOut("RXTX?"); return; }
  printKeypadStatus(tx ? "TX" : "RX");
  if (!g_speechEnabled) return;
  speakToken("transceiver");
  playSilenceMs(60);
  speakSimpleBinaryState(tx);
}

void queryBank3BandStack(uint8_t reg) {
  printKeypadCommand(String("BANK3 ") + String(reg + 6) + " SHORT -> BSTACK? " + String(reg));
  if (keypadReportIfUnsupported(protocolSupportsBandStack(), "BSTACK?")) return;
  keypadSendNow(String("BSTACK? ") + String(reg));
}

void recallBank3BandStack(uint8_t reg) {
  printKeypadCommand(String("BANK3 ") + String(reg + 6) + " LONG -> BSTACK " + String(reg));
  if (keypadReportIfUnsupported(protocolSupportsBandStack(), "BSTACK")) return;
  keypadSendNow(String("BSTACK ") + String(reg));
}
