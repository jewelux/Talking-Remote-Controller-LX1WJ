// Bank 3 keypad actions: split, VFO A/B, VFO mode, RX/TX and band stack.
#include "ui_keypad_bank.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_state.h"
#include "radio_utils.h"

void queryBank3Ft8x7Rit() {
  printKeypadAction("RIT?");
  bool on = false;
  if (!queryRitEnabled(on, 800)) { keypadReportIfTimedOut("RIT?"); return; }
  printKeypadStatus("RIT {}", on ? "ON" : "OFF");
  speakTokenState("rit", on);
}

void toggleBank3Ft8x7Rit() {
  printKeypadAction("RIT");
  bool on = false;
  if (!toggleRitEnabled(on, 800)) { keypadReportIfTimedOut("RIT"); return; }
  printKeypadStatus("RIT {}", on ? "ON" : "OFF");
  speakTokenState("rit", on);
}

void setBank3Ft857Ptt(bool on) {
  printKeypadAction("PTT {}", on ? "ON" : "OFF");
  yaesuCatSetPtt(on);
  printKeypadStatus("PTT {}", on ? "ON" : "OFF");
  speakLabel("ptt");
  speakSimpleBinaryState(on);
}

void queryBank3Split() {
  printKeypadAction("SPLIT?");
  bool on = false;
  if (!querySplit(on, 800)) { keypadReportIfTimedOut("SPLIT?"); return; }
  printKeypadStatus("SPLIT {}", on ? "ON" : "OFF");
  speakTokenState("split", on);
}

void toggleBank3Split() {
  printKeypadAction("SPLIT");
  bool on = false;
  if (!querySplit(on, 800)) { keypadReportIfTimedOut("SPLIT"); return; }
  if (!setSplit(!on)) { keypadReportIfTimedOut("SPLIT"); return; }
  printKeypadStatus("SPLIT {}", !on ? "ON" : "OFF");
  speakTokenState("split", !on);
}

void queryBank3TxFrequency() {
  printKeypadAction("TXFREQ?");
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, 800)) {
    if (currentProtocolType() == PROTO_YAESU_FT8X7) {
      bool splitOn = false;
      if (querySplit(splitOn, 800) && !splitOn && queryFrequency(hz, 800)) {
        printKeypadStatus("TXFREQ: {} MHz", RadioFrequency::fromHz(hz));
        speakQueriedFrequencyHz(hz);
      } else {
        printKeypadStatus("TXFREQ -> unavailable");
        if (g_speechEnabled) speakNotAvailable();
      }
    }
    return;
  }
  printKeypadStatus("TXFREQ: {} MHz", RadioFrequency::fromHz(hz));
  speakQueriedFrequencyHz(hz);
}

void queryBank3VfoA() {
  printKeypadAction("VFOA?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(true, hz, 800)) { keypadReportIfTimedOut("VFOA?"); return; }
  printKeypadStatus("VFOA: {} MHz", RadioFrequency::fromHz(hz));
  speakVfoFrequency('A', hz);
}

void queryBank3Ft8x7CurrentVfo() {
  refreshFt8x7ActiveVfo();
  const char which = ft8x7CurrentVfoLabel();
  const FormattedLine label("VFO{}?", which);
  printKeypadAction("{}", label.c_str());
  uint64_t hz = 0;
  if (!queryFrequency(hz, 800)) { keypadReportIfTimedOut(label.c_str()); return; }
  printKeypadStatus("VFO{}: {} MHz", which, RadioFrequency::fromHz(hz));
  speakVfoFrequency(which, hz);
}

void selectBank3VfoA() {
  printKeypadAction("VFO A");
  muteTuningSpeechAfterOwnChange();
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
  muteTuningSpeechAfterOwnChange();
  refreshFt8x7ActiveVfo();
  if (!guardFt8x7VfoToggleLock()) return;
  yaesuCatToggleVfo();
  rememberActiveVfo(!live.activeVfoA);
  const char which = ft8x7CurrentVfoLabel();
  printKeypadStatus("VFO{}", which);
  speakVfoLabel(which);
}

void beginBank3VfoAFrequencySet() {
  printKeypadAction("VFOA FREQ");
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::A);
  speakPrompt("frequency");
}

void beginBank3Ft8x7CurrentVfoFrequencySet() {
  refreshFt8x7ActiveVfo();
  const char which = ft8x7CurrentVfoLabel();
  printKeypadAction("VFO{} FREQ", which);
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::Current);
  speakVfoFrequencyLabel(which);
  speakPlease();
}

void queryBank3VfoB() {
  printKeypadAction("VFOB?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(false, hz, 800)) { keypadReportIfTimedOut("VFOB?"); return; }
  printKeypadStatus("VFOB: {} MHz", RadioFrequency::fromHz(hz));
  speakVfoFrequency('B', hz);
}

// Switches to the other VFO, reads it and switches back.
void queryBank3Ft8x7OtherVfo() {
  refreshFt8x7ActiveVfo();
  const char other = ft8x7OtherVfoLabel();
  const FormattedLine label("VFO{}?", other);
  printKeypadAction("{}", label.c_str());
  if (!guardFt8x7VfoToggleLock()) return;
  uint64_t hz = 0;
  if (!ft8x7QueryOtherVfoFrequency(hz, 800)) { keypadReportIfTimedOut(label.c_str()); return; }
  printKeypadStatus("VFO{}: {} MHz", other, RadioFrequency::fromHz(hz));
  speakVfoFrequency(other, hz);
}

void selectBank3VfoB() {
  printKeypadAction("VFO B");
  muteTuningSpeechAfterOwnChange();
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
  muteTuningSpeechAfterOwnChange();
  refreshFt8x7ActiveVfo();
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
  refreshFt8x7ActiveVfo();
  const char which = ft8x7OtherVfoLabel();
  printKeypadAction("VFO{} FREQ", which);
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::Other);
  speakVfoFrequencyLabel(which);
  speakPlease();
}

void selectBank3Ft817ActiveVfoA() {
  printKeypadAction("VFO A ACTIVE");
  refreshFt8x7ActiveVfo();
  if (!live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    yaesuCatToggleVfo();
    rememberActiveVfo(true);
    delay(YAESU_CAT_VFO_SETTLE_MS);
  }
  printKeypadStatus("VFO A");
  speakVfoLabel('A');
}

void selectBank3Ft817ActiveVfoB() {
  printKeypadAction("VFO B ACTIVE");
  refreshFt8x7ActiveVfo();
  if (live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    yaesuCatToggleVfo();
    rememberActiveVfo(false);
    delay(YAESU_CAT_VFO_SETTLE_MS);
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
  printKeypadStatus("VFOA MODE: {}", modeToString(mode));
  speakModeName(mode);
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
  printKeypadStatus("VFOB MODE: {}", modeToString(mode));
  speakModeName(mode);
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
  printKeypadStatus("{}", tx ? "TX" : "RX");
  speakRxTxState(tx);
}

void queryBank3BandStack(uint8_t reg) {
  printKeypadAction("BSTACK? {}", reg);
  if (keypadReportIfUnsupported(protocolSupportsBandStack(), "BSTACK?")) return;
  keypadSendNow(String("BSTACK? ") + String(reg));
}

void recallBank3BandStack(uint8_t reg) {
  printKeypadAction("BSTACK {}", reg);
  if (keypadReportIfUnsupported(protocolSupportsBandStack(), "BSTACK")) return;
  keypadSendNow(String("BSTACK ") + String(reg));
}
