// Bank 3 keypad actions: split, VFO A/B, VFO mode, RX/TX and band stack.
#include "ui_keypad_bank.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_state.h"
#include "radio_utils.h"

void queryBank3Ft8x7Rit() {
  printKeypadAction("RIT?");
  KeypadAnswer answer("RIT?");
  bool on = false;
  if (!queryRitEnabled(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("RIT {}", on ? "ON" : "OFF");
  speakTokenState("rit", on);
}

void toggleBank3Ft8x7Rit() {
  printKeypadAction("RIT");
  KeypadAnswer answer("RIT");
  bool on = false;
  if (!toggleRitEnabled(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("RIT {}", on ? "ON" : "OFF");
  speakTokenState("rit", on);
}

void setBank3Ft857Ptt(bool on) {
  printKeypadAction("PTT {}", on ? "ON" : "OFF");
  KeypadAnswer answer("PTT");
  yaesuCatSetPtt(on);
  printKeypadStatus("PTT {}", on ? "ON" : "OFF");
  speakLabel("ptt");
  speakSimpleBinaryState(on);
}

void queryBank3Split() {
  printKeypadAction("SPLIT?");
  KeypadAnswer answer("SPLIT?");
  bool on = false;
  if (!querySplit(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("SPLIT {}", on ? "ON" : "OFF");
  speakTokenState("split", on);
}

void toggleBank3Split() {
  printKeypadAction("SPLIT");
  KeypadAnswer answer("SPLIT");
  bool on = false;
  if (!querySplit(on, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  if (!setSplit(!on)) { answer.radioFailed(); return; }
  printKeypadStatus("SPLIT {}", !on ? "ON" : "OFF");
  speakTokenState("split", !on);
}

void queryBank3TxFrequency() {
  printKeypadAction("TXFREQ?");
  KeypadAnswer answer("TXFREQ?");
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, kCatReplyTimeoutMs)) {
    if (currentProtocolType() != PROTO_YAESU_FT8X7) {
      answer.radioFailed();
      return;
    }
    // The FT-8x7 says the frequency while split is off; with it on, the TX VFO is not readable.
    bool splitOn = false;
    if (!querySplit(splitOn, kCatReplyTimeoutMs) || splitOn || !queryFrequency(hz, kCatReplyTimeoutMs)) {
      if (!answer.timedOut()) answer.fail(KeypadFailure::NotAvailable);
      return;
    }
  }
  printKeypadStatus("TXFREQ: {} MHz", RadioFrequency::fromHz(hz));
  speakQueriedFrequencyHz(hz);
}

void queryBank3VfoA() {
  printKeypadAction("VFOA?");
  KeypadAnswer answer("VFOA?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(true, hz, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("VFOA: {} MHz", RadioFrequency::fromHz(hz));
  speakVfoFrequency('A', hz);
}

void queryBank3Ft8x7CurrentVfo() {
  refreshFt8x7ActiveVfo();
  const char which = ft8x7CurrentVfoLabel();
  const FormattedLine label("VFO{}?", which);
  printKeypadAction("{}", label.c_str());
  KeypadAnswer answer(label.c_str());
  uint64_t hz = 0;
  if (!queryFrequency(hz, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("VFO{}: {} MHz", which, RadioFrequency::fromHz(hz));
  speakVfoFrequency(which, hz);
}

void selectBank3VfoA() {
  printKeypadAction("VFO A");
  KeypadAnswer answer("VFO A");
  muteTuningSpeechAfterOwnChange();
  if (!selectVfoA()) { answer.radioFailed(); return; }
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    printKeypadStatus("VFO A");
    answer.done();
    return;
  }
  queryBank3VfoA();
}

// "vfo", then the letter now active.
void toggleBank3Ft8x7Vfo() {
  printKeypadAction("A/B");
  KeypadAnswer answer("VFO");
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
  if (!canSetVfoFrequency(true)) {
    keypadReportNotAvailable("VFOA FREQ");
    return;
  }
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
  KeypadAnswer answer("VFOB?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(false, hz, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("VFOB: {} MHz", RadioFrequency::fromHz(hz));
  speakVfoFrequency('B', hz);
}

// Switches to the other VFO, reads it and switches back.
void queryBank3Ft8x7OtherVfo() {
  refreshFt8x7ActiveVfo();
  const char other = ft8x7OtherVfoLabel();
  const FormattedLine label("VFO{}?", other);
  printKeypadAction("{}", label.c_str());
  KeypadAnswer answer(label.c_str());
  if (!guardFt8x7VfoToggleLock()) return;
  uint64_t hz = 0;
  if (!ft8x7QueryOtherVfoFrequency(hz, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("VFO{}: {} MHz", other, RadioFrequency::fromHz(hz));
  speakVfoFrequency(other, hz);
}

void selectBank3VfoB() {
  printKeypadAction("VFO B");
  KeypadAnswer answer("VFO B");
  muteTuningSpeechAfterOwnChange();
  if (!selectVfoB()) { answer.radioFailed(); return; }
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    printKeypadStatus("VFO B");
    answer.done();
    return;
  }
  queryBank3VfoB();
}

void copyBank3Ft817VfoToOther() {
  printKeypadAction("A=B");
  KeypadAnswer answer("A=B");
  muteTuningSpeechAfterOwnChange();
  refreshFt8x7ActiveVfo();
  if (!guardFt8x7VfoToggleLock()) return;
  if (!ft8x7CopyActiveVfoToOther()) { answer.radioFailed(); return; }
  printKeypadStatus("A=B");
  answer.done();
}

void reportBank3Ft857VfoBUnsupported() {
  printKeypadAction("VFO B");
  keypadReportNotAvailable("VFO B");
}

void beginBank3VfoBFrequencySet() {
  printKeypadAction("VFOB FREQ");
  if (!canSetVfoFrequency(false)) {
    keypadReportNotAvailable("VFOB FREQ");
    return;
  }
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
  KeypadAnswer answer("VFO A");
  refreshFt8x7ActiveVfo();
  if (!live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    yaesuCatToggleVfo();
    rememberActiveVfo(true);
    delay(YAESU_CAT_VFO_SETTLE_MS);
  }
  printKeypadStatus("VFO A");
  answer.done();
}

void selectBank3Ft817ActiveVfoB() {
  printKeypadAction("VFO B ACTIVE");
  KeypadAnswer answer("VFO B");
  refreshFt8x7ActiveVfo();
  if (live.activeVfoA) {
    if (!guardFt8x7VfoToggleLock()) return;
    yaesuCatToggleVfo();
    rememberActiveVfo(false);
    delay(YAESU_CAT_VFO_SETTLE_MS);
  }
  printKeypadStatus("VFO B");
  answer.done();
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
  KeypadAnswer answer("VFOA MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(true, mode, filter, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("VFOA MODE: {}", modeToString(mode));
  speakModeName(mode);
}

void beginBank3VfoAModeSet() {
  printKeypadAction("VFOA MODE");
  // Kenwood and Elecraft set only the current VFO's mode.
  if (!canSetVfoMode()) {
    keypadReportNotAvailable("VFOA MODE");
    return;
  }
  keypadBeginModeSelect(TargetVfo::A);
  speakPrompt("mode");
}

void queryBank3VfoBMode() {
  printKeypadAction("VFOB MODE?");
  KeypadAnswer answer("VFOB MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(false, mode, filter, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("VFOB MODE: {}", modeToString(mode));
  speakModeName(mode);
}

void beginBank3VfoBModeSet() {
  printKeypadAction("VFOB MODE");
  if (!canSetVfoMode()) {
    keypadReportNotAvailable("VFOB MODE");
    return;
  }
  keypadBeginModeSelect(TargetVfo::B);
  speakPrompt("mode");
}

void queryBank3RxTx() {
  printKeypadAction("RXTX?");
  KeypadAnswer answer("RXTX?");
  bool tx = false;
  if (!queryRxTxStatus(tx, kCatReplyTimeoutMs)) { answer.radioFailed(); return; }
  printKeypadStatus("{}", tx ? "TX" : "RX");
  speakRxTxState(tx);
}

void queryBank3BandStack(uint8_t reg) {
  printKeypadAction("BSTACK? {}", reg);
  KeypadAnswer answer("BSTACK?");
  if (answer.unavailable(protocolSupportsBandStack())) return;
  keypadSendNow(String("BSTACK? ") + String(reg));
}

void recallBank3BandStack(uint8_t reg) {
  printKeypadAction("BSTACK {}", reg);
  KeypadAnswer answer("BSTACK");
  if (answer.unavailable(protocolSupportsBandStack())) return;
  keypadSendNow(String("BSTACK ") + String(reg));
}
