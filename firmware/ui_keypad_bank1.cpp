// Bank 1 keypad actions: frequency, lock, power, meters and mode.
#include "ui_keypad_bank.h"
#include "engine_civ.h"
#include "radio_frequency.h"
#include "radio_state.h"
#include "radio_utils.h"

static bool queryDialLockReliable(bool& onOut) {
  // One EEPROM read on the FT-8x7. No stale fallback: a toggle must start from the radio's real
  // state.
  if (currentProtocolType() == PROTO_YAESU_FT8X7) return queryDialLock(onOut, 800);
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (queryDialLock(onOut, 800)) {
      rememberDialLockState(onOut);
      return true;
    }
    if (attempt < 2) {
      pumpIncoming(20);
      delay(25);
    }
  }
  if (live.lockKnown) {
    onOut = live.lockOn;
    return true;
  }
  return false;
}

// Same wording as a tuning announcement: digits only, no "frequency" prefix.
static void speakTunedFrequencyHz(uint64_t hz) {
  if (!g_speechEnabled) return;
  speakDigitsAndPoint(hzToMHzString3(hz));
}

void queryBank1RxTx() {
  printKeypadAction("RXTX?");
  bool tx = false;
  if (!queryRxTxStatus(tx, 800)) { keypadReportIfTimedOut("RXTX?"); return; }
  printKeypadStatus(tx ? "TX" : "RX");
  if (!g_speechEnabled) return;
  speakToken("transceiver");
  playSilenceMs(60);
  speakSimpleBinaryState(tx);
}

void queryBank1Frequency() {
  printKeypadAction("FREQ?");
  uint64_t hz = 0;
  if (!queryFrequency(hz, 800)) {
    if (!keypadReportIfTimedOut("FREQ?")) {
      printKeypadStatus("FREQ? -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }
  printKeypadStatus(String("FREQ: ") + hzToMHzString3(hz) + " MHz");
  speakQueriedFrequencyHz(hz);
  rememberAnnouncedFrequency(hz);
}

void queryBank1TxFrequency() {
  printKeypadAction("TXFREQ?");
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, 800)) {
    // FT-8x7 without a TX frequency reply: say the frequency instead.
    if (currentProtocolType() == PROTO_YAESU_FT8X7) {
      if (queryFrequency(hz, 800)) {
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

void queryBank1Ft857TxFrequency() {
  printKeypadAction("TXFREQ?");
  uint64_t hz = 0;
  bool splitOn = false;
  if (querySplit(splitOn, 800) && !splitOn && queryFrequency(hz, 800)) {
    printKeypadStatus(String("TXFREQ: ") + hzToMHzString3(hz) + " MHz");
    speakQueriedFrequencyHz(hz);
  } else {
    printKeypadStatus("TXFREQ unavailable on FT-857/897");
    if (g_speechEnabled) speakNotAvailable();
  }
}

void queryBank1Lock() {
  printKeypadAction("LOCK?");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (!queryDialLockReliable(on)) {
    if (keypadReportIfTimedOut("LOCK?")) return;
    printKeypadStatus("LOCK UNKNOWN");
    if (g_speechEnabled) {
      speakToken("lock");
      playSilenceMs(60);
      speakError();
    }
    return;
  }
  printKeypadStatus(on ? "LOCK ON" : "LOCK OFF");
  speakTokenState("lock", on);
}

void beginBank1FrequencySet() {
  printKeypadAction("FREQ");
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::Current);
  speakPrompt("frequency");
}

void roundActiveFrequency(uint32_t stepHz) {
  printKeypadAction(String("ROUND ") + String((unsigned long)stepHz) + " Hz");
  prepareKeypadRadioWrite();

  uint64_t hz = 0;
  if (!queryFrequency(hz, 800)) {
    // Radio not responding: do not round or announce a stale value.
    if (!keypadReportIfTimedOut("ROUND")) {
      printKeypadStatus("ROUND -> no reply");
      if (g_speechEnabled) speakError();
    }
    return;
  }

  const uint64_t rounded = RadioFrequency::fromHz(hz).roundedTo(stepHz).hz();
  // Serial monitor reports old -> new; speech reports only the new frequency.
  if (rounded == hz) {
    // Already on a step boundary.
    printKeypadStatus(String("FREQ: ") + hzToMHzString3(rounded) + " MHz (already rounded)");
    speakTunedFrequencyHz(rounded);
    rememberAnnouncedFrequency(rounded);
    return;
  }

  if (keypadApplyFrequencyHz(rounded, TargetVfo::Current)) {
    printKeypadStatus(String("ROUND: ") + hzToMHzString3(hz) + " -> " + hzToMHzString3(rounded) + " MHz");
    speakTunedFrequencyHz(rounded);
    rememberAnnouncedFrequency(rounded);
  } else if (!keypadReportIfTimedOut("ROUND")) {
    printKeypadStatus(currentProtocolType() == PROTO_YAESU_FT8X7 ? "ROUND -> no change" : "ROUND -> failed");
    if (g_speechEnabled) speakError();
  }
}

void beginBank1RfPowerSet() {
  if (!currentStoredProfile().caps.setRfPower) {
    printKeypadStatus("RFPOWER -> unavailable");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  printKeypadAction("RFPOWER");
  keypadBeginEntry(InputMode::RfPowerEntry);
  printKeypadStatus("POWER PLEASE");
  speakPrompt("power");
}

void toggleBank1Lock() {
  printKeypadAction("LOCK");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (!queryDialLockReliable(on)) {
    if (keypadReportIfTimedOut("LOCK?")) return;
    printKeypadStatus("LOCK UNKNOWN");
    if (g_speechEnabled) {
      speakToken("lock");
      playSilenceMs(60);
      speakError();
    }
    return;
  }
  if (!setDialLock(!on)) { keypadReportIfTimedOut("LOCK"); return; }
  printKeypadStatus(!on ? "LOCK ON" : "LOCK OFF");
  speakTokenState("lock", !on);
}

static void sendOrStageBank1Command(const String& cmd) {
  printKeypadAction(cmd);
  if (AUTO_SEND_BANK1_QUERIES) {
    // The SWR and RF power replies say their own word, so saying it here too doubles it.
    if (cmd != "SWR?" && cmd != "RFPOWER?") {
      speakKeypadCommandWord(cmd);
      playSilenceMs(60);
    }
    keypadSendNow(cmd);
  } else {
    keypadStageCommand(cmd);
  }
}

void queryBank1Power() { sendOrStageBank1Command("PO?"); }

void queryBank1RfPower() { sendOrStageBank1Command("RFPOWER?"); }

void queryBank1Smeter() { sendOrStageBank1Command("SM?"); }

void queryBank1Swr() { sendOrStageBank1Command("SWR?"); }

void queryBank1Mode() { sendOrStageBank1Command("MODE?"); }

void beginBank1ModeSelect() {
  keypadBeginModeSelect(TargetVfo::Current);
  printKeypadAction("MODE");
  printKeypadStatus("MODE PLEASE");
  speakPrompt("mode");
}
