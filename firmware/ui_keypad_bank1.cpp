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
  // An FT-8x7 profile without get_rxtx, e.g. an ft817.ini from before RXTX? was verified.
  if (currentProtocolType() == PROTO_YAESU_FT8X7 && !currentStoredProfile().caps.getRxTx) {
    printKeypadStatus("RXTX -> unavailable");
    if (g_speechEnabled) {
      speakToken("transceiver");
      playSilenceMs(60);
      speakNotAvailable();
    }
    return;
  }
  bool tx = false;
  if (!queryRxTxStatus(tx, 800)) { keypadReportIfTimedOut("RXTX?"); return; }
  printKeypadStatus("{}", tx ? "TX" : "RX");
  speakRxTxState(tx);
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
  printKeypadStatus("FREQ: {} MHz", RadioFrequency::fromHz(hz));
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

void queryBank1Ft857TxFrequency() {
  printKeypadAction("TXFREQ?");
  uint64_t hz = 0;
  bool splitOn = false;
  if (querySplit(splitOn, 800) && !splitOn && queryFrequency(hz, 800)) {
    printKeypadStatus("TXFREQ: {} MHz", RadioFrequency::fromHz(hz));
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
  printKeypadStatus("LOCK {}", on ? "ON" : "OFF");
  speakTokenState("lock", on);
}

void beginBank1FrequencySet() {
  printKeypadAction("FREQ");
  keypadBeginEntry(InputMode::FreqEntry, TargetVfo::Current);
  speakPrompt("frequency");
}

void roundActiveFrequency(uint32_t stepHz) {
  printKeypadAction("ROUND {} Hz", stepHz);
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
    printKeypadStatus("FREQ: {} MHz (already rounded)", RadioFrequency::fromHz(rounded));
    speakTunedFrequencyHz(rounded);
    rememberAnnouncedFrequency(rounded);
    return;
  }

  if (keypadApplyFrequencyHz(rounded, TargetVfo::Current)) {
    printKeypadStatus("ROUND: {} -> {} MHz", RadioFrequency::fromHz(hz), RadioFrequency::fromHz(rounded));
    speakTunedFrequencyHz(rounded);
    rememberAnnouncedFrequency(rounded);
  } else if (!keypadReportIfTimedOut("ROUND")) {
    printKeypadStatus("ROUND -> {}", currentProtocolType() == PROTO_YAESU_FT8X7 ? "no change" : "failed");
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
  printKeypadStatus("LOCK {}", !on ? "ON" : "OFF");
  speakTokenState("lock", !on);
}

static void sendBank1Query(const String& cmd) {
  printKeypadAction("{}", cmd.c_str());
  // The SWR and RF power replies say their own word, so saying it here too doubles it.
  if (cmd != "SWR?" && cmd != "RFPOWER?") speakKeypadCommandWord(cmd);
  keypadSendNow(cmd);
}

void queryBank1Power() { sendBank1Query("PO?"); }

void queryBank1RfPower() { sendBank1Query("RFPOWER?"); }

void queryBank1Smeter() { sendBank1Query("SM?"); }

void queryBank1Swr() { sendBank1Query("SWR?"); }

void queryBank1Mode() { sendBank1Query("MODE?"); }

void beginBank1ModeSelect() {
  keypadBeginModeSelect(TargetVfo::Current);
  printKeypadAction("MODE");
  printKeypadStatus("MODE PLEASE");
  speakPrompt("mode");
}
