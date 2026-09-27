// Bank 1 keypad actions: frequency, lock, power, meters and mode.
#include "ui_keypad_bank.h"
#include "engine_civ.h"
#include "protocol_ops_yaesu.h"
#include "radio_frequency.h"
#include "radio_state.h"
#include "radio_utils.h"

static bool queryDialLockReliable(bool& onOut) {
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    if (!live.lockKnown) return false;
    onOut = live.lockOn;
    return true;
  }
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
  printKeypadCommand("BANK1 1 SHORT -> RXTX?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("RXTX?");
    return;
  }
  if (isFt8x7Ft817Keypad()) {
    printKeypadStatus("RXTX unreliable");
    if (g_speechEnabled) {
      speakToken("transceiver");
      playSilenceMs(60);
      speakNotAvailable();
    }
    return;
  }
  bool tx = false;
  if (!queryRxTxStatus(tx, 800)) { keypadReportIfTimedOut("RXTX?"); return; }
  printKeypadStatus(tx ? "TX" : "RX");
  if (!g_speechEnabled) return;
  speakToken("transceiver");
  playSilenceMs(60);
  speakSimpleBinaryState(tx);
}

void queryBank1Frequency() {
  printKeypadCommand("BANK1 0 SHORT -> FREQ?");
  if (isFtdx10KeypadProfile()) { keypadSendNow("FREQ?"); return; }
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
  printKeypadCommand("BANK1 2 SHORT -> TXFREQ?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("TXFREQ?");
    return;
  }
  if (isFt8x7Ft857FamilyKeypad()) {
    uint64_t hz = 0;
    if (g_ft8x7SplitKnown && !g_ft8x7SplitOn && queryFrequency(hz, 800)) {
      printKeypadStatus(String("TXFREQ: ") + hzToMHzString3(hz) + " MHz");
      speakQueriedFrequencyHz(hz);
    } else {
      printKeypadStatus("TXFREQ unavailable on FT-857/897");
      if (g_speechEnabled) speakNotAvailable();
    }
    return;
  }
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, 800)) {
    if (currentProtocolType() == PROTO_YAESU_FT8X7) {
      if (isFt8x7Ft857FamilyKeypad()) {
        bool splitOn = false;
        if (querySplit(splitOn, 800) && splitOn) {
          if (!guardFt8x7VfoToggleLock()) return;
          if (yaesuCatToggleVfo()) {
            delay(120);
            bool ok = queryFrequency(hz, 800);
            delay(40);
            yaesuCatToggleVfo();
            delay(120);
            if (ok) {
              printKeypadStatus(String("TXFREQ: ") + hzToMHzString3(hz) + " MHz");
              speakQueriedFrequencyHz(hz);
              return;
            }
          }
        }
      }
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

void queryBank1Lock() {
  printKeypadCommand("BANK1 3 SHORT -> LOCK?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("LOCK?");
    return;
  }
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
  printKeypadCommand("BANK1 0 LONG -> FREQ");
  keypadInput().beginEntry(InputMode::FreqEntry, KEYPAD_VFO_CURRENT);
  if (g_speechEnabled) {
    speakFrequencyWord();
    playSilenceMs(80);
    speakToken("please");
  }
}

void roundActiveFrequency(uint32_t stepHz) {
  printKeypadCommand(String("BANK1 0 DOUBLE -> ROUND ") + String((unsigned long)stepHz) + " Hz");
  g_suspendPollingUntilMs = millis() + 1400;
  g_suppressFreqSpeakUntilMs = millis() + 2000;

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

  if (keypadApplyFrequencyHz(rounded, 0)) {
    printKeypadStatus(String("ROUND: ") + hzToMHzString3(hz) + " -> " + hzToMHzString3(rounded) + " MHz");
    speakTunedFrequencyHz(rounded);
    rememberAnnouncedFrequency(rounded);
  } else if (!keypadReportIfTimedOut("ROUND")) {
    printKeypadStatus(currentProtocolType() == PROTO_YAESU_FT8X7 ? "ROUND -> no change" : "ROUND -> failed");
    if (g_speechEnabled) speakError();
  }
}

void beginBank1RfPowerSet() {
  if (currentProtocolType() != PROTO_CIV || !currentStoredProfile().caps.setRfPower) {
    printKeypadStatus("RFPOWER -> unavailable");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  printKeypadCommand("BANK1 6 LONG -> RFPOWER");
  keypadInput().beginEntry(InputMode::RfPowerEntry);
  printKeypadStatus("POWER PLEASE");
  if (g_speechEnabled) {
    speakToken("power");
    playSilenceMs(80);
    speakToken("please");
  }
}

void toggleBank1Lock() {
  printKeypadCommand("BANK1 3 LONG -> LOCK");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("LOCK TOGGLE");
    return;
  }
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

static void sendOrStageBank1Command(const String& keyLabel, const String& cmd, bool suppressModePrefix = false) {
  printKeypadCommand(keyLabel + " -> " + cmd);
  if (AUTO_SEND_BANK1_QUERIES) {
    speakKeypadCommandWord(cmd);
    playSilenceMs(60);
    if (suppressModePrefix) g_suppressModePrefixOnce = true;
    keypadSendNow(cmd);
  } else {
    keypadStageCommand(cmd);
  }
}

void queryBank1Power() { sendOrStageBank1Command("BANK1 4 SHORT", "PO?"); }

void queryBank1RfPower() { sendOrStageBank1Command("BANK1 6 SHORT", "RFPOWER?"); }

void queryBank1Smeter() { sendOrStageBank1Command("BANK1 7 SHORT", "SM?"); }

void queryBank1Swr() { sendOrStageBank1Command("BANK1 8 SHORT", "SWR?"); }

void queryBank1Mode() { sendOrStageBank1Command("BANK1 9 SHORT", "MODE?", true); }

void beginBank1ModeSelect() {
  keypadInput().beginModeSelect(KEYPAD_VFO_CURRENT);
  printKeypadCommand("BANK1 9 LONG -> MODE");
  printKeypadStatus("MODE PLEASE");
  if (g_speechEnabled) {
    speakToken("mode");
    playSilenceMs(80);
    speakToken("please");
  }
}

void ftdx10QueryTuner() { sendKeypadCommand("BANK1 5 SHORT -> TUNER?", "TUNER?"); }

void ftdx10ToggleTuner() { sendKeypadCommand("BANK1 5 LONG -> TUNER TOGGLE", "TUNER TOGGLE"); }

void ftdx10Tune() { sendKeypadCommand("BANK1 5 DOUBLE -> TUNE", "TUNE"); }

void ftdx10QueryPreamp() { sendKeypadCommand("BANK1 6 SHORT -> PA?", "PA?"); }

void ftdx10TogglePreamp() { sendKeypadCommand("BANK1 6 LONG -> PA TOGGLE", "PA TOGGLE"); }
