// Bank 1 keypad actions: frequency, lock, power, meters and mode.
#include "ui_keypad_bank.h"
#include "engine_civ.h"
#include "radio_frequency.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "ui_features.h"

static bool queryDialLockReliable(bool& onOut) {
  // One EEPROM read on the FT-8x7. No stale fallback: a toggle must start from the radio's real
  // state.
  if (currentProtocolType() == PROTO_YAESU_FT8X7) return queryDialLock(onOut, kCatReplyTimeoutMs);
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (queryDialLock(onOut, kCatReplyTimeoutMs)) {
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
  // An FT-8x7 profile that leaves out getRxTx.
  if (currentProtocolType() == PROTO_YAESU_FT8X7 && !currentProfile().caps.getRxTx) {
    printKeypadStatus("RXTX -> unavailable");
    speakKeypadFailure("RXTX", KeypadFailure::NotAvailable);
    return;
  }
  bool tx = false;
  if (!queryRxTxStatus(tx, kCatReplyTimeoutMs)) { keypadReportFailure("RXTX?"); return; }
  printKeypadStatus("{}", tx ? "TX" : "RX");
  speakRxTxState(tx);
}

void queryBank1Frequency() {
  printKeypadAction("FREQ?");
  uint64_t hz = 0;
  if (!queryFrequency(hz, kCatReplyTimeoutMs)) {
    keypadReportFailure("FREQ?");
    return;
  }
  printKeypadStatus("FREQ: {} MHz", RadioFrequency::fromHz(hz));
  speakQueriedFrequencyHz(hz);
  rememberAnnouncedFrequency(hz);
}

void queryBank1TxFrequency() {
  printKeypadAction("TXFREQ?");
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, kCatReplyTimeoutMs)) {
    // FT-8x7 without a TX frequency reply: say the frequency instead.
    if (currentProtocolType() == PROTO_YAESU_FT8X7) {
      if (queryFrequency(hz, kCatReplyTimeoutMs)) {
        printKeypadStatus("TXFREQ: {} MHz", RadioFrequency::fromHz(hz));
        speakQueriedFrequencyHz(hz);
      } else {
        printKeypadStatus("TXFREQ -> unavailable");
        speakKeypadFailure("TXFREQ", KeypadFailure::NotAvailable);
      }
    } else {
      keypadReportFailure("TXFREQ?");
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
  if (querySplit(splitOn, kCatReplyTimeoutMs) && !splitOn && queryFrequency(hz, kCatReplyTimeoutMs)) {
    printKeypadStatus("TXFREQ: {} MHz", RadioFrequency::fromHz(hz));
    speakQueriedFrequencyHz(hz);
  } else {
    printKeypadStatus("TXFREQ unavailable on FT-857/897");
    speakKeypadFailure("TXFREQ", KeypadFailure::NotAvailable);
  }
}

void queryBank1Lock() {
  printKeypadAction("LOCK?");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (!queryDialLockReliable(on)) {
    if (keypadReportIfTimedOut("LOCK?")) return;
    printKeypadStatus("LOCK UNKNOWN");
    speakKeypadFailure("LOCK", KeypadFailure::Error);
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
  if (!queryFrequency(hz, kCatReplyTimeoutMs)) {
    // Radio not responding: do not round or announce a stale value.
    keypadReportFailure("ROUND");
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
    speakKeypadFailure("ROUND", KeypadFailure::Error);
  }
}

// FT-817/818: a higher key is more power.
static constexpr ChoiceOption kFt817PowerKeys[] = {
    {'1', (uint8_t)Ft817PowerLevel::L1},
    {'2', (uint8_t)Ft817PowerLevel::L2},
    {'3', (uint8_t)Ft817PowerLevel::L3},
    {'4', (uint8_t)Ft817PowerLevel::High},
};

static void sayPickedFt817Power(char key, uint8_t level) {
  printKeypadCommand("RFPOWER LEVEL -> {}", key);
  printKeypadStatus("RFPOWER STAGE: {}", ft817PowerLevelName((Ft817PowerLevel)level));
  speakFt817PowerLevel((Ft817PowerLevel)level);
}

static void commitFt817Power(uint8_t level, TargetVfo) {
  printKeypadCommand("ENTER -> RFPOWER {}", ft817PowerLevelName((Ft817PowerLevel)level));
  prepareKeypadRadioWrite();
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(ft817PowerLevelSet((Ft817PowerLevel)level, state), "RFPOWER", state);
}

static constexpr ChoiceSpec kFt817PowerChoice = {
    "RFPOWER LEVEL", kFt817PowerKeys, sizeof(kFt817PowerKeys) / sizeof(kFt817PowerKeys[0]),
    nullptr, sayPickedFt817Power, commitFt817Power};

// The FT-817/818 picks one of its levels, read first so that B and C walk from it.
static void beginFt817PowerSelect() {
  printKeypadAction("RFPOWER");
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  if (keypadReportFeatureFailure(ft8x7SettingQuery(Ft8x7Setting::RfPower, state), "RFPOWER")) return;
  EntrySeed seed;
  seed.known = state.hasPowerLevel;
  seed.value = (uint8_t)state.powerLevel;
  keypadBeginChoiceSelect(kFt817PowerChoice, TargetVfo::Current, seed);
  printKeypadStatus("POWER PLEASE");
  speakPrompt("power");
}

static bool picksFt817PowerLevel() {
  return currentProtocolType() == PROTO_YAESU_FT8X7 && currentIsFt817Family() && currentProfile().caps.setRfPower;
}

void beginBank1RfPowerSet() {
  if (picksFt817PowerLevel()) {
    beginFt817PowerSelect();
    return;
  }
  keypadSettingEntry(Setting::RfPower);
}

void nudgeBank1RfPower(int8_t dir, bool big) {
  if (!picksFt817PowerLevel()) {
    keypadSettingNudge(Setting::RfPower, dir, big);
    return;
  }
  // Four levels: a big step is one too.
  printKeypadAction("RFPOWER {}", dir > 0 ? "UP" : "DOWN");
  prepareKeypadSpeechResponse();
  Ft8x7SettingState state;
  if (keypadReportFeatureFailure(ft8x7SettingQuery(Ft8x7Setting::RfPower, state), "RFPOWER")) return;
  if (!state.hasPowerLevel) {
    keypadReportFeatureFailure(FeatureStatus::NoReply, "RFPOWER");
    return;
  }
  EntrySeed seed;
  seed.known = true;
  seed.value = (uint8_t)state.powerLevel;
  if (!keypadNudgeChoice(kFt817PowerChoice, seed, dir)) {
    keypadValueLimit("RFPOWER", dir > 0);
    return;
  }
  speakPlease();
}

void toggleBank1Lock() {
  printKeypadAction("LOCK");
  prepareKeypadSpeechResponse();
  bool on = false;
  if (!queryDialLockReliable(on)) {
    if (keypadReportIfTimedOut("LOCK?")) return;
    printKeypadStatus("LOCK UNKNOWN");
    speakKeypadFailure("LOCK", KeypadFailure::Error);
    return;
  }
  if (!setDialLock(!on)) { keypadReportFailure("LOCK"); return; }
  printKeypadStatus("LOCK {}", !on ? "ON" : "OFF");
  speakTokenState("lock", !on);
}

static void sendBank1Query(const String& cmd) {
  printKeypadAction("{}", cmd.c_str());
  // The SWR, RF power and ALC replies say their own word, so saying it here too doubles it.
  if (cmd != "SWR?" && cmd != "RFPOWER?" && cmd != "ALC?") speakKeypadCommandWord(cmd);
  keypadSendNow(cmd);
}

void queryBank1Power() { sendBank1Query("PO?"); }

void queryBank1RfPower() { sendBank1Query("RFPOWER?"); }

void queryBank1Smeter() { sendBank1Query("SM?"); }

void queryBank1Swr() { sendBank1Query("SWR?"); }

void queryBank1Alc() { sendBank1Query("ALC?"); }

void queryBank1Mode() { sendBank1Query("MODE?"); }

void beginBank1ModeSelect() {
  keypadBeginModeSelect(TargetVfo::Current);
  printKeypadAction("MODE");
  printKeypadStatus("MODE PLEASE");
  speakPrompt("mode");
}

// The mode is read, never stepped live: the pick opens one mode away, in digit order.
void nudgeBank1Mode(int8_t dir) {
  printKeypadAction("MODE {}", dir > 0 ? "UP" : "DOWN");
  prepareKeypadSpeechResponse();
  uint8_t mode = 0;
  if (!queryMode(mode, kCatReplyTimeoutMs)) {
    keypadReportFailure("MODE");
    return;
  }
  if (!keypadNudgeModeSelect(mode, dir)) {
    keypadValueLimit("MODE", dir > 0);
    return;
  }
  speakPlease();
}
