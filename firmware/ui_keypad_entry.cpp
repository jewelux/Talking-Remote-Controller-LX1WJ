// Entries and selections: the digit feedback while typing, and what Enter
// commits. KeypadInput decides which digits an entry takes; this file acts on
// them.
#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_monitor.h"
#include "radio_profile.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "ui_console_support.h"
#include "ui_keypad.h"
#include "ui_keypad_common.h"
#include "ui_speech.h"

static bool rejectFt8x7WriteWhileTx(const char* statusLabel) {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return false;
  if (currentIsFt857Family()) return false;
  if (!currentStoredProfile().caps.getRxTx) return false;
  bool tx = false;
  if (!queryRxTxStatus(tx, 800) || !tx) return false;
  printKeypadStatus(String(statusLabel) + " -> TX");
  speakRxTxState(true);
  return true;
}

// Shared frequency writer used by keypad entry commit and the round-to-500 Hz
// action. The write is not read back: FT8x7 CAT writes are effectively
// write-only, and an immediate readback can race the radio and falsely report
// "no change" after a successful write.
bool keypadApplyFrequencyHz(uint64_t hz, TargetVfo targetVfo) {
  bool ok = false;
  if (targetVfo == TargetVfo::A) {
    ok = setVfoFrequency(true, hz);
  } else if (targetVfo == TargetVfo::B) {
    ok = setVfoFrequency(false, hz);
  } else if (targetVfo == TargetVfo::Other && currentProtocolType() == PROTO_YAESU_FT8X7 &&
             (currentIsFt817Family() || currentIsFt857Family())) {
    if (yaesuCatToggleVfo()) {
      delay(120);
      ok = setFrequency(hz);
      delay(120);
      yaesuCatToggleVfo();
      delay(120);
    }
  } else {
    ok = applyFrequencyAndTrack(hz, true);
  }
  return ok;
}

static void speakRepeaterOffsetHz(uint64_t hz) {
  if (!g_speechEnabled) return;
  speakLabel("repeater frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

void keypadEntryDigit(const EntrySpec& entry, char key, const char* digits) {
  if (key == '*') {
    printKeypadCommand(String(entry.name) + " POINT -> *");
    printKeypadStatus(String(entry.name) + " STAGE: " + digits);
    if (g_speechEnabled) speakToken("point");
    return;
  }
  printKeypadCommand(String(entry.name) + " DIGIT -> " + String(key));
  switch (entry.mode) {
    case InputMode::ProfileSelect:
      printKeypadStatus(String("PROFILE ") + digits);
      if (g_speechEnabled) playDigit((uint8_t)(key - '0'));
      return;
    default:
      printKeypadStatus(String(entry.name) + " STAGE: " + digits + entry.unit);
      if (g_speechEnabled) speakDigitsAndPoint(String(key));
      return;
  }
}

// Bank select commits on its digit, without Enter.
static void commitBank() {
  printKeypadCommand(String("BANK SELECT DIGIT -> ") + String((int)uiGetBank()));
  printKeypadStatus(String("BANK ") + String((int)uiGetBank()));
  speakBankNumber();
}

static void commitProfile(const char* digits) {
  printKeypadCommand("ENTER -> PROFILE");
  int slot = atoi(digits);
  if (slot >= 1 && slot <= MAX_PROFILE_SLOTS && storedProfileForId((uint8_t)slot)) {
    applyProfile((uint8_t)slot);
    printKeypadStatus(String("PROFILE ") + String(slot));
    speakCurrentProfile();
  } else {
    printKeypadStatus("PROFILE -> not available");
    if (g_speechEnabled) speakNotAvailable();
  }
}

static void commitFrequency(const char* digits, TargetVfo targetVfo) {
  prepareKeypadRadioWrite();
  if (rejectFt8x7WriteWhileTx("FREQ")) return;
  RadioFrequency parsedFreq;
  if (!RadioFrequency::parseEntry(String(digits), parsedFreq)) {
    printKeypadStatus("FREQ -> invalid");
    if (g_speechEnabled) speakError();
    return;
  }
  const uint64_t hz = parsedFreq.hz();
  if (targetVfo == TargetVfo::A) printKeypadCommand("ENTER -> VFOA FREQ");
  else if (targetVfo == TargetVfo::B) printKeypadCommand("ENTER -> VFOB FREQ");
  else if (targetVfo == TargetVfo::Other) {
    const char which = ft8x7OtherVfoLabel();
    printKeypadCommand(String("ENTER -> VFO") + which + " FREQ");
  } else if (isFt8x7Ft817Keypad() || isFt8x7Ft857FamilyKeypad()) {
    const char which = ft8x7CurrentVfoLabel();
    printKeypadCommand(String("ENTER -> VFO") + which + " FREQ");
  }
  else printKeypadCommand("ENTER -> FREQ");
  bool ok = keypadApplyFrequencyHz(hz, targetVfo);
  if (ok) {
    if (targetVfo == TargetVfo::A) printKeypadStatus(String("VFOA: ") + hzToMHzString3(hz) + " MHz");
    else if (targetVfo == TargetVfo::B) printKeypadStatus(String("VFOB: ") + hzToMHzString3(hz) + " MHz");
    else if (targetVfo == TargetVfo::Other) {
      const char which = ft8x7OtherVfoLabel();
      printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
    } else if (isFt8x7Ft817Keypad() || isFt8x7Ft857FamilyKeypad()) {
      const char which = ft8x7CurrentVfoLabel();
      printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
    }
    else printKeypadStatus(String("FREQ: ") + hzToMHzString3(hz) + " MHz");
    if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
  } else if (!keypadReportIfTimedOut("FREQ")) {
    printKeypadStatus(currentProtocolType() == PROTO_YAESU_FT8X7 ? "FREQ -> no change" : "FREQ -> failed");
    if (g_speechEnabled && currentProtocolType() == PROTO_YAESU_FT8X7) speakError();
  }
}

static void commitRfPower(const char* digits) {
  printKeypadCommand("ENTER -> RFPOWER");
  keypadSendNow(String("RFPOWER ") + digits);
}

static void commitCivAddress(const char* digits) {
  printKeypadCommand("ENTER -> CIVADDR");
  int addr = atoi(digits);
  if (addr < 0 || addr > 255 || !setCurrentCivConnection((uint8_t)addr, currentConnectionProfile().baud)) {
    printKeypadStatus("CIVADDR -> invalid");
    if (g_speechEnabled) speakError();
    return;
  }
  char hex[3] = "";
  formatHexByte((uint8_t)addr, hex, sizeof(hex));
  printKeypadStatus(String("CI ") + hex);
  speakCivAddressValue((uint8_t)addr, true);
}

static void commitRepeaterOffset(const char* digits) {
  printKeypadCommand("ENTER -> RPTSHIFT");
  uint64_t hz = (uint64_t)atoi(digits) * 1000ULL;
  if (hz > 0 && yaesuCatSetRepeaterOffsetHzRaw(hz)) {
    printKeypadStatus(String("RPTSHIFT ") + hzToMHzString3(hz) + " MHz");
    speakRepeaterOffsetHz(hz);
  } else {
    printKeypadStatus("RPTSHIFT -> failed");
  }
}

static void commitCtcss(const char* digits) {
  printKeypadCommand("ENTER -> CTCSS");
  uint16_t toneTenths = (uint16_t)atoi(digits);
  char label[12] = "";
  formatCtcssTenthsLabel(toneTenths, label, sizeof(label));
  if (toneTenths > 0 && yaesuCatSetCtcssTenths(toneTenths)) {
    printKeypadStatus(String("CTCSS ") + label);
    if (g_speechEnabled) {
      speakLabel("ctcss");
      speakDigitsAndPoint(label);
    }
  } else if (!yaesuCtcssTenthsValid(toneTenths)) {
    printKeypadStatus("CTCSS -> invalid");
  } else {
    printKeypadStatus("CTCSS -> failed");
  }
}

static void commitDcs(const char* digits) {
  printKeypadCommand("ENTER -> DCS");
  uint16_t dcsCode = (uint16_t)atoi(digits);
  char label[8] = "";
  snprintf(label, sizeof(label), "%03u", (unsigned)dcsCode);
  if (yaesuCatSetDcsCode(dcsCode)) {
    printKeypadStatus(String("DCS ") + label);
    if (g_speechEnabled) {
      speakLabel("dcs");
      speakDigitsAndPoint(label);
    }
  } else if (!yaesuDcsCodeValid(dcsCode)) {
    printKeypadStatus("DCS -> invalid");
  } else {
    printKeypadStatus("DCS -> failed");
  }
}

void keypadEntryCommit(InputMode mode, const char* digits, TargetVfo targetVfo) {
  switch (mode) {
    case InputMode::BankSelect: commitBank(); return;
    case InputMode::ProfileSelect: commitProfile(digits); return;
    case InputMode::FreqEntry: commitFrequency(digits, targetVfo); return;
    case InputMode::RfPowerEntry: commitRfPower(digits); return;
    case InputMode::CivAddrEntry: commitCivAddress(digits); return;
    case InputMode::RptOffsetEntry: commitRepeaterOffset(digits); return;
    case InputMode::CtcssEntry: commitCtcss(digits); return;
    case InputMode::DcsEntry: commitDcs(digits); return;
    case InputMode::Normal:
    case InputMode::ModeSelect: return;
  }
}

// A key that picks no mode, or a mode the profile cannot set, is rejected by
// the state machine.
bool keypadModeDigit(char key, uint8_t& mode) {
  if (!modeFromDigit(key, mode) || !canSetMode(mode)) return false;
  printKeypadCommand(String("MODE DIGIT -> ") + String(key));
  printKeypadStatus(String("MODE STAGE: ") + modeToString(mode));
  speakModeName(mode);
  return true;
}

void keypadModeCommit(uint8_t mode, TargetVfo targetVfo) {
  prepareKeypadRadioWrite();
  if (rejectFt8x7WriteWhileTx("MODE")) return;
  if (targetVfo == TargetVfo::A) printKeypadCommand("ENTER -> VFOA MODE");
  else if (targetVfo == TargetVfo::B) printKeypadCommand("ENTER -> VFOB MODE");
  else printKeypadCommand("ENTER -> MODE");
  bool ok = false;
  if (isFtdx10KeypadProfile()) {
    String cmd;
    if (targetVfo == TargetVfo::A) cmd = String("VFOA MODE ") + modeToString(mode);
    else if (targetVfo == TargetVfo::B) cmd = String("VFOB MODE ") + modeToString(mode);
    else cmd = String("MODE ") + modeToString(mode);
    keypadSendNow(cmd);
    ok = true;
  } else if (targetVfo == TargetVfo::A) ok = setVfoMode(true, mode, 1);
  else if (targetVfo == TargetVfo::B) ok = setVfoMode(false, mode, 1);
  else ok = applyModeAndTrack(mode, 1);
  if (ok) {
    if (targetVfo == TargetVfo::A) printKeypadStatus(String("VFOA MODE: ") + modeToString(mode));
    else if (targetVfo == TargetVfo::B) printKeypadStatus(String("VFOB MODE: ") + modeToString(mode));
    else printKeypadStatus(String("MODE: ") + modeToString(mode));
    if (g_speechEnabled) speakModeName(mode);
  } else if (!keypadReportIfTimedOut("MODE")) {
    printKeypadStatus(currentProtocolType() == PROTO_YAESU_FT8X7 ? "MODE -> no change" : "MODE -> failed");
    if (g_speechEnabled && currentProtocolType() == PROTO_YAESU_FT8X7) speakError();
  }
}
