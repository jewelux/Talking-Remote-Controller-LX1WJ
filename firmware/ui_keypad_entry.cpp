// Entries and selections: the digit feedback while typing, and what Enter
// commits. KeypadInput decides which digits an entry takes; this file acts on
// them.
#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_monitor.h"
#include "radio_prefs.h"
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
  if (currentProfileVariantIs("ft857_897")) return false;
  if (!currentStoredProfile().caps.getRxTx) return false;
  const bool isFt817 = currentProfileVariantIs("ft817");
  uint8_t txHits = 0;
  const uint8_t attempts = isFt817 ? 3 : 1;
  for (uint8_t i = 0; i < attempts; ++i) {
    bool tx = false;
    if (queryRxTxStatus(tx, 800) && tx) ++txHits;
    if (i + 1 < attempts) delay(40);
  }
  if ((isFt817 && txHits < attempts) || (!isFt817 && txHits == 0)) return false;
  printKeypadStatus(String(statusLabel) + " -> TX");
  if (g_speechEnabled) {
    speakToken("transceiver");
    playSilenceMs(60);
    speakToken("tx");
  }
  return true;
}

static bool verifyKeypadFrequencyWrite(uint8_t targetVfo, uint64_t expectedHz) {
  // FT8x7 CAT write commands are effectively write-only; immediate readback can
  // race the radio and falsely report "no change" after a successful write.
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return true;
  (void)targetVfo;
  (void)expectedHz;
  return true;
}

// Shared frequency writer used by keypad entry commit and the round-to-500 Hz
// action. targetVfo: 0 = current VFO, 1 = VFO A, 2 = VFO B, 3 = other VFO (FT8x7).
bool keypadApplyFrequencyHz(uint64_t hz, uint8_t targetVfo) {
  bool ok = false;
  if (targetVfo == 1) {
    ok = setVfoFrequency(true, hz);
  } else if (targetVfo == 2) {
    ok = setVfoFrequency(false, hz);
  } else if (targetVfo == 3 && currentProtocolType() == PROTO_YAESU_FT8X7 &&
             (currentProfileVariantIs("ft817") || currentProfileVariantIs("ft857_897"))) {
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
  if (ok && targetVfo != 3) ok = verifyKeypadFrequencyWrite(targetVfo, hz);
  return ok;
}

static bool verifyKeypadModeWrite(uint8_t targetVfo, uint8_t expectedMode) {
  // Same as frequency: trust the write result and update the local cache.
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return true;
  (void)targetVfo;
  (void)expectedMode;
  return true;
}

static constexpr uint16_t kValidCtcssTenths[] = {
  670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
  948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
  1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
  1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
  2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541
};

static constexpr uint16_t kValidDcsCodes[] = {
  23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72, 73,
  74, 114, 115, 116, 122, 125, 131, 132, 134, 143, 145, 152, 155, 156, 162,
  165, 172, 174, 205, 212, 223, 225, 226, 243, 244, 245, 246, 251, 252, 255,
  261, 263, 265, 266, 271, 274, 306, 311, 315, 325, 331, 332, 343, 346, 351,
  356, 364, 365, 371, 411, 412, 413, 423, 431, 432, 445, 446, 452, 454, 455,
  462, 464, 465, 466, 503, 506, 516, 523, 526, 532, 546, 565, 606, 612, 624,
  627, 631, 632, 654, 662, 664, 703, 712, 723, 731, 732, 734, 743, 754
};

template <size_t N>
static bool containsU16(const uint16_t (&values)[N], uint16_t needle) {
  for (size_t i = 0; i < N; ++i) {
    if (values[i] == needle) return true;
  }
  return false;
}

static bool isValidCtcssTenths(uint16_t toneTenths) {
  return containsU16(kValidCtcssTenths, toneTenths);
}

static bool isValidDcsCode(uint16_t dcsCode) {
  return containsU16(kValidDcsCodes, dcsCode);
}

static void speakRepeaterOffsetHz(uint64_t hz) {
  if (!g_speechEnabled) return;
  speakToken("repeater");
  playSilenceMs(60);
  speakFrequencyWord();
  playSilenceMs(60);
  speakDigitsAndPoint(hzToMHzString3(hz));
}

static bool applyCtcssTenths(uint16_t toneTenths) {
  if (!isValidCtcssTenths(toneTenths)) return false;
  uint8_t b0 = 0;
  uint8_t b1 = 0;
  if (!encodeCtcssTenths(toneTenths, b0, b1)) return false;
  uint8_t data[4] = {b0, b1, 0x00, 0x00};
  if (currentProfileVariantIs("ft857_897")) {
    data[2] = b0;
    data[3] = b1;
  }
  if (!yaesuCatSetCtcssToneRaw(data)) return false;
  live.ctcssValid = true;
  live.ctcssTenths = toneTenths;
  return true;
}

static bool applyDcsCode(uint16_t dcsCode) {
  if (!isValidDcsCode(dcsCode)) return false;
  uint8_t b0 = 0;
  uint8_t b1 = 0;
  if (!encodeDcsCode(dcsCode, b0, b1)) return false;
  uint8_t data[4] = {b0, b1, 0x00, 0x00};
  if (currentProfileVariantIs("ft857_897")) {
    data[2] = b0;
    data[3] = b1;
  }
  if (!yaesuCatSetDcsCodeRaw(data)) return false;
  live.dcsValid = true;
  live.dcsCode = dcsCode;
  return true;
}

void keypadEntryDigit(InputMode mode, char key, const char* digits) {
  switch (mode) {
    case InputMode::BankSelect:
      printKeypadCommand(String("BANK SELECT DIGIT -> ") + String(key));
      printKeypadStatus(String("BANK ") + String((int)(key - '0')));
      if (g_speechEnabled) playDigit((uint8_t)(key - '0'));
      return;
    case InputMode::ProfileSelect:
      printKeypadCommand(String("PROFILE DIGIT -> ") + String(key));
      printKeypadStatus(String("PROFILE ") + digits);
      if (g_speechEnabled) playDigit((uint8_t)(key - '0'));
      return;
    case InputMode::FreqEntry:
      if (key == '*') {
        printKeypadCommand("FREQ POINT -> *");
        printKeypadStatus(String("FREQ STAGE: ") + digits);
        if (g_speechEnabled) speakToken("point");
        return;
      }
      printKeypadCommand(String("FREQ DIGIT -> ") + String(key));
      printKeypadStatus(String("FREQ STAGE: ") + digits);
      break;
    case InputMode::RfPowerEntry:
      printKeypadCommand(String("RFPOWER DIGIT -> ") + String(key));
      printKeypadStatus(String("RFPOWER STAGE: ") + digits + " W");
      break;
    case InputMode::CivAddrEntry:
      printKeypadCommand(String("CIVADDR DIGIT -> ") + String(key));
      printKeypadStatus(String("CIVADDR STAGE: ") + digits);
      break;
    case InputMode::RptOffsetEntry:
      printKeypadCommand(String("RPTSHIFT DIGIT -> ") + String(key));
      printKeypadStatus(String("RPTSHIFT STAGE: ") + digits + " kHz");
      break;
    case InputMode::CtcssEntry:
      printKeypadCommand(String("CTCSS DIGIT -> ") + String(key));
      printKeypadStatus(String("CTCSS STAGE: ") + digits);
      break;
    case InputMode::DcsEntry:
      printKeypadCommand(String("DCS DIGIT -> ") + String(key));
      printKeypadStatus(String("DCS STAGE: ") + digits);
      break;
    case InputMode::Normal:
    case InputMode::ModeSelect:
      return;
  }
  if (g_speechEnabled) speakDigitsAndPoint(String(key));
}

static void commitBank(const char* digits) {
  printKeypadCommand("ENTER -> BANK");
  if (digits[0]) {
    printKeypadStatus(String("BANK ") + String((int)keypadInput().bank()));
    speakBankNumber();
  } else {
    printKeypadStatus("BANK -> no selection");
    playBeep();
  }
}

static void commitProfile(const char* digits) {
  printKeypadCommand("ENTER -> PROFILE");
  int slot = atoi(digits);
  if (slot >= 1 && slot <= MAX_PROFILE_SLOTS && storedProfileForId((uint8_t)slot)) {
    applyProfile((uint8_t)slot);
    printKeypadStatus(String("PROFILE ") + String(slot));
    speakCurrentProfile();
  } else {
    printKeypadStatus("PROFILE -> no selection");
    playBeep();
  }
}

static void commitFrequency(const char* digits, uint8_t targetVfo) {
  g_suspendPollingUntilMs = millis() + 1400;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  if (rejectFt8x7WriteWhileTx("FREQ")) return;
  RadioFrequency parsedFreq;
  if (!RadioFrequency::parseEntry(String(digits), parsedFreq)) {
    printKeypadStatus("FREQ -> invalid");
    if (g_speechEnabled) speakError();
    return;
  }
  const uint64_t hz = parsedFreq.hz();
  if (targetVfo == KEYPAD_VFO_A) printKeypadCommand("ENTER -> VFOA FREQ");
  else if (targetVfo == KEYPAD_VFO_B) printKeypadCommand("ENTER -> VFOB FREQ");
  else if (targetVfo == KEYPAD_VFO_OTHER) {
    const char which = ft8x7OtherVfoLabel();
    printKeypadCommand(String("ENTER -> VFO") + which + " FREQ");
  } else if (isFt8x7Ft817Keypad() || isFt8x7Ft857FamilyKeypad()) {
    const char which = ft8x7CurrentVfoLabel();
    printKeypadCommand(String("ENTER -> VFO") + which + " FREQ");
  }
  else printKeypadCommand("ENTER -> FREQ");
  bool ok = keypadApplyFrequencyHz(hz, targetVfo);
  if (ok) {
    if (targetVfo == KEYPAD_VFO_A) printKeypadStatus(String("VFOA: ") + hzToMHzString3(hz) + " MHz");
    else if (targetVfo == KEYPAD_VFO_B) printKeypadStatus(String("VFOB: ") + hzToMHzString3(hz) + " MHz");
    else if (targetVfo == KEYPAD_VFO_OTHER) {
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
  if (!digits[0]) {
    printKeypadStatus("RFPOWER -> no value");
    if (g_speechEnabled) speakError();
  } else {
    keypadSendNow(String("RFPOWER ") + digits);
  }
}

static void commitCivAddress(const char* digits) {
  printKeypadCommand("ENTER -> CIVADDR");
  if (!digits[0]) {
    printKeypadStatus("CIVADDR -> no value");
    if (g_speechEnabled) speakError();
    return;
  }
  int addr = atoi(digits);
  StoredProfile* sp = (isValidProfileId(g_profileId) && g_slotProfiles[g_profileId - 1].valid) ? &g_slotProfiles[g_profileId - 1] : nullptr;
  if (!sp || sp->protocolType != PROTO_CIV || addr < 0 || addr > 255) {
    printKeypadStatus("CIVADDR -> invalid");
    if (g_speechEnabled) speakError();
    return;
  }
  sp->civ.civAddr = (uint8_t)addr;
  saveConnectionOverrideToNvs(g_profileId, sp->civ.civAddr, sp->civ.baud);
  applyProfile(g_profileId);
  char hex[3] = "";
  formatHexByte(sp->civ.civAddr, hex, sizeof(hex));
  printKeypadStatus(String("CI ") + hex);
  speakCivAddressValue(sp->civ.civAddr, true);
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
  if (toneTenths > 0 && applyCtcssTenths(toneTenths)) {
    printKeypadStatus(String("CTCSS ") + label);
    if (g_speechEnabled) {
      speakToken("ctcss");
      playSilenceMs(60);
      speakDigitsAndPoint(label);
    }
  } else if (!isValidCtcssTenths(toneTenths)) {
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
  if (applyDcsCode(dcsCode)) {
    printKeypadStatus(String("DCS ") + label);
    if (g_speechEnabled) {
      speakToken("dcs");
      playSilenceMs(60);
      speakDigitsAndPoint(label);
    }
  } else if (!isValidDcsCode(dcsCode)) {
    printKeypadStatus("DCS -> invalid");
  } else {
    printKeypadStatus("DCS -> failed");
  }
}

void keypadEntryCommit(InputMode mode, const char* digits, uint8_t targetVfo) {
  switch (mode) {
    case InputMode::BankSelect: commitBank(digits); return;
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

bool keypadModeDigit(char key, uint8_t& mode) {
  printKeypadCommand(String("MODE DIGIT -> ") + String(key));
  // A digit the profile has no mode code for still stages its mode, as before.
  mode = 0xFF;
  (void)profileModeFromDigit(key, mode);
  if (mode == 0xFF) {
    printKeypadStatus("MODE DIGIT -> invalid");
    playBeep();
    return false;
  }
  g_suppressModePrefixOnce = true;
  printKeypadStatus(String("MODE STAGE: ") + modeToString(mode));
  speakMode(mode);
  return true;
}

void keypadModeCommit(uint8_t mode, uint8_t targetVfo) {
  g_suspendPollingUntilMs = millis() + 1400;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  if (rejectFt8x7WriteWhileTx("MODE")) return;
  if (targetVfo == KEYPAD_VFO_A) printKeypadCommand("ENTER -> VFOA MODE");
  else if (targetVfo == KEYPAD_VFO_B) printKeypadCommand("ENTER -> VFOB MODE");
  else printKeypadCommand("ENTER -> MODE");
  bool ok = false;
  if (isFtdx10KeypadProfile()) {
    String cmd;
    if (targetVfo == KEYPAD_VFO_A) cmd = String("VFOA MODE ") + modeToString(mode);
    else if (targetVfo == KEYPAD_VFO_B) cmd = String("VFOB MODE ") + modeToString(mode);
    else cmd = String("MODE ") + modeToString(mode);
    keypadSendNow(cmd);
    ok = true;
  } else if (targetVfo == KEYPAD_VFO_A) ok = setVfoMode(true, mode, 1);
  else if (targetVfo == KEYPAD_VFO_B) ok = setVfoMode(false, mode, 1);
  else ok = applyModeAndTrack(mode, 1);
  if (ok && !isFtdx10KeypadProfile()) ok = verifyKeypadModeWrite(targetVfo, mode);
  if (ok) {
    if (targetVfo == KEYPAD_VFO_A) printKeypadStatus(String("VFOA MODE: ") + modeToString(mode));
    else if (targetVfo == KEYPAD_VFO_B) printKeypadStatus(String("VFOB MODE: ") + modeToString(mode));
    else printKeypadStatus(String("MODE: ") + modeToString(mode));
    if (g_speechEnabled) {
      g_suppressModePrefixOnce = true;
      speakMode(mode);
    }
  } else if (!keypadReportIfTimedOut("MODE")) {
    printKeypadStatus(currentProtocolType() == PROTO_YAESU_FT8X7 ? "MODE -> no change" : "MODE -> failed");
    if (g_speechEnabled && currentProtocolType() == PROTO_YAESU_FT8X7) speakError();
  }
}
