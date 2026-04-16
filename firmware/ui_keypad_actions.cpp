#include "ui_keypad_actions.h"

#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_protocol.h"
#include "radio_profile.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "protocol_ops_yaesu.h"
#include "ui_console.h"
#include "ui_console_support.h"
#include "ui_keypad.h"
#include "ui_keypad_state.h"
#include "ui_speech.h"
#include "debug_log.h"

static bool isFtdx10KeypadProfile() {
  const StoredProfile& sp = currentStoredProfile();
  return sp.protocolType == PROTO_YAESU_FTDX_ASCII &&
         strcmp(sp.voiceVendor, "yaesu") == 0 &&
         strcmp(sp.voiceDigits, "10") == 0;
}

static void reportFtdx10HiddenKeypadAction(const char* label) {
  if ((bool)Serial) Serial.println(String(label) + " hidden on FTDX10");
  if (g_speechEnabled) speakError();
}

static void printKeypadStatus(const String& line) {
  if ((bool)Serial) Serial.println(line);
}

static void printKeypadCommand(const String& line) {
  if ((bool)Serial) {
    Serial.print("CMD ");
    Serial.println(line);
  }
}

static char ft817CurrentVfoLabelForKeypad() {
  if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817") && !live.activeVfoKnown) {
    rememberActiveVfo(true);
  }
  if (!(currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817") && live.activeVfoKnown)) return '?';
  return live.activeVfoA ? 'A' : 'B';
}

static char ft817OtherVfoLabelForKeypad() {
  if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817") && !live.activeVfoKnown) {
    rememberActiveVfo(true);
  }
  if (!(currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817") && live.activeVfoKnown)) return '?';
  return live.activeVfoA ? 'B' : 'A';
}

static char ft857CurrentVfoLabelForKeypad() {
  if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897") && !live.activeVfoKnown) {
    rememberActiveVfo(true);
  }
  if (!(currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897") && live.activeVfoKnown)) return '?';
  return live.activeVfoA ? 'A' : 'B';
}

static char ft857OtherVfoLabelForKeypad() {
  if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897") && !live.activeVfoKnown) {
    rememberActiveVfo(true);
  }
  if (!(currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897") && live.activeVfoKnown)) return '?';
  return live.activeVfoA ? 'B' : 'A';
}

static void speakFrequencyWord() {
  if (!g_speechEnabled) return;
  speakToken("frequency");
}

static bool rejectFt8x7WriteWhileTx(const char* statusLabel) {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return false;
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
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return true;
  delay(220);
  uint64_t readHz = 0;
  bool ok = false;
  if (targetVfo == 1) ok = queryVfoFrequency(true, readHz, 900);
  else if (targetVfo == 2) ok = queryVfoFrequency(false, readHz, 900);
  else ok = queryFrequency(readHz, 900);
  return ok && readHz == expectedHz;
}

static bool verifyKeypadModeWrite(uint8_t targetVfo, uint8_t expectedMode) {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return true;
  delay(220);
  uint8_t readMode = 0xFF;
  uint8_t filter = 0xFF;
  bool ok = false;
  if (targetVfo == 1) ok = queryVfoMode(true, readMode, filter, 900);
  else if (targetVfo == 2) ok = queryVfoMode(false, readMode, filter, 900);
  else ok = queryMode(readMode, 900);
  return ok && readMode == expectedMode;
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

static void formatCtcssTenthsLabel(uint16_t toneTenths, char* out, size_t outSize) {
  if (!out || outSize < 2) return;
  snprintf(out, outSize, "%u.%u", (unsigned)(toneTenths / 10), (unsigned)(toneTenths % 10));
}

static bool encodeCtcssTenths(uint16_t toneTenths, uint8_t& b0, uint8_t& b1) {
  if (toneTenths > 9999) return false;
  uint16_t value = toneTenths;
  uint8_t d1 = (uint8_t)(value % 10); value /= 10;
  uint8_t d10 = (uint8_t)(value % 10); value /= 10;
  uint8_t d100 = (uint8_t)(value % 10); value /= 10;
  uint8_t d1000 = (uint8_t)(value % 10);
  b0 = (uint8_t)((d1000 << 4) | d100);
  b1 = (uint8_t)((d10 << 4) | d1);
  return true;
}

static bool encodeDcsCode(uint16_t dcsCode, uint8_t& b0, uint8_t& b1) {
  if (dcsCode > 999) return false;
  uint16_t value = dcsCode;
  uint8_t d1 = (uint8_t)(value % 10); value /= 10;
  uint8_t d10 = (uint8_t)(value % 10); value /= 10;
  uint8_t d100 = (uint8_t)(value % 10);
  b0 = d100;
  b1 = (uint8_t)((d10 << 4) | d1);
  return true;
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

static void speakBinaryFeatureState(const uint8_t* featureData, size_t featureLen, bool on) {
  if (!g_speechEnabled) return;
  playClipProgmem(featureData, featureLen);
  playSilenceMs(60);
  playClipProgmem(on ? voice_on : voice_off, on ? voice_on_len : voice_off_len);
}

static void speakNotchCycleState(bool on, NotchWidth width) {
  if (!g_speechEnabled) return;
  speakToken("notch filter");
  playSilenceMs(60);
  if (!on) {
    speakToken("off");
    return;
  }
  switch (width) {
    case NOTCH_WIDTH_NAR: playDigit(1); break;
    case NOTCH_WIDTH_MID: playDigit(2); break;
    case NOTCH_WIDTH_WIDE: playDigit(3); break;
    default: speakToken("on"); break;
  }
}

static void queryBank2Nr() {
  printKeypadCommand("BANK2 1 SHORT -> NR?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NR?");
    return;
  }
  if (!currentStoredProfile().caps.getNr) return;
  g_suspendPollingUntilMs = millis() + 900;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  live.tuning = false;
  live.pendingHz = 0;
  live.tuningStartSpokenHz = 0;
  if (g_audioPlaying) audioAbortNow();
  if (!refreshLiveNr()) {
    printKeypadStatus("NR? -> no reply");
    if (g_speechEnabled) speakError();
    return;
  }
  printKeypadStatus(live.nrOn ? "NR ON" : "NR OFF");
  speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, live.nrOn);
}

static void queryBank2Nb() {
  printKeypadCommand("BANK2 2 SHORT -> NB?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NB?");
    return;
  }
  if (!currentStoredProfile().caps.getNb) return;
  g_suspendPollingUntilMs = millis() + 900;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  live.tuning = false;
  live.pendingHz = 0;
  live.tuningStartSpokenHz = 0;
  if (g_audioPlaying) audioAbortNow();
  if (!refreshLiveNb()) {
    printKeypadStatus("NB? -> no reply");
    if (g_speechEnabled) speakError();
    return;
  }
  printKeypadStatus(live.nbOn ? "NB ON" : "NB OFF");
  speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, live.nbOn);
}

static void queryBank2Notch() {
  printKeypadCommand("BANK2 3 SHORT -> NOTCH?");
  if (isFtdx10KeypadProfile()) {
    keypadSendNow("NOTCH?");
    return;
  }
  if (!currentStoredProfile().caps.getNotch) return;
  g_suspendPollingUntilMs = millis() + 900;
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  live.tuning = false;
  live.pendingHz = 0;
  live.tuningStartSpokenHz = 0;
  if (g_audioPlaying) audioAbortNow();
  if (!refreshLiveNotch()) {
    printKeypadStatus("NOTCH? -> no reply");
    if (g_speechEnabled) speakError();
    return;
  }
  if (!live.notchOn) {
    printKeypadStatus("NOTCH OFF");
    speakNotchCycleState(false, NOTCH_WIDTH_UNKNOWN);
    return;
  }
  if (live.notchWidthValid) {
    if (live.notchWidth == NOTCH_WIDTH_NAR) printKeypadStatus("NOTCH NAR");
    else if (live.notchWidth == NOTCH_WIDTH_MID) printKeypadStatus("NOTCH MID");
    else if (live.notchWidth == NOTCH_WIDTH_WIDE) printKeypadStatus("NOTCH WIDE");
    speakNotchCycleState(true, live.notchWidth);
    return;
  }
  printKeypadStatus("NOTCH ON");
  speakTokenState("notch filter", true);
}

void keypadBank2QueryNr() {
  queryBank2Nr();
}

void keypadBank2QueryNb() {
  queryBank2Nb();
}

void keypadBank2QueryNotch() {
  queryBank2Notch();
}

static void speakKeypadCommandWord(const String& cmd) {
  if (!g_speechEnabled) return;
  if (cmd == "FREQ?") speakFrequencyWord();
  else if (cmd == "MODE?") speakToken("mode");
  else if (cmd == "SM?") speakToken("s_meter");
  else if (cmd == "SWR?") speakToken("swr");
  else if (cmd == "NOTCH?") speakToken("notch filter");
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

void keypadClearAll() {
  g_kpStagedCmd = "";
  g_kpHasStagedCmd = false;
  g_bankSelectActive = false;
  g_bankStage = 0;
  g_freqEntryActive = false;
  g_freqEntryDigits = "";
  g_freqEntryIsMHz = false;
  g_freqEntryTargetVfo = 0;
  g_bank6EntryMode = BANK6_ENTRY_NONE;
  g_bank6EntryDigits = "";
  g_modeSetActive = false;
  g_modeStageActive = false;
  g_modeStageMode = 0xFF;
  g_modeStageTargetVfo = 0;
  g_modeStageKey = 0;
  g_profileSelectActive = false;
  g_profileStageDigits = "";
  g_volStageActive = false;
  g_pendingClickActive = false;
  g_pendingClickBank = 0;
  g_pendingClickKey = 0;
  g_pendingClickAtMs = 0;
  g_oneHoldConsumed = false;
  g_twoHoldConsumed = false;
  g_threeHoldConsumed = false;
  g_fourHoldConsumed = false;
  g_fiveHoldConsumed = false;
  g_sixHoldConsumed = false;
  g_sevenHoldConsumed = false;
  g_eightHoldConsumed = false;
  printKeypadCommand("CLEAR");
  if (g_speechEnabled) playClipProgmem(voice_ok, voice_ok_len);
}

void keypadStageCommand(const String& cmd) {
  g_kpStagedCmd = cmd;
  g_kpHasStagedCmd = true;
  if ((bool)Serial) {
    Serial.print("CMD STAGE ");
    Serial.println(cmd);
  }
  if (g_speechEnabled) {
    speakKeypadCommandWord(cmd);
    playSilenceMs(60);
    speakToken("ok");
  }
}

void keypadSendNow(const String& cmd) {
  if ((bool)Serial) {
    Serial.print("CMD SEND ");
    Serial.println(cmd);
  }
  g_suppressFreqSpeakUntilMs = millis() + 2000;
  live.tuning = false;
  live.pendingHz = 0;
  live.tuningStartSpokenHz = 0;
  if (g_audioPlaying) audioAbortNow();
  g_suspendPollingUntilMs = millis() + 900;
  g_keypadExecuting = true;
  processCommand(cmd);
  g_keypadExecuting = false;
  g_suspendPollingUntilMs = millis() + 900;
}

void keypadEnter() {
  if (g_bankSelectActive) {
    printKeypadCommand("ENTER -> BANK");
    if (g_bankStage >= 1 && g_bankStage <= 9) {
      g_bank = g_bankStage;
      printKeypadStatus(String("BANK ") + String((int)g_bank));
      speakBankNumber();
    } else {
      printKeypadStatus("BANK -> no selection");
    }
    g_bankSelectActive = false;
    g_bankStage = 0;
    return;
  }

  if (g_profileSelectActive) {
    printKeypadCommand("ENTER -> PROFILE");
    int slot = g_profileStageDigits.toInt();
    if (slot >= 1 && slot <= MAX_PROFILE_SLOTS && storedProfileForId((uint8_t)slot)) {
      applyProfile((uint8_t)slot);
      printKeypadStatus(String("PROFILE ") + String(slot));
      speakCurrentProfile();
    } else {
      printKeypadStatus("PROFILE -> no selection");
    }
    g_profileSelectActive = false;
    g_profileStageDigits = "";
    return;
  }

  if (g_freqEntryActive) {
    g_suspendPollingUntilMs = millis() + 1400;
    g_suppressFreqSpeakUntilMs = millis() + 2000;
    if (rejectFt8x7WriteWhileTx("FREQ")) {
      g_freqEntryActive = false;
      g_freqEntryDigits = "";
      g_freqEntryIsMHz = false;
      g_freqEntryTargetVfo = 0;
      return;
    }
    uint64_t hz = g_freqEntryIsMHz ? (uint64_t)g_freqEntryDigits.toInt() * 1000000ULL : (uint64_t)g_freqEntryDigits.toInt() * 1000ULL;
    const uint64_t khz = hz / 1000ULL;
    if (g_freqEntryTargetVfo == 1) printKeypadCommand("ENTER -> VFOA FREQ");
    else if (g_freqEntryTargetVfo == 2) printKeypadCommand("ENTER -> VFOB FREQ");
    else if (g_freqEntryTargetVfo == 3) {
      char which = '?';
      if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817")) which = ft817OtherVfoLabelForKeypad();
      else if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897")) which = ft857OtherVfoLabelForKeypad();
      printKeypadCommand(String("ENTER -> VFO") + which + " FREQ");
    } else if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817")) {
      const char which = ft817CurrentVfoLabelForKeypad();
      printKeypadCommand(String("ENTER -> VFO") + which + " FREQ");
    } else if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897")) {
      const char which = ft857CurrentVfoLabelForKeypad();
      printKeypadCommand(String("ENTER -> VFO") + which + " FREQ");
    }
    else printKeypadCommand("ENTER -> FREQ");
    bool ok = false;
    if (isFtdx10KeypadProfile()) {
      String cmd;
      if (g_freqEntryTargetVfo == 1) cmd = String("VFOA ") + String((unsigned long long)khz);
      else if (g_freqEntryTargetVfo == 2) cmd = String("VFOB ") + String((unsigned long long)khz);
      else cmd = String("FREQ ") + String((unsigned long long)khz);
      keypadSendNow(cmd);
      ok = true;
    } else if (g_freqEntryTargetVfo == 1) ok = setVfoFrequency(true, hz);
    else if (g_freqEntryTargetVfo == 2) ok = setVfoFrequency(false, hz);
    else if (g_freqEntryTargetVfo == 3 && currentProtocolType() == PROTO_YAESU_FT8X7 &&
             (currentProfileVariantIs("ft817") || currentProfileVariantIs("ft857_897"))) {
      if (yaesuCatToggleVfo()) {
        delay(120);
        ok = setFrequency(hz);
        delay(120);
        yaesuCatToggleVfo();
        delay(120);
      }
    }
    else ok = applyFrequencyAndTrack(hz, true);
    if (ok && !isFtdx10KeypadProfile() && g_freqEntryTargetVfo != 3) ok = verifyKeypadFrequencyWrite(g_freqEntryTargetVfo, hz);
    if (ok) {
      if (g_freqEntryTargetVfo == 1) printKeypadStatus(String("VFOA: ") + hzToMHzString3(hz) + " MHz");
      else if (g_freqEntryTargetVfo == 2) printKeypadStatus(String("VFOB: ") + hzToMHzString3(hz) + " MHz");
      else if (g_freqEntryTargetVfo == 3) {
        char which = '?';
        if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817")) which = ft817OtherVfoLabelForKeypad();
        else if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897")) which = ft857OtherVfoLabelForKeypad();
        printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
      } else if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft817")) {
        const char which = ft817CurrentVfoLabelForKeypad();
        printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
      } else if (currentProtocolType() == PROTO_YAESU_FT8X7 && currentProfileVariantIs("ft857_897")) {
        const char which = ft857CurrentVfoLabelForKeypad();
        printKeypadStatus(String("VFO") + which + ": " + hzToMHzString3(hz) + " MHz");
      }
      else printKeypadStatus(String("FREQ: ") + hzToMHzString3(hz) + " MHz");
      if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    } else {
      printKeypadStatus(currentProtocolType() == PROTO_YAESU_FT8X7 ? "FREQ -> no change" : "FREQ -> failed");
      if (g_speechEnabled && currentProtocolType() == PROTO_YAESU_FT8X7) speakError();
    }
    g_freqEntryActive = false;
    g_freqEntryDigits = "";
    g_freqEntryIsMHz = false;
    g_freqEntryTargetVfo = 0;
    return;
  }

  if (g_bank6EntryMode != BANK6_ENTRY_NONE) {
    if (g_bank6EntryMode == BANK6_ENTRY_OFFSET) {
      printKeypadCommand("ENTER -> RPTSHIFT");
      uint64_t hz = (uint64_t)g_bank6EntryDigits.toInt() * 1000ULL;
      if (hz > 0 && yaesuCatSetRepeaterOffsetHzRaw(hz)) {
        printKeypadStatus(String("RPTSHIFT ") + hzToMHzString3(hz) + " MHz");
        speakRepeaterOffsetHz(hz);
      } else {
        printKeypadStatus("RPTSHIFT -> failed");
      }
    } else if (g_bank6EntryMode == BANK6_ENTRY_CTCSS) {
      printKeypadCommand("ENTER -> CTCSS");
      uint16_t toneTenths = (uint16_t)g_bank6EntryDigits.toInt();
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
    } else if (g_bank6EntryMode == BANK6_ENTRY_DCS) {
      printKeypadCommand("ENTER -> DCS");
      uint16_t dcsCode = (uint16_t)g_bank6EntryDigits.toInt();
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
    g_bank6EntryMode = BANK6_ENTRY_NONE;
    g_bank6EntryDigits = "";
    return;
  }

  if (g_modeStageActive) {
    g_suspendPollingUntilMs = millis() + 1400;
    g_suppressFreqSpeakUntilMs = millis() + 2000;
    if (rejectFt8x7WriteWhileTx("MODE")) {
      g_modeStageActive = false;
      g_modeStageMode = 0xFF;
      g_modeStageTargetVfo = 0;
      g_modeStageKey = 0;
      return;
    }
    if (g_modeStageTargetVfo == 1) printKeypadCommand("ENTER -> VFOA MODE");
    else if (g_modeStageTargetVfo == 2) printKeypadCommand("ENTER -> VFOB MODE");
    else printKeypadCommand("ENTER -> MODE");
    bool ok = false;
    if (isFtdx10KeypadProfile()) {
      String cmd;
      if (g_modeStageTargetVfo == 1) cmd = String("VFOA MODE ") + modeToString(g_modeStageMode);
      else if (g_modeStageTargetVfo == 2) cmd = String("VFOB MODE ") + modeToString(g_modeStageMode);
      else cmd = String("MODE ") + modeToString(g_modeStageMode);
      keypadSendNow(cmd);
      ok = true;
    } else if (g_modeStageTargetVfo == 1) ok = setVfoMode(true, g_modeStageMode, 1);
    else if (g_modeStageTargetVfo == 2) ok = setVfoMode(false, g_modeStageMode, 1);
    else ok = applyModeAndTrack(g_modeStageMode, 1);
    if (ok && !isFtdx10KeypadProfile()) ok = verifyKeypadModeWrite(g_modeStageTargetVfo, g_modeStageMode);
    if (ok) {
      if (g_modeStageTargetVfo == 1) printKeypadStatus(String("VFOA MODE: ") + modeToString(g_modeStageMode));
      else if (g_modeStageTargetVfo == 2) printKeypadStatus(String("VFOB MODE: ") + modeToString(g_modeStageMode));
      else printKeypadStatus(String("MODE: ") + modeToString(g_modeStageMode));
      if (g_speechEnabled) {
        g_suppressModePrefixOnce = true;
        speakMode(g_modeStageMode);
      }
    } else {
      printKeypadStatus(currentProtocolType() == PROTO_YAESU_FT8X7 ? "MODE -> no change" : "MODE -> failed");
      if (g_speechEnabled && currentProtocolType() == PROTO_YAESU_FT8X7) speakError();
    }
    g_modeStageActive = false;
    g_modeStageMode = 0xFF;
    g_modeStageTargetVfo = 0;
    g_modeStageKey = 0;
    return;
  }

  if (g_volStageActive) {
    printKeypadCommand("ENTER -> VOLUME");
    applyVolumeLevel(g_volStageLevel);
    printKeypadStatus(String("VOLUME ") + String((int)g_volStageLevel));
    if (g_speechEnabled) speakVolumeLevel(g_volStageLevel);
    g_volStageActive = false;
    return;
  }

  if (g_kpHasStagedCmd) {
    keypadSendNow(g_kpStagedCmd);
    g_kpStagedCmd = "";
    g_kpHasStagedCmd = false;
  }
}

void keypadHandleReleased(char k) {
  if (k == '#') {
    keypadClearAll();
    return;
  }

  if (g_modeSetActive) {
    printKeypadCommand(String("MODE DIGIT -> ") + String(k));
    g_modeStageMode = 0xFF;
    (void)profileModeFromDigit(k, g_modeStageMode);
    if (g_modeStageMode != 0xFF) {
      g_modeStageActive = true;
      g_modeStageKey = k;
      g_suppressModePrefixOnce = true;
      printKeypadStatus(String("MODE STAGE: ") + modeToString(g_modeStageMode));
      speakMode(g_modeStageMode);
    } else {
      printKeypadStatus("MODE DIGIT -> invalid");
    }
    g_modeSetActive = false;
    return;
  }

  if (g_freqEntryActive) {
    if (k >= '0' && k <= '9' && g_freqEntryDigits.length() < 9) {
      g_freqEntryDigits += k;
      printKeypadCommand(String("FREQ DIGIT -> ") + String(k));
      printKeypadStatus(String("FREQ STAGE: ") + g_freqEntryDigits);
      if (g_speechEnabled) speakDigitsAndPoint(String(k));
    }
    return;
  }

  if (g_bank6EntryMode != BANK6_ENTRY_NONE) {
    size_t maxDigits = 4;
    if (g_bank6EntryMode == BANK6_ENTRY_DCS) maxDigits = 3;
    if (k >= '0' && k <= '9' && g_bank6EntryDigits.length() < maxDigits) {
      g_bank6EntryDigits += k;
      if (g_bank6EntryMode == BANK6_ENTRY_OFFSET) {
        printKeypadCommand(String("RPTSHIFT DIGIT -> ") + String(k));
        printKeypadStatus(String("RPTSHIFT STAGE: ") + g_bank6EntryDigits + " kHz");
      } else if (g_bank6EntryMode == BANK6_ENTRY_CTCSS) {
        printKeypadCommand(String("CTCSS DIGIT -> ") + String(k));
        printKeypadStatus(String("CTCSS STAGE: ") + g_bank6EntryDigits);
      } else {
        printKeypadCommand(String("DCS DIGIT -> ") + String(k));
        printKeypadStatus(String("DCS STAGE: ") + g_bank6EntryDigits);
      }
      if (g_speechEnabled) speakDigitsAndPoint(String(k));
    }
    return;
  }

  if (g_bank == 1) {
    switch (k) {
      case '1':
        printKeypadCommand("BANK1 1 SHORT -> RXTX?");
        keypadSendNow("RXTX?");
        return;
      case '2':
        printKeypadCommand("BANK1 2 SHORT -> TXFREQ?");
        keypadSendNow("TXFREQ?");
        return;
      case '3':
        keypadQueryBank1Lock();
        return;
      case '0':
        sendOrStageBank1Command("BANK1 0 SHORT", "FREQ?");
        return;
      case '7':
        sendOrStageBank1Command("BANK1 7 SHORT", "SM?");
        return;
      case '8':
        sendOrStageBank1Command("BANK1 8 SHORT", "SWR?");
        return;
      case '9':
        sendOrStageBank1Command("BANK1 9 SHORT", "MODE?", true);
        return;
      case '4':
        sendOrStageBank1Command("BANK1 4 SHORT", "PO?");
        return;
      case '5':
        if (isFtdx10KeypadProfile()) {
          printKeypadCommand("BANK1 5 SHORT -> TUNER?");
          keypadSendNow("TUNER?");
          return;
        }
        return;
      case '6':
        if (isFtdx10KeypadProfile()) {
          printKeypadCommand("BANK1 6 SHORT -> PA?");
          keypadSendNow("PA?");
          return;
        }
        return;
      default: break;
    }
  } else if (g_bank == 2) {
    switch (k) {
      case '1':
        queryBank2Nr();
        return;
      case '2':
        queryBank2Nb();
        return;
      case '3':
        queryBank2Notch();
        return;
      case '4':
        if (currentProtocolType() == PROTO_CIV) {
          return;
        }
        if (isFtdx10KeypadProfile()) {
          printKeypadCommand("BANK2 4 SHORT -> GT?");
          keypadSendNow("GT?");
          return;
        }
        return;
      case '5':
        if (currentProtocolType() == PROTO_CIV) {
          return;
        }
        if (isFtdx10KeypadProfile()) {
          printKeypadCommand("BANK2 5 SHORT -> PS?");
          keypadSendNow("PS?");
          return;
        }
        return;
      case '6':
        if (currentProtocolType() == PROTO_CIV) {
          return;
        }
        if (isFtdx10KeypadProfile()) {
          printKeypadCommand("BANK2 6 SHORT -> IF?");
          keypadSendNow("IF?");
          return;
        }
        return;
      case '7':
        if (currentProtocolType() == PROTO_CIV) {
          return;
        }
        if (isFtdx10KeypadProfile()) {
          printKeypadCommand("BANK2 7 SHORT -> ID?");
          keypadSendNow("ID?");
          return;
        }
        return;
      case '8':
        if (currentProtocolType() == PROTO_CIV) {
          printKeypadCommand("BANK2 8 SHORT -> FILSHAPE?");
          keypadSendNow("FILSHAPE?");
          return;
        }
        return;
      case '9':
        if (currentProtocolType() == PROTO_CIV) {
          return;
        }
        return;
      default: break;
    }
  } else if (g_bank == 3) {
    switch (k) {
      case '7':
        if (isFtdx10KeypadProfile()) {
          reportFtdx10HiddenKeypadAction("BANK3 BSTACK");
          return;
        }
        printKeypadCommand("BANK3 7 SHORT -> BSTACK? 1");
        keypadSendNow("BSTACK? 1");
        return;
      case '8':
        if (isFtdx10KeypadProfile()) {
          reportFtdx10HiddenKeypadAction("BANK3 BSTACK");
          return;
        }
        printKeypadCommand("BANK3 8 SHORT -> BSTACK? 2");
        keypadSendNow("BSTACK? 2");
        return;
      case '9':
        if (isFtdx10KeypadProfile()) {
          reportFtdx10HiddenKeypadAction("BANK3 BSTACK");
          return;
        }
        printKeypadCommand("BANK3 9 SHORT -> BSTACK? 3");
        keypadSendNow("BSTACK? 3");
        return;
      default:
        break;
    }
  } else if (g_bank == 4) {
    switch (k) {
      case '0':
        return;
      case '1':
        if (isFtdx10KeypadProfile()) {
          reportFtdx10HiddenKeypadAction("BANK4");
          return;
        }
        printKeypadCommand("BANK4 1 SHORT -> MONITOR?");
        keypadSendNow("MONITOR?");
        return;
      case '2':
        if (isFtdx10KeypadProfile()) {
          reportFtdx10HiddenKeypadAction("BANK4");
        }
        return;
      case '3':
        if (isFtdx10KeypadProfile()) {
          reportFtdx10HiddenKeypadAction("BANK4");
          return;
        }
        printKeypadCommand("BANK4 3 SHORT -> TRANSCEIVE?");
        keypadSendNow("TRANSCEIVE?");
        return;
      default:
        break;
    }
  } else if (g_bank == 9) {
    switch (k) {
      case 'A':
        printKeypadCommand("BANK9 A SHORT -> PROFILE?");
        printKeypadStatus("PROFILE CURRENT");
        speakCurrentProfile();
        return;
      case '4':
        printKeypadCommand("BANK9 4 SHORT -> TUNINGSPEECH?");
        printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
        speakTuningSpeechState();
        return;
      case '7':
        g_volStageLevel = 1;
        g_volStageActive = true;
        printKeypadCommand("BANK9 7 SHORT -> VOLUME 1");
        printKeypadStatus("VOLUME STAGE 1");
        if (g_speechEnabled) speakVolumeLevel(g_volStageLevel);
        return;
      case '8':
        g_volStageLevel = 2;
        g_volStageActive = true;
        printKeypadCommand("BANK9 8 SHORT -> VOLUME 2");
        printKeypadStatus("VOLUME STAGE 2");
        if (g_speechEnabled) speakVolumeLevel(g_volStageLevel);
        return;
      case '9':
        g_volStageLevel = 3;
        g_volStageActive = true;
        printKeypadCommand("BANK9 9 SHORT -> VOLUME 3");
        printKeypadStatus("VOLUME STAGE 3");
        if (g_speechEnabled) speakVolumeLevel(g_volStageLevel);
        return;
      default:
        break;
    }
  }
}
