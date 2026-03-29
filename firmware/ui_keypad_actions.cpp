#include "ui_keypad_actions.h"

#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_protocol.h"
#include "radio_profile.h"
#include "radio_runtime.h"
#include "radio_utils.h"
#include "ui_console.h"
#include "ui_console_support.h"
#include "ui_keypad.h"
#include "ui_keypad_state.h"
#include "ui_speech.h"
#include "debug_log.h"

static void printKeypadStatus(const String& line) {
  if ((bool)Serial) Serial.println(line);
}

static void printKeypadCommand(const String& line) {
  if ((bool)Serial) {
    Serial.print("CMD ");
    Serial.println(line);
  }
}

static void speakFrequencyWord() {
  if (!g_speechEnabled) return;
  speakToken("frequency");
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
  if (!currentStoredProfile().caps.getNr) return;
  if (!refreshLiveNr()) return;
  printKeypadStatus(live.nrOn ? "NR ON" : "NR OFF");
  speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, live.nrOn);
}

static void queryBank2Nb() {
  printKeypadCommand("BANK2 2 SHORT -> NB?");
  if (!currentStoredProfile().caps.getNb) return;
  if (!refreshLiveNb()) return;
  printKeypadStatus(live.nbOn ? "NB ON" : "NB OFF");
  speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, live.nbOn);
}

static void queryBank2Notch() {
  printKeypadCommand("BANK2 3 SHORT -> NOTCH?");
  if (!currentStoredProfile().caps.getNotch) return;
  if (!refreshLiveNotch()) return;
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
  g_keypadExecuting = true;
  processCommand(cmd);
  g_keypadExecuting = false;
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
    uint64_t hz = g_freqEntryIsMHz ? (uint64_t)g_freqEntryDigits.toInt() * 1000000ULL : (uint64_t)g_freqEntryDigits.toInt() * 1000ULL;
    if (g_freqEntryTargetVfo == 1) printKeypadCommand("ENTER -> VFOA FREQ");
    else if (g_freqEntryTargetVfo == 2) printKeypadCommand("ENTER -> VFOB FREQ");
    else printKeypadCommand("ENTER -> FREQ");
    bool ok = false;
    if (g_freqEntryTargetVfo == 1) ok = setVfoFrequency(true, hz);
    else if (g_freqEntryTargetVfo == 2) ok = setVfoFrequency(false, hz);
    else ok = applyFrequencyAndTrack(hz, true);
    if (ok) {
      if (g_freqEntryTargetVfo == 1) printKeypadStatus(String("VFOA: ") + hzToMHzString3(hz) + " MHz");
      else if (g_freqEntryTargetVfo == 2) printKeypadStatus(String("VFOB: ") + hzToMHzString3(hz) + " MHz");
      else printKeypadStatus(String("FREQ: ") + hzToMHzString3(hz) + " MHz");
      if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    } else {
      printKeypadStatus("FREQ -> failed");
    }
    g_freqEntryActive = false;
    g_freqEntryDigits = "";
    g_freqEntryIsMHz = false;
    g_freqEntryTargetVfo = 0;
    return;
  }

  if (g_modeStageActive) {
    if (g_modeStageTargetVfo == 1) printKeypadCommand("ENTER -> VFOA MODE");
    else if (g_modeStageTargetVfo == 2) printKeypadCommand("ENTER -> VFOB MODE");
    else printKeypadCommand("ENTER -> MODE");
    bool ok = false;
    if (g_modeStageTargetVfo == 1) ok = setVfoMode(true, g_modeStageMode, 1);
    else if (g_modeStageTargetVfo == 2) ok = setVfoMode(false, g_modeStageMode, 1);
    else ok = applyModeAndTrack(g_modeStageMode, 1);
    if (ok) {
      if (g_modeStageTargetVfo == 1) printKeypadStatus(String("VFOA MODE: ") + modeToString(g_modeStageMode));
      else if (g_modeStageTargetVfo == 2) printKeypadStatus(String("VFOB MODE: ") + modeToString(g_modeStageMode));
      else printKeypadStatus(String("MODE: ") + modeToString(g_modeStageMode));
      if (g_speechEnabled) playClipProgmem(voice_ok, voice_ok_len);
    } else {
      printKeypadStatus("MODE -> failed");
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
        printKeypadCommand("BANK1 3 SHORT -> LOCK?");
        keypadSendNow("LOCK?");
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
        return;
      case '5':
        if (currentProtocolType() == PROTO_CIV) {
          return;
        }
        return;
      case '6':
        if (currentProtocolType() == PROTO_CIV) {
          return;
        }
        return;
      case '7':
        if (currentProtocolType() == PROTO_CIV) {
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
        printKeypadCommand("BANK3 7 SHORT -> BSTACK? 1");
        keypadSendNow("BSTACK? 1");
        return;
      case '8':
        printKeypadCommand("BANK3 8 SHORT -> BSTACK? 2");
        keypadSendNow("BSTACK? 2");
        return;
      case '9':
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
        printKeypadCommand("BANK4 1 SHORT -> MONITOR?");
        keypadSendNow("MONITOR?");
        return;
      case '2':
        return;
      case '3':
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
      case '1':
        printKeypadCommand("BANK9 1 SHORT -> PROFILE NEXT");
        applyProfile(findAdjacentValidProfile(1));
        printKeypadStatus("PROFILE NEXT");
        speakCurrentProfile();
        return;
      case '2':
        printKeypadCommand("BANK9 2 SHORT -> PROFILE PREV");
        applyProfile(findAdjacentValidProfile(-1));
        printKeypadStatus("PROFILE PREV");
        speakCurrentProfile();
        return;
      case '3':
        return;
      case '4':
        printKeypadCommand("BANK9 4 SHORT -> TUNINGSPEECH?");
        printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
        speakTuningSpeechState();
        return;
      case '5':
        printKeypadCommand("BANK9 5 SHORT -> VOLUME?");
        printKeypadStatus(String("VOLUME ") + String((int)g_volumeLevel));
        if (g_speechEnabled) speakVolumeLevel(g_volumeLevel);
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
