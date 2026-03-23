#include "ui_keypad_actions.h"

#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_profile.h"
#include "radio_runtime.h"
#include "radio_utils.h"
#include "ui_console.h"
#include "ui_keypad.h"
#include "ui_keypad_state.h"
#include "ui_speech.h"
#include "debug_log.h"

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
  playClipProgmem(voice_notchfilter, voice_notchfilter_len);
  playSilenceMs(60);
  if (!on) {
    playClipProgmem(voice_off, voice_off_len);
    return;
  }
  switch (width) {
    case NOTCH_WIDTH_NAR: playClipProgmem(voice_one, voice_one_len); break;
    case NOTCH_WIDTH_MID: playClipProgmem(voice_two, voice_two_len); break;
    case NOTCH_WIDTH_WIDE: playClipProgmem(voice_three, voice_three_len); break;
    default: playClipProgmem(voice_on, voice_on_len); break;
  }
}

static void queryBank2Nr() {
  if (!currentStoredProfile().caps.getNr) return;
  if (!refreshLiveNr()) return;
  DBG_PRINTLN(live.nrOn ? "NR ON" : "NR OFF");
  speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, live.nrOn);
}

static void queryBank2Nb() {
  if (!currentStoredProfile().caps.getNb) return;
  if (!refreshLiveNb()) return;
  DBG_PRINTLN(live.nbOn ? "NB ON" : "NB OFF");
  speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, live.nbOn);
}

static void queryBank2Notch() {
  if (!currentStoredProfile().caps.getNotch) return;
  if (!refreshLiveNotch()) return;
  if (!live.notchOn) {
    DBG_PRINTLN("NOTCH OFF");
    speakNotchCycleState(false, NOTCH_WIDTH_UNKNOWN);
    return;
  }
  if (live.notchWidthValid) {
    if (live.notchWidth == NOTCH_WIDTH_NAR) DBG_PRINTLN("NOTCH NAR");
    else if (live.notchWidth == NOTCH_WIDTH_MID) DBG_PRINTLN("NOTCH MID");
    else if (live.notchWidth == NOTCH_WIDTH_WIDE) DBG_PRINTLN("NOTCH WIDE");
    speakNotchCycleState(true, live.notchWidth);
    return;
  }
  DBG_PRINTLN("NOTCH ON");
  speakBinaryFeatureState(voice_notchfilter, voice_notchfilter_len, true);
}

static void speakKeypadCommandWord(const String& cmd) {
  if (!g_speechEnabled) return;
  if (cmd == "FREQ?") speakFrequencyWord();
  else if (cmd == "MODE?") speakToken("mode");
  else if (cmd == "SM?") speakToken("s");
  else if (cmd == "SWR?") speakToken("swr");
  else if (cmd == "PO?") speakToken("power");
  else if (cmd == "NOTCH?") speakToken("notchfilter");
}

static void sendOrStageBank1Command(const String& cmd, bool suppressModePrefix = false) {
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
  g_modeSetActive = false;
  g_modeStageActive = false;
  g_modeStageMode = 0xFF;
  g_modeStageKey = 0;
  g_profileSelectActive = false;
  g_profileStage = 0;
  g_volStageActive = false;
  g_oneHoldConsumed = false;
  g_twoHoldConsumed = false;
  g_threeHoldConsumed = false;
  g_fourHoldConsumed = false;
  g_fiveHoldConsumed = false;
  g_sixHoldConsumed = false;
  g_sevenHoldConsumed = false;
  g_eightHoldConsumed = false;
  DBG_PRINTLN("[KP] CLEAR");
  if (g_speechEnabled) playClipProgmem(voice_ok, voice_ok_len);
}

void keypadStageCommand(const String& cmd) {
  g_kpStagedCmd = cmd;
  g_kpHasStagedCmd = true;
  DBG_PRINT("[KP STAGE] ");
  DBG_PRINTLN(cmd);
  if (g_speechEnabled) {
    speakKeypadCommandWord(cmd);
    playSilenceMs(60);
    speakToken("ok");
  }
}

void keypadSendNow(const String& cmd) {
  DBG_PRINT("[KP SEND] ");
  DBG_PRINTLN(cmd);
  g_keypadExecuting = true;
  processCommand(cmd);
  g_keypadExecuting = false;
}

void keypadEnter() {
  if (g_bankSelectActive) {
    if (g_bankStage >= 1 && g_bankStage <= 9) {
      g_bank = g_bankStage;
      speakBankNumber();
    }
    g_bankSelectActive = false;
    g_bankStage = 0;
    return;
  }

  if (g_profileSelectActive) {
    if (g_profileStage >= 1 && g_profileStage <= 9) {
      applyProfile(g_profileStage);
      speakCurrentProfile();
    }
    g_profileSelectActive = false;
    g_profileStage = 0;
    return;
  }

  if (g_freqEntryActive) {
    uint64_t hz = g_freqEntryIsMHz ? (uint64_t)g_freqEntryDigits.toInt() * 1000000ULL : (uint64_t)g_freqEntryDigits.toInt() * 1000ULL;
    if (applyFrequencyAndTrack(hz, true)) {
      if (g_speechEnabled) speakDigitsAndPoint(hzToMHzString3(hz));
    }
    g_freqEntryActive = false;
    g_freqEntryDigits = "";
    g_freqEntryIsMHz = false;
    return;
  }

  if (g_modeStageActive) {
    if (applyModeAndTrack(g_modeStageMode, 1)) {
      g_suppressModePrefixOnce = true;
      speakMode(g_modeStageMode);
    }
    g_modeStageActive = false;
    g_modeStageMode = 0xFF;
    g_modeStageKey = 0;
    return;
  }

  if (g_volStageActive) {
    applyVolumeLevel(g_volStageLevel);
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
    g_modeStageMode = 0xFF;
    (void)profileModeFromDigit(k, g_modeStageMode);
    if (g_modeStageMode != 0xFF) {
      g_modeStageActive = true;
      g_modeStageKey = k;
      speakMode(g_modeStageMode);
    }
    g_modeSetActive = false;
    return;
  }

  if (g_freqEntryActive) {
    if (k >= '0' && k <= '9' && g_freqEntryDigits.length() < 9) {
      g_freqEntryDigits += k;
      if (g_speechEnabled) speakDigitsAndPoint(String(k));
    }
    return;
  }

  if (g_bank == 1) {
    switch (k) {
      case '1':
        setTuningSpeechEnabled(!g_tuningSpeakEnabled);
        speakTuningSpeechState();
        return;
      case '0':
        sendOrStageBank1Command("FREQ?");
        return;
      case '7':
        sendOrStageBank1Command("SM?");
        return;
      case '8':
        sendOrStageBank1Command("SWR?");
        return;
      case '9':
        sendOrStageBank1Command("MODE?", true);
        return;
      case '4':
        sendOrStageBank1Command("PO?");
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
        if (currentProtocolType() == PROTO_YAESU_FT8X7) keypadSendNow("YSTATUS?");
        return;
      case '5':
        if (currentProtocolType() == PROTO_YAESU_FT8X7) DBG_PRINTLN("[KP] VFO A");
        if (currentProtocolType() == PROTO_YAESU_FT8X7) yaesuCatSelectVfoA();
        return;
      case '6':
        if (currentProtocolType() == PROTO_YAESU_FT8X7) DBG_PRINTLN("[KP] CLAR ON");
        if (currentProtocolType() == PROTO_YAESU_FT8X7) yaesuCatSetClarifier(true);
        return;
      case '7':
        if (currentProtocolType() == PROTO_YAESU_FT8X7) DBG_PRINTLN("[KP] SPLIT ON");
        if (currentProtocolType() == PROTO_YAESU_FT8X7) yaesuCatSetSplit(true);
        return;
      case '8':
        if (currentProtocolType() == PROTO_YAESU_FT8X7) DBG_PRINTLN("[KP] PTT ON");
        if (currentProtocolType() == PROTO_YAESU_FT8X7) yaesuCatSetPtt(true);
        return;
      case '9':
        if (currentProtocolType() == PROTO_YAESU_FT8X7) keypadSendNow("YALL?");
        return;
      default: break;
    }
  } else if (g_bank == 3) {
    bool mapped = true;
    switch (k) {
      case '0': g_volStageLevel = 0; break;
      case '7': g_volStageLevel = 1; break;
      case '8': g_volStageLevel = 2; break;
      case '9': g_volStageLevel = 3; break;
      default: mapped = false; break;
    }
    if (mapped) {
      g_volStageActive = true;
      if (g_speechEnabled) speakVolumeLevel(g_volStageLevel);
      return;
    }
  }
}
