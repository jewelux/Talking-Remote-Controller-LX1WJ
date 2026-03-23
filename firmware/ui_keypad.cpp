#include "ui_keypad.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_profile.h"
#include "radio_prefs.h"
#include "ui_keypad_actions.h"
#include "ui_keypad_state.h"
#include "radio_runtime.h"
#include "ui_speech.h"
#include "debug_log.h"

extern Keypad keypad;

bool g_keypadExecuting = false;
bool g_suppressModePrefixOnce = false;

static void speakBankPlease() {
  if (!g_speechEnabled) return;
  speakToken("bank");
  playSilenceMs(60);
  speakToken("please");
}

static void speakChoosePlease() {
  if (!g_speechEnabled) return;
  speakToken("choose");
  playSilenceMs(60);
  speakToken("please");
}

static void speakFrequencyWord() {
  if (!g_speechEnabled) return;
  speakToken("frequency");
}

void speakTuningSpeechState() {
  if (!g_speechEnabled) return;
  speakFrequencyWord();
  playSilenceMs(60);
  speakToken(g_tuningSpeakEnabled ? "on" : "off");
}

bool profileModeFromDigit(char digit, uint8_t& modeOut) {
  switch (digit) {
    case '1': modeOut = 0x00; break;
    case '2': modeOut = 0x01; break;
    case '3': modeOut = 0x03; break;
    case '4': modeOut = 0x02; break;
    case '5': modeOut = 0x05; break;
    case '6': modeOut = 0x11; break;
    case '7': modeOut = 0x04; break;
    case '8': modeOut = 0x07; break;
    case '9': modeOut = 0x08; break;
    default: return false;
  }
  String code;
  return profileModeCodeForInternal(currentStoredProfile(), modeOut, code);
}

void setTuningSpeechEnabled(bool enabled) {
  g_tuningSpeakEnabled = enabled;
  saveTuningSpeakToNvs(g_tuningSpeakEnabled);
  if (!g_tuningSpeakEnabled) {
    live.tuning = false;
    live.pendingHz = 0;
    live.tuningStartSpokenHz = 0;
  }
}

void speakBankNumber() {
  if (!g_speechEnabled) return;
  speakToken("bank");
  playSilenceMs(60);
  if (g_bank >= 1 && g_bank <= 9) playDigit(g_bank);
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

static void speakNrLevel(int level) {
  if (!g_speechEnabled) return;
  playClipProgmem(voice_noisereduction, voice_noisereduction_len);
  playSilenceMs(60);
  if (level <= 0) playClipProgmem(voice_off, voice_off_len);
  else if (level == 1) playClipProgmem(voice_one, voice_one_len);
  else playClipProgmem(voice_two, voice_two_len);
}

static void toggleBank2Nr() {
  if (!currentStoredProfile().caps.setNr) return;
  if (currentProtocolType() == PROTO_KENWOOD_ASCII && String(currentProfile().name).indexOf("TS-480") >= 0) {
    String line;
    int nextLevel = 1;
    if (transactAsciiCommand(currentStoredProfile().ascii.nrGet, line, currentStoredProfile().ascii.nrReplyPrefix, 800)) {
      int start = (int)strlen(currentStoredProfile().ascii.nrReplyPrefix);
      int semi = line.indexOf(';', start);
      if (semi < 0) semi = line.length();
      String value = line.substring(start, semi);
      value.trim();
      int currentLevel = value.toInt();
      if (currentLevel <= 0) nextLevel = 1;
      else if (currentLevel == 1) nextLevel = 2;
      else nextLevel = 0;
    }
    const char* cmd = (nextLevel == 0) ? "NR0;" : (nextLevel == 1) ? "NR1;" : "NR2;";
    if (asciiPacketSendCommand(cmd)) {
      live.nrOn = nextLevel != 0;
      live.nrValid = true;
      if (debugLogEnabled()) {
        DBG_PRINT("[KP] NR ");
        if (nextLevel == 0) DBG_PRINTLN("OFF");
        else DBG_PRINTLN(nextLevel);
      }
      speakNrLevel(nextLevel);
    }
    return;
  }
  if (!live.nrValid && !refreshLiveNr()) return;
  bool next = !live.nrOn;
  if (applyNrAndTrack(next)) {
    DBG_PRINTLN(next ? "[KP] NR ON" : "[KP] NR OFF");
    speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, next);
  }
}

static void toggleBank2Nb() {
  if (!currentStoredProfile().caps.setNb) return;
  if (!live.nbValid && !refreshLiveNb()) return;
  bool next = !live.nbOn;
  if (applyNbAndTrack(next)) {
    DBG_PRINTLN(next ? "[KP] NB ON" : "[KP] NB OFF");
    speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, next);
  }
}

static void toggleBank2Notch() {
  if (!currentStoredProfile().caps.setNotch) return;

  if (currentProtocolType() != PROTO_CIV) {
    if (!live.notchValid && !refreshLiveNotch()) return;
    bool next = !live.notchOn;
    if (applyNotchAndTrack(next)) {
      DBG_PRINTLN(next ? "[KP] NOTCH ON" : "[KP] NOTCH OFF");
      speakBinaryFeatureState(voice_notchfilter, voice_notchfilter_len, next);
    }
    return;
  }

  if (!live.notchValid && !refreshLiveNotch()) return;

  if (!live.notchOn) {
    if (!applyNotchAndTrack(true)) return;
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_NAR)) return;
    DBG_PRINTLN("[KP] NOTCH NAR");
    speakNotchCycleState(true, NOTCH_WIDTH_NAR);
    return;
  }

  if (!live.notchWidthValid && !refreshLiveNotchWidth()) return;

  if (live.notchWidth == NOTCH_WIDTH_NAR) {
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_MID)) return;
    DBG_PRINTLN("[KP] NOTCH MID");
    speakNotchCycleState(true, NOTCH_WIDTH_MID);
    return;
  }

  if (live.notchWidth == NOTCH_WIDTH_MID) {
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_WIDE)) return;
    DBG_PRINTLN("[KP] NOTCH WIDE");
    speakNotchCycleState(true, NOTCH_WIDTH_WIDE);
    return;
  }

  if (applyNotchAndTrack(false)) {
    DBG_PRINTLN("[KP] NOTCH OFF");
    speakNotchCycleState(false, NOTCH_WIDTH_UNKNOWN);
  }
}

static bool isCurrentYaesuFt8x7() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
}

static void sendYaesuBank2Action(const char* label, bool ok) {
  if (!ok) return;
  if (!debugLogEnabled()) return;
  DBG_PRINT("[KP] ");
  DBG_PRINTLN(label);
}

void keypadEvent(KeypadEvent k) {
  KeyState s = keypad.getState();
  if (s == PRESSED && g_audioPlaying) audioAbortNow();

  if (g_bankSelectActive) {
    if (k >= '1' && k <= '9' && s == RELEASED) {
      g_bankStage = (uint8_t)(k - '0');
      if (g_speechEnabled) playDigit(g_bankStage);
      return;
    }
    if (k == 'D' && s == RELEASED) { keypadEnter(); return; }
    if (k == '#' && s == RELEASED) { g_bankSelectActive = false; g_bankStage = 0; return; }
    return;
  }

  if (g_profileSelectActive) {
    if (k >= '1' && k <= '9' && s == RELEASED) {
      g_profileStage = (uint8_t)(k - '0');
      if (g_speechEnabled) playDigit(g_profileStage);
      return;
    }
    if (k == 'D' && s == RELEASED) { keypadEnter(); return; }
    if (k == '#' && s == RELEASED) { g_profileSelectActive = false; g_profileStage = 0; return; }
    return;
  }

  if (k == 'D' && s == RELEASED) { keypadEnter(); return; }
  if (k == '*' && s == HOLD) {
    g_bankSelectActive = true;
    g_bankStage = 0;
    g_starHoldConsumed = true;
    speakBankPlease();
    return;
  }
  if (k == '*' && s == RELEASED && g_starHoldConsumed) { g_starHoldConsumed = false; return; }
  if (k == '*' && s == RELEASED) { speakBankNumber(); return; }

  if (g_bank == 1 && k == 'A') {
    if (s == HOLD) {
      applyVolumeLevel(g_volumeLevel < 3 ? g_volumeLevel + 1 : 3);
      if (g_speechEnabled) speakVolumeLevel(g_volumeLevel);
      g_aHoldConsumed = true;
      return;
    }
    if (s == RELEASED && g_aHoldConsumed) { g_aHoldConsumed = false; return; }
    if (s == RELEASED) {
      applyVolumeLevel(g_volumeLevel > 0 ? g_volumeLevel - 1 : 0);
      if (g_speechEnabled) speakVolumeLevel(g_volumeLevel);
      return;
    }
  }

  if (g_bank == 1 && k == '9' && s == HOLD) {
    g_modeSetActive = true;
    g_nineHoldConsumed = true;
    if (g_speechEnabled) {
      speakToken("mode");
      playSilenceMs(80);
      speakToken("please");
    }
    return;
  }
  if (g_bank == 1 && k == '9' && s == RELEASED && g_nineHoldConsumed) { g_nineHoldConsumed = false; return; }

  if (g_bank == 2 && k == '1' && s == HOLD) {
    toggleBank2Nr();
    g_oneHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '1' && s == RELEASED && g_oneHoldConsumed) { g_oneHoldConsumed = false; return; }

  if (g_bank == 2 && k == '2' && s == HOLD) {
    toggleBank2Nb();
    g_twoHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '2' && s == RELEASED && g_twoHoldConsumed) { g_twoHoldConsumed = false; return; }

  if (g_bank == 2 && k == '3' && s == HOLD) {
    toggleBank2Notch();
    g_threeHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '3' && s == RELEASED && g_threeHoldConsumed) { g_threeHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '4' && s == HOLD) {
    sendYaesuBank2Action("VFO TOGGLE", yaesuCatToggleVfo());
    g_fourHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '4' && s == RELEASED && g_fourHoldConsumed) { g_fourHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '5' && s == HOLD) {
    sendYaesuBank2Action("VFO B", yaesuCatSelectVfoB());
    g_fiveHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '5' && s == RELEASED && g_fiveHoldConsumed) { g_fiveHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '6' && s == HOLD) {
    sendYaesuBank2Action("CLAR OFF", yaesuCatSetClarifier(false));
    g_sixHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '6' && s == RELEASED && g_sixHoldConsumed) { g_sixHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '7' && s == HOLD) {
    sendYaesuBank2Action("SPLIT OFF", yaesuCatSetSplit(false));
    g_sevenHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '7' && s == RELEASED && g_sevenHoldConsumed) { g_sevenHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '8' && s == HOLD) {
    sendYaesuBank2Action("PTT OFF", yaesuCatSetPtt(false));
    g_eightHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '8' && s == RELEASED && g_eightHoldConsumed) { g_eightHoldConsumed = false; return; }

  if (g_bank == 3 && k == 'A' && s == HOLD) {
    g_profileSelectActive = true;
    g_profileStage = 0;
    speakChoosePlease();
    return;
  }
  if (g_bank == 3 && k == 'A' && s == RELEASED) { speakCurrentProfile(); return; }
  if (g_bank == 3 && k == 'B' && s == HOLD) {
    speakTuningSpeechState();
    return;
  }
  if (g_bank == 3 && k == 'B' && s == RELEASED) {
    setTuningSpeechEnabled(!g_tuningSpeakEnabled);
    speakTuningSpeechState();
    return;
  }

  if (k == '0' && s == HOLD && g_bank == 1 && !g_freqEntryActive) {
    g_freqEntryActive = true;
    g_freqEntryIsMHz = false;
    g_freqEntryDigits = "";
    g_zeroHoldConsumed = true;
    if (g_speechEnabled) {
      speakFrequencyWord();
      playSilenceMs(80);
      speakToken("please");
    }
    return;
  }
  if (k == '0' && s == RELEASED && g_zeroHoldConsumed) { g_zeroHoldConsumed = false; return; }

  if (s == RELEASED) keypadHandleReleased((char)k);
}

void initKeypadUi() {
  keypad.setDebounceTime(KEYPAD_DEBOUNCE_MS);
  keypad.setHoldTime(KEYPAD_HOLD_MS);
  keypad.addEventListener(keypadEvent);
}

void pollKeypadUi() {
  (void)keypad.getKey();
}

uint8_t uiGetBank() {
  return g_bank;
}

void uiSetBank(uint8_t bank) {
  g_bank = bank;
}
