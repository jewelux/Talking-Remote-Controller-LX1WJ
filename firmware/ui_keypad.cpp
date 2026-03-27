#include "ui_keypad.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_profile.h"
#include "radio_prefs.h"
#include "ui_keypad_actions.h"
#include "ui_keypad_state.h"
#include "radio_runtime.h"
#include "radio_protocol.h"
#include "radio_utils.h"
#include "ui_speech.h"
#include "ui_console_support.h"
#include "debug_log.h"

extern Keypad keypad;

bool g_keypadExecuting = false;
bool g_suppressModePrefixOnce = false;
static constexpr uint32_t KEYPAD_DOUBLE_CLICK_MS = 350;

static void queryBank2Tuner();
static void toggleBank2Tuner();
static void triggerBank2Tune();
static void queryBank2NrLevel();
static void adjustBank2NrLevel(int deltaPercent);
static void queryBank2NbLevel();
static void adjustBank2NbLevel(int deltaPercent);
static void queryBank2PbtInner();
static void adjustBank2PbtInner(int delta);
static void queryBank2PbtOuter();
static void adjustBank2PbtOuter(int delta);
static void queryBank2FilterShape();
static void toggleBank2FilterShape();
static void queryBank2FilterWidth();
static void cycleBank2FilterWidth(int delta);
static void queryBank1RxTx();
static void queryBank1TxFrequency();
static void queryBank1Lock();
static void toggleBank1Lock();
static void beginBank1FrequencySet();
static void queryBank3Split();
static void toggleBank3Split();
static void queryBank3TxFrequency();
static void queryBank3VfoA();
static void selectBank3VfoA();
static void beginBank3VfoAFrequencySet();
static void queryBank3VfoB();
static void selectBank3VfoB();
static void beginBank3VfoBFrequencySet();
static void queryBank4VfoAMode(uint8_t sourceBank = 4);
static void beginBank4VfoAModeSet(uint8_t sourceBank = 4);
static void queryBank4VfoBMode(uint8_t sourceBank = 4);
static void beginBank4VfoBModeSet(uint8_t sourceBank = 4);
static void queryBank5Rit();
static void toggleBank5Rit();
static void adjustBank5Rit(int32_t deltaHz);
static void setBank5RitOffset(int32_t hz);
static void queryBank3BandStack(uint8_t reg);
static void recallBank3BandStack(uint8_t reg);
static void queryBank4Tuner();
static void toggleBank4Tuner();
static void triggerBank4Tune();
static void queryBank4Monitor();
static void toggleBank4Monitor();
static void queryBank4MonitorLevel();
static void adjustBank4MonitorLevel(int deltaPercent);
static void queryBank4Transceive();
static void toggleBank4Transceive();
static void queryBank9Profile();
static void beginBank9ProfileSelect();
static void selectNextProfile();
static void selectPrevProfile();
static void queryBank9Bank();
static void queryBank9TuningSpeech();
static void toggleBank9TuningSpeech();
static void queryBank9Volume();
static bool handleDeferredShortRelease(uint8_t bank, char key);
static bool handleDoubleClick(uint8_t bank, char key);
static bool shouldDelayShortRelease(uint8_t bank, char key);

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
  speakToken("tune");
  playSilenceMs(60);
  speakFrequencyWord();
  playSilenceMs(60);
  speakToken(g_tuningSpeakEnabled ? "on" : "off");
}

static void speakVfoFrequencyLabel(char which) {
  if (!g_speechEnabled) return;
  speakToken("vfo");
  playSilenceMs(60);
  if (which == 'A') speakToken("a");
  else if (which == 'B') speakToken("b");
  playSilenceMs(60);
  speakFrequencyWord();
}

static void speakRitLabel() {
  if (!g_speechEnabled) return;
  speakToken("rit");
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

static void speakNrLevel(int level) {
  if (!g_speechEnabled) return;
  playClipProgmem(voice_noisereduction, voice_noisereduction_len);
  playSilenceMs(60);
  if (level <= 0) playClipProgmem(voice_off, voice_off_len);
  else if (level == 1) playClipProgmem(voice_one, voice_one_len);
  else playClipProgmem(voice_two, voice_two_len);
}

static void speakFeatureValue(const uint8_t* featureData, size_t featureLen, uint8_t value) {
  if (!g_speechEnabled) return;
  playClipProgmem(featureData, featureLen);
  playSilenceMs(60);
  speakDigitsAndPoint(String((int)value));
}

static uint8_t levelRawToPercent(uint16_t raw) {
  if (raw >= 255) return 100;
  return (uint8_t)((raw * 100U + 127U) / 255U);
}

static uint16_t levelPercentToRaw(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return (uint16_t)((percent * 255 + 50) / 100);
}

static int pbtRawToOffset(uint16_t raw) {
  if (raw > 255) raw = 255;
  return (int)raw - 128;
}

static uint16_t pbtOffsetToRaw(int offset) {
  if (offset < -128) offset = -128;
  if (offset > 127) offset = 127;
  return (uint16_t)(offset + 128);
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

static void speakSimpleBinaryState(bool on) {
  if (!g_speechEnabled) return;
  playClipProgmem(on ? voice_on : voice_off, on ? voice_on_len : voice_off_len);
}

static void speakSignedStepValue(const String& label, int value) {
  if (!g_speechEnabled) return;
  speakToken(label);
  playSilenceMs(60);
  if (value < 0) {
    speakToken("minus");
    playSilenceMs(60);
    value = -value;
  }
  speakDigitsAndPoint(String(value));
  playSilenceMs(60);
  speakToken("step");
}

static void toggleBank2Nr() {
  printKeypadCommand("BANK2 1 LONG -> NR");
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
      printKeypadStatus(nextLevel == 0 ? "NR OFF" : (nextLevel == 1 ? "NR 1" : "NR 2"));
      speakNrLevel(nextLevel);
    }
    return;
  }
  if (!live.nrValid && !refreshLiveNr()) return;
  bool next = !live.nrOn;
  if (applyNrAndTrack(next)) {
    printKeypadStatus(next ? "NR ON" : "NR OFF");
    speakBinaryFeatureState(voice_noisereduction, voice_noisereduction_len, next);
  }
}

static void toggleBank2Nb() {
  printKeypadCommand("BANK2 2 LONG -> NB");
  if (!currentStoredProfile().caps.setNb) return;
  if (!live.nbValid && !refreshLiveNb()) return;
  bool next = !live.nbOn;
  if (applyNbAndTrack(next)) {
    printKeypadStatus(next ? "NB ON" : "NB OFF");
    speakBinaryFeatureState(voice_noiseblanker, voice_noiseblanker_len, next);
  }
}

static void toggleBank2Notch() {
  printKeypadCommand("BANK2 3 LONG -> NOTCH");
  if (!currentStoredProfile().caps.setNotch) return;

  if (currentProtocolType() != PROTO_CIV) {
    if (!live.notchValid && !refreshLiveNotch()) return;
    bool next = !live.notchOn;
    if (applyNotchAndTrack(next)) {
      printKeypadStatus(next ? "NOTCH ON" : "NOTCH OFF");
      speakTokenState("notch filter", next);
    }
    return;
  }

  if (!live.notchValid && !refreshLiveNotch()) return;

  if (!live.notchOn) {
    if (!applyNotchAndTrack(true)) return;
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_NAR)) return;
    printKeypadStatus("NOTCH NAR");
    speakNotchCycleState(true, NOTCH_WIDTH_NAR);
    return;
  }

  if (!live.notchWidthValid && !refreshLiveNotchWidth()) return;

  if (live.notchWidth == NOTCH_WIDTH_NAR) {
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_MID)) return;
    printKeypadStatus("NOTCH MID");
    speakNotchCycleState(true, NOTCH_WIDTH_MID);
    return;
  }

  if (live.notchWidth == NOTCH_WIDTH_MID) {
    if (!applyNotchWidthAndTrack(NOTCH_WIDTH_WIDE)) return;
    printKeypadStatus("NOTCH WIDE");
    speakNotchCycleState(true, NOTCH_WIDTH_WIDE);
    return;
  }

  if (applyNotchAndTrack(false)) {
    printKeypadStatus("NOTCH OFF");
    speakNotchCycleState(false, NOTCH_WIDTH_UNKNOWN);
  }
}

static void queryBank2Tuner() {
  printKeypadCommand("BANK2 0 SHORT -> TUNER?");
  bool on = false;
  if (!queryTuner(on, 800)) return;
  printKeypadStatus(on ? "TUNER ON" : "TUNER OFF");
  speakTokenState("tuner", on);
}

static void toggleBank2Tuner() {
  printKeypadCommand("BANK2 0 LONG -> TUNER");
  bool on = false;
  if (!queryTuner(on, 800)) return;
  if (!setTuner(!on)) return;
  printKeypadStatus(!on ? "TUNER ON" : "TUNER OFF");
  speakTokenState("tuner", !on);
}

static void triggerBank2Tune() {
  printKeypadCommand("BANK2 0 DOUBLE -> TUNE");
  if (!startTune()) return;
  printKeypadStatus("TUNE");
  if (g_speechEnabled) speakToken("tune");
}

static void speakFeaturePercent(const uint8_t* featureData, size_t featureLen, uint8_t percent) {
  if (!g_speechEnabled) return;
  playClipProgmem(featureData, featureLen);
  playSilenceMs(60);
  speakDigitsAndPoint(String((int)percent));
}

static void queryBank2NrLevel() {
  printKeypadCommand("BANK2 4 SHORT -> NRLEVEL?");
  uint16_t raw = 0;
  if (!queryNrLevel(raw, 800)) return;
  uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus(String("NRLEVEL ") + String((int)percent) + "%");
  speakFeatureValue(voice_noisereduction, voice_noisereduction_len, percent);
}

static void adjustBank2NrLevel(int deltaPercent) {
  printKeypadCommand(String("BANK2 4 ") + (deltaPercent > 0 ? "LONG" : "DOUBLE") + " -> NRLEVEL");
  uint16_t raw = 0;
  if (!queryNrLevel(raw, 800)) return;
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  const uint16_t targetRaw = levelPercentToRaw(percent);

  bool wrote = setNrLevel(targetRaw);
  if (!wrote) {
    bool nrOn = false;
    if (queryNr(nrOn, 800) && !nrOn) {
      (void)setNr(true);
      wrote = setNrLevel(targetRaw);
    }
  }

  uint16_t readBack = 0;
  if (!queryNrLevel(readBack, 800)) {
    printKeypadStatus("NRLEVEL -> failed");
    return;
  }

  const uint8_t readPercent = levelRawToPercent(readBack);
  printKeypadStatus(String("NRLEVEL ") + String((int)readPercent) + "%");
  if (!wrote && (bool)Serial) {
    Serial.println("WARN NRLEVEL write not confirmed; using readback value");
  }
  speakFeatureValue(voice_noisereduction, voice_noisereduction_len, readPercent);
}

static void queryBank2NbLevel() {
  printKeypadCommand("BANK2 5 SHORT -> NBLEVEL?");
  uint16_t raw = 0;
  if (!queryNbLevel(raw, 800)) return;
  uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus(String("NBLEVEL ") + String((int)percent) + "%");
  speakTokenPercent("noiseblanker", percent);
}

static void adjustBank2NbLevel(int deltaPercent) {
  printKeypadCommand(String("BANK2 5 ") + (deltaPercent > 0 ? "LONG" : "DOUBLE") + " -> NBLEVEL");
  uint16_t raw = 0;
  if (!queryNbLevel(raw, 800)) return;
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  if (!setNbLevel(levelPercentToRaw(percent))) return;
  printKeypadStatus(String("NBLEVEL ") + String(percent) + "%");
  speakTokenPercent("noiseblanker", (uint8_t)percent);
}

static void speakQueriedFrequencyHz(uint64_t hz) {
  if (!g_speechEnabled) return;
  speakFrequencyWord();
  playSilenceMs(60);
  speakDigitsAndPoint(hzToMHzString3(hz));
}

static void queryBank3Split() {
  printKeypadCommand("BANK3 0 SHORT -> SPLIT?");
  bool on = false;
  if (!querySplit(on, 800)) return;
  printKeypadStatus(on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", on);
}

static void toggleBank3Split() {
  printKeypadCommand("BANK3 0 LONG -> SPLIT");
  bool on = false;
  if (!querySplit(on, 800)) return;
  if (!setSplit(!on)) return;
  printKeypadStatus(!on ? "SPLIT ON" : "SPLIT OFF");
  speakTokenState("split", !on);
}

static void queryBank3TxFrequency() {
  printKeypadCommand("BANK3 0 DOUBLE -> TXFREQ?");
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, 800)) return;
  printKeypadStatus(String("TXFREQ: ") + hzToMHzString3(hz) + " MHz");
  speakQueriedFrequencyHz(hz);
}

static void queryBank3VfoA() {
  printKeypadCommand("BANK3 1 SHORT -> VFOA?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(true, hz, 800)) return;
  printKeypadStatus(String("VFOA: ") + hzToMHzString3(hz) + " MHz");
  if (g_speechEnabled) {
    speakVfoFrequencyLabel('A');
    playSilenceMs(60);
    speakDigitsAndPoint(hzToMHzString3(hz));
  }
}

static void selectBank3VfoA() {
  printKeypadCommand("BANK3 1 LONG -> VFO A");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  if (!selectVfoA()) return;
  queryBank3VfoA();
}

static void beginBank3VfoAFrequencySet() {
  printKeypadCommand("BANK3 1 DOUBLE -> VFOA FREQ");
  g_freqEntryActive = true;
  g_freqEntryIsMHz = false;
  g_freqEntryDigits = "";
  g_freqEntryTargetVfo = 1;
  if (g_speechEnabled) {
    speakFrequencyWord();
    playSilenceMs(80);
    speakToken("please");
  }
}

static void queryBank3VfoB() {
  printKeypadCommand("BANK3 2 SHORT -> VFOB?");
  uint64_t hz = 0;
  if (!queryVfoFrequency(false, hz, 800)) return;
  printKeypadStatus(String("VFOB: ") + hzToMHzString3(hz) + " MHz");
  if (g_speechEnabled) {
    speakVfoFrequencyLabel('B');
    playSilenceMs(60);
    speakDigitsAndPoint(hzToMHzString3(hz));
  }
}

static void selectBank3VfoB() {
  printKeypadCommand("BANK3 2 LONG -> VFO B");
  g_suppressFreqSpeakUntilMs = millis() + 1500;
  if (!selectVfoB()) return;
  queryBank3VfoB();
}

static void beginBank3VfoBFrequencySet() {
  printKeypadCommand("BANK3 2 DOUBLE -> VFOB FREQ");
  g_freqEntryActive = true;
  g_freqEntryIsMHz = false;
  g_freqEntryDigits = "";
  g_freqEntryTargetVfo = 2;
  if (g_speechEnabled) {
    speakFrequencyWord();
    playSilenceMs(80);
    speakToken("please");
  }
}

static void queryBank4VfoAMode(uint8_t sourceBank) {
  printKeypadCommand(String("BANK") + String((int)sourceBank) + (sourceBank == 3 ? " 4" : " 1") + " SHORT -> VFOA MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(true, mode, filter, 800)) return;
  printKeypadStatus(String("VFOA MODE: ") + modeToString(mode));
  g_suppressModePrefixOnce = true;
  speakMode(mode);
}

static void beginBank4VfoAModeSet(uint8_t sourceBank) {
  printKeypadCommand(String("BANK") + String((int)sourceBank) + (sourceBank == 3 ? " 4" : " 1") + " LONG -> VFOA MODE");
  g_modeSetActive = true;
  g_modeStageTargetVfo = 1;
  g_oneHoldConsumed = true;
  if (g_speechEnabled) {
    speakToken("mode");
    playSilenceMs(80);
    speakToken("please");
  }
}

static void queryBank4VfoBMode(uint8_t sourceBank) {
  printKeypadCommand(String("BANK") + String((int)sourceBank) + (sourceBank == 3 ? " 5" : " 2") + " SHORT -> VFOB MODE?");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!queryVfoMode(false, mode, filter, 800)) return;
  printKeypadStatus(String("VFOB MODE: ") + modeToString(mode));
  g_suppressModePrefixOnce = true;
  speakMode(mode);
}

static void beginBank4VfoBModeSet(uint8_t sourceBank) {
  printKeypadCommand(String("BANK") + String((int)sourceBank) + (sourceBank == 3 ? " 5" : " 2") + " LONG -> VFOB MODE");
  g_modeSetActive = true;
  g_modeStageTargetVfo = 2;
  g_twoHoldConsumed = true;
  if (g_speechEnabled) {
    speakToken("mode");
    playSilenceMs(80);
    speakToken("please");
  }
}

static void queryBank3RxTx() {
  printKeypadCommand("BANK3 6 SHORT -> RXTX?");
  bool tx = false;
  if (!queryRxTxStatus(tx, 800)) return;
  printKeypadStatus(tx ? "TX" : "RX");
  if (!g_speechEnabled) return;
  speakToken("transceiver");
  playSilenceMs(60);
  speakSimpleBinaryState(tx);
}

static bool ensureActiveVfoKnownForKeypad() {
  if (live.activeVfoKnown) return true;
  uint64_t dummy = 0;
  return queryVfoFrequency(true, dummy, 800);
}

static bool queryCurrentFilterSlotForKeypad(uint8_t& filterOut) {
  if (!ensureActiveVfoKnownForKeypad()) return false;
  uint8_t mode = 0xFF;
  return queryVfoMode(live.activeVfoA, mode, filterOut, 800);
}

static void queryBank1RxTx() {
  printKeypadCommand("BANK1 1 SHORT -> RXTX?");
  bool tx = false;
  if (!queryRxTxStatus(tx, 800)) return;
  printKeypadStatus(tx ? "TX" : "RX");
  if (!g_speechEnabled) return;
  speakToken("transceiver");
  playSilenceMs(60);
  speakSimpleBinaryState(tx);
}

static void queryBank1TxFrequency() {
  printKeypadCommand("BANK1 2 SHORT -> TXFREQ?");
  uint64_t hz = 0;
  if (!queryTxFrequency(hz, 800)) return;
  printKeypadStatus(String("TXFREQ: ") + hzToMHzString3(hz) + " MHz");
  speakQueriedFrequencyHz(hz);
}

static void queryBank1Lock() {
  printKeypadCommand("BANK1 3 SHORT -> LOCK?");
  bool on = false;
  if (!queryDialLock(on, 800)) return;
  printKeypadStatus(on ? "LOCK ON" : "LOCK OFF");
  speakTokenState("lock", on);
}

static void beginBank1FrequencySet() {
  printKeypadCommand("BANK1 0 LONG -> FREQ");
  g_freqEntryActive = true;
  g_freqEntryIsMHz = false;
  g_freqEntryDigits = "";
  g_freqEntryTargetVfo = 0;
  if (g_speechEnabled) {
    speakFrequencyWord();
    playSilenceMs(80);
    speakToken("please");
  }
}

static void toggleBank1Lock() {
  printKeypadCommand("BANK1 3 LONG -> LOCK");
  bool on = false;
  if (!queryDialLock(on, 800)) return;
  if (!setDialLock(!on)) return;
  printKeypadStatus(!on ? "LOCK ON" : "LOCK OFF");
  speakTokenState("lock", !on);
}

static void queryBank2PbtInner() {
  printKeypadCommand("BANK2 6 SHORT -> PBT1?");
  uint16_t raw = 0;
  if (!queryPbtInner(raw, 800)) return;
  printKeypadStatus(String("PBT1 ") + String(pbtRawToOffset(raw)) + " step");
  speakSignedStepValue("pbt", pbtRawToOffset(raw));
}

static void adjustBank2PbtInner(int delta) {
  printKeypadCommand(String("BANK2 6 ") + (delta > 0 ? "LONG" : "DOUBLE") + " -> PBT1");
  uint16_t raw = 0;
  if (!queryPbtInner(raw, 800)) return;
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtInner(pbtOffsetToRaw(next))) return;
  queryBank2PbtInner();
}

static void queryBank2PbtOuter() {
  printKeypadCommand("BANK2 7 SHORT -> PBT2?");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) return;
  printKeypadStatus(String("PBT2 ") + String(pbtRawToOffset(raw)) + " step");
  speakSignedStepValue("pbt", pbtRawToOffset(raw));
}

static void adjustBank2PbtOuter(int delta) {
  printKeypadCommand(String("BANK2 7 ") + (delta > 0 ? "LONG" : "DOUBLE") + " -> PBT2");
  uint16_t raw = 0;
  if (!queryPbtOuter(raw, 800)) return;
  const int next = pbtRawToOffset(raw) + delta;
  if (!setPbtOuter(pbtOffsetToRaw(next))) return;
  queryBank2PbtOuter();
}

static void queryBank2FilterShape() {
  printKeypadCommand("BANK2 8 SHORT -> FILSHAPE?");
  bool soft = false;
  if (!queryFilterShape(soft, 800)) return;
  printKeypadStatus(soft ? "FILSHAPE SOFT" : "FILSHAPE SHARP");
  if (g_speechEnabled) {
    speakToken("filtershape");
    playSilenceMs(60);
    speakToken(soft ? "soft" : "sharp");
  }
}

static void toggleBank2FilterShape() {
  printKeypadCommand("BANK2 8 LONG -> FILSHAPE");
  bool soft = false;
  if (!queryFilterShape(soft, 800)) return;
  if (!setFilterShape(!soft)) return;
  printKeypadStatus(!soft ? "FILSHAPE SOFT" : "FILSHAPE SHARP");
  if (g_speechEnabled) {
    speakToken("filtershape");
    playSilenceMs(60);
    speakToken(!soft ? "soft" : "sharp");
  }
}

static void queryBank2FilterWidth() {
  printKeypadCommand("BANK2 9 SHORT -> FILWIDTH?");
  uint8_t filter = 0xFF;
  if (!queryCurrentFilterSlotForKeypad(filter)) return;
  printKeypadStatus(String("FILWIDTH ") + String((int)filter));
  if (g_speechEnabled) {
    speakToken("filterwidth");
    playSilenceMs(60);
    playDigit(filter);
  }
}

static void cycleBank2FilterWidth(int delta) {
  printKeypadCommand(String("BANK2 9 ") + (delta > 0 ? "LONG" : "DOUBLE") + " -> FILWIDTH");
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
  if (!ensureActiveVfoKnownForKeypad()) return;
  if (!queryVfoMode(live.activeVfoA, mode, filter, 800)) return;
  int next = (int)filter + delta;
  if (next < 1) next = 3;
  if (next > 3) next = 1;
  if (!setMode(mode, (uint8_t)next)) return;
  printKeypadStatus(String("FILWIDTH ") + String(next));
  if (g_speechEnabled) {
    speakToken("filterwidth");
    playSilenceMs(60);
    playDigit((uint8_t)next);
  }
}

static void queryBank3BandStack(uint8_t reg) {
  printKeypadCommand(String("BANK3 ") + String(reg + 6) + " SHORT -> BSTACK? " + String(reg));
  keypadSendNow(String("BSTACK? ") + String(reg));
}

static void recallBank3BandStack(uint8_t reg) {
  printKeypadCommand(String("BANK3 ") + String(reg + 6) + " LONG -> BSTACK " + String(reg));
  keypadSendNow(String("BSTACK ") + String(reg));
}

static void queryBank4Tuner() {
  printKeypadCommand("BANK4 0 SHORT -> TUNER?");
  bool on = false;
  if (!queryTuner(on, 800)) return;
  printKeypadStatus(on ? "TUNER ON" : "TUNER OFF");
  speakTokenState("tuner", on);
}

static void toggleBank4Tuner() {
  printKeypadCommand("BANK4 0 LONG -> TUNER");
  bool on = false;
  if (!queryTuner(on, 800)) return;
  if (!setTuner(!on)) return;
  printKeypadStatus(!on ? "TUNER ON" : "TUNER OFF");
  speakTokenState("tuner", !on);
}

static void triggerBank4Tune() {
  printKeypadCommand("BANK4 0 DOUBLE -> TUNE");
  if (!startTune()) return;
  printKeypadStatus("TUNE");
  if (g_speechEnabled) speakToken("tune");
}

static void queryBank4Monitor() {
  printKeypadCommand("BANK4 1 SHORT -> MONITOR?");
  bool on = false;
  if (!queryMonitorEnabled(on, 800)) return;
  printKeypadStatus(on ? "MONITOR ON" : "MONITOR OFF");
  speakTokenState("monitor", on);
}

static void toggleBank4Monitor() {
  printKeypadCommand("BANK4 1 LONG -> MONITOR");
  bool on = false;
  if (!queryMonitorEnabled(on, 800)) return;
  if (!setMonitorEnabled(!on)) return;
  printKeypadStatus(!on ? "MONITOR ON" : "MONITOR OFF");
  speakTokenState("monitor", !on);
}

static void queryBank4MonitorLevel() {
  printKeypadCommand("BANK4 2 SHORT -> MONLEVEL?");
  uint16_t raw = 0;
  if (!queryMonitorLevel(raw, 800)) return;
  const uint8_t percent = levelRawToPercent(raw);
  printKeypadStatus(String("MONLEVEL ") + String((int)percent) + "%");
  speakFeatureValue(voice_monitor, voice_monitor_len, percent);
}

static void adjustBank4MonitorLevel(int deltaPercent) {
  printKeypadCommand(String("BANK4 2 ") + (deltaPercent > 0 ? "LONG" : "DOUBLE") + " -> MONLEVEL");
  uint16_t raw = 0;
  if (!queryMonitorLevel(raw, 800)) return;
  int percent = (int)levelRawToPercent(raw) + deltaPercent;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  if (!setMonitorLevel(levelPercentToRaw(percent))) return;
  queryBank4MonitorLevel();
}

static void queryBank4Transceive() {
  printKeypadCommand("BANK4 3 SHORT -> TRANSCEIVE?");
  bool on = false;
  if (!queryTransceiveEnabled(on, 800)) return;
  printKeypadStatus(on ? "TRANSCEIVE ON" : "TRANSCEIVE OFF");
  speakTokenState("transceiver", on);
}

static void toggleBank4Transceive() {
  printKeypadCommand("BANK4 3 LONG -> TRANSCEIVE");
  bool on = false;
  if (!queryTransceiveEnabled(on, 800)) return;
  if (!setTransceiveEnabled(!on)) return;
  printKeypadStatus(!on ? "TRANSCEIVE ON" : "TRANSCEIVE OFF");
  speakTokenState("transceiver", !on);
}

static void queryBank9Profile() {
  printKeypadCommand("BANK9 0 SHORT -> PROFILE?");
  printKeypadStatus("PROFILE CURRENT");
  speakCurrentProfile();
}

static void beginBank9ProfileSelect() {
  g_profileSelectActive = true;
  g_profileStage = 0;
  printKeypadCommand("BANK9 0 LONG -> PROFILE SELECT");
  printKeypadStatus("CHOOSE PLEASE");
  speakChoosePlease();
}

static void selectNextProfile() {
  printKeypadCommand("BANK9 1 SHORT -> PROFILE NEXT");
  uint8_t next = findAdjacentValidProfile(1);
  applyProfile(next);
  printKeypadStatus(String("PROFILE ") + String((int)next));
  speakCurrentProfile();
}

static void selectPrevProfile() {
  printKeypadCommand("BANK9 2 SHORT -> PROFILE PREV");
  uint8_t prev = findAdjacentValidProfile(-1);
  applyProfile(prev);
  printKeypadStatus(String("PROFILE ") + String((int)prev));
  speakCurrentProfile();
}

static void queryBank9Bank() {
  printKeypadCommand("BANK9 3 SHORT -> BANK?");
  printKeypadStatus(String("BANK ") + String((int)g_bank));
  speakBankNumber();
}

static void queryBank9TuningSpeech() {
  printKeypadCommand("BANK9 4 SHORT -> TUNINGSPEECH?");
  printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
  speakTuningSpeechState();
}

static void toggleBank9TuningSpeech() {
  printKeypadCommand("BANK9 4 LONG -> TUNINGSPEECH");
  setTuningSpeechEnabled(!g_tuningSpeakEnabled);
  printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
  speakTuningSpeechState();
}

static void queryBank9Volume() {
  printKeypadCommand("BANK9 5 SHORT -> VOLUME?");
  printKeypadStatus(String("VOLUME ") + String((int)g_volumeLevel));
  if (g_speechEnabled) speakVolumeLevel(g_volumeLevel);
}

static void speakRitOffsetValue(int32_t hz) {
  if (!g_speechEnabled) return;
  speakRitLabel();
  playSilenceMs(60);
  if (hz > 0) {
    speakToken("plus");
    playSilenceMs(60);
  }
  if (hz < 0) {
    speakToken("minus");
    playSilenceMs(60);
  }
  speakDigitsAndPoint(String(hz < 0 ? -hz : hz));
  playSilenceMs(60);
  speakToken("hertz");
}

static void queryBank5Rit() {
  printKeypadCommand("BANK5 0 SHORT -> RIT?");
  bool on = false;
  int32_t offset = 0;
  if (!queryRitEnabled(on, 800)) return;
  if (!queryRitOffsetHz(offset, 800)) offset = 0;
  printKeypadStatus(String(on ? "RIT ON " : "RIT OFF ") + String(offset) + " Hz");
  if (!g_speechEnabled) return;
  speakRitLabel();
  playSilenceMs(60);
  speakToken(on ? "on" : "off");
  if (offset != 0) {
    playSilenceMs(60);
    speakRitOffsetValue(offset);
  }
}

static void toggleBank5Rit() {
  printKeypadCommand("BANK5 0 LONG -> RIT");
  bool on = false;
  if (!queryRitEnabled(on, 800)) return;
  if (!setRitEnabled(!on)) return;
  printKeypadStatus(!on ? "RIT ON" : "RIT OFF");
  speakRitLabel();
  playSilenceMs(60);
  speakToken(!on ? "on" : "off");
}

static void setBank5RitOffset(int32_t hz) {
  printKeypadCommand(hz == 0 ? "BANK5 0 DOUBLE / 3 SHORT -> RIT 0" : String("BANK5 RIT -> ") + String(hz) + " Hz");
  if (!setRitOffsetHz(hz)) return;
  printKeypadStatus(String("RIT ") + String(hz) + " Hz");
  speakRitOffsetValue(hz);
}

static void adjustBank5Rit(int32_t deltaHz) {
  printKeypadCommand(String("BANK5 STEP -> ") + (deltaHz >= 0 ? "+" : "") + String(deltaHz) + " Hz");
  int32_t offset = 0;
  if (!queryRitOffsetHz(offset, 800)) return;
  int32_t next = offset + deltaHz;
  if (next < -9999) next = -9999;
  if (next > 9999) next = 9999;
  setBank5RitOffset(next);
}

static bool isCurrentYaesuFt8x7() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
}

static void sendYaesuBank2Action(const char* label, bool ok) {
  if (!ok) return;
  printKeypadStatus(label);
}

static bool handleDeferredShortRelease(uint8_t bank, char key) {
  if (bank == 1) {
    switch (key) {
      case '1': queryBank1RxTx(); return true;
      case '2': queryBank1TxFrequency(); return true;
      default: break;
    }
  }
  if (bank == 2) {
    switch (key) {
      case '1': keypadBank2QueryNr(); return true;
      case '2': keypadBank2QueryNb(); return true;
      case '3': keypadBank2QueryNotch(); return true;
      case '4': queryBank2NrLevel(); return true;
      case '5': queryBank2NbLevel(); return true;
      case '6': queryBank2PbtInner(); return true;
      case '7': queryBank2PbtOuter(); return true;
      case '9': queryBank2FilterWidth(); return true;
      default: break;
    }
  }
  if (bank == 3) {
    switch (key) {
      case '0': queryBank3Split(); return true;
      case '1': queryBank3VfoA(); return true;
      case '2': queryBank3VfoB(); return true;
      case '4': queryBank4VfoAMode(); return true;
      case '5': queryBank4VfoBMode(); return true;
      case '6': queryBank3RxTx(); return true;
      case '7': queryBank3BandStack(1); return true;
      case '8': queryBank3BandStack(2); return true;
      case '9': queryBank3BandStack(3); return true;
      default: break;
    }
  }
  if (bank == 4) {
    switch (key) {
      case '0': queryBank4Tuner(); return true;
      case '1': queryBank4Monitor(); return true;
      case '2': queryBank4MonitorLevel(); return true;
      case '3': queryBank4Transceive(); return true;
      default: break;
    }
  }
  if (bank == 5) {
    switch (key) {
      case '0': queryBank5Rit(); return true;
      case '1': adjustBank5Rit(-10); return true;
      case '2': adjustBank5Rit(10); return true;
      case '3': setBank5RitOffset(0); return true;
      default: break;
    }
  }
  if (bank == 9) {
    switch (key) {
      case '0': queryBank9Profile(); return true;
      default: break;
    }
  }
  return false;
}

static bool handleDoubleClick(uint8_t bank, char key) {
  if (bank == 2 && key == '6') {
    adjustBank2PbtInner(-10);
    return true;
  }
  if (bank == 2 && key == '7') {
    adjustBank2PbtOuter(-10);
    return true;
  }
  if (bank == 2 && key == '0') {
    triggerBank2Tune();
    return true;
  }
  if (bank == 2 && key == '4') {
    adjustBank2NrLevel(-10);
    return true;
  }
  if (bank == 2 && key == '5') {
    adjustBank2NbLevel(-10);
    return true;
  }
  if (bank == 3 && key == '0') {
    queryBank3TxFrequency();
    return true;
  }
  if (bank == 3 && key == '1') {
    beginBank3VfoAFrequencySet();
    return true;
  }
  if (bank == 3 && key == '2') {
    beginBank3VfoBFrequencySet();
    return true;
  }
  if (bank == 4 && key == '0') {
    triggerBank4Tune();
    return true;
  }
  if (bank == 4 && key == '2') {
    adjustBank4MonitorLevel(-10);
    return true;
  }
  if (bank == 5 && key == '0') {
    setBank5RitOffset(0);
    return true;
  }
  if (bank == 2 && key == '9') {
    cycleBank2FilterWidth(-1);
    return true;
  }
  return false;
}

static bool shouldDelayShortRelease(uint8_t bank, char key) {
  if (g_freqEntryActive || g_modeSetActive) return false;
  return (bank == 1 && (key == '1' || key == '2')) ||
         (bank == 2 && (key == '0' || (currentProtocolType() == PROTO_CIV && (key == '4' || key == '5' || key == '6' || key == '7' || key == '9')))) ||
         (bank == 3 && (key == '0' || key == '1' || key == '2')) ||
         (bank == 4 && (key == '0' || key == '2')) ||
         (bank == 5 && key == '0');
}

void keypadEvent(KeypadEvent k) {
  KeyState s = keypad.getState();
  if (s == PRESSED && g_audioPlaying) audioAbortNow();

  if (g_bankSelectActive) {
    if (k >= '1' && k <= '9' && s == RELEASED) {
      g_bankStage = (uint8_t)(k - '0');
      printKeypadCommand(String("BANK SELECT DIGIT -> ") + String(k));
      printKeypadStatus(String("BANK ") + String((int)g_bankStage));
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
      printKeypadCommand(String("PROFILE DIGIT -> ") + String(k));
      printKeypadStatus(String("PROFILE ") + String((int)g_profileStage));
      if (g_speechEnabled) speakProfileIdentityFromSlot(g_profileStage, false);
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
    printKeypadCommand("* HOLD -> BANK SELECT");
    printKeypadStatus("BANK PLEASE");
    speakBankPlease();
    return;
  }
  if (k == '*' && s == RELEASED && g_starHoldConsumed) { g_starHoldConsumed = false; return; }
  if (k == '*' && s == RELEASED) {
    printKeypadCommand("* SHORT -> BANK?");
    printKeypadStatus(String("BANK ") + String((int)g_bank));
    speakBankNumber();
    return;
  }

  if (g_bank == 1 && k == '9' && s == HOLD) {
    g_modeSetActive = true;
    g_nineHoldConsumed = true;
    printKeypadCommand("BANK1 9 LONG -> MODE");
    printKeypadStatus("MODE PLEASE");
    if (g_speechEnabled) {
      speakToken("mode");
      playSilenceMs(80);
      speakToken("please");
    }
    return;
  }
  if (g_bank == 1 && k == '9' && s == RELEASED && g_nineHoldConsumed) { g_nineHoldConsumed = false; return; }

  if (g_bank == 1 && k == '3' && s == HOLD) {
    toggleBank1Lock();
    g_threeHoldConsumed = true;
    return;
  }
  if (g_bank == 1 && k == '3' && s == RELEASED && g_threeHoldConsumed) { g_threeHoldConsumed = false; return; }

  if (g_bank == 1 && k == '0' && s == HOLD) {
    beginBank1FrequencySet();
    g_zeroHoldConsumed = true;
    return;
  }
  if (g_bank == 1 && k == '0' && s == RELEASED && g_zeroHoldConsumed) { g_zeroHoldConsumed = false; return; }

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

  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '4' && s == HOLD) {
    adjustBank2NrLevel(10);
    g_fourHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '4' && s == RELEASED && g_fourHoldConsumed) { g_fourHoldConsumed = false; return; }

  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '5' && s == HOLD) {
    adjustBank2NbLevel(10);
    g_fiveHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '5' && s == RELEASED && g_fiveHoldConsumed) { g_fiveHoldConsumed = false; return; }

  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '6' && s == HOLD) {
    adjustBank2PbtInner(10);
    g_sixHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '6' && s == RELEASED && g_sixHoldConsumed) { g_sixHoldConsumed = false; return; }

  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '7' && s == HOLD) {
    adjustBank2PbtOuter(10);
    g_sevenHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '7' && s == RELEASED && g_sevenHoldConsumed) { g_sevenHoldConsumed = false; return; }

  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '8' && s == HOLD) {
    toggleBank2FilterShape();
    g_eightHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '8' && s == RELEASED && g_eightHoldConsumed) { g_eightHoldConsumed = false; return; }

  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '9' && s == HOLD) {
    cycleBank2FilterWidth(1);
    g_nineHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 2 && currentProtocolType() == PROTO_CIV && k == '9' && s == RELEASED && g_nineHoldConsumed) { g_nineHoldConsumed = false; return; }

  if (g_bank == 3 && k == '0' && s == HOLD) {
    toggleBank3Split();
    g_zeroHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 3 && k == '0' && s == RELEASED && g_zeroHoldConsumed) { g_zeroHoldConsumed = false; return; }

  if (g_bank == 3 && !g_modeSetActive && !g_freqEntryActive && k == '1' && s == HOLD) {
    selectBank3VfoA();
    g_oneHoldConsumed = true;
    return;
  }
  if (g_bank == 3 && k == '1' && s == RELEASED && g_oneHoldConsumed) { g_oneHoldConsumed = false; return; }

  if (g_bank == 3 && !g_modeSetActive && !g_freqEntryActive && k == '2' && s == HOLD) {
    selectBank3VfoB();
    g_twoHoldConsumed = true;
    return;
  }
  if (g_bank == 3 && k == '2' && s == RELEASED && g_twoHoldConsumed) { g_twoHoldConsumed = false; return; }

  if (g_bank == 3 && !g_modeSetActive && !g_freqEntryActive && k == '4' && s == HOLD) {
    beginBank4VfoAModeSet(3);
    g_fourHoldConsumed = true;
    return;
  }
  if (g_bank == 3 && k == '4' && s == RELEASED && g_fourHoldConsumed) { g_fourHoldConsumed = false; return; }
  if (g_bank == 3 && !g_modeSetActive && !g_freqEntryActive && k == '4' && s == RELEASED) {
    queryBank4VfoAMode(3);
    return;
  }

  if (g_bank == 3 && !g_modeSetActive && !g_freqEntryActive && k == '5' && s == HOLD) {
    beginBank4VfoBModeSet(3);
    g_fiveHoldConsumed = true;
    return;
  }
  if (g_bank == 3 && k == '5' && s == RELEASED && g_fiveHoldConsumed) { g_fiveHoldConsumed = false; return; }
  if (g_bank == 3 && !g_modeSetActive && !g_freqEntryActive && k == '5' && s == RELEASED) {
    queryBank4VfoBMode(3);
    return;
  }

  if (g_bank == 3 && !g_freqEntryActive && k == '6' && s == RELEASED) {
    queryBank3RxTx();
    return;
  }

  if (g_bank == 3 && k == '7' && s == HOLD) {
    recallBank3BandStack(1);
    g_sevenHoldConsumed = true;
    return;
  }
  if (g_bank == 3 && k == '7' && s == RELEASED && g_sevenHoldConsumed) { g_sevenHoldConsumed = false; return; }
  if (g_bank == 3 && k == '7' && s == RELEASED) {
    queryBank3BandStack(1);
    return;
  }

  if (g_bank == 3 && k == '8' && s == HOLD) {
    recallBank3BandStack(2);
    g_eightHoldConsumed = true;
    return;
  }
  if (g_bank == 3 && k == '8' && s == RELEASED && g_eightHoldConsumed) { g_eightHoldConsumed = false; return; }
  if (g_bank == 3 && k == '8' && s == RELEASED) {
    queryBank3BandStack(2);
    return;
  }

  if (g_bank == 3 && k == '9' && s == HOLD) {
    recallBank3BandStack(3);
    g_nineHoldConsumed = true;
    return;
  }
  if (g_bank == 3 && k == '9' && s == RELEASED && g_nineHoldConsumed) { g_nineHoldConsumed = false; return; }
  if (g_bank == 3 && k == '9' && s == RELEASED) {
    queryBank3BandStack(3);
    return;
  }

  if (g_bank == 4 && k == '0' && s == HOLD) {
    toggleBank4Tuner();
    g_zeroHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 4 && k == '0' && s == RELEASED && g_zeroHoldConsumed) { g_zeroHoldConsumed = false; return; }

  if (g_bank == 4 && k == '1' && s == HOLD) {
    toggleBank4Monitor();
    g_oneHoldConsumed = true;
    return;
  }
  if (g_bank == 4 && k == '1' && s == RELEASED && g_oneHoldConsumed) { g_oneHoldConsumed = false; return; }

  if (g_bank == 4 && k == '2' && s == HOLD) {
    adjustBank4MonitorLevel(10);
    g_twoHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 4 && k == '2' && s == RELEASED && g_twoHoldConsumed) { g_twoHoldConsumed = false; return; }

  if (g_bank == 4 && k == '3' && s == HOLD) {
    toggleBank4Transceive();
    g_threeHoldConsumed = true;
    return;
  }
  if (g_bank == 4 && k == '3' && s == RELEASED && g_threeHoldConsumed) { g_threeHoldConsumed = false; return; }

  if (g_bank == 5 && k == '0' && s == HOLD) {
    toggleBank5Rit();
    g_zeroHoldConsumed = true;
    g_pendingClickActive = false;
    return;
  }
  if (g_bank == 5 && k == '0' && s == RELEASED && g_zeroHoldConsumed) { g_zeroHoldConsumed = false; return; }

  if (g_bank == 5 && k == '1' && s == HOLD) {
    adjustBank5Rit(-100);
    g_oneHoldConsumed = true;
    return;
  }
  if (g_bank == 5 && k == '1' && s == RELEASED && g_oneHoldConsumed) { g_oneHoldConsumed = false; return; }
  if (g_bank == 5 && k == '1' && s == RELEASED) {
    adjustBank5Rit(-10);
    return;
  }

  if (g_bank == 5 && k == '2' && s == HOLD) {
    adjustBank5Rit(100);
    g_twoHoldConsumed = true;
    return;
  }
  if (g_bank == 5 && k == '2' && s == RELEASED && g_twoHoldConsumed) { g_twoHoldConsumed = false; return; }
  if (g_bank == 5 && k == '2' && s == RELEASED) {
    adjustBank5Rit(10);
    return;
  }

  if (g_bank == 5 && k == '4' && s == HOLD) {
    adjustBank5Rit(-500);
    g_fourHoldConsumed = true;
    return;
  }
  if (g_bank == 5 && k == '4' && s == RELEASED && g_fourHoldConsumed) { g_fourHoldConsumed = false; return; }
  if (g_bank == 5 && k == '4' && s == RELEASED) {
    adjustBank5Rit(-1);
    return;
  }

  if (g_bank == 5 && k == '5' && s == HOLD) {
    adjustBank5Rit(500);
    g_fiveHoldConsumed = true;
    return;
  }
  if (g_bank == 5 && k == '5' && s == RELEASED && g_fiveHoldConsumed) { g_fiveHoldConsumed = false; return; }
  if (g_bank == 5 && k == '5' && s == RELEASED) {
    adjustBank5Rit(1);
    return;
  }

  if (g_bank == 5 && k == '3' && s == HOLD) {
    printKeypadCommand("BANK5 3 LONG -> RIT OFF");
    if (setRitEnabled(false)) {
      printKeypadStatus("RIT OFF");
      if (g_speechEnabled) speakToken("off");
    }
    g_threeHoldConsumed = true;
    return;
  }
  if (g_bank == 5 && k == '3' && s == RELEASED && g_threeHoldConsumed) { g_threeHoldConsumed = false; return; }
  if (g_bank == 5 && k == '3' && s == RELEASED) {
    setBank5RitOffset(0);
    return;
  }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '4' && s == HOLD) {
    printKeypadCommand("BANK2 4 LONG -> VFO TOGGLE");
    sendYaesuBank2Action("VFO TOGGLE", yaesuCatToggleVfo());
    g_fourHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '4' && s == RELEASED && g_fourHoldConsumed) { g_fourHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '5' && s == HOLD) {
    printKeypadCommand("BANK2 5 LONG -> VFO B");
    sendYaesuBank2Action("VFO B", yaesuCatSelectVfoB());
    g_fiveHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '5' && s == RELEASED && g_fiveHoldConsumed) { g_fiveHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '6' && s == HOLD) {
    printKeypadCommand("BANK2 6 LONG -> CLAR OFF");
    sendYaesuBank2Action("CLAR OFF", yaesuCatSetClarifier(false));
    g_sixHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '6' && s == RELEASED && g_sixHoldConsumed) { g_sixHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '7' && s == HOLD) {
    printKeypadCommand("BANK2 7 LONG -> SPLIT OFF");
    sendYaesuBank2Action("SPLIT OFF", yaesuCatSetSplit(false));
    g_sevenHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '7' && s == RELEASED && g_sevenHoldConsumed) { g_sevenHoldConsumed = false; return; }

  if (g_bank == 2 && isCurrentYaesuFt8x7() && k == '8' && s == HOLD) {
    printKeypadCommand("BANK2 8 LONG -> PTT OFF");
    sendYaesuBank2Action("PTT OFF", yaesuCatSetPtt(false));
    g_eightHoldConsumed = true;
    return;
  }
  if (g_bank == 2 && k == '8' && s == RELEASED && g_eightHoldConsumed) { g_eightHoldConsumed = false; return; }

  if (g_bank == 9 && k == 'A' && s == HOLD) {
    beginBank9ProfileSelect();
    g_aHoldConsumed = true;
    return;
  }
  if (g_bank == 9 && k == 'A' && s == RELEASED && g_aHoldConsumed) { g_aHoldConsumed = false; return; }

  if (g_bank == 9 && k == '4' && s == HOLD) {
    toggleBank9TuningSpeech();
    g_fourHoldConsumed = true;
    return;
  }
  if (g_bank == 9 && k == '4' && s == RELEASED && g_fourHoldConsumed) { g_fourHoldConsumed = false; return; }

  if (s == RELEASED) {
    char key = (char)k;
    if (g_pendingClickActive &&
        g_pendingClickBank == g_bank &&
        g_pendingClickKey == key &&
        (uint32_t)(millis() - g_pendingClickAtMs) <= KEYPAD_DOUBLE_CLICK_MS) {
      g_pendingClickActive = false;
      if (handleDoubleClick(g_bank, key)) return;
    }
    if (shouldDelayShortRelease(g_bank, key)) {
      g_pendingClickActive = true;
      g_pendingClickBank = g_bank;
      g_pendingClickKey = key;
      g_pendingClickAtMs = millis();
      return;
    }
    keypadHandleReleased(key);
  }
}

void initKeypadUi() {
  keypad.setDebounceTime(KEYPAD_DEBOUNCE_MS);
  keypad.setHoldTime(KEYPAD_HOLD_MS);
  keypad.addEventListener(keypadEvent);
}

void pollKeypadUi() {
  (void)keypad.getKey();
  if (g_pendingClickActive && (uint32_t)(millis() - g_pendingClickAtMs) > KEYPAD_DOUBLE_CLICK_MS) {
    uint8_t bank = g_pendingClickBank;
    char key = g_pendingClickKey;
    g_pendingClickActive = false;
    if (!handleDeferredShortRelease(bank, key)) keypadHandleReleased(key);
  }
}

uint8_t uiGetBank() {
  return g_bank;
}

void uiSetBank(uint8_t bank) {
  g_bank = bank;
}
