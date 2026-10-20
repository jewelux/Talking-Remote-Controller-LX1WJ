#include "ui_features.h"
#include "ui_speech.h"

const char* featureStatusText(FeatureStatus status) {
  switch (status) {
    case FeatureStatus::Unsupported: return "unsupported";
    case FeatureStatus::Timeout: return "timeout";
    case FeatureStatus::NoReply: return "no reply";
    case FeatureStatus::Failed: return "failed";
    case FeatureStatus::Invalid: return "invalid";
    case FeatureStatus::Unexpected: return "unexpected value";
    default: return "";
  }
}

String nrStateText(const NrState& state) {
  if (!state.on) return "NR OFF";
  if (state.level > 0) return String("NR ") + String((int)state.level);
  return "NR ON";
}

void speakNrState(const NrState& state) {
  if (!g_speechEnabled) return;
  if (state.level == 0) {
    speakTokenState("noisereduction", state.on);
    return;
  }
  speakLabel("noisereduction");
  playDigit(state.level);
}

String nbStateText(bool on) {
  return on ? "NB ON" : "NB OFF";
}

void speakNbState(bool on) {
  speakTokenState("noiseblanker", on);
}

String notchStateText(const NotchState& state) {
  if (!state.on) return "NOTCH OFF";
  switch (state.width) {
    case NOTCH_WIDTH_NAR: return "NOTCH NAR";
    case NOTCH_WIDTH_MID: return "NOTCH MID";
    case NOTCH_WIDTH_WIDE: return "NOTCH WIDE";
    default: return "NOTCH ON";
  }
}

// The width as its number: 1 NAR, 2 MID, 3 WIDE.
void speakNotchState(const NotchState& state) {
  if (!g_speechEnabled) return;
  speakLabel("notch filter");
  if (!state.on) {
    speakToken("off");
    return;
  }
  switch (state.width) {
    case NOTCH_WIDTH_NAR: playDigit(1); break;
    case NOTCH_WIDTH_MID: playDigit(2); break;
    case NOTCH_WIDTH_WIDE: playDigit(3); break;
    default: speakToken("on"); break;
  }
}

String procStateText(bool on) {
  return on ? "PROC ON" : "PROC OFF";
}

void speakProcState(bool on) {
  speakTokenState("processor", on);
}

String procLevelText(uint8_t level) {
  return String("PROCLEVEL ") + String((int)level);
}

void speakProcLevel(uint8_t level) {
  if (!g_speechEnabled) return;
  speakLabel("processor level");
  speakDigitsAndPoint(String((int)level));
}

String micGainText(uint8_t level) {
  return String("MICGAIN ") + String((int)level);
}

void speakMicGain(uint8_t level) {
  if (!g_speechEnabled) return;
  speakLabel("micgain");
  speakDigitsAndPoint(String((int)level));
}

String voxStateText(bool on) {
  return on ? "VOX ON" : "VOX OFF";
}

void speakVoxState(bool on) {
  speakTokenState("vox", on);
}

String voxGainText(uint8_t level) {
  return String("VOXGAIN ") + String((int)level);
}

void speakVoxGain(uint8_t level) {
  if (!g_speechEnabled) return;
  speakLabel("voxgain");
  speakDigitsAndPoint(String((int)level));
}

String voxDelayText(uint16_t ms) {
  return String("VOXDELAY ") + String((int)ms) + " ms";
}

void speakVoxDelay(uint16_t ms) {
  if (!g_speechEnabled) return;
  speakLabel("voxdelay");
  speakNumber(String((int)ms));
  playSilenceMs(60);
  speakToken("ms");
}

static const char* toneTypeWord(YaesuToneType type) {
  return type == YaesuToneType::Ctcss ? "ctcss" : "dcs";
}

// "off", "ctcss", "dcs", or for a mixed mode the awaited type then the sent one
// ("ctcss rx dcs tx").
static String toneTypeWords(const YaesuToneMode& mode) {
  if (mode.isOff()) return "off";
  if (mode.mixed()) {
    return String(toneTypeWord(mode.rx)) + " rx " + toneTypeWord(mode.tx) + " tx";
  }
  return toneTypeWord(mode.type());
}

static const char* toneDirWord(const YaesuToneMode& mode) {
  if (mode.txOnly()) return "tx";
  if (mode.rxOnly()) return "rx";
  return mode.mixed() ? "split" : "both";
}

static String toneModeWords(const YaesuToneMode& mode) {
  if (mode.isOff() || mode.mixed()) return toneTypeWords(mode);
  return toneTypeWords(mode) + " " + toneDirWord(mode);
}

static String upper(String s) {
  s.toUpperCase();
  return s;
}

static void speakTone(const String& words) {
  if (!g_speechEnabled) return;
  speakLabel("tone");
  speakToken(words);
}

String toneTypeText(const YaesuToneMode& mode) {
  return upper(String("TONE ") + toneTypeWords(mode));
}

void speakToneType(const YaesuToneMode& mode) {
  speakTone(toneTypeWords(mode));
}

String toneDirText(const YaesuToneMode& mode) {
  if (mode.isOff()) return "TONE OFF";
  return upper(String("TONEDIR ") + toneDirWord(mode));
}

void speakToneDir(const YaesuToneMode& mode) {
  speakTone(mode.isOff() ? "off" : toneDirWord(mode));
}

String toneModeText(const YaesuToneMode& mode) {
  return upper(String("TONE ") + toneModeWords(mode));
}

void speakToneMode(const YaesuToneMode& mode) {
  speakTone(toneModeWords(mode));
}

String splitToneText(bool on) {
  return on ? "SPLITTONE ON" : "SPLITTONE OFF";
}

void speakSplitTone(bool on) {
  speakTokenState("split tone", on);
}

const char* ft8x7AgcName(YaesuAgc agc) {
  switch (agc) {
    case YaesuAgc::Fast: return "FAST";
    case YaesuAgc::Slow: return "SLOW";
    case YaesuAgc::Auto: return "AUTO";
    default: return "OFF";
  }
}

// The soft key label on the radio's display.
static const char* ft8x7SettingName(Ft8x7Setting setting) {
  switch (setting) {
    case Ft8x7Setting::Agc: return "AGC";
    case Ft8x7Setting::Ipo: return "IPO";
    case Ft8x7Setting::Att: return "ATT";
    case Ft8x7Setting::Nar: return "NAR";
    case Ft8x7Setting::Dbf: return "DBF";
    case Ft8x7Setting::BreakIn: return "BK";
    case Ft8x7Setting::Keyer: return "KYR";
    case Ft8x7Setting::RfPower: return "RFPOWER";
    case Ft8x7Setting::Menu: return "MENU";
    case Ft8x7Setting::Row: return "ROW";
    case Ft8x7Setting::IfShift: return "IFSHIFT";
    case Ft8x7Setting::NrLevel: return "NRLEVEL";
    case Ft8x7Setting::NbLevel: return "NBLEVEL";
    case Ft8x7Setting::BpfWidth: return "BPF";
    case Ft8x7Setting::LowCut: return "HPF";
    case Ft8x7Setting::HighCut: return "LPF";
    case Ft8x7Setting::MicEq: return "MICEQ";
    case Ft8x7Setting::Antenna: return "ANT";
    case Ft8x7Setting::CwSpeed: return "CWSPEED";
    case Ft8x7Setting::CwPitch: return "CWPITCH";
    case Ft8x7Setting::CwDelay: return "CWDELAY";
    case Ft8x7Setting::CwWeight: return "CWWEIGHT";
    case Ft8x7Setting::Sidetone: return "SIDETONE";
    case Ft8x7Setting::Qsk: return "QSK";
    case Ft8x7Setting::KeyReverse: return "KEYREV";
    case Ft8x7Setting::CwAutoMode: return "CWAUTO";
    case Ft8x7Setting::Paddle: return "PADDLE";
    case Ft8x7Setting::CwBfo: return "CWBFO";
  }
  return "";
}

const char* ft857CwBfoName(Ft857CwBfo bfo) {
  switch (bfo) {
    case Ft857CwBfo::Usb: return "USB";
    case Ft857CwBfo::Lsb: return "LSB";
    default: return "AUTO";
  }
}

String cwWeightText(uint16_t tenths) {
  return String((int)(tenths / 10)) + "." + String((int)(tenths % 10));
}

const char* ft8x7MicEqName(YaesuFt857MicEq eq) {
  switch (eq) {
    case YaesuFt857MicEq::Lpf: return "LPF";
    case YaesuFt857MicEq::Hpf: return "HPF";
    case YaesuFt857MicEq::Both: return "BOTH";
    default: return "OFF";
  }
}

// "LPF" -> "l p f"
static String spelledLetters(const char* name) {
  String spelled;
  for (const char* c = name; *c; ++c) {
    if (spelled.length()) spelled += ' ';
    spelled += *c;
  }
  return spelled;
}

// 25 -> "2.5", 50 -> "5".
static String wattsText(uint16_t tenths) {
  String text = String((int)(tenths / 10));
  if (tenths % 10) text += "." + String((int)(tenths % 10));
  return text;
}

const char* ft817PowerLevelName(Ft817PowerLevel level) {
  switch (level) {
    case Ft817PowerLevel::L3: return "L3";
    case Ft817PowerLevel::L2: return "L2";
    case Ft817PowerLevel::L1: return "L1";
    default: return "HIGH";
  }
}

void speakFt817PowerLevel(Ft817PowerLevel level) {
  if (!g_speechEnabled) return;
  if (level == Ft817PowerLevel::High) {
    speakToken("high");
    return;
  }
  speakToken("l");
  playDigit(level == Ft817PowerLevel::L1 ? 1 : level == Ft817PowerLevel::L2 ? 2 : 3);
}

String ft8x7SettingText(const Ft8x7SettingState& state) {
  String text = String(ft8x7SettingName(state.setting)) + " ";
  // The watts hold on an external supply only, so they are not said.
  if (state.setting == Ft8x7Setting::RfPower && state.hasPowerLevel) {
    return text + ft817PowerLevelName(state.powerLevel) + " (" + wattsText(state.wattsTenths) + " W)";
  }
  if (state.setting == Ft8x7Setting::RfPower) return text + wattsText(state.wattsTenths) + " W";
  if (state.setting == Ft8x7Setting::Agc) return text + ft8x7AgcName(state.agc);
  if (state.setting == Ft8x7Setting::Menu || state.setting == Ft8x7Setting::Row) return text + String((int)state.number);
  if (state.setting == Ft8x7Setting::NrLevel || state.setting == Ft8x7Setting::NbLevel) return text + String(state.value);
  if (state.setting == Ft8x7Setting::BpfWidth || state.setting == Ft8x7Setting::LowCut ||
      state.setting == Ft8x7Setting::HighCut) {
    return text + String(state.value) + " Hz";
  }
  if (state.setting == Ft8x7Setting::MicEq) return text + ft8x7MicEqName(state.micEq);
  if (state.setting == Ft8x7Setting::Antenna) return text + (state.on ? "REAR" : "FRONT");
  switch (state.setting) {
    case Ft8x7Setting::CwSpeed: return text + String(state.value) + " WPM";
    case Ft8x7Setting::CwPitch: return text + String(state.value) + " Hz";
    case Ft8x7Setting::CwDelay: return text + (state.value == 0 ? String("FULL") : String(state.value) + " ms");
    case Ft8x7Setting::CwWeight: return text + "1:" + cwWeightText(state.value);
    case Ft8x7Setting::Sidetone: return text + String(state.value);
    case Ft8x7Setting::Qsk: return text + String(state.value) + " ms";
    case Ft8x7Setting::Paddle: return text + (state.on ? "MICKEY" : "ELEKEY");
    case Ft8x7Setting::CwBfo: return text + ft857CwBfoName(state.cwBfo);
    default: break;
  }
  return text + (state.on ? "ON" : "OFF");
}

// "c w speed 23", "c w pitch 700 hertz", "c w delay full", "c w delay 250 m s", "c w weight 3
// point 0", "side tone 50", "q s k 10 m s", "key reverse", "c w auto on", "paddle m i c",
// "c w b f o u s b". True when state is one of them.
static bool speakFt8x7CwSetting(const Ft8x7SettingState& state) {
  switch (state.setting) {
    case Ft8x7Setting::CwSpeed:
      speakLabel("cw speed");
      speakDigitsAndPoint(String(state.value));
      return true;
    case Ft8x7Setting::CwPitch:
      speakLabel("cw pitch");
      speakNumber(String(state.value));
      playSilenceMs(60);
      speakToken("hertz");
      return true;
    case Ft8x7Setting::CwDelay:
      speakLabel("cw delay");
      if (state.value == 0) {
        speakToken("full");
        return true;
      }
      speakNumber(String(state.value));
      playSilenceMs(60);
      speakToken("ms");
      return true;
    case Ft8x7Setting::CwWeight:
      speakLabel("cw weight");
      speakDigitsAndPoint(cwWeightText(state.value));
      return true;
    case Ft8x7Setting::Sidetone:
      speakLabel("sidetone");
      speakDigitsAndPoint(String(state.value));
      return true;
    case Ft8x7Setting::Qsk:
      speakLabel("q s k");
      speakNumber(String(state.value));
      playSilenceMs(60);
      speakToken("ms");
      return true;
    case Ft8x7Setting::KeyReverse:
      speakLabel("key");
      speakToken(state.on ? "reverse" : "normal");
      return true;
    case Ft8x7Setting::CwAutoMode:
      speakTokenState("cw auto", state.on);
      return true;
    case Ft8x7Setting::Paddle:
      speakLabel("paddle");
      speakToken(state.on ? "m i c" : "e l e");
      return true;
    case Ft8x7Setting::CwBfo:
      speakLabel("cw b f o");
      speakToken(state.cwBfo == Ft857CwBfo::Usb ? "usb" : state.cwBfo == Ft857CwBfo::Lsb ? "lsb" : "auto");
      return true;
    default: return false;
  }
}

void speakFt8x7Setting(const Ft8x7SettingState& state) {
  if (!g_speechEnabled) return;
  if (state.setting == Ft8x7Setting::RfPower && state.hasPowerLevel) {
    speakLabel("power");
    speakFt817PowerLevel(state.powerLevel);
    return;
  }
  if (state.setting == Ft8x7Setting::RfPower) {
    speakLabel("power");
    speakNumber(wattsText(state.wattsTenths));
    playSilenceMs(60);
    speakToken("watts");
    return;
  }
  if (state.setting == Ft8x7Setting::Menu || state.setting == Ft8x7Setting::Row) {
    speakLabel(state.setting == Ft8x7Setting::Menu ? "menu" : "row");
    speakDigitsAndPoint(String((int)state.number));
    return;
  }
  // "noise reduction level 8", "noise blanker level 50"
  if (state.setting == Ft8x7Setting::NrLevel || state.setting == Ft8x7Setting::NbLevel) {
    speakLabel(state.setting == Ft8x7Setting::NrLevel ? "noisereduction level" : "noiseblanker level");
    speakDigitsAndPoint(String(state.value));
    return;
  }
  // "low cut 300 hertz": the radio's HPF cutoff; high cut is its LPF. "b p f 240 hertz": the
  // DSP bandpass width.
  if (state.setting == Ft8x7Setting::BpfWidth || state.setting == Ft8x7Setting::LowCut ||
      state.setting == Ft8x7Setting::HighCut) {
    speakLabel(state.setting == Ft8x7Setting::BpfWidth ? "b p f"
               : state.setting == Ft8x7Setting::LowCut ? "lowcut"
                                                       : "highcut");
    speakNumber(String(state.value));
    playSilenceMs(60);
    speakToken("hertz");
    return;
  }
  // "tx equalizer high cut": the radio's LPF cuts the highs, its HPF the lows.
  if (state.setting == Ft8x7Setting::MicEq) {
    speakLabel("tx equalizer");
    switch (state.micEq) {
      case YaesuFt857MicEq::Lpf: speakToken("highcut"); break;
      case YaesuFt857MicEq::Hpf: speakToken("lowcut"); break;
      case YaesuFt857MicEq::Both: speakToken("both"); break;
      default: speakToken("off"); break;
    }
    return;
  }
  if (state.setting == Ft8x7Setting::Dbf) {
    speakTokenState("bandpassfilter", state.on);
    return;
  }
  if (state.setting == Ft8x7Setting::Nar) {
    speakTokenState("narrow", state.on);
    return;
  }
  // "antenna front", "antenna rear"
  if (state.setting == Ft8x7Setting::Antenna) {
    speakLabel("antenna");
    speakToken(state.on ? "rear" : "front");
    return;
  }
  // No "shift" clip yet: printed only.
  if (state.setting == Ft8x7Setting::IfShift) return;
  if (speakFt8x7CwSetting(state)) return;
  if (state.setting == Ft8x7Setting::Agc) {
    speakLabel("agc");
    speakToken(ft8x7AgcName(state.agc));
    return;
  }
  // IPO bypasses the preamp: "IPO on" is "preamplifier off".
  if (state.setting == Ft8x7Setting::Ipo) {
    speakTokenState("preamplifier", !state.on);
    return;
  }
  if (state.setting == Ft8x7Setting::Att) {
    speakTokenState("attenuator", state.on);
    return;
  }
  // "BK" -> "b k"
  speakTokenState(spelledLetters(ft8x7SettingName(state.setting)), state.on);
}
