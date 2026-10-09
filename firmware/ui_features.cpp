#include "ui_features.h"
#include "ui_speech.h"

const char* featureStatusText(FeatureStatus status) {
  switch (status) {
    case FeatureStatus::Unsupported: return "unsupported";
    case FeatureStatus::Timeout: return "timeout";
    case FeatureStatus::NoReply: return "no reply";
    case FeatureStatus::Failed: return "failed";
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

static const char* ft8x7AgcText(YaesuAgc agc) {
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
    case Ft8x7Setting::LowCut: return "HPF";
    case Ft8x7Setting::HighCut: return "LPF";
    case Ft8x7Setting::MicEq: return "MICEQ";
    case Ft8x7Setting::Antenna: return "ANT";
  }
  return "";
}

// Menu 48 as the radio shows it.
static const char* ft8x7MicEqText(YaesuFt857MicEq eq) {
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

String ft8x7SettingText(const Ft8x7SettingState& state) {
  String text = String(ft8x7SettingName(state.setting)) + " ";
  if (state.setting == Ft8x7Setting::RfPower) return text + wattsText(state.wattsTenths) + " W";
  if (state.setting == Ft8x7Setting::Agc) return text + ft8x7AgcText(state.agc);
  if (state.setting == Ft8x7Setting::Menu || state.setting == Ft8x7Setting::Row) return text + String((int)state.number);
  if (state.setting == Ft8x7Setting::NrLevel || state.setting == Ft8x7Setting::NbLevel) return text + String(state.value);
  if (state.setting == Ft8x7Setting::LowCut || state.setting == Ft8x7Setting::HighCut) {
    return text + String(state.value) + " Hz";
  }
  if (state.setting == Ft8x7Setting::MicEq) return text + ft8x7MicEqText(state.micEq);
  if (state.setting == Ft8x7Setting::Antenna) return text + (state.on ? "REAR" : "FRONT");
  return text + (state.on ? "ON" : "OFF");
}

void speakFt8x7Setting(const Ft8x7SettingState& state) {
  if (!g_speechEnabled) return;
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
  // "h p f 300 hertz": the radio's menu name, low cut being the high pass filter.
  if (state.setting == Ft8x7Setting::LowCut || state.setting == Ft8x7Setting::HighCut) {
    speakLabel(spelledLetters(ft8x7SettingName(state.setting)));
    speakNumber(String(state.value));
    playSilenceMs(60);
    speakToken("hertz");
    return;
  }
  // "equalizer both", "equalizer l p f"
  if (state.setting == Ft8x7Setting::MicEq) {
    speakLabel("equalizer");
    if (state.micEq == YaesuFt857MicEq::Lpf || state.micEq == YaesuFt857MicEq::Hpf) {
      speakToken(spelledLetters(ft8x7MicEqText(state.micEq)));
    } else {
      speakToken(ft8x7MicEqText(state.micEq));
    }
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
  // "AGC" -> "a g c"
  const String spelled = spelledLetters(ft8x7SettingName(state.setting));
  if (state.setting == Ft8x7Setting::Agc) {
    speakLabel(spelled);
    speakToken(ft8x7AgcText(state.agc));
    return;
  }
  speakTokenState(spelled, state.on);
}
