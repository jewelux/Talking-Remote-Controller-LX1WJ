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

// Speech by token, not by clip: a clip named in a new file is another copy of
// it in flash (voice_data.h).
void speakNrState(const NrState& state) {
  if (!g_speechEnabled) return;
  if (state.level == 0) {
    speakTokenState("noisereduction", state.on);
    return;
  }
  speakToken("noisereduction");
  playSilenceMs(60);
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
  speakToken("notch filter");
  playSilenceMs(60);
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
  }
  return "";
}

String ft8x7SettingText(const Ft8x7SettingState& state) {
  String text = String(ft8x7SettingName(state.setting)) + " ";
  if (state.setting == Ft8x7Setting::RfPower) return text + String((int)state.watts) + " W";
  if (state.setting == Ft8x7Setting::Agc) return text + ft8x7AgcText(state.agc);
  if (state.setting == Ft8x7Setting::Menu || state.setting == Ft8x7Setting::Row) return text + String((int)state.number);
  return text + (state.on ? "ON" : "OFF");
}

void speakFt8x7Setting(const Ft8x7SettingState& state) {
  if (!g_speechEnabled) return;
  if (state.setting == Ft8x7Setting::RfPower) {
    speakToken("power");
    playSilenceMs(60);
    speakDigitsAndPoint(String((int)state.watts));
    playSilenceMs(60);
    speakToken("watts");
    return;
  }
  if (state.setting == Ft8x7Setting::Menu || state.setting == Ft8x7Setting::Row) {
    speakToken(state.setting == Ft8x7Setting::Menu ? "menu" : "row");
    playSilenceMs(60);
    speakDigitsAndPoint(String((int)state.number));
    return;
  }
  // "AGC" -> "a g c"
  String spelled;
  for (const char* c = ft8x7SettingName(state.setting); *c; ++c) {
    if (spelled.length()) spelled += ' ';
    spelled += *c;
  }
  if (state.setting == Ft8x7Setting::Agc) {
    speakToken(spelled);
    playSilenceMs(60);
    speakToken(ft8x7AgcText(state.agc));
    return;
  }
  speakTokenState(spelled, state.on);
}
