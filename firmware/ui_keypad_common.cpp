#include "ui_keypad_common.h"
#include "keypad_labels.h"
#include "ui_features.h"

#include "radio_catalog.h"
#include "radio_monitor.h"
#include "radio_protocol.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "transport_serial.h"
#include "ui_speech.h"

static constexpr uint32_t KEYPAD_POLL_SUSPEND_MS = 900;
static constexpr uint32_t KEYPAD_WRITE_POLL_SUSPEND_MS = 1400;
static constexpr uint32_t KEYPAD_ANSWER_SPEECH_QUIET_MS = 2000;

static uint32_t s_writesBeforeKey = 0;

void keypadForgetRadioActivity() {
  g_radioReplyTimedOut = false;
  s_writesBeforeKey = serialTransportWriteCount();
}

// A label kSpokenLabels (keypad_labels.cpp) has no row for (CWAUTO?, ...) is
// spelled, as its answer says it.
static String spokenLabel(const char* label) {
  String name(label);
  if (name.endsWith("?")) name.remove(name.length() - 1);
  if (const char* words = spokenLabelWords(name.c_str())) return words;
  String spelled;
  for (size_t i = 0; i < name.length(); ++i) {
    if (name[i] < 'A' || name[i] > 'Z') return String();
    if (spelled.length()) spelled += ' ';
    spelled += name[i];
  }
  return spelled;
}

// The name, said whatever the label mode, and the gap after it.
static void speakName(const char* label) {
  if (!g_speechEnabled) return;
  const String name = spokenLabel(label);
  if (!name.length()) return;
  speakToken(name);
  playSilenceMs(60);
}

static void speakFailureWord(KeypadFailure failure) {
  if (!g_speechEnabled) return;
  switch (failure) {
    case KeypadFailure::NotAvailable: speakNotAvailable(); break;
    case KeypadFailure::Experimental: speakToken("experimental"); break;
    case KeypadFailure::Timeout: speakTimeout(); break;
    case KeypadFailure::Error: speakError(); break;
  }
}

static const char* failureText(KeypadFailure failure) {
  switch (failure) {
    case KeypadFailure::NotAvailable: return "not available";
    case KeypadFailure::Experimental: return "experimental";
    case KeypadFailure::Timeout: return "timeout";
    default: return "failed";
  }
}

void speakKeypadFailure(const char* label, KeypadFailure failure) {
  if (!g_speechEnabled) return;
  if (labelSpoken()) speakName(label);
  speakFailureWord(failure);
}

void keypadReportNotAvailable(const char* label) {
  printKeypadStatus("{} -> not available", label);
  speakKeypadFailure(label, KeypadFailure::NotAvailable);
}

void keypadReportExperimental(const char* label) {
  printKeypadStatus("{} -> experimental", label);
  speakKeypadFailure(label, KeypadFailure::Experimental);
}

void keypadReportRefusal(const char* label, FeatureStatus refusal) {
  if (refusal == FeatureStatus::Experimental) keypadReportExperimental(label);
  else keypadReportNotAvailable(label);
}

// Holds polling and says the name now where naming wants it: true when the user
// has heard it, now or from an answer around this one (LabelMode::Never).
static bool beginAnswer(const char* label, AnswerNaming naming) {
  prepareKeypadSpeechResponse();
  if (labelMode() == LabelMode::Never) return true;
  if (naming == AnswerNaming::OnFailure) return false;
  if (naming == AnswerNaming::Labels && !labelSpoken()) return false;
  speakName(label);
  return true;
}

KeypadAnswer::KeypadAnswer(const char* label, AnswerNaming naming)
    : label_(label),
      named_(beginAnswer(label, naming)),
      labels_(named_ || naming == AnswerNaming::OnFailure ? LabelMode::Never : labelMode()) {}

void KeypadAnswer::sayName() const { speakName(label_); }

void KeypadAnswer::fail(KeypadFailure failure) {
  printKeypadStatus("{} -> {}", label_, failureText(failure));
  if (!g_speechEnabled) return;
  if (!named_ && g_verboseSpeech) sayName();
  speakFailureWord(failure);
}

bool KeypadAnswer::failed(FeatureStatus status) {
  switch (status) {
    case FeatureStatus::Ok: return false;
    case FeatureStatus::Unsupported: fail(KeypadFailure::NotAvailable); return true;
    case FeatureStatus::Experimental: fail(KeypadFailure::Experimental); return true;
    case FeatureStatus::Timeout: fail(KeypadFailure::Timeout); return true;
    default: break;
  }
  printKeypadStatus("{} -> {}", label_, featureStatusText(status));
  if (!g_speechEnabled) return true;
  if (!named_ && g_verboseSpeech) sayName();
  speakFailureWord(KeypadFailure::Error);
  return true;
}

bool KeypadAnswer::unavailable(bool available) {
  if (available) return false;
  fail(KeypadFailure::NotAvailable);
  return true;
}

bool KeypadAnswer::timedOut() {
  if (!g_radioReplyTimedOut) return false;
  fail(KeypadFailure::Timeout);
  return true;
}

void KeypadAnswer::radioFailed() {
  if (timedOut()) return;
  // Nothing went to the radio: the profile has no command for it.
  if (serialTransportWriteCount() == s_writesBeforeKey) fail(KeypadFailure::NotAvailable);
  else fail(KeypadFailure::Error);
}

void KeypadAnswer::done() { speakDone(spokenLabel(label_)); }

AnswerNaming keypadStepNaming() {
  return keypadActiveFocusIsNew() ? AnswerNaming::Always : AnswerNaming::OnFailure;
}

void queryKeypadFt8x7Setting(Ft8x7Setting setting) {
  const char* label = ft8x7SettingLabel(setting);
  printKeypadAction("{}", label);
  KeypadAnswer answer(label);
  Ft8x7SettingState state;
  keypadReportFt8x7Setting(answer, ft8x7SettingQuery(setting, state), state);
}

void keypadReportFt8x7Setting(KeypadAnswer& answer, FeatureStatus status, const Ft8x7SettingState& state) {
  if (answer.failed(status)) return;
  printKeypadStatus("{}", ft8x7SettingText(state).c_str());
  speakFt8x7Setting(state);
}

void keypadReportUnassigned(const char* label) {
  printKeypadStatus("{} -> unassigned", label);
  playBeep();
}

void reportFtdx10HiddenKey() {
  const char* key = keypadActiveKey();
  printKeypadStatus("{} hidden on FTDX10", key ? key : "KEY");
  if (g_speechEnabled) speakNotAvailable();
}

bool isFtdx10KeypadProfile() {
  return currentRadioModel() == RadioModel::Ftdx10;
}

bool isFt8x7Ft817Keypad() {
  return currentIsFt817Family();
}

bool isFt8x7Ft857FamilyKeypad() {
  return currentIsFt857Family();
}

static bool tracksFt8x7Vfo() {
  return isFt8x7Ft817Keypad() || isFt8x7Ft857FamilyKeypad();
}

// Without a reply (or on the FT-817, which has no readback) the tracked VFO stays, starting at A.
void refreshFt8x7ActiveVfo() {
  if (!tracksFt8x7Vfo()) return;
  bool vfoA = true;
  if (queryActiveVfo(vfoA, 300)) return;
  if (!live.activeVfoKnown) rememberActiveVfo(true);
}

char ft8x7CurrentVfoLabel() {
  if (!tracksFt8x7Vfo()) return '?';
  if (!live.activeVfoKnown) rememberActiveVfo(true);
  return live.activeVfoA ? 'A' : 'B';
}

char ft8x7OtherVfoLabel() {
  if (!tracksFt8x7Vfo()) return '?';
  if (!live.activeVfoKnown) rememberActiveVfo(true);
  return live.activeVfoA ? 'B' : 'A';
}

void formatHexByte(uint8_t value, char* out, size_t outSize) {
  if (!out || outSize < 3) return;
  snprintf(out, outSize, "%02X", (unsigned)value);
}

void speakHexNibble(char c) {
  if (c >= '0' && c <= '9') {
    playDigit(c - '0');
    return;
  }
  speakToken(String(c));  // A-F: the letter clip
}

void speakCivAddressValue(uint8_t addr, bool ok) {
  if (!g_speechEnabled) return;
  char hex[3] = "";
  formatHexByte(addr, hex, sizeof(hex));
  if (labelSpoken()) {
    speakToken("c");
    playSilenceMs(50);
    speakToken("i");
    playSilenceMs(80);
  }
  speakHexNibble(hex[0]);
  playSilenceMs(50);
  speakHexNibble(hex[1]);
  if (ok) speakValueOk();
}

void speakBaudValue(uint32_t baud, bool ok) {
  if (!g_speechEnabled) return;
  speakLabel("baud");
  speakNumber(String((unsigned long)baud));
  if (ok && g_verboseSpeech) speakValueOk();
  else playSilenceMs(250);
}

void speakProfileReset() {
  if (!g_speechEnabled) return;
  speakToken("profile");
  playSilenceMs(60);
  speakToken("reset");
}

void formatCtcssTenthsLabel(uint16_t toneTenths, char* out, size_t outSize) {
  if (!out || outSize < 2) return;
  snprintf(out, outSize, "%u.%u", (unsigned)(toneTenths / 10), (unsigned)(toneTenths % 10));
}

void speakFrequencyWord() {
  if (!g_speechEnabled) return;
  speakToken("frequency");
}

void speakPlease() {
  if (!g_speechEnabled) return;
  speakToken("please");
}

void speakHelpMode(bool on) {
  LabelModeScope named(LabelMode::Always);
  speakTokenState("help", on);
}

void speakHelp(const GestureHelp& help) {
  if (!g_speechEnabled || help.verb == HelpVerb::None) return;
  LabelModeScope named(LabelMode::Always);
  switch (help.verb) {
    case HelpVerb::Next: speakToken("next"); break;
    case HelpVerb::Entry:
    case HelpVerb::Pick: speakToken("choose"); break;
    case HelpVerb::Centre: speakToken("reset"); break;
    case HelpVerb::Up: speakToken("up"); break;
    case HelpVerb::Down: speakToken("down"); break;
    case HelpVerb::UpFast: speakToken("up fast"); break;
    case HelpVerb::DownFast: speakToken("down fast"); break;
    default: break;
  }
  const String name = help.label ? spokenLabel(help.label) : String();
  if (name.length()) speakLabel(name);
  if (help.verb == HelpVerb::Toggle) speakToken("on off");
  if (help.verb == HelpVerb::NotAvailable) speakNotAvailable();
}

void speakPrompt(const char* token) {
  if (!g_speechEnabled) return;
  speakToken(token);
  speakPlease();
}

static void speakVfoLetter(char which) {
  if (which == 'A') speakToken("a");
  else if (which == 'B') speakToken("b");
}

// The letter is the value, so verbose off keeps it.
void speakVfoLabel(char which) {
  if (!g_speechEnabled) return;
  speakLabel("vfo");
  speakVfoLetter(which);
}

// A prompt: the same words with verbose off.
void speakVfoFrequencyLabel(char which) {
  if (!g_speechEnabled) return;
  speakToken("vfo");
  playSilenceMs(60);
  speakVfoLetter(which);
  playSilenceMs(60);
  speakFrequencyWord();
}

// Verbose off: "a 7.1". After the name ("vfo a" from a KeypadAnswer): "7.1".
void speakVfoFrequency(char which, uint64_t hz) {
  if (!g_speechEnabled) return;
  if (labelMode() != LabelMode::Never) {
    speakVfoLabel(which);
    playSilenceMs(60);
  }
  speakLabel("frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

void holdKeypadPolling() {
  g_suspendPollingUntilMs = millis() + KEYPAD_POLL_SUSPEND_MS;
}

void prepareKeypadSpeechResponse() {
  holdKeypadPolling();
  g_suppressFreqSpeakUntilMs = millis() + KEYPAD_ANSWER_SPEECH_QUIET_MS;
  cancelPendingFreqAnnouncement();
}

void prepareKeypadRadioWrite() {
  g_suspendPollingUntilMs = millis() + KEYPAD_WRITE_POLL_SUSPEND_MS;
  g_suppressFreqSpeakUntilMs = millis() + KEYPAD_ANSWER_SPEECH_QUIET_MS;
  cancelPendingFreqAnnouncement();
}

bool guardFt8x7VfoToggleLock() {
  if (currentProtocolType() != PROTO_YAESU_FT8X7) return true;
  // Reads the radio's lock, so one set on the front panel counts too. No reply: let the toggle
  // try.
  bool on = false;
  if (!queryDialLock(on, 300) || !on) return true;
  printKeypadStatus("LOCK ON");
  // Not the key's own value: named as verbose has it, inside an answer too.
  LabelModeScope labels(LabelMode::Verbose);
  if (g_speechEnabled) speakTokenState("lock", true);
  return false;
}

void speakSimpleBinaryState(bool on) {
  if (!g_speechEnabled) return;
  speakToken(on ? "on" : "off");
}

void speakRxTxState(bool tx) {
  if (!g_speechEnabled) return;
  speakLabel("transceiver");
  speakToken(tx ? "tx" : "rx");
}

void speakQueriedFrequencyHz(uint64_t hz) {
  if (!g_speechEnabled) return;
  speakLabel("frequency");
  speakDigitsAndPoint(hzToMHzString3(hz));
}

void speakFeatureValue(const char* featureToken, uint8_t value) {
  if (!g_speechEnabled) return;
  speakLabel(featureToken);
  speakDigitsAndPoint(String((int)value));
}

void sendKeypadCommand(const char* cmd) {
  printKeypadAction("{}", cmd);
  keypadSendNow(cmd);
}
