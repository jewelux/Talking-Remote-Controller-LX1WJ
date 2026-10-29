#pragma once

#include "formatted_line.h"
#include "keypad_actions.h"
#include "keypad_help.h"
#include "keypad_input.h"
#include "radio_features.h"
#include "radio_globals.h"
#include "ui_speech.h"

// Helpers shared by the keypad UI files.

// What an action may ask of the keypad state machine (ui_keypad.cpp): the keys
// after it go to an entry, a choice select or profile select.
void keypadBeginEntry(InputMode mode, TargetVfo targetVfo = TargetVfo::Current);
void keypadBeginChoiceSelect(const ChoiceSpec& spec, TargetVfo targetVfo = TargetVfo::Current,
                             const EntrySeed& seed = EntrySeed());
void keypadBeginValueEntry(const ValueSpec& spec, const EntrySeed& seed);
// A B or C nudge: the choice select opened from seed, one value away, staged and said by its
// picked(). False, with nothing opened, at the end of the choices.
bool keypadNudgeChoice(const ChoiceSpec& spec, const EntrySeed& seed, int8_t dir,
                       TargetVfo targetVfo = TargetVfo::Current);
// The mode keys (ui_keypad_entry.cpp).
void keypadBeginModeSelect(TargetVfo targetVfo);
// The mode select of the current VFO, opened from mode one mode away (keypadNudgeChoice).
bool keypadNudgeModeSelect(uint8_t mode, int8_t dir);
void keypadBeginProfileSelect();

// Runs a console command for a key in a KeypadAnswer named by keypadCommandLabel
// (keypad_labels.h): the name at once, the console's reply without it.
void keypadSendNow(const String& cmd);

// Entries and selections (ui_keypad_entry.cpp), called by the state machine.
void keypadEntryDigit(const EntrySpec& entry, char key, const char* digits);
void keypadEntryCommit(InputMode mode, const char* digits, TargetVfo targetVfo);
// The settings engine (ui_keypad_settings.cpp). B or C on setting (big: a big step): a live
// setting is set and said at once; a Confirm one opens its entry one step away, "power 45
// watts please".
void keypadSettingNudge(Setting setting, int8_t dir, bool big);
// B and C on the Bank 1 focus (ui_keypad_bank1.cpp), read once and staged for Enter: RF power
// (the FT-817/818 level pick) and the mode pick.
void nudgeBank1RfPower(int8_t dir, bool big);
void nudgeBank1Mode(int8_t dir);
// B and C on the AGC focus (ui_keypad_bank2.cpp): over CAT the next speed the radio can be set
// to, set at once; on the FT-8x7 the AGC pick one away, staged for Enter. A big step is one too.
void nudgeBank2Agc(int8_t dir, bool big);
// B and C on the Bank 8 focuses (ui_keypad_bank8.cpp), set at once: the next or previous baud
// rate, going round (no big step); the FT-8x7 menu item or row one up or down (big 10 or 5).
void stepBank8Baud(int8_t dir, bool big);
void stepBank8Ft8x7Menu(int8_t dir, bool big);
void stepBank8Ft8x7Row(int8_t dir, bool big);
// "UP", "DOWN FAST" and so on, for a B or C trace.
const char* keypadStepWord(int8_t dir, bool big);
// What a value entry says, called by the state machine.
void keypadValueDigit(const ValueSpec& spec, char key, const char* digits);
void keypadValueSay(const ValueSpec& spec, int32_t value);
void keypadValueStepped(const ValueSpec& spec, int32_t value);
// The value, then "please": staged, not yet set.
void keypadValueStaged(const ValueSpec& spec, int32_t value);
// "maximum" or "minimum": nothing changed.
void keypadValueLimit(const char* name, bool atMax);
void keypadValueCommit(const ValueSpec& spec, int32_t value);
// Shared frequency writer for entry commit and the round-to-500 Hz action.
bool keypadApplyFrequencyHz(uint64_t hz, TargetVfo targetVfo);

// Holds background polling briefly, so it does not talk over a key's exchange.
void holdKeypadPolling();
// Holds polling and tuning speech while a key's answer is prepared.
void prepareKeypadSpeechResponse();
// Holds polling a little longer while a key writes to the radio and reads it back.
void prepareKeypadRadioWrite();
// FT-8x7 with the dial lock on: say "lock on" and return false.
bool guardFt8x7VfoToggleLock();
void speakSimpleBinaryState(bool on);
// "transceiver rx" or "transceiver tx".
void speakRxTxState(bool tx);
void speakQueriedFrequencyHz(uint64_t hz);
void speakFeatureValue(const char* featureToken, uint8_t value);

// Serial traces. Each takes a format with {} placeholders and its arguments (see
// FormattedLine); text that is not a literal goes through "{}":
//   printKeypadStatus("LOCK {}", on ? "ON" : "OFF");
//   printKeypadStatus("VFO{}: {} MHz", which, RadioFrequency::fromHz(hz));
// Nothing is formatted while no console is attached.
template <class... Args>
void printKeypadStatus(LineFormat<sizeof...(Args)> format, const Args&... args) {
  if (Serial) Serial.println(FormattedLine(format, args...).c_str());
}

// "CMD <line>" trace.
template <class... Args>
void printKeypadCommand(LineFormat<sizeof...(Args)> format, const Args&... args) {
  if (!Serial) return;
  FormattedLine line("CMD ");
  line.appendFormat(format, args...);
  Serial.println(line.c_str());
}

// Trace of a bank key action: "CMD <key> -> <what>", the key being keypadActiveKey()
// (e.g. "BANK3 2 LONG"). Just "CMD <what>" outside a key action.
template <class... Args>
void printKeypadAction(LineFormat<sizeof...(Args)> format, const Args&... args) {
  if (!Serial) return;
  FormattedLine line("CMD ");
  if (const char* key = keypadActiveKey()) line.appendFormat("{} -> ", key);
  line.appendFormat(format, args...);
  Serial.println(line.c_str());
}

// A key event: radio traffic and timeouts from before it are not this key's doing.
void keypadForgetRadioActivity();

// What went wrong, said "not available", "timeout" or "error".
enum class KeypadFailure : uint8_t { NotAvailable, Timeout, Error };
// The failure of a key that asks nothing of the radio, with verbose on after its
// name ("profile not available"). label is the trace label, e.g. "PROFILE"; a
// label with no row in kSpokenLabels is spelled.
void speakKeypadFailure(const char* label, KeypadFailure failure);
// A key that asks nothing of the radio cannot run here: prints "<label> -> not
// available" and says it. (A beep is only for a key with no action.)
void keypadReportNotAvailable(const char* label);

// How a KeypadAnswer names its key.
enum class AnswerNaming : uint8_t {
  Labels,     // at once, where a label is said: verbose on, or a LabelModeScope says so
  Always,     // at once, verbose off too: an entry that ends in "please"
  OnFailure,  // not now, the user just heard it: only before a failure, with verbose on
};

// The answer to a key that talks to the radio. It holds polling and says the
// key's name at once, so the name plays while the radio is asked; then, for the
// answer's life, speakLabel says nothing (LabelMode::Never), so a value speaker
// (speakNrState, ops->speak, a console reply) gives just the value. A failure
// names the key when verbose is on and the name was not said yet. An answer made
// inside another one (a key that runs another action) says no name again.
//
//   printKeypadAction("NR?");
//   KeypadAnswer answer("NR?");                      // "noisereduction"
//   NrState state;
//   if (answer.failed(nrQuery(state))) return;       // "timeout"
//   printKeypadStatus("{}", nrStateText(state).c_str());
//   speakNrState(state);                             // "on"
//
// Every failure prints "<label> -> <what>" too.
class KeypadAnswer {
 public:
  explicit KeypadAnswer(const char* label, AnswerNaming naming = AnswerNaming::Labels);
  KeypadAnswer(const KeypadAnswer&) = delete;
  KeypadAnswer& operator=(const KeypadAnswer&) = delete;

  // A shared feature operation: "not available" when unsupported, "timeout" or
  // "error" otherwise, and true. Ok: false.
  bool failed(FeatureStatus status);
  // available false: the radio or profile has no such function. Says "not
  // available" and returns true.
  bool unavailable(bool available);
  // The radio gave no answer: says "timeout" and returns true. Otherwise false, so
  // the caller says its own failure.
  bool timedOut();
  // A radio call returned false: "timeout" when the radio gave no answer, "not
  // available" when nothing was sent (the profile has no command for it), and
  // "error" otherwise (the radio refused it or answered something else).
  void radioFailed();
  void fail(KeypadFailure failure);
  // A command with no value to say worked: "ok" after its name (verbose off: nothing),
  // or its name where it was not said.
  void done();
  // The name was said, now or by an answer around this one.
  bool named() const { return named_; }

 private:
  void sayName() const;

  const char* label_;
  bool named_;
  LabelModeScope labels_;
};

// How a B or C step names its setting: at once on the first step after another
// key, verbose off too; on a repeat only before a failure.
AnswerNaming keypadStepNaming();

// Reads an FT-8x7 EEPROM setting, prints and speaks it; the FT-817 says "not available" for what it lacks.
void queryKeypadFt8x7Setting(Ft8x7Setting setting);
// An FT-8x7 setting read or written: the failure as answer.failed says it, else state printed
// and spoken.
void keypadReportFt8x7Setting(KeypadAnswer& answer, FeatureStatus status, const Ft8x7SettingState& state);
// The key has no action here: short beep.
void keypadReportUnassigned(const char* label);
// The key is hidden on the FTDX10 layout: say "not available".
void reportFtdx10HiddenKey();

// The radio checks behind KeypadTraits::layout. Bank actions do not use them:
// the keymap already picked the action for the radio.
bool isFtdx10KeypadProfile();
bool isFt8x7Ft817Keypad();
bool isFt8x7Ft857FamilyKeypad();

// The FT-817 and FT-857/897 keypad workflows track VFO A/B. The FT-857/897 reads the
// active VFO from the radio; the FT-817 assumes VFO A until a toggle or sync says otherwise.
void refreshFt8x7ActiveVfo();
// Tracked VFO letter ('A' or 'B') on FT-817 and FT-857/897, '?' otherwise.
char ft8x7CurrentVfoLabel();
char ft8x7OtherVfoLabel();

void formatHexByte(uint8_t value, char* out, size_t outSize);
void speakHexNibble(char c);
void speakCivAddressValue(uint8_t addr, bool ok);
// "baud 4800" ("4800" with verbose off), then "ok" if ok.
void speakBaudValue(uint32_t baud, bool ok);
// "profile reset", after the profile's baud and CI-V address went back to its defaults.
void speakProfileReset();

void formatCtcssTenthsLabel(uint16_t toneTenths, char* out, size_t outSize);

void speakFrequencyWord();
// A short gap, then "please". Ends a prompt such as "mode please", and the value
// a Confirm entry opens with.
void speakPlease();
// "help on" or "help off", with verbose off too.
void speakHelpMode(bool on);
// What a gesture would do in help mode ("up noisereduction level"). The name is
// said with verbose off too; HelpVerb::None says nothing.
void speakHelp(const GestureHelp& help);
// "<token> please".
void speakPrompt(const char* token);
// "vfo a" or "vfo b" (just "vfo" for any other letter).
void speakVfoLabel(char which);
// "vfo a frequency".
void speakVfoFrequencyLabel(char which);
// "vfo a frequency" and the frequency in MHz.
void speakVfoFrequency(char which, uint64_t hz);
