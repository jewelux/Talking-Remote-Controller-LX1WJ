#pragma once

#include "formatted_line.h"
#include "keypad_actions.h"
#include "keypad_help.h"
#include "keypad_input.h"
#include "radio_features.h"
#include "radio_globals.h"

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

// Runs a console command for a key, with the keypad's polling and speech holds.
void keypadSendNow(const String& cmd);
void speakKeypadCommandWord(const String& cmd);

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
// The answer to a key whose function did not work: with verbose on, the
// function's name first ("tuner not available", "split timeout"). label is the
// trace label, e.g. "TUNER?"; a label with no spoken name gives the bare answer.
enum class KeypadFailure : uint8_t { NotAvailable, Timeout, Error };
void speakKeypadFailure(const char* label, KeypadFailure failure);
// Keypad feedback. Each prints a status line and gives the matching audio cue.
// Radio gave no answer: say "timeout" and return true. Otherwise return false, so
// the caller keeps its own handling of the failure (unsupported, rejected).
bool keypadReportIfTimedOut(const char* label);
// A radio operation failed: say "timeout" when the radio gave no answer, "not
// available" when nothing was sent (the profile has no command for it), and
// "error" otherwise (the radio refused it or answered something else).
void keypadReportFailure(const char* label);
// A shared feature operation did not succeed: say "not available" when
// unsupported, "timeout" or "error" otherwise, and return true. Ok: false.
bool keypadReportFeatureFailure(FeatureStatus status, const char* label);
// Reads an FT-8x7 EEPROM setting, prints and speaks it; the FT-817 says "not available" for what it lacks.
void queryKeypadFt8x7Setting(Ft8x7Setting setting);
// An FT-8x7 setting read or written: the failure as keypadReportFeatureFailure says it, else
// state printed and spoken.
void keypadReportFt8x7Setting(FeatureStatus status, const char* label, const Ft8x7SettingState& state);
// The key has no action here: short beep.
void keypadReportUnassigned(const char* label);
// When supported is false the key's feature is missing on this radio or profile:
// say "not available" and return true. (A beep is only for a key with no action.)
bool keypadReportIfUnsupported(bool supported, const char* label);
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
