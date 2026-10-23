#pragma once

#include <stddef.h>
#include <stdint.h>

#include "keypad_digit_buffer.h"
#include "keypad_settings.h"

// Keypad input state machine. It decides what each key event means: which
// mode is active, which digits an entry takes, when a release is swallowed
// (after a hold, or after a press an entry took), when a short press waits for
// a double click or a double hold, and when two keys are down at once.
// Everything it decides goes out through KeypadInputListener. Pure C++ with no
// Arduino.h, so it runs in the host unit tests (tests/keypad).
//
// Keys: '0'-'9' and 'A'-'C' are bank keys and go to the keymap through the
// listener. '*' (bank), 'D' (Enter) and '#' (Clear) belong to the state machine.

// Exactly one mode is active. Bank select, profile select and the entries
// take digits by the rules of their EntrySpec; choice select takes one key by
// its ChoiceSpec.
enum class InputMode : uint8_t {
  Normal,
  BankSelect,
  ProfileSelect,
  ChoiceSelect,  // a key picks one value of a ChoiceSpec; Enter applies it
  FreqEntry,
  RfPowerEntry,
  CivAddrEntry,
  RptOffsetEntry,
  CtcssEntry,
  DcsEntry,
  MenuEntry,  // FT-8x7 menu item
  RowEntry,   // FT-8x7 soft key row
};

// How bank select, profile select or an entry takes digits. One row per mode
// in keypad_input.cpp; a new entry is an InputMode, a row there and its commit.
struct EntrySpec {
  InputMode mode;
  const char* name;     // label of its beeps and trace: "<name> 5", "<name> D"
  uint8_t maxLen;       // characters, a point included
  bool commitsWhenFull; // the last digit commits it, without Enter
  bool leadingZero;     // '0' may be the first digit
  uint8_t maxFraction;  // digits after the '*' point; 0 = takes no point
  const char* unit;     // shown after the digits while typing
};

// The rules of mode, or nullptr for Normal and ChoiceSelect.
const EntrySpec* keypadEntrySpec(InputMode mode);

// Target VFO for a frequency entry and mode select.
enum class TargetVfo : uint8_t {
  Current,
  A,
  B,
  Other,  // FT-8x7: the VFO that is not active
};

// One value of a choice select and the key that picks it.
struct ChoiceOption {
  char key;
  uint8_t value;
};

// A choice select: a key picks a value and says it, another key replaces it, and Enter applies
// it. Any other key beeps ("<name> 7") and the choice stays; only Clear cancels. The mode, the
// FT-817 power level, ... each have one, kept for as long as the selection runs.
struct ChoiceSpec {
  const char* name;  // label of its beeps and trace: "MODE SELECT"
  // The keys and their values; or nullptr, and pick decides.
  const ChoiceOption* options;
  uint8_t optionCount;
  // With no options: the value key picks, or false. nullptr with options.
  bool (*pick)(char key, uint8_t& value);
  // Says the value key just picked.
  void (*picked)(char key, uint8_t value);
  // Enter: applies value. The mode is already back to Normal.
  void (*commit)(uint8_t value, TargetVfo targetVfo);
};

// Keypad library KeyState, without the Arduino types.
enum class KeyGesture : uint8_t { Pressed, Held, Released, Idle };

// A bank key action.
using KeyAction = void (*)();

// What one bank key does. nullptr: nothing for that gesture.
struct KeyBinding {
  KeyAction shortAction = nullptr;
  KeyAction holdAction = nullptr;
  KeyAction doubleAction = nullptr;
  // A short press followed by a hold within the double-click time. A gesture of
  // its own: with no action it beeps, and the long action does not run instead.
  KeyAction doubleHoldAction = nullptr;
  // A short press waits for a possible double click or double hold. A key may
  // wait with no double action: a quick second press restarts the wait, and the
  // short action then runs once.
  bool waitsForDouble = false;
  // What A, B and C move once this key has acted on its bank.
  Setting focus = Setting::None;
};

// While KeypadInput runs a bank key action: the key and gesture, e.g.
// "BANK3 2 LONG". nullptr otherwise. The actions' serial trace starts with it,
// so an action does not spell out which key runs it.
const char* keypadActiveKey();

class KeypadInputListener {
 public:
  virtual ~KeypadInputListener() = default;

  // Called for every key event, and before a deferred short action runs:
  // whatever happened since the last key is not this key's doing. pressed is
  // true for a key press.
  virtual void onActivity(bool pressed) = 0;

  // The keymap: what key does on bank, looked up each time it is needed.
  virtual KeyBinding keyBinding(uint8_t bank, char key) = 0;

  // '*' short: say the current bank.
  virtual void onBankQuery(uint8_t bank) = 0;
  // '*' hold: bank select started.
  virtual void onBankSelectStart() = 0;
  // Bank select took its digit while '*' was still held: only the next key
  // acts on bank, then the current bank is back. The mode is already back to
  // Normal.
  virtual void onOneShotBank(uint8_t bank) = 0;

  // A digit (or the frequency point) was taken into entry.
  virtual void onDigitAccepted(const EntrySpec& entry, char key, const char* digits) = 0;
  // The key does nothing here: an unassigned key, a digit an entry does not
  // take, a key a choice select has no value for, Enter with nothing picked, '#' with nothing to cancel, a second key
  // pressed while another is down ("TWO KEYS"). "<label> ->
  // unassigned" and the beep.
  virtual void onRejected(const char* label) = 0;
  // Enter in profile select or an entry, with at least one digit typed (with
  // none, Enter beeps and the mode stays), or the digit that fills an entry
  // that commits when full: bank select commits on its digit, once '*' is let
  // go. The mode is already back to Normal. For bank select the new bank is
  // already set.
  virtual void onCommit(InputMode mode, const char* digits, TargetVfo targetVfo) = 0;

  // '#': everything was cancelled. With nothing to cancel, '#' gives
  // onRejected("CLEAR") instead.
  virtual void onClear() = 0;
  // An entry or selection had no key event for kEntryTimeoutMs and was
  // dropped. The mode is already back to Normal. name: its EntrySpec or
  // ChoiceSpec name.
  virtual void onEntryTimeout(InputMode mode, const char* name) = 0;
};

class KeypadInput {
 public:
  // A second short press of the same key within this time is a double click.
  static constexpr uint32_t kDoubleClickMs = 220;
  // An entry or selection with no key event for this long goes back to Normal.
  static constexpr uint32_t kEntryTimeoutMs = 30000;

  explicit KeypadInput(KeypadInputListener& listener) : listener_(listener) {}

  void onKey(char key, KeyGesture gesture, uint32_t nowMs);
  // Runs a deferred short action once the double-click wait has passed, and
  // ends an entry or selection that waited kEntryTimeoutMs for a key.
  void poll(uint32_t nowMs);

  // Called by keymap actions.
  void beginEntry(InputMode mode, TargetVfo targetVfo = TargetVfo::Current);
  void beginProfileSelect() { beginEntry(InputMode::ProfileSelect); }
  // spec must outlive the selection.
  void beginChoiceSelect(const ChoiceSpec& spec, TargetVfo targetVfo = TargetVfo::Current);

  uint8_t bank() const { return bank_; }
  void setBank(uint8_t bank) { bank_ = bank; }

  InputMode mode() const { return mode_; }
  const char* digits() const { return digits_.c_str(); }
  TargetVfo entryTargetVfo() const { return entryVfo_; }
  // Choice select with a value picked, waiting for Enter.
  bool choiceStaged() const { return mode_ == InputMode::ChoiceSelect && choiceStaged_; }
  bool doubleClickPending() const { return pending_.active; }

 private:
  // A set of keys, one bit each. Used for the keys that are down and for the
  // keys whose release is swallowed: their hold was handled, or their press
  // was taken by bank, profile or choice select or an entry.
  class KeySet {
   public:
    void set(char key);
    bool has(char key) const;
    // Clears key's bit. Returns true when it was set.
    bool release(char key);
    bool any() const { return bits_ != 0; }
    // key is the only key in the set.
    bool isOnly(char key) const;
    // Some key other than key is in the set.
    bool hasOtherThan(char key) const;

   private:
    uint16_t bits_ = 0;
  };

  // A key that belongs to the state machine: the same on every bank.
  struct GlobalKey {
    char key;
    const char* name;  // for the beep of a long press without an action
    bool normalOnly;  // the other modes take the key as their input
    void (KeypadInput::*onShort)();
    void (KeypadInput::*onHold)();  // nullptr: no long action
  };

  struct DoubleClick {
    bool active = false;
    uint8_t bank = 0;
    char key = 0;
    uint32_t atMs = 0;
    // The key went down again within the time and is still down: its release
    // makes a double click, its hold a double hold.
    bool repressed = false;
  };

  const GlobalKey* globalKey(char key) const;
  void sayBank();
  void beginBankSelect();
  // Tracks the keys that are down. True when the event belongs to a co-press
  // and is to be ignored.
  bool coPressed(char key, KeyGesture gesture);
  // The bank a key acts on: the one-shot bank while one is set.
  uint8_t keyBank() const { return oneShotBank_ ? oneShotBank_ : bank_; }
  void held(char key);
  void runPending();
  // key is the waiting short's key, down again.
  bool secondPressDown(char key) const;
  void releasedNormal(char key, uint32_t nowMs);
  void pressedInMode(char key);
  void pressedChoice(char key);
  // The value key picks in the running choice select.
  bool choiceValue(char key, uint8_t& value) const;
  void pressedEntry(const EntrySpec& entry, char key);
  bool takesDigit(const EntrySpec& entry, char key) const;
  void enter();
  void commitEntry();
  void clearAll();
  // Drops the entry or selection that waited too long.
  void timeOut();
  // Runs action with keypadActiveKey() naming it, e.g. "BANK3 2 LONG".
  void runAction(KeyAction action, uint8_t bank, char key, const char* gesture);
  void runShortOrReject(const KeyBinding& binding, uint8_t bank, char key);
  // "<mode> <what>" to onRejected, e.g. "FREQ 6".
  void reportRejected(const char* mode, const char* what);

  KeypadInputListener& listener_;
  uint8_t bank_ = 1;
  // The bank of the next key only, or 0. It ends once that key has acted or
  // beeped; a short waiting for a double click keeps it until then.
  uint8_t oneShotBank_ = 0;
  InputMode mode_ = InputMode::Normal;
  // The longest entry is a frequency: 12 characters, plus a point the old code
  // accepted even after them.
  DigitBuffer<13> digits_;
  // Target VFO of a frequency entry and mode select.
  TargetVfo entryVfo_ = TargetVfo::Current;
  // The running choice select, and the value picked once choiceStaged_.
  const ChoiceSpec* choice_ = nullptr;
  bool choiceStaged_ = false;
  uint8_t choiceValue_ = 0;
  KeySet swallowed_;
  DoubleClick pending_;
  // Keys that are down now, and whether two were down at once: every event is
  // then ignored until all keys are up.
  KeySet down_;
  bool coPress_ = false;
  // Time of the last key event; an entry or selection times out from here.
  uint32_t lastKeyMs_ = 0;
};
