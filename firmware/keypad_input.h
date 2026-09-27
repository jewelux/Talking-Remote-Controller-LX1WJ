#pragma once

#include <stddef.h>
#include <stdint.h>

#include "keypad_digit_buffer.h"

// Keypad input state machine. It decides what each key event means: which
// mode is active, which digits an entry takes, when a release is swallowed
// after a hold, and when a short press waits for a double click. Everything it
// decides goes out through KeypadInputListener. Pure C++ with no Arduino.h, so
// it runs in the host unit tests (tests/keypad).
//
// Keys: '0'-'9' and 'A'-'C' are bank keys and go to the keymap through the
// listener. '*' (bank), 'D' (Enter) and '#' (Clear) belong to the state machine.

// Exactly one mode is active. Bank select, profile select and the entries
// take digits by the rules of their EntrySpec.
enum class InputMode : uint8_t {
  Normal,
  BankSelect,
  ProfileSelect,
  ModeSelect,  // a mode key, validated by the listener; Enter applies it
  FreqEntry,
  RfPowerEntry,
  CivAddrEntry,
  RptOffsetEntry,
  CtcssEntry,
  DcsEntry,
};

// How bank select, profile select or an entry takes digits. One row per mode
// in keypad_input.cpp; a new entry is an InputMode, a row there and its commit.
struct EntrySpec {
  InputMode mode;
  const char* name;     // label of its beeps and trace: "<name> 5", "<name> D"
  uint8_t maxLen;       // characters, a point included
  bool replaces;        // when full, a digit starts it over instead of beeping
  bool leadingZero;     // '0' may be the first digit
  uint8_t maxFraction;  // digits after the '*' point; 0 = takes no point
  const char* unit;     // shown after the digits while typing
};

// The rules of mode, or nullptr for Normal and ModeSelect.
const EntrySpec* keypadEntrySpec(InputMode mode);

// Keypad library KeyState, without the Arduino types.
enum class KeyGesture : uint8_t { Pressed, Held, Released, Idle };

// Target VFO for a frequency entry and mode select.
constexpr uint8_t KEYPAD_VFO_CURRENT = 0;
constexpr uint8_t KEYPAD_VFO_A = 1;
constexpr uint8_t KEYPAD_VFO_B = 2;
constexpr uint8_t KEYPAD_VFO_OTHER = 3;  // FT-8x7: the VFO that is not active

class KeypadInputListener {
 public:
  virtual ~KeypadInputListener() = default;

  // Called for every key event, and before a deferred short action runs.
  // pressed is true for a key press.
  virtual void onActivity(bool pressed) = 0;

  // Keymap. Each returns true when the key has an action there and it ran.
  virtual bool runHold(uint8_t bank, char key) = 0;
  virtual bool runShort(uint8_t bank, char key) = 0;
  virtual bool runDoubleClick(uint8_t bank, char key) = 0;
  // True when a short press of the key waits for a possible double click.
  virtual bool wantsDoubleClick(uint8_t bank, char key) = 0;

  // '*' short: say the current bank.
  virtual void onBankQuery(uint8_t bank) = 0;
  // '*' hold: bank select started.
  virtual void onBankSelectStart() = 0;

  // A digit (or the frequency point) was taken into entry.
  virtual void onDigitAccepted(const EntrySpec& entry, char key, const char* digits) = 0;
  // The key does nothing here: "<label> -> unassigned" and the beep.
  virtual void onUnassigned(const char* label) = 0;
  // Enter in bank select, profile select or an entry, with at least one digit
  // typed (with none, Enter beeps and the mode stays). The mode is already back
  // to Normal. For bank select the new bank is already set.
  virtual void onCommit(InputMode mode, const char* digits, uint8_t targetVfo) = 0;

  // A key typed during mode select. Gives the feedback (a beep when it picks no
  // mode) and returns true with mode set when the key picks a mode.
  virtual bool onModeDigit(char key, uint8_t& mode) = 0;
  // Enter in mode select once a mode is picked: apply it. The mode is already
  // back to Normal.
  virtual void onModeCommit(uint8_t mode, uint8_t targetVfo) = 0;
  // Enter in Normal mode with a staged command: send it. It is already
  // unstaged.
  virtual void onStagedCommandSend(const char* cmd) = 0;

  // '#': everything was cancelled. With nothing to cancel, '#' gives
  // onUnassigned("CLEAR") instead.
  virtual void onClear() = 0;
};

class KeypadInput {
 public:
  // A second short press of the same key within this time is a double click.
  static constexpr uint32_t kDoubleClickMs = 220;

  explicit KeypadInput(KeypadInputListener& listener) : listener_(listener) {}

  void onKey(char key, KeyGesture gesture, uint32_t nowMs);
  // Runs a deferred short action once the double-click wait has passed.
  void poll(uint32_t nowMs);

  // Called by keymap actions.
  void beginEntry(InputMode mode, uint8_t targetVfo = KEYPAD_VFO_CURRENT);
  void beginProfileSelect() { beginEntry(InputMode::ProfileSelect); }
  void beginModeSelect(uint8_t targetVfo);
  // Keeps cmd for Enter in Normal mode, replacing any staged command. '#'
  // drops it. cmd is copied, up to kMaxStagedCommand characters.
  void stageCommand(const char* cmd);

  uint8_t bank() const { return bank_; }
  void setBank(uint8_t bank) { bank_ = bank; }

  InputMode mode() const { return mode_; }
  const char* digits() const { return digits_.c_str(); }
  uint8_t entryTargetVfo() const { return entryVfo_; }
  // Mode select with a mode picked, waiting for Enter.
  bool stagedModeActive() const { return mode_ == InputMode::ModeSelect && stagedMode_ != kNoMode; }
  bool doubleClickPending() const { return pending_.active; }
  bool hasStagedCommand() const { return stagedCommand_[0] != '\0'; }
  const char* stagedCommand() const { return stagedCommand_; }

  static constexpr size_t kMaxStagedCommand = 23;

 private:
  // Keys whose hold was handled. Their release is swallowed.
  class HoldTracker {
   public:
    void set(char key);
    // Clears key's bit. Returns true when it was set.
    bool release(char key);

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

  static constexpr uint8_t kNoMode = 0xFF;

  struct DoubleClick {
    bool active = false;
    uint8_t bank = 0;
    char key = 0;
    uint32_t atMs = 0;
  };

  const GlobalKey* globalKey(char key) const;
  void sayBank();
  void beginBankSelect();
  void held(char key);
  void runPending();
  void releasedNormal(char key, uint32_t nowMs);
  void releasedModeSelect(char key);
  void releasedEntry(const EntrySpec& entry, char key);
  bool takesDigit(const EntrySpec& entry, char key) const;
  void enter();
  void clearAll();
  void runShortOrUnassigned(uint8_t bank, char key);
  // "<mode> <what>" to onUnassigned, e.g. "FREQ 6".
  void reportUnassigned(const char* mode, const char* what);

  KeypadInputListener& listener_;
  uint8_t bank_ = 1;
  InputMode mode_ = InputMode::Normal;
  // The longest entry is a frequency: 12 characters, plus a point the old code
  // accepted even after them.
  DigitBuffer<13> digits_;
  // Target VFO of a frequency entry and mode select.
  uint8_t entryVfo_ = KEYPAD_VFO_CURRENT;
  // The mode picked in mode select, or kNoMode.
  uint8_t stagedMode_ = kNoMode;
  // The command waiting for Enter in Normal mode, or empty.
  char stagedCommand_[kMaxStagedCommand + 1] = "";
  HoldTracker holds_;
  DoubleClick pending_;
};
