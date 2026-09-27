#include "keypad_input.h"

#include <stdio.h>
#include <string.h>

namespace {

// Bit positions for the hold tracker.
constexpr char kTrackedKeys[] = "0123456789ABCD*#";

int holdBit(char key) {
  if (key == '\0') return -1;
  const char* p = strchr(kTrackedKeys, key);
  return p ? (int)(p - kTrackedKeys) : -1;
}

bool isDigit(char key) { return key >= '0' && key <= '9'; }

// The key keypadActiveKey() reports; empty while no action runs.
char g_activeKey[20] = "";

// Names the key in g_activeKey while one action runs.
class ActiveKey {
 public:
  ActiveKey(uint8_t bank, char key, const char* gesture) {
    snprintf(g_activeKey, sizeof(g_activeKey), "BANK%u %c %s", (unsigned)bank, key, gesture);
  }
  ~ActiveKey() { g_activeKey[0] = '\0'; }
};

// Bank select keeps the last digit typed; profile select takes up to two.
// The frequency took a point even after 12 digits in the old code, so the
// point does not check maxLen.
constexpr EntrySpec kEntries[] = {
    // mode                     name           len  replaces zero  fraction unit
    {InputMode::BankSelect,     "BANK SELECT", 1,   true,    false, 0,      ""},
    {InputMode::ProfileSelect,  "PROFILE",     2,   false,   false, 0,      ""},
    {InputMode::FreqEntry,      "FREQ",        12,  false,   true,  5,      ""},
    {InputMode::RfPowerEntry,   "RFPOWER",     3,   false,   true,  0,      " W"},
    {InputMode::CivAddrEntry,   "CIVADDR",     3,   false,   true,  0,      ""},
    {InputMode::RptOffsetEntry, "RPTSHIFT",    4,   false,   true,  0,      " kHz"},
    {InputMode::CtcssEntry,     "CTCSS",       4,   false,   true,  0,      ""},
    {InputMode::DcsEntry,       "DCS",         3,   false,   true,  0,      ""},
};

}  // namespace

const char* keypadActiveKey() { return g_activeKey[0] ? g_activeKey : nullptr; }

const EntrySpec* keypadEntrySpec(InputMode mode) {
  for (const EntrySpec& e : kEntries) {
    if (e.mode == mode) return &e;
  }
  return nullptr;
}

void KeypadInput::HoldTracker::set(char key) {
  const int bit = holdBit(key);
  if (bit >= 0) bits_ |= (uint16_t)(1u << bit);
}

bool KeypadInput::HoldTracker::release(char key) {
  const int bit = holdBit(key);
  if (bit < 0) return false;
  const uint16_t mask = (uint16_t)(1u << bit);
  const bool wasSet = (bits_ & mask) != 0;
  bits_ &= (uint16_t)~mask;
  return wasSet;
}

void KeypadInput::onKey(char key, KeyGesture gesture, uint32_t nowMs) {
  listener_.onActivity(gesture == KeyGesture::Pressed);
  // A short waiting for a double click answers before any later key: when its
  // wait is over, or when another key goes down. '#' cancels it instead.
  poll(nowMs);
  if (gesture == KeyGesture::Pressed && pending_.active && key != '#' &&
      (key != pending_.key || bank_ != pending_.bank)) {
    runPending();
  }
  if (gesture == KeyGesture::Held) {
    held(key);
    return;
  }
  if (gesture != KeyGesture::Released) return;

  // The '*' hold that opened bank select ends here. Its release is silent even
  // when the hold was not tracked.
  if (mode_ == InputMode::BankSelect && key == '*') {
    holds_.release(key);
    return;
  }
  // The release that ends a handled hold is not a short press.
  if (holds_.release(key)) return;

  if (const GlobalKey* g = globalKey(key)) {
    (this->*g->onShort)();
    return;
  }
  switch (mode_) {
    case InputMode::Normal:
      releasedNormal(key, nowMs);
      return;
    case InputMode::ModeSelect:
      releasedModeSelect(key);
      return;
    case InputMode::BankSelect:
    case InputMode::ProfileSelect:
    case InputMode::FreqEntry:
    case InputMode::RfPowerEntry:
    case InputMode::CivAddrEntry:
    case InputMode::RptOffsetEntry:
    case InputMode::CtcssEntry:
    case InputMode::DcsEntry:
      releasedEntry(*keypadEntrySpec(mode_), key);
      return;
  }
}

void KeypadInput::poll(uint32_t nowMs) {
  if (!pending_.active || (uint32_t)(nowMs - pending_.atMs) <= kDoubleClickMs) return;
  runPending();
}

void KeypadInput::runPending() {
  pending_.active = false;
  listener_.onActivity(false);
  runShortOrReject(listener_.keyBinding(pending_.bank, pending_.key), pending_.bank, pending_.key);
}

void KeypadInput::beginEntry(InputMode mode, TargetVfo targetVfo) {
  mode_ = mode;
  digits_.clear();
  entryVfo_ = targetVfo;
}

void KeypadInput::beginModeSelect(TargetVfo targetVfo) {
  beginEntry(InputMode::ModeSelect, targetVfo);
  stagedMode_ = kNoMode;
}

void KeypadInput::stageCommand(const char* cmd) {
  snprintf(stagedCommand_, sizeof(stagedCommand_), "%s", cmd ? cmd : "");
}

// The keys that belong to the state machine rather than the keymap, the same
// on every bank, or nullptr. A key that works in Normal mode only is input for
// the other modes and is not global there.
const KeypadInput::GlobalKey* KeypadInput::globalKey(char key) const {
  static const GlobalKey kKeys[] = {
      {'*', "BANK", true, &KeypadInput::sayBank, &KeypadInput::beginBankSelect},
      {'D', "ENTER", false, &KeypadInput::enter, nullptr},
      {'#', "CLEAR", false, &KeypadInput::clearAll, nullptr},
  };
  for (const GlobalKey& g : kKeys) {
    if (g.key == key && (!g.normalOnly || mode_ == InputMode::Normal)) return &g;
  }
  return nullptr;
}

void KeypadInput::sayBank() { listener_.onBankQuery(bank_); }

void KeypadInput::beginBankSelect() {
  mode_ = InputMode::BankSelect;
  digits_.clear();
  listener_.onBankSelectStart();
}

// Bank, profile and mode select and the entries ignore holds; their keys act on
// release. In Normal mode a key with no long action beeps now, and its release
// is swallowed like after a long action.
void KeypadInput::held(char key) {
  if (mode_ != InputMode::Normal) return;
  holds_.set(key);
  const GlobalKey* g = globalKey(key);
  bool handled;
  if (g) {
    handled = g->onHold != nullptr;
    if (handled) (this->*g->onHold)();
  } else {
    const KeyAction action = listener_.keyBinding(bank_, key).holdAction;
    handled = action != nullptr;
    if (handled) runAction(action, bank_, key, "LONG");
  }
  if (handled) {
    // Only the waiting key itself can still be waiting here: its long action
    // replaces its short.
    pending_.active = false;
    return;
  }
  // The beep is no action: the key's short still waiting for a double click
  // stays.
  char label[24];
  if (g) {
    snprintf(label, sizeof(label), "%s LONG", g->name);
  } else {
    snprintf(label, sizeof(label), "BANK%u %c LONG", (unsigned)bank_, key);
  }
  listener_.onRejected(label);
}

void KeypadInput::releasedNormal(char key, uint32_t nowMs) {
  const KeyBinding binding = listener_.keyBinding(bank_, key);
  if (pending_.active && pending_.bank == bank_ && pending_.key == key &&
      (uint32_t)(nowMs - pending_.atMs) <= kDoubleClickMs) {
    pending_.active = false;
    if (binding.doubleAction) {
      runAction(binding.doubleAction, bank_, key, "DOUBLE");
      return;
    }
  }
  if (binding.waitsForDouble) {
    // Any other key still waiting has run when this key went down.
    pending_.active = true;
    pending_.bank = bank_;
    pending_.key = key;
    pending_.atMs = nowMs;
    return;
  }
  runShortOrReject(binding, bank_, key);
}

// Mode select takes a key that picks a mode; another one replaces it, and
// Enter applies it. Any other key beeps and the mode stays; only '#' cancels.
void KeypadInput::releasedModeSelect(char key) {
  uint8_t mode = 0;
  if (listener_.onModeDigit(key, mode)) {
    stagedMode_ = mode;
    return;
  }
  const char label[2] = {key, '\0'};
  reportRejected("MODE SELECT", label);
}

// Bank select, profile select and the entries: a digit the entry takes, or
// the frequency point. Any other key beeps and the entry stays.
void KeypadInput::releasedEntry(const EntrySpec& entry, char key) {
  if (key == '*' && entry.maxFraction > 0) {
    // One point only, and not first.
    if (!digits_.empty() && digits_.indexOf('*') < 0) {
      digits_.push(key);
      listener_.onDigitAccepted(entry, key, digits_.c_str());
    } else {
      reportRejected(entry.name, "POINT");
    }
    return;
  }
  if (!takesDigit(entry, key)) {
    const char label[2] = {key, '\0'};
    reportRejected(entry.name, label);
    return;
  }
  if (digits_.length() >= entry.maxLen) digits_.clear();  // replaces
  digits_.push(key);
  listener_.onDigitAccepted(entry, key, digits_.c_str());
}

bool KeypadInput::takesDigit(const EntrySpec& entry, char key) const {
  if (!isDigit(key)) return false;
  const size_t len = digits_.length();
  const bool full = len >= entry.maxLen;
  if (full && !entry.replaces) return false;
  // A full entry that replaces starts over with this digit.
  if ((len == 0 || full) && key == '0' && !entry.leadingZero) return false;
  const int point = digits_.indexOf('*');
  return point < 0 || (int)len - point - 1 < entry.maxFraction;
}

// Enter ('D'). Entries and selections commit; in Normal mode a staged command
// is sent. With no mode, bank or digit chosen yet, Enter beeps and the mode
// stays: only Clear cancels.
void KeypadInput::enter() {
  if (mode_ == InputMode::Normal) {
    if (!hasStagedCommand()) {
      listener_.onRejected("ENTER");
      return;
    }
    char cmd[sizeof(stagedCommand_)];
    memcpy(cmd, stagedCommand_, sizeof(cmd));
    stagedCommand_[0] = '\0';
    listener_.onStagedCommandSend(cmd);
    return;
  }
  if (mode_ == InputMode::ModeSelect) {
    if (stagedMode_ == kNoMode) {
      reportRejected("MODE SELECT", "D");
      return;
    }
    const TargetVfo targetVfo = entryVfo_;
    mode_ = InputMode::Normal;
    entryVfo_ = TargetVfo::Current;
    listener_.onModeCommit(stagedMode_, targetVfo);
    return;
  }

  if (digits_.empty()) {
    reportRejected(keypadEntrySpec(mode_)->name, "D");
    return;
  }
  const InputMode mode = mode_;
  const DigitBuffer<13> digits = digits_;
  const TargetVfo targetVfo = entryVfo_;
  if (mode == InputMode::BankSelect) bank_ = (uint8_t)(digits.c_str()[0] - '0');
  mode_ = InputMode::Normal;
  digits_.clear();
  entryVfo_ = TargetVfo::Current;
  listener_.onCommit(mode, digits.c_str(), targetVfo);
}

// Clear ('#') cancels every mode, entry, staged mode, staged command and
// waiting double click, and beeps when there is none. Keys still held keep their
// hold, so their release stays swallowed.
void KeypadInput::clearAll() {
  if (mode_ == InputMode::Normal && !pending_.active && !hasStagedCommand()) {
    listener_.onRejected("CLEAR");
    return;
  }
  mode_ = InputMode::Normal;
  digits_.clear();
  entryVfo_ = TargetVfo::Current;
  stagedCommand_[0] = '\0';
  pending_.active = false;
  listener_.onClear();
}

void KeypadInput::runAction(KeyAction action, uint8_t bank, char key, const char* gesture) {
  ActiveKey named(bank, key, gesture);
  action();
}

void KeypadInput::runShortOrReject(const KeyBinding& binding, uint8_t bank, char key) {
  if (binding.shortAction) {
    runAction(binding.shortAction, bank, key, "SHORT");
    return;
  }
  char label[16];
  snprintf(label, sizeof(label), "BANK%u %c", (unsigned)bank, key);
  listener_.onRejected(label);
}

void KeypadInput::reportRejected(const char* mode, const char* what) {
  char label[24];
  snprintf(label, sizeof(label), "%s %s", mode, what);
  listener_.onRejected(label);
}
