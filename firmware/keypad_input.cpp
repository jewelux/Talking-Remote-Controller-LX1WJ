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

}  // namespace

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

  switch (mode_) {
    case InputMode::Normal:
      releasedNormal(key, nowMs);
      return;
    case InputMode::BankSelect:
      releasedBankSelect(key);
      return;
    case InputMode::ProfileSelect:
      releasedProfileSelect(key);
      return;
    case InputMode::ModeSelect:
    case InputMode::ModeStaged:
      releasedModeSelect(key);
      return;
    default:
      releasedEntry(key);
      return;
  }
}

void KeypadInput::poll(uint32_t nowMs) {
  if (!pending_.active || (uint32_t)(nowMs - pending_.atMs) <= kDoubleClickMs) return;
  pending_.active = false;
  listener_.onActivity(false);
  runShortOrUnassigned(pending_.bank, pending_.key);
}

void KeypadInput::beginEntry(InputMode mode, uint8_t targetVfo) {
  mode_ = mode;
  digits_.clear();
  entryVfo_ = targetVfo;
}

void KeypadInput::beginModeSelect(uint8_t targetVfo) {
  beginEntry(InputMode::ModeSelect, targetVfo);
}

// Bank, profile and mode select and the entries ignore holds; their keys act on
// release.
void KeypadInput::held(char key) {
  if (mode_ != InputMode::Normal) return;
  bool handled = false;
  if (key == '*') {
    mode_ = InputMode::BankSelect;
    digits_.clear();
    listener_.onBankSelectStart();
    handled = true;
  } else if (key == 'D' || key == '#') {
    handled = false;
  } else {
    handled = listener_.runHold(bank_, key);
  }
  if (handled) {
    holds_.set(key);
    pending_.active = false;
  }
}

void KeypadInput::releasedNormal(char key, uint32_t nowMs) {
  if (key == 'D') {
    enter();
    return;
  }
  if (key == '#') {
    clearAll();
    return;
  }
  if (key == '*') {
    listener_.onBankQuery(bank_);
    return;
  }
  if (pending_.active && pending_.bank == bank_ && pending_.key == key &&
      (uint32_t)(nowMs - pending_.atMs) <= kDoubleClickMs) {
    pending_.active = false;
    if (listener_.runDoubleClick(bank_, key)) return;
  }
  if (listener_.wantsDoubleClick(bank_, key)) {
    // A key that waits replaces any other key still waiting; that one is
    // dropped, as before the state machine.
    pending_.active = true;
    pending_.bank = bank_;
    pending_.key = key;
    pending_.atMs = nowMs;
    return;
  }
  runShortOrUnassigned(bank_, key);
}

// Mode select takes a key that picks a mode and stages it; Enter applies the
// staged mode, and another mode digit replaces it. Any other key beeps and the
// mode stays; only '#' cancels.
void KeypadInput::releasedModeSelect(char key) {
  if (key == '#') {
    clearAll();
    return;
  }
  if (key == 'D') {
    if (mode_ == InputMode::ModeStaged) {
      enter();
    } else {
      reportUnassigned("MODE SELECT ", key);
    }
    return;
  }
  uint8_t mode = 0;
  if (!listener_.onModeDigit(key, mode)) return;
  mode_ = InputMode::ModeStaged;
  stagedMode_ = mode;
}

void KeypadInput::releasedBankSelect(char key) {
  if (key >= '1' && key <= '9') {
    digits_.clear();
    digits_.push(key);
    listener_.onDigitAccepted(mode_, key, digits_.c_str());
    return;
  }
  if (key == 'D') {
    enter();
    return;
  }
  if (key == '#') {
    clearAll();
    return;
  }
  reportUnassigned("BANK SELECT ", key);
}

void KeypadInput::releasedProfileSelect(char key) {
  if (isDigit(key)) {
    if (digits_.length() >= 2 || (digits_.empty() && key == '0')) {
      reportUnassigned("PROFILE ", key);
      return;
    }
    digits_.push(key);
    listener_.onDigitAccepted(mode_, key, digits_.c_str());
    return;
  }
  if (key == 'D') {
    enter();
    return;
  }
  if (key == '#') {
    clearAll();
    return;
  }
  reportUnassigned("PROFILE ", key);
}

void KeypadInput::releasedEntry(char key) {
  if (key == 'D') {
    enter();
    return;
  }
  if (key == '#') {
    clearAll();
    return;
  }
  if (mode_ == InputMode::FreqEntry && key == '*') {
    // '*' is the decimal point: one only, and not first.
    if (!digits_.empty() && digits_.indexOf('*') < 0) {
      digits_.push(key);
      listener_.onDigitAccepted(mode_, key, digits_.c_str());
    } else {
      listener_.onUnassigned("FREQ POINT");
    }
    return;
  }
  if (!isDigit(key)) {
    reportUnassigned("ENTRY ", key);
    return;
  }
  if (entryTakesDigit()) {
    digits_.push(key);
    listener_.onDigitAccepted(mode_, key, digits_.c_str());
    return;
  }
  switch (mode_) {
    case InputMode::FreqEntry:
      reportUnassigned("FREQ ", key);
      return;
    case InputMode::RfPowerEntry:
      reportUnassigned("RFPOWER ", key);
      return;
    case InputMode::CivAddrEntry:
      reportUnassigned("CIVADDR ", key);
      return;
    default:
      reportUnassigned("BANK6 ENTRY ", key);
      return;
  }
}

bool KeypadInput::entryTakesDigit() const {
  const size_t len = digits_.length();
  switch (mode_) {
    case InputMode::FreqEntry: {
      // Up to 12 characters, and up to 5 digits after the point.
      const int point = digits_.indexOf('*');
      const bool fractionFull = point >= 0 && (int)len - point - 1 >= 5;
      return !fractionFull && len < 12;
    }
    case InputMode::RfPowerEntry:
    case InputMode::CivAddrEntry:
    case InputMode::DcsEntry:
      return len < 3;
    case InputMode::RptOffsetEntry:
    case InputMode::CtcssEntry:
      return len < 4;
    default:
      return false;
  }
}

// Enter ('D'). Entries and selections commit; in Normal mode a staged command
// is sent.
void KeypadInput::enter() {
  if (mode_ == InputMode::Normal) {
    if (!listener_.sendStagedCommand()) listener_.onUnassigned("ENTER");
    return;
  }
  if (mode_ == InputMode::ModeStaged) {
    const uint8_t targetVfo = entryVfo_;
    mode_ = InputMode::Normal;
    entryVfo_ = KEYPAD_VFO_CURRENT;
    listener_.onModeCommit(stagedMode_, targetVfo);
    return;
  }

  const InputMode mode = mode_;
  const DigitBuffer<13> digits = digits_;
  const uint8_t targetVfo = entryVfo_;
  if (mode == InputMode::BankSelect && !digits.empty()) bank_ = (uint8_t)(digits.c_str()[0] - '0');
  mode_ = InputMode::Normal;
  digits_.clear();
  entryVfo_ = KEYPAD_VFO_CURRENT;
  listener_.onCommit(mode, digits.c_str(), targetVfo);
}

// Clear ('#') cancels every mode, entry, staged mode and waiting double click.
// Keys still held keep their hold, so their release stays swallowed.
void KeypadInput::clearAll() {
  mode_ = InputMode::Normal;
  digits_.clear();
  entryVfo_ = KEYPAD_VFO_CURRENT;
  pending_.active = false;
  listener_.onClear();
}

void KeypadInput::runShortOrUnassigned(uint8_t bank, char key) {
  if (listener_.runShort(bank, key)) return;
  char label[16];
  snprintf(label, sizeof(label), "BANK%u %c", (unsigned)bank, key);
  listener_.onUnassigned(label);
}

void KeypadInput::reportUnassigned(const char* prefix, char key) {
  char label[24];
  snprintf(label, sizeof(label), "%s%c", prefix, key);
  listener_.onUnassigned(label);
}
