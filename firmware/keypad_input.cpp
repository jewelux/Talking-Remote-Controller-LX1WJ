#include "keypad_input.h"

#include <stdio.h>
#include <string.h>

namespace {

// Bit positions in a KeySet.
constexpr char kTrackedKeys[] = "0123456789ABCD*#";

int keyBit(char key) {
  if (key == '\0') return -1;
  const char* p = strchr(kTrackedKeys, key);
  return p ? (int)(p - kTrackedKeys) : -1;
}

bool isDigit(char key) { return key >= '0' && key <= '9'; }

bool isFocusKey(char key) { return key == 'A' || key == 'B' || key == 'C'; }

// The key keypadActiveKey() reports; empty while no action runs. The focus
// keypadActiveFocus() reports with it.
char g_activeKey[24] = "";
Setting g_activeFocus = Setting::None;
bool g_activeFocusNew = false;

// Names the key and its bank's focus while one action runs.
class ActiveKey {
 public:
  ActiveKey(uint8_t bank, char key, const char* gesture, Setting focus, bool focusNew) {
    snprintf(g_activeKey, sizeof(g_activeKey), "BANK%u %c %s", (unsigned)bank, key, gesture);
    g_activeFocus = focus;
    g_activeFocusNew = focusNew;
  }
  ~ActiveKey() {
    g_activeKey[0] = '\0';
    g_activeFocus = Setting::None;
    g_activeFocusNew = false;
  }
};

// Bank select commits on its one digit; profile select takes up to two.
// The frequency took a point even after 12 digits in the old code, so the
// point does not check maxLen.
constexpr EntrySpec kEntries[] = {
    // mode                     name           len  commits zero  fraction unit
    {InputMode::BankSelect,     "BANK SELECT", 1,   true,   false, 0,      ""},
    {InputMode::ProfileSelect,  "PROFILE",     2,   false,  false, 0,      ""},
    {InputMode::FreqEntry,      "FREQ",        12,  false,  true,  5,      ""},
    {InputMode::CivAddrEntry,   "CIVADDR",     3,   false,  true,  0,      ""},
    {InputMode::RptOffsetEntry, "RPTSHIFT",    4,   false,  true,  0,      " kHz"},
    {InputMode::MenuEntry,      "MENU",        2,   false,  false, 0,      ""},
    {InputMode::RowEntry,       "ROW",         2,   false,  false, 0,      ""},
};

}  // namespace

const char* keypadActiveKey() { return g_activeKey[0] ? g_activeKey : nullptr; }

Setting keypadActiveFocus() { return g_activeFocus; }

bool keypadActiveFocusIsNew() { return g_activeFocusNew; }

const EntrySpec* keypadEntrySpec(InputMode mode) {
  for (const EntrySpec& e : kEntries) {
    if (e.mode == mode) return &e;
  }
  return nullptr;
}

void KeypadInput::KeySet::set(char key) {
  const int bit = keyBit(key);
  if (bit >= 0) bits_ |= (uint16_t)(1u << bit);
}

bool KeypadInput::KeySet::has(char key) const {
  const int bit = keyBit(key);
  return bit >= 0 && (bits_ & (uint16_t)(1u << bit)) != 0;
}

bool KeypadInput::KeySet::hasOtherThan(char key) const {
  const int bit = keyBit(key);
  const uint16_t own = bit >= 0 ? (uint16_t)(1u << bit) : (uint16_t)0;
  return (bits_ & (uint16_t)~own) != 0;
}

bool KeypadInput::KeySet::isOnly(char key) const {
  const int bit = keyBit(key);
  return bit >= 0 && bits_ == (uint16_t)(1u << bit);
}

bool KeypadInput::KeySet::release(char key) {
  const int bit = keyBit(key);
  if (bit < 0) return false;
  const uint16_t mask = (uint16_t)(1u << bit);
  const bool wasSet = (bits_ & mask) != 0;
  bits_ &= (uint16_t)~mask;
  return wasSet;
}

void KeypadInput::onKey(char key, KeyGesture gesture, uint32_t nowMs) {
  lastKeyMs_ = nowMs;
  listener_.onActivity(gesture == KeyGesture::Pressed);
  if (coPressed(key, gesture)) return;
  // The second press is over: its release is judged by the double-click time.
  if (gesture == KeyGesture::Released && key == pending_.key) pending_.repressed = false;
  // A short waiting for a double click answers before any later key: when its
  // wait is over, or when another key goes down. '#' cancels it instead.
  poll(nowMs);
  if (gesture == KeyGesture::Pressed && pending_.active && key != '#') {
    if (key != pending_.key || keyBank() != pending_.bank) {
      runPending();
    } else {
      pending_.repressed = true;
    }
  }
  if (gesture == KeyGesture::Pressed) {
    // Normal mode waits for the release: it tells short, long and double apart.
    if (mode_ != InputMode::Normal) pressedInMode(key);
    return;
  }
  if (gesture == KeyGesture::Held) {
    if (key == valueStepKey_) {
      valueStepKey_ = 0;
      if (mode_ == InputMode::ValueEntry) stepValue(key, value_.spec().bigSteps);
      return;
    }
    held(key);
    return;
  }
  if (gesture != KeyGesture::Released) return;
  // A B or C short in value entry: one step, unless a key ended the entry meanwhile.
  if (key == valueStepKey_) {
    valueStepKey_ = 0;
    if (mode_ == InputMode::ValueEntry) stepValue(key, 1);
  }

  // The '*' hold that opened bank select ends here. Its release is silent even
  // when the hold was not tracked.
  if (mode_ == InputMode::BankSelect && key == '*') {
    swallowed_.release(key);
    return;
  }
  // The release that ends a handled hold, or a press an entry took, is not a
  // short press.
  if (swallowed_.release(key)) return;
  // A key that went down in Normal mode before another key opened an entry
  // was never typed into it.
  if (mode_ != InputMode::Normal) return;

  if (const GlobalKey* g = globalKey(key)) {
    (this->*g->onShort)();
    oneShotBank_ = 0;
    return;
  }
  releasedNormal(key, nowMs);
}

// Two keys down at once are no gesture. The second press beeps, drops a waiting
// short and a one-shot bank and starts a co-press: every event of every key,
// the releases too, is ignored until all keys are up. What an entry or
// selection already took stays. The one exception: a digit pressed while the
// '*' that opened bank select is still held picks a one-shot bank.
bool KeypadInput::coPressed(char key, KeyGesture gesture) {
  if (gesture == KeyGesture::Pressed) {
    const bool oneShotChord = mode_ == InputMode::BankSelect && isDigit(key) && down_.isOnly('*');
    if (!coPress_ && down_.hasOtherThan(key) && !oneShotChord) {
      coPress_ = true;
      pending_.active = false;
      oneShotBank_ = 0;
      valueStepKey_ = 0;
      listener_.onRejected("TWO KEYS");
    }
    down_.set(key);
  }
  const bool ignored = coPress_;
  if (gesture == KeyGesture::Released) {
    down_.release(key);
    if (ignored) swallowed_.release(key);
    if (!down_.any()) coPress_ = false;
  }
  return ignored;
}

// Bank, profile and choice select and the entries have no long or double
// action, so their keys act as soon as they go down. The hold and release
// that follow do nothing, even once the key has ended the mode. Except B and C
// in value entry: they wait, and held they take a big step.
void KeypadInput::pressedInMode(char key) {
  swallowed_.set(key);
  // The '*' that opened bank select, pressed again: silent.
  if (mode_ == InputMode::BankSelect && key == '*') return;
  if (const GlobalKey* g = globalKey(key)) {
    (this->*g->onShort)();
    return;
  }
  if (mode_ == InputMode::ChoiceSelect) {
    pressedChoice(key);
    return;
  }
  if (mode_ == InputMode::ValueEntry) {
    pressedValue(key);
    return;
  }
  pressedEntry(*keypadEntrySpec(mode_), key);
}

void KeypadInput::poll(uint32_t nowMs) {
  // A waiting short runs once its time is over. While its second press is down,
  // that press's release or hold decides, however long it takes.
  if (pending_.active && (uint32_t)(nowMs - pending_.atMs) > kDoubleClickMs && !pending_.repressed) {
    runPending();
  }
  if (mode_ != InputMode::Normal && (uint32_t)(nowMs - lastKeyMs_) > kEntryTimeoutMs) timeOut();
}

bool KeypadInput::secondPressDown(char key) const {
  return pending_.active && pending_.repressed && key == pending_.key && keyBank() == pending_.bank;
}

void KeypadInput::runPending() {
  pending_.active = false;
  oneShotBank_ = 0;
  listener_.onActivity(false);
  runShortOrReject(bindingOf(pending_.bank, pending_.key), pending_.bank, pending_.key);
}

void KeypadInput::beginEntry(InputMode mode, TargetVfo targetVfo) {
  mode_ = mode;
  digits_.clear();
  entryVfo_ = targetVfo;
}

void KeypadInput::beginChoiceSelect(const ChoiceSpec& spec, TargetVfo targetVfo, const EntrySeed& seed) {
  beginEntry(InputMode::ChoiceSelect, targetVfo);
  choice_ = &spec;
  choiceKnown_ = seed.known;
  choiceStaged_ = seed.known && seed.staged;
  choiceValue_ = (uint8_t)seed.value;
}

bool KeypadInput::nudgeChoice(const ChoiceSpec& spec, const EntrySeed& seed, int8_t dir,
                              TargetVfo targetVfo) {
  beginChoiceSelect(spec, targetVfo, seed);
  if (seed.known && walkChoice(dir > 0 ? 'B' : 'C', dir)) return true;
  mode_ = InputMode::Normal;
  entryVfo_ = TargetVfo::Current;
  return false;
}

Setting KeypadInput::focus(uint8_t bank) const {
  const Setting f = bank < 10 ? focus_[bank] : Setting::None;
  return f == Setting::Home ? listener_.homeFocus(bank) : f;
}

void KeypadInput::resetFocus() {
  for (Setting& f : focus_) f = Setting::Home;
  named_ = false;
}

void KeypadInput::beginValueEntry(const ValueSpec& spec, const EntrySeed& seed, TargetVfo targetVfo) {
  beginEntry(InputMode::ValueEntry, targetVfo);
  valueStepKey_ = 0;
  value_.begin(spec, seed);
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

// Bank, profile and choice select and the entries ignore holds; their keys act on
// press. So does a key whose press one of them took, even when that press
// ended the mode. In Normal mode a key with no long action beeps now, and its
// release is swallowed like after a long action.
void KeypadInput::held(char key) {
  if (mode_ != InputMode::Normal || swallowed_.has(key)) return;
  swallowed_.set(key);
  const GlobalKey* g = globalKey(key);
  // A hold right after a short press of the same key is a double hold, a
  // gesture of its own: with no action it beeps, it never turns into a long
  // press, and the short it follows does not run.
  const bool doubleHold = !g && secondPressDown(key);
  const uint8_t bank = keyBank();
  bool handled;
  if (g) {
    handled = g->onHold != nullptr;
    if (handled) (this->*g->onHold)();
  } else {
    const KeyBinding binding = bindingOf(bank, key);
    const KeyAction action = doubleHold ? binding.doubleHoldAction : binding.holdAction;
    handled = action != nullptr;
    if (handled) runAction(action, bank, key, doubleHold ? "DOUBLE LONG" : "LONG", binding.focus);
  }
  if (handled || doubleHold) {
    // Only the waiting key itself can still be waiting here: its long or
    // double-hold action replaces its short.
    pending_.active = false;
  }
  // A one-shot bank ends with this key, unless a short still waits on it.
  if (!pending_.active) oneShotBank_ = 0;
  if (handled) return;
  // The beep is no action: a short still waiting for a double click stays. Only
  // a '#' hold gets here with one waiting.
  char label[24];
  if (g) {
    snprintf(label, sizeof(label), "%s LONG", g->name);
  } else {
    snprintf(label, sizeof(label), "BANK%u %c %s", (unsigned)bank, key,
             doubleHold ? "DOUBLE LONG" : "LONG");
  }
  listener_.onRejected(label);
}

void KeypadInput::releasedNormal(char key, uint32_t nowMs) {
  const uint8_t bank = keyBank();
  const KeyBinding binding = bindingOf(bank, key);
  if (pending_.active && pending_.bank == bank && pending_.key == key &&
      (uint32_t)(nowMs - pending_.atMs) <= kDoubleClickMs) {
    pending_.active = false;
    if (binding.doubleAction) {
      oneShotBank_ = 0;
      runAction(binding.doubleAction, bank, key, "DOUBLE", binding.focus);
      return;
    }
  }
  if (binding.waitsForDouble) {
    // Any other key still waiting has run when this key went down. A one-shot
    // bank stays until this short has run.
    pending_.active = true;
    pending_.repressed = false;
    pending_.bank = bank;
    pending_.key = key;
    pending_.atMs = nowMs;
    return;
  }
  oneShotBank_ = 0;
  runShortOrReject(binding, bank, key);
}

// Choice select takes a key that picks a value; another one replaces it, and
// Enter applies it. Any other key beeps and the choice stays; only '#' cancels.
void KeypadInput::pressedChoice(char key) {
  if (key == 'A' && (choiceStaged_ || choiceKnown_)) {
    choice_->picked(key, choiceValue_);
    return;
  }
  if ((key == 'B' || key == 'C') && (choiceStaged_ || choiceKnown_)) {
    const int8_t dir = key == 'B' ? 1 : -1;
    if (!walkChoice(key, dir)) listener_.onLimit(choice_->name, dir > 0);
    return;
  }
  uint8_t value = 0;
  if (choiceValue(key, value)) {
    choiceStaged_ = true;
    choiceValue_ = value;
    choice_->picked(key, value);
    return;
  }
  const char label[2] = {key, '\0'};
  reportRejected(choice_->name, label);
}

// From the value picked or the radio's value, to the next candidate the choice
// takes. A value that is none of them (a mode the keys do not pick) walks from
// before the first or after the last. A choice that cycles goes on from the
// other end; it stops only when no other candidate is left.
bool KeypadInput::walkChoice(char key, int8_t dir) {
  int count = 0;
  int at = -1;
  for (char k; (k = choiceKeyAt(count)) != 0; ++count) {
    uint8_t v = 0;
    if (at < 0 && choiceValue(k, v) && v == choiceValue_) at = count;
  }
  if (at < 0) at = dir > 0 ? -1 : count;
  for (int n = 1; n <= count; ++n) {
    int i = at + dir * n;
    if (choice_->cycles) i = (i % count + count) % count;
    else if (i < 0 || i >= count) break;
    if (i == at) break;
    uint8_t v = 0;
    if (!choiceValue(choiceKeyAt(i), v)) continue;
    choiceStaged_ = true;
    choiceValue_ = v;
    choice_->picked(key, v);
    return true;
  }
  return false;
}

char KeypadInput::choiceKeyAt(int at) const {
  if (choice_->options) return at < choice_->optionCount ? choice_->options[at].key : 0;
  return at < 10 ? (char)('0' + at) : 0;
}

// Value entry: digits and '*' type, A says the value so far, B and C step it
// once released, or by its big step once held.
void KeypadInput::pressedValue(char key) {
  const ValueSpec& spec = value_.spec();
  if (key == 'A') {
    int32_t v = 0;
    if (value_.current(v)) {
      listener_.onValueSay(spec, v);
      return;
    }
  } else if (key == 'B' || key == 'C') {
    valueStepKey_ = key;
    return;
  } else if (value_.type(key)) {
    listener_.onValueDigit(spec, key, value_.digits());
    return;
  }
  const char label[2] = {key, '\0'};
  reportRejected(spec.name, label);
}

void KeypadInput::stepValue(char key, uint8_t times) {
  const ValueSpec& spec = value_.spec();
  const int8_t dir = key == 'B' ? 1 : -1;
  switch (value_.step(dir, times)) {
    case ValueEntry::Step::Moved: {
      int32_t v = 0;
      value_.current(v);
      listener_.onValueStepped(spec, v);
      return;
    }
    case ValueEntry::Step::AtLimit: listener_.onLimit(spec.name, dir > 0); return;
    case ValueEntry::Step::NoValue: break;
  }
  const char label[2] = {key, '\0'};
  reportRejected(spec.name, label);
}

bool KeypadInput::choiceValue(char key, uint8_t& value) const {
  if (!choice_->options) return choice_->pick(key, value);
  for (uint8_t i = 0; i < choice_->optionCount; ++i) {
    if (choice_->options[i].key != key) continue;
    value = choice_->options[i].value;
    return true;
  }
  return false;
}

// Bank select, profile select and the entries: a digit the entry takes, or
// the frequency point. Any other key beeps and the entry stays.
void KeypadInput::pressedEntry(const EntrySpec& entry, char key) {
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
  digits_.push(key);
  if (entry.commitsWhenFull && digits_.length() >= entry.maxLen) {
    commitEntry();
    return;
  }
  listener_.onDigitAccepted(entry, key, digits_.c_str());
}

bool KeypadInput::takesDigit(const EntrySpec& entry, char key) const {
  if (!isDigit(key)) return false;
  const size_t len = digits_.length();
  if (len >= entry.maxLen) return false;
  if (len == 0 && key == '0' && !entry.leadingZero) return false;
  const int point = digits_.indexOf('*');
  return point < 0 || (int)len - point - 1 < entry.maxFraction;
}

// Enter ('D'). Entries and selections commit; in Normal mode it beeps. With no
// choice, bank or digit picked yet, Enter beeps and the mode stays: only Clear
// cancels.
void KeypadInput::enter() {
  if (mode_ == InputMode::Normal) {
    listener_.onRejected("ENTER");
    return;
  }
  if (mode_ == InputMode::ChoiceSelect) {
    if (!choiceStaged_) {
      reportRejected(choice_->name, "D");
      return;
    }
    const ChoiceSpec& spec = *choice_;
    const TargetVfo targetVfo = entryVfo_;
    mode_ = InputMode::Normal;
    entryVfo_ = TargetVfo::Current;
    choiceStaged_ = false;
    spec.commit(choiceValue_, targetVfo);
    return;
  }
  if (mode_ == InputMode::ValueEntry) {
    // A value past the range says so and becomes that end, staged for another Enter.
    int32_t v = 0;
    const ValueEntry::Result result = value_.result(v);
    if (result == ValueEntry::Result::AboveMax || result == ValueEntry::Result::BelowMin) {
      value_.snapToRange();
      value_.current(v);
      listener_.onLimit(value_.spec().name, result == ValueEntry::Result::AboveMax);
      listener_.onValueStaged(value_.spec(), v);
      return;
    }
    switch (result) {
      case ValueEntry::Result::Ok: {
        const TargetVfo targetVfo = entryVfo_;
        mode_ = InputMode::Normal;
        entryVfo_ = TargetVfo::Current;
        listener_.onValueCommit(value_.spec(), v, targetVfo);
        return;
      }
      case ValueEntry::Result::None: reportRejected(value_.spec().name, "D"); return;
      default: return;
    }
  }

  if (digits_.empty()) {
    reportRejected(keypadEntrySpec(mode_)->name, "D");
    return;
  }
  commitEntry();
}

void KeypadInput::commitEntry() {
  const InputMode mode = mode_;
  const decltype(digits_) digits = digits_;
  const TargetVfo targetVfo = entryVfo_;
  mode_ = InputMode::Normal;
  digits_.clear();
  entryVfo_ = TargetVfo::Current;
  if (mode == InputMode::BankSelect) {
    const uint8_t bank = (uint8_t)(digits.c_str()[0] - '0');
    // The digit came while '*' was still held: the next key only.
    if (down_.has('*')) {
      oneShotBank_ = bank;
      named_ = false;
      listener_.onOneShotBank(bank);
      return;
    }
    bank_ = bank;
    named_ = false;
  }
  listener_.onCommit(mode, digits.c_str(), targetVfo);
}

// Clear ('#') cancels every mode, entry, staged mode, waiting double click and
// one-shot bank, and beeps when there is none. Keys still held keep their
// swallowed release.
void KeypadInput::clearAll() {
  if (mode_ == InputMode::Normal && !pending_.active && !oneShotBank_) {
    listener_.onRejected("CLEAR");
    return;
  }
  mode_ = InputMode::Normal;
  digits_.clear();
  entryVfo_ = TargetVfo::Current;
  choiceStaged_ = false;
  pending_.active = false;
  oneShotBank_ = 0;
  listener_.onClear();
}

// Like Clear for the mode only: the swallowed release of a key still held
// stays.
void KeypadInput::timeOut() {
  const InputMode mode = mode_;
  const EntrySpec* entry = keypadEntrySpec(mode);
  const char* name = mode == InputMode::ChoiceSelect ? choice_->name
                     : mode == InputMode::ValueEntry ? value_.spec().name
                     : entry                         ? entry->name
                                                     : "ENTRY";
  mode_ = InputMode::Normal;
  digits_.clear();
  entryVfo_ = TargetVfo::Current;
  choiceStaged_ = false;
  listener_.onEntryTimeout(mode, name);
}

// A digit key moves the focus and ends the naming: the next A, B or C says the focus name
// again. A, B and C name it once.
void KeypadInput::runAction(KeyAction action, uint8_t bank, char key, const char* gesture,
                            Setting keyFocus) {
  const bool focusKey = isFocusKey(key);
  if (!focusKey) {
    if (keyFocus != Setting::None && bank < 10) focus_[bank] = keyFocus;
    named_ = false;
  }
  {
    ActiveKey active(bank, key, gesture, focus(bank), !named_);
    action();
  }
  if (focusKey) named_ = true;
}

void KeypadInput::runShortOrReject(const KeyBinding& binding, uint8_t bank, char key) {
  if (binding.shortAction) {
    runAction(binding.shortAction, bank, key, "SHORT", binding.focus);
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
