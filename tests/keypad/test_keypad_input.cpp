// Tests for the keypad input state machine (firmware/keypad_input.*) and
// DigitBuffer. A recording fake stands in for the listener and the keymap.
#include "keypad_input.h"
#include "test_runner.h"

#include <cstring>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

std::string keyId(uint8_t bank, char key) { return std::to_string(bank) + " " + key; }

const char *modeName(InputMode m) {
  switch (m) {
    case InputMode::Normal: return "Normal";
    case InputMode::BankSelect: return "BankSelect";
    case InputMode::ProfileSelect: return "ProfileSelect";
    case InputMode::ModeSelect: return "ModeSelect";
    case InputMode::FreqEntry: return "FreqEntry";
    case InputMode::RfPowerEntry: return "RfPowerEntry";
    case InputMode::CivAddrEntry: return "CivAddrEntry";
    case InputMode::RptOffsetEntry: return "RptOffsetEntry";
    case InputMode::CtcssEntry: return "CtcssEntry";
    case InputMode::DcsEntry: return "DcsEntry";
  }
  return "?";
}

struct Fake;
Fake *g_fake = nullptr;  // the Fake whose keymap is running an action
void runFakeAction();

// Records every listener call as one line. The keymap is a set of
// "bank key" ids per gesture; an action may also run a hook, e.g. to begin
// an entry. Actions log the key keypadActiveKey() names.
struct Fake : KeypadInputListener {
  std::vector<std::string> log;
  int activity = 0;
  int pressedActivity = 0;

  std::set<std::string> holds, shorts, doubles, doubleHolds, waits;
  std::map<std::string, std::function<void()>> hooks;  // "hold 1 0" -> hook
  std::string validModeDigits = "123456789";

  Fake() { g_fake = this; }

  // activeKey is "BANK<digit> <key> <SHORT|LONG|DOUBLE>"; logs "<gesture> <bank> <key>".
  void ran(const char *activeKey) {
    const std::string k = activeKey ? activeKey : "(null)";
    const std::string gesture = k.size() > 8 ? k.substr(8) : "";
    const char *name = gesture == "LONG" ? "hold" : gesture == "SHORT" ? "short"
                       : gesture == "DOUBLE" ? "double"
                       : gesture == "DOUBLE LONG" ? "doublehold" : nullptr;
    if (k.size() < 9 || k.compare(0, 4, "BANK") != 0 || k[5] != ' ' || k[7] != ' ' || !name) {
      log.push_back("action without key: " + k);
      return;
    }
    const std::string entry = std::string(name) + " " + keyId((uint8_t)(k[4] - '0'), k[6]);
    log.push_back(entry);
    auto hook = hooks.find(entry);
    if (hook != hooks.end()) hook->second();
  }

  void onActivity(bool pressed) override {
    ++activity;
    if (pressed) ++pressedActivity;
  }
  KeyBinding keyBinding(uint8_t bank, char key) override {
    const std::string id = keyId(bank, key);
    KeyBinding b;
    if (shorts.count(id)) b.shortAction = runFakeAction;
    if (holds.count(id)) b.holdAction = runFakeAction;
    if (doubles.count(id)) b.doubleAction = runFakeAction;
    if (doubleHolds.count(id)) b.doubleHoldAction = runFakeAction;
    b.waitsForDouble = waits.count(id) > 0;
    return b;
  }
  void onBankQuery(uint8_t bank) override { log.push_back("bank? " + std::to_string(bank)); }
  void onBankSelectStart() override { log.push_back("bank please"); }
  void onDigitAccepted(const EntrySpec &entry, char key, const char *digits) override {
    log.push_back(std::string("digit ") + modeName(entry.mode) + " " + key + " " + digits);
  }
  void onRejected(const char *label) override {
    log.push_back(std::string("rejected ") + label);
  }
  void onCommit(InputMode mode, const char *digits, TargetVfo targetVfo) override {
    log.push_back(std::string("commit ") + modeName(mode) + " [" + digits + "] vfo" +
                  std::to_string((int)targetVfo));
  }
  bool onModeDigit(char key, uint8_t &mode) override {
    if (key == '\0' || validModeDigits.find(key) == std::string::npos) return false;
    log.push_back(std::string("mode digit ") + key);
    mode = (uint8_t)(key - '0');
    return true;
  }
  void onModeCommit(uint8_t mode, TargetVfo targetVfo) override {
    log.push_back("mode commit " + std::to_string(mode) + " vfo" +
                  std::to_string((int)targetVfo));
  }
  void onStagedCommandSend(const char *cmd) override {
    log.push_back(std::string("send staged ") + cmd);
  }
  void onClear() override { log.push_back("clear"); }

  // The log since the last call, then cleared.
  std::vector<std::string> take() {
    std::vector<std::string> out;
    out.swap(log);
    return out;
  }
};

void runFakeAction() { g_fake->ran(keypadActiveKey()); }

using Log = std::vector<std::string>;

void printLog(const char *title, const Log &log) {
  fprintf(stderr, "    %s:\n", title);
  for (const std::string &line : log) fprintf(stderr, "      %s\n", line.c_str());
}

void checkLog(const char *file, int line, const Log &actual, const Log &expected) {
  if (actual == expected) return;
  fprintf(stderr, "  %s:%d: listener calls differ\n", file, line);
  printLog("actual", actual);
  printLog("expected", expected);
  ++test::g_checkFailures;
}

// The listener calls since the last check are exactly the given lines.
#define CHECK_LOG(fake, ...) checkLog(__FILE__, __LINE__, (fake).take(), Log{__VA_ARGS__})

// A key tapped and released at nowMs.
void tap(KeypadInput &in, char key, uint32_t nowMs = 0) {
  in.onKey(key, KeyGesture::Pressed, nowMs);
  in.onKey(key, KeyGesture::Released, nowMs);
}

// A key held until the HOLD event, then released.
void hold(KeypadInput &in, char key, uint32_t nowMs = 0) {
  in.onKey(key, KeyGesture::Pressed, nowMs);
  in.onKey(key, KeyGesture::Held, nowMs);
  in.onKey(key, KeyGesture::Released, nowMs);
}

void typeKeys(KeypadInput &in, const char *keys) {
  for (const char *k = keys; *k; ++k) tap(in, *k);
}

}  // namespace

// --- DigitBuffer ---------------------------------------------------------

TEST(digit_buffer_appends_until_full) {
  DigitBuffer<3> b;
  CHECK(b.empty());
  CHECK_EQ(b.c_str(), "");
  CHECK(b.push('1'));
  CHECK(b.push('*'));
  CHECK(b.push('5'));
  CHECK(!b.push('6'));
  CHECK_EQ(b.c_str(), "1*5");
  CHECK_EQ(b.length(), 3u);
  CHECK_EQ(b.indexOf('*'), 1);
  CHECK_EQ(b.indexOf('9'), -1);
  b.clear();
  CHECK(b.empty());
  CHECK_EQ(b.c_str(), "");
  CHECK_EQ(b.indexOf('1'), -1);
}

// --- Short, long and events that do nothing ---------------------------------

TEST(input_short_press_runs_short_action) {
  Fake f;
  f.shorts = {"1 7"};
  KeypadInput in(f);
  tap(in, '7');
  CHECK_LOG(f, "short 1 7");
}

TEST(input_short_press_without_action_is_unassigned) {
  Fake f;
  KeypadInput in(f);
  in.setBank(4);
  tap(in, 'B');
  CHECK_LOG(f, "rejected BANK4 B");
}

TEST(input_names_the_key_only_while_an_action_runs) {
  Fake f;
  f.holds = {"3 2"};
  std::string named;
  f.hooks["hold 3 2"] = [&] { named = keypadActiveKey() ? keypadActiveKey() : "(null)"; };
  KeypadInput in(f);
  in.setBank(3);
  CHECK(keypadActiveKey() == nullptr);
  hold(in, '2');
  CHECK_LOG(f, "hold 3 2");
  CHECK_EQ(named.c_str(), "BANK3 2 LONG");
  CHECK(keypadActiveKey() == nullptr);
  // The state machine's own keys run no bank action.
  tap(in, '*');
  CHECK_LOG(f, "bank? 3");
  CHECK(keypadActiveKey() == nullptr);
}

TEST(input_press_and_idle_only_report_activity) {
  Fake f;
  f.shorts = {"1 7"};
  KeypadInput in(f);
  in.onKey('7', KeyGesture::Pressed, 0);
  in.onKey('7', KeyGesture::Idle, 0);
  CHECK_LOG(f);
  CHECK_EQ(f.activity, 2);
  CHECK_EQ(f.pressedActivity, 1);
  in.onKey('7', KeyGesture::Released, 0);
  CHECK_EQ(f.activity, 3);
  CHECK_EQ(f.pressedActivity, 1);
}

// --- Hold swallowing ---------------------------------------------------------

TEST(input_hold_runs_long_action_and_swallows_release) {
  Fake f;
  f.holds = {"2 1"};
  f.shorts = {"2 1"};
  KeypadInput in(f);
  in.setBank(2);
  hold(in, '1');
  CHECK_LOG(f, "hold 2 1");
  // Only that one release is swallowed.
  tap(in, '1');
  CHECK_LOG(f, "short 2 1");
}

TEST(input_hold_without_long_action_is_unassigned_and_swallows_release) {
  Fake f;
  f.shorts = {"1 7"};
  KeypadInput in(f);
  in.onKey('7', KeyGesture::Pressed, 0);
  in.onKey('7', KeyGesture::Held, 0);
  CHECK_LOG(f, "rejected BANK1 7 LONG");
  in.onKey('7', KeyGesture::Released, 0);
  CHECK_LOG(f);
  // Only that one release is swallowed.
  tap(in, '7');
  CHECK_LOG(f, "short 1 7");
}

TEST(input_hold_without_any_action_is_unassigned_at_hold) {
  Fake f;
  KeypadInput in(f);
  hold(in, 'C');
  CHECK_LOG(f, "rejected BANK1 C LONG");
}

// Fixed by construction: FT-817 Bank 3 '1' long used to type its own release
// into the frequency entry it began.
TEST(input_release_of_hold_that_began_entry_is_not_typed) {
  Fake f;
  KeypadInput in(f);
  in.setBank(3);
  f.holds = {"3 1"};
  f.hooks["hold 3 1"] = [&] { in.beginEntry(InputMode::FreqEntry, TargetVfo::Current); };
  hold(in, '1');
  CHECK_LOG(f, "hold 3 1");
  CHECK_EQ(in.mode(), InputMode::FreqEntry);
  CHECK_EQ(in.digits(), "");
  tap(in, '1');
  CHECK_LOG(f, "digit FreqEntry 1 1");
}

// A key held through '#' is a co-press: '#' beeps instead of clearing, and
// the release of the held key does not run its short action.
TEST(input_clear_while_another_key_is_down_is_a_co_press) {
  Fake f;
  f.holds = {"1 3"};
  f.shorts = {"1 3"};
  KeypadInput in(f);
  f.hooks["hold 1 3"] = [&] { in.beginEntry(InputMode::FreqEntry); };
  in.onKey('3', KeyGesture::Pressed, 0);
  in.onKey('3', KeyGesture::Held, 0);
  tap(in, '#');
  in.onKey('3', KeyGesture::Released, 0);
  CHECK_LOG(f, "hold 1 3", "rejected TWO KEYS");
  // Nothing was cancelled.
  CHECK_EQ(in.mode(), InputMode::FreqEntry);
}

TEST(input_hold_of_d_and_hash_is_unassigned_and_swallows_release) {
  Fake f;
  KeypadInput in(f);
  in.stageCommand("PO?");
  hold(in, 'D');
  hold(in, '#');
  CHECK_LOG(f, "rejected ENTER LONG", "rejected CLEAR LONG");
  CHECK(in.hasStagedCommand());
}

TEST(input_hold_of_d_and_hash_in_entry_acts_on_release) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::RfPowerEntry);
  typeKeys(in, "5");
  hold(in, 'D');
  in.beginEntry(InputMode::RfPowerEntry);
  hold(in, '#');
  CHECK_LOG(f, "digit RfPowerEntry 5 5", "commit RfPowerEntry [5] vfo0", "clear");
}

// --- Double click ------------------------------------------------------------

TEST(input_waiting_key_runs_short_after_double_click_time) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  CHECK_LOG(f);
  CHECK(in.doubleClickPending());
  in.poll(1000 + KeypadInput::kDoubleClickMs);
  CHECK_LOG(f);
  const int before = f.activity;
  in.poll(1000 + KeypadInput::kDoubleClickMs + 1);
  CHECK_LOG(f, "short 1 0");
  CHECK_EQ(f.activity, before + 1);
  CHECK(!in.doubleClickPending());
  in.poll(5000);
  CHECK_LOG(f);
}

TEST(input_second_press_within_double_click_time_is_double) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  f.doubles = {"1 0"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  tap(in, '0', 1000 + KeypadInput::kDoubleClickMs);
  CHECK_LOG(f, "double 1 0");
  CHECK(!in.doubleClickPending());
  in.poll(5000);
  CHECK_LOG(f);
}

TEST(input_second_press_after_double_click_time_is_two_shorts) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  f.doubles = {"1 0"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  in.poll(1221);
  tap(in, '0', 1300);
  in.poll(1600);
  CHECK_LOG(f, "short 1 0", "short 1 0");
}

TEST(input_double_without_action_restarts_wait_and_runs_short_once) {
  Fake f;
  f.waits = {"8 2"};
  f.shorts = {"8 2"};
  KeypadInput in(f);
  in.setBank(8);
  tap(in, '2', 1000);
  tap(in, '2', 1100);
  in.poll(1300);
  CHECK_LOG(f);
  in.poll(1321);
  CHECK_LOG(f, "short 8 2");
}

// A tap then a hold is a double hold, never the key's long action: with none
// assigned it beeps, and the short it follows does not run.
TEST(input_hold_after_short_of_key_without_double_hold_beeps_and_drops_the_short) {
  Fake f;
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  f.holds = {"4 0"};
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('0', KeyGesture::Pressed, 1100);
  in.onKey('0', KeyGesture::Held, 1600);
  in.onKey('0', KeyGesture::Released, 1900);
  in.poll(5000);
  CHECK_LOG(f, "rejected BANK4 0 DOUBLE LONG");
  CHECK(!in.doubleClickPending());
  // Only that release is swallowed.
  tap(in, '0', 6000);
  in.poll(6300);
  CHECK_LOG(f, "short 4 0");
}

// --- Double hold: a short press, then a hold within the double-click time ----

// A key with a double hold, a short and a long action, bank 4 key '0'.
void giveDoubleHold(Fake &f) {
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  f.holds = {"4 0"};
  f.doubleHolds = {"4 0"};
}

TEST(input_hold_right_after_short_is_double_hold) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  hold(in, '0', 1100);
  CHECK_LOG(f, "doublehold 4 0");
  CHECK(!in.doubleClickPending());
  in.poll(5000);
  CHECK_LOG(f);
}

TEST(input_double_hold_action_is_named_double_long) {
  Fake f;
  giveDoubleHold(f);
  std::string named;
  f.hooks["doublehold 4 0"] = [&] { named = keypadActiveKey() ? keypadActiveKey() : "(null)"; };
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  hold(in, '0', 1100);
  CHECK_EQ(named.c_str(), "BANK4 0 DOUBLE LONG");
  CHECK(keypadActiveKey() == nullptr);
}

// The hold event comes long after the second press went down; the short must
// not run meanwhile, however often the main loop polls.
TEST(input_second_press_waits_for_the_double_hold) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('0', KeyGesture::Pressed, 1100);
  in.poll(1230);
  in.poll(1500);
  CHECK_LOG(f);
  CHECK(in.doubleClickPending());
  in.onKey('0', KeyGesture::Held, 1600);
  in.onKey('0', KeyGesture::Released, 1900);
  CHECK_LOG(f, "doublehold 4 0");
  in.poll(5000);
  CHECK_LOG(f);
}

TEST(input_double_hold_swallows_its_release) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  hold(in, '0', 1100);
  CHECK_LOG(f, "doublehold 4 0");
  // Only that release is swallowed.
  tap(in, '0', 3000);
  in.poll(3300);
  CHECK_LOG(f, "short 4 0");
}

TEST(input_hold_without_short_before_is_plain_long) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  hold(in, '0', 1000);
  CHECK_LOG(f, "hold 4 0");
}

TEST(input_hold_after_double_click_time_is_short_then_long) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.poll(1300);
  CHECK_LOG(f, "short 4 0");
  hold(in, '0', 1400);
  CHECK_LOG(f, "hold 4 0");
}

// The second press went down after the time, and the main loop had not polled:
// its own event runs the expired short first, so the hold is a plain long press.
TEST(input_second_press_after_double_click_time_is_not_double_hold) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('0', KeyGesture::Pressed, 1000 + KeypadInput::kDoubleClickMs + 1);
  in.onKey('0', KeyGesture::Held, 2000);
  in.onKey('0', KeyGesture::Released, 2100);
  CHECK_LOG(f, "short 4 0", "hold 4 0");
}

TEST(input_double_hold_key_still_takes_double_click) {
  Fake f;
  giveDoubleHold(f);
  f.doubles = {"4 0"};
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  tap(in, '0', 1100);
  CHECK_LOG(f, "double 4 0");
}

// A second press released after the time, with no hold, is two shorts like
// for any waiting key.
TEST(input_slow_second_press_of_double_hold_key_is_two_shorts) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('0', KeyGesture::Pressed, 1100);
  in.onKey('0', KeyGesture::Released, 1500);
  CHECK_LOG(f, "short 4 0");
  in.poll(2000);
  CHECK_LOG(f, "short 4 0");
}

// The short keeps waiting while the second press is down, however long that is.
TEST(input_second_press_holds_back_the_short_of_any_waiting_key) {
  Fake f;
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('0', KeyGesture::Pressed, 1100);
  in.poll(1500);
  CHECK_LOG(f);
  // Released after the time, with no hold: two shorts.
  in.onKey('0', KeyGesture::Released, 1600);
  CHECK_LOG(f, "short 4 0");
  in.poll(2000);
  CHECK_LOG(f, "short 4 0");
}

TEST(input_plain_hold_of_double_hold_key_without_long_beeps_as_long) {
  Fake f;
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  f.doubleHolds = {"4 0"};
  KeypadInput in(f);
  in.setBank(4);
  hold(in, '0', 1000);
  CHECK_LOG(f, "rejected BANK4 0 LONG");
}

// Two keys down at once beep and drop the waiting short; the second press of
// the double hold is part of it, so the hold does nothing either.
TEST(input_other_key_during_second_press_is_a_co_press) {
  Fake f;
  giveDoubleHold(f);
  f.shorts.insert("4 7");
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('0', KeyGesture::Pressed, 1100);
  tap(in, '7', 1150);
  in.onKey('0', KeyGesture::Held, 1600);
  in.onKey('0', KeyGesture::Released, 1900);
  in.poll(5000);
  CHECK_LOG(f, "rejected TWO KEYS");
  CHECK(!in.doubleClickPending());
}

TEST(input_clear_during_second_press_is_a_co_press) {
  Fake f;
  giveDoubleHold(f);
  KeypadInput in(f);
  in.setBank(4);
  in.stageCommand("PO?");
  tap(in, '0', 1000);
  in.onKey('0', KeyGesture::Pressed, 1100);
  tap(in, '#', 1150);
  in.onKey('0', KeyGesture::Held, 1600);
  in.onKey('0', KeyGesture::Released, 1900);
  CHECK_LOG(f, "rejected TWO KEYS");
  CHECK(!in.doubleClickPending());
  CHECK(in.hasStagedCommand());
}

TEST(input_double_hold_needs_same_bank) {
  Fake f;
  giveDoubleHold(f);
  f.waits.insert("3 0");
  f.shorts.insert("3 0");
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.setBank(3);
  hold(in, '0', 1100);
  CHECK_LOG(f, "short 4 0", "rejected BANK3 0 LONG");
}

// A waiting short answers before any later key, never after it.
TEST(input_waiting_short_runs_before_other_key) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0", "1 7"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  in.onKey('7', KeyGesture::Pressed, 1100);
  CHECK_LOG(f, "short 1 0");
  CHECK(!in.doubleClickPending());
  in.onKey('7', KeyGesture::Released, 1150);
  in.poll(1300);
  CHECK_LOG(f, "short 1 7");
}

TEST(input_waiting_short_runs_before_other_key_hold) {
  Fake f;
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  f.holds = {"4 2"};
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  hold(in, '2', 1100);
  in.poll(2000);
  CHECK_LOG(f, "short 4 0", "hold 4 2");
}

TEST(input_waiting_short_runs_before_unassigned_hold) {
  Fake f;
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('7', KeyGesture::Pressed, 1100);
  in.onKey('7', KeyGesture::Held, 1100);
  in.poll(2000);
  CHECK_LOG(f, "short 4 0", "rejected BANK4 7 LONG");
}

TEST(input_waiting_short_runs_before_other_waiting_key) {
  Fake f;
  f.waits = {"1 0", "1 1"};
  f.shorts = {"1 0", "1 1"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  tap(in, '1', 1100);
  CHECK_LOG(f, "short 1 0");
  CHECK(in.doubleClickPending());
  in.poll(1400);
  CHECK_LOG(f, "short 1 1");
}

TEST(input_waiting_short_runs_before_star_and_enter) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  KeypadInput in(f);
  in.stageCommand("SM?");
  tap(in, '0', 1000);
  tap(in, '*', 1100);
  tap(in, '0', 1400);
  tap(in, 'D', 1500);
  in.poll(2000);
  CHECK_LOG(f, "short 1 0", "bank? 1", "short 1 0", "send staged SM?");
}

TEST(input_waiting_short_runs_before_bank_select) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  hold(in, '*', 1100);
  CHECK_LOG(f, "short 1 0", "bank please");
}

// The main loop may not have polled since the wait ran out: the next key runs
// the waiting short first, even the same key.
TEST(input_expired_wait_runs_before_same_key_without_poll) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  f.doubles = {"1 0"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  tap(in, '0', 1000 + KeypadInput::kDoubleClickMs + 1);
  CHECK_LOG(f, "short 1 0");
  in.poll(2000);
  CHECK_LOG(f, "short 1 0");
}

TEST(input_double_click_needs_same_bank) {
  Fake f;
  f.waits = {"1 0", "3 0"};
  f.shorts = {"1 0", "3 0"};
  f.doubles = {"1 0", "3 0"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  in.setBank(3);
  tap(in, '0', 1100);
  in.poll(1400);
  CHECK_LOG(f, "short 1 0", "short 3 0");
}

TEST(input_deferred_short_without_action_is_unassigned) {
  Fake f;
  f.waits = {"2 9"};
  KeypadInput in(f);
  in.setBank(2);
  tap(in, '9', 1000);
  in.poll(1300);
  CHECK_LOG(f, "rejected BANK2 9");
}

// --- Bank ('*') ----------------------------------------------------------------

TEST(input_star_short_says_bank) {
  Fake f;
  KeypadInput in(f);
  in.setBank(6);
  tap(in, '*');
  CHECK_LOG(f, "bank? 6");
}

TEST(input_bank_select_commits_on_the_digit) {
  Fake f;
  KeypadInput in(f);
  hold(in, '*');
  CHECK_EQ(in.mode(), InputMode::BankSelect);
  tap(in, '5');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_EQ(in.bank(), 5);
  CHECK_LOG(f, "bank please", "commit BankSelect [5] vfo0");
  // The next short '*' is not mistaken for the release of the hold.
  tap(in, '*');
  CHECK_LOG(f, "bank? 5");
  // The next digit is a bank key again.
  tap(in, '3');
  CHECK_LOG(f, "rejected BANK5 3");
}

// A digit pressed while '*' is still down is a co-press: bank select stays
// open and the digit is not taken. Released, a digit commits.
TEST(input_bank_select_digit_during_the_star_hold_is_a_co_press) {
  Fake f;
  KeypadInput in(f);
  in.onKey('*', KeyGesture::Pressed, 0);
  in.onKey('*', KeyGesture::Held, 0);
  tap(in, '4');
  in.onKey('*', KeyGesture::Released, 0);
  CHECK_EQ(in.mode(), InputMode::BankSelect);
  CHECK_LOG(f, "bank please", "rejected TWO KEYS");
  tap(in, '4');
  CHECK_EQ(in.bank(), 4);
  CHECK_LOG(f, "commit BankSelect [4] vfo0");
}

TEST(input_bank_select_enter_is_unassigned_and_stays) {
  Fake f;
  KeypadInput in(f);
  in.setBank(2);
  hold(in, '*');
  tap(in, 'D');
  CHECK_EQ(in.bank(), 2);
  CHECK_EQ(in.mode(), InputMode::BankSelect);
  tap(in, '4');
  CHECK_EQ(in.bank(), 4);
  CHECK_LOG(f, "bank please", "rejected BANK SELECT D", "commit BankSelect [4] vfo0");
}

TEST(input_bank_select_rejects_other_keys) {
  Fake f;
  f.holds = {"1 3"};
  KeypadInput in(f);
  hold(in, '*');
  typeKeys(in, "0A");
  tap(in, '*');  // silent in bank select
  hold(in, '3');  // holds are ignored, the press is a digit
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_EQ(in.bank(), 3);
  CHECK_LOG(f, "bank please", "rejected BANK SELECT 0", "rejected BANK SELECT A",
                        "commit BankSelect [3] vfo0");
}

TEST(input_bank_select_clear) {
  Fake f;
  KeypadInput in(f);
  hold(in, '*');
  tap(in, '#');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_EQ(in.bank(), 1);
  CHECK_LOG(f, "bank please", "clear");
}

// --- Profile select --------------------------------------------------------------

TEST(input_profile_select_digit_rules) {
  Fake f;
  KeypadInput in(f);
  in.setBank(9);
  f.holds = {"9 A"};
  f.hooks["hold 9 A"] = [&] { in.beginProfileSelect(); };
  hold(in, 'A');  // its release is swallowed
  CHECK_EQ(in.mode(), InputMode::ProfileSelect);
  typeKeys(in, "0123B*");
  tap(in, 'A');
  tap(in, 'D');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "hold 9 A", "rejected PROFILE 0", "digit ProfileSelect 1 1",
                        "digit ProfileSelect 2 12", "rejected PROFILE 3",
                        "rejected PROFILE B", "rejected PROFILE *", "rejected PROFILE A",
                        "commit ProfileSelect [12] vfo0");
}

TEST(input_profile_select_zero_after_first_digit) {
  Fake f;
  KeypadInput in(f);
  in.beginProfileSelect();
  typeKeys(in, "10D");
  CHECK_LOG(f, "digit ProfileSelect 1 1", "digit ProfileSelect 0 10",
                        "commit ProfileSelect [10] vfo0");
}

// --- Entries -----------------------------------------------------------------------

TEST(input_frequency_entry_point_rules) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::FreqEntry, TargetVfo::B);
  typeKeys(in, "*14**");
  CHECK_LOG(f, "rejected FREQ POINT", "digit FreqEntry 1 1", "digit FreqEntry 4 14",
                        "digit FreqEntry * 14*", "rejected FREQ POINT");
  typeKeys(in, "123456");
  CHECK_LOG(f, "digit FreqEntry 1 14*1", "digit FreqEntry 2 14*12",
                        "digit FreqEntry 3 14*123", "digit FreqEntry 4 14*1234",
                        "digit FreqEntry 5 14*12345", "rejected FREQ 6");
  typeKeys(in, "AD");
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "rejected FREQ A", "commit FreqEntry [14*12345] vfo2");
}

TEST(input_frequency_entry_length_limit) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::FreqEntry);
  typeKeys(in, "123456789012");
  f.take();
  tap(in, '3');
  CHECK_LOG(f, "rejected FREQ 3");
  // The old code took a point after 12 digits.
  tap(in, '*');
  CHECK_LOG(f, "digit FreqEntry * 123456789012*");
  tap(in, 'D');
  CHECK_LOG(f, "commit FreqEntry [123456789012*] vfo0");
}

TEST(input_frequency_entry_point_counts_toward_length) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::FreqEntry);
  typeKeys(in, "1234567*123");
  f.take();
  typeKeys(in, "45");
  CHECK_LOG(f, "digit FreqEntry 4 1234567*1234", "rejected FREQ 5");
}

TEST(input_rf_power_entry_limits) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::RfPowerEntry);
  typeKeys(in, "1004*BD");
  CHECK_LOG(f, "digit RfPowerEntry 1 1", "digit RfPowerEntry 0 10",
                        "digit RfPowerEntry 0 100", "rejected RFPOWER 4",
                        "rejected RFPOWER *", "rejected RFPOWER B",
                        "commit RfPowerEntry [100] vfo0");
}

TEST(input_civ_address_entry_limits) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::CivAddrEntry);
  typeKeys(in, "1488*D");
  CHECK_LOG(f, "digit CivAddrEntry 1 1", "digit CivAddrEntry 4 14",
                        "digit CivAddrEntry 8 148", "rejected CIVADDR 8",
                        "rejected CIVADDR *", "commit CivAddrEntry [148] vfo0");
}

TEST(input_bank6_entry_limits) {
  struct Case {
    InputMode mode;
    const char *accepted;
    const char *name;
  };
  const Case cases[] = {
      {InputMode::RptOffsetEntry, "5000", "RPTSHIFT"},
      {InputMode::CtcssEntry, "1318", "CTCSS"},
      {InputMode::DcsEntry, "023", "DCS"},
  };
  for (const Case &c : cases) {
    Fake f;
    KeypadInput in(f);
    in.beginEntry(c.mode);
    typeKeys(in, c.accepted);
    f.take();
    typeKeys(in, "9*");
    CHECK_LOG(f, std::string("rejected ") + c.name + " 9", std::string("rejected ") + c.name + " *");
    tap(in, 'D');
    CHECK_LOG(f, std::string("commit ") + modeName(c.mode) + " [" + c.accepted + "] vfo0");
  }
}

TEST(input_entry_ignores_holds) {
  Fake f;
  f.holds = {"1 5"};
  KeypadInput in(f);
  in.beginEntry(InputMode::FreqEntry);
  hold(in, '5');
  hold(in, '*');
  CHECK_EQ(in.mode(), InputMode::FreqEntry);
  CHECK_LOG(f, "digit FreqEntry 5 5", "digit FreqEntry * 5*");
}

// Entries and selections take a key as it goes down, not when it comes up.
TEST(input_entry_takes_the_key_on_press) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::FreqEntry);
  in.onKey('1', KeyGesture::Pressed, 0);
  CHECK_LOG(f, "digit FreqEntry 1 1");
  in.onKey('1', KeyGesture::Released, 0);
  CHECK_LOG(f);
  in.beginModeSelect(TargetVfo::Current);
  in.onKey('2', KeyGesture::Pressed, 0);
  CHECK(in.stagedModeActive());
  CHECK_LOG(f, "mode digit 2");
}

// A key that ends the mode on its press does nothing more in Normal mode: its
// hold runs no long action and beeps no "LONG", its release no short.
TEST(input_key_that_ends_the_mode_is_silent_until_released) {
  Fake f;
  f.shorts = {"5 5", "1 7"};
  f.holds = {"5 5"};
  KeypadInput in(f);
  hold(in, '*');
  f.take();
  in.onKey('5', KeyGesture::Pressed, 0);
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_EQ(in.bank(), 5);
  CHECK_LOG(f, "commit BankSelect [5] vfo0");
  in.onKey('5', KeyGesture::Held, 0);
  in.onKey('5', KeyGesture::Released, 0);
  CHECK_LOG(f);

  in.beginEntry(InputMode::FreqEntry);
  tap(in, '7');
  f.take();
  hold(in, 'D');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "commit FreqEntry [7] vfo0");

  in.beginEntry(InputMode::FreqEntry);
  hold(in, '#');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "clear");

  // The next press of the same key is a Normal-mode key again.
  tap(in, '5');
  CHECK_LOG(f, "short 5 5");
}

// A press while another key is down never reaches the keymap, so it cannot open
// an entry the first key's release would then type into.
TEST(input_press_while_another_key_is_down_is_a_co_press) {
  Fake f;
  f.shorts = {"1 1"};
  KeypadInput in(f);
  in.onKey('4', KeyGesture::Pressed, 0);
  tap(in, '1');
  in.onKey('4', KeyGesture::Released, 0);
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "rejected TWO KEYS");
}

// --- Co-press: two keys down at once -----------------------------------------

// One beep, then silence until every key is up, whatever the order of events.
TEST(input_co_press_ignores_everything_until_all_keys_are_up) {
  Fake f;
  f.shorts = {"1 4", "1 5", "1 6"};
  f.holds = {"1 4", "1 5", "1 6"};
  KeypadInput in(f);
  in.onKey('4', KeyGesture::Pressed, 0);
  in.onKey('5', KeyGesture::Pressed, 10);
  in.onKey('6', KeyGesture::Pressed, 20);
  in.onKey('4', KeyGesture::Held, 500);
  in.onKey('5', KeyGesture::Held, 510);
  in.onKey('4', KeyGesture::Released, 600);
  in.onKey('5', KeyGesture::Released, 610);
  in.onKey('6', KeyGesture::Held, 620);
  in.onKey('6', KeyGesture::Released, 700);
  CHECK_LOG(f, "rejected TWO KEYS");
  // The next press is a plain one again.
  tap(in, '4');
  CHECK_LOG(f, "short 1 4");
}

TEST(input_co_press_beeps_again_in_the_next_one) {
  Fake f;
  KeypadInput in(f);
  for (int i = 0; i < 2; ++i) {
    in.onKey('4', KeyGesture::Pressed, 0);
    in.onKey('5', KeyGesture::Pressed, 0);
    in.onKey('4', KeyGesture::Released, 0);
    in.onKey('5', KeyGesture::Released, 0);
  }
  CHECK_LOG(f, "rejected TWO KEYS", "rejected TWO KEYS");
}

// What an entry took before the co-press stays, and a release the entry had
// swallowed does not stay swallowed for the next press of that key.
TEST(input_co_press_keeps_the_entry_and_forgets_swallowed_releases) {
  Fake f;
  f.shorts = {"1 5"};
  KeypadInput in(f);
  in.beginEntry(InputMode::RfPowerEntry);
  in.onKey('5', KeyGesture::Pressed, 0);
  in.onKey('6', KeyGesture::Pressed, 0);
  in.onKey('5', KeyGesture::Released, 0);
  in.onKey('6', KeyGesture::Released, 0);
  CHECK_EQ(in.mode(), InputMode::RfPowerEntry);
  CHECK_EQ(std::string(in.digits()), std::string("5"));
  CHECK_LOG(f, "digit RfPowerEntry 5 5", "rejected TWO KEYS");
  tap(in, 'D');
  in.beginEntry(InputMode::RfPowerEntry);
  CHECK_LOG(f, "commit RfPowerEntry [5] vfo0");
  tap(in, '5');
  CHECK_LOG(f, "digit RfPowerEntry 5 5");
}

// A key tapped and released before the next goes down is no co-press.
TEST(input_keys_one_after_the_other_are_no_co_press) {
  Fake f;
  f.shorts = {"1 4", "1 5"};
  KeypadInput in(f);
  tap(in, '4');
  tap(in, '5');
  CHECK_LOG(f, "short 1 4", "short 1 5");
}

TEST(input_entry_starts_empty_each_time) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::RfPowerEntry);
  typeKeys(in, "5#");
  in.beginEntry(InputMode::RfPowerEntry);
  typeKeys(in, "7D");
  CHECK_LOG(f, "digit RfPowerEntry 5 5", "clear", "digit RfPowerEntry 7 7",
                        "commit RfPowerEntry [7] vfo0");
}

// Like bank and mode select: Enter with nothing typed beeps and the entry
// stays; only '#' cancels.
TEST(input_enter_with_nothing_typed_is_unassigned_and_stays) {
  struct Case {
    InputMode mode;
    const char *label;
  };
  const Case cases[] = {
      {InputMode::FreqEntry, "FREQ D"},
      {InputMode::RfPowerEntry, "RFPOWER D"},
      {InputMode::CivAddrEntry, "CIVADDR D"},
      {InputMode::RptOffsetEntry, "RPTSHIFT D"},
      {InputMode::CtcssEntry, "CTCSS D"},
      {InputMode::DcsEntry, "DCS D"},
      {InputMode::ProfileSelect, "PROFILE D"},
  };
  for (const Case &c : cases) {
    Fake f;
    KeypadInput in(f);
    in.beginEntry(c.mode, TargetVfo::B);
    tap(in, 'D');
    CHECK_LOG(f, std::string("rejected ") + c.label);
    CHECK_EQ(in.mode(), c.mode);
    CHECK_EQ(in.entryTargetVfo(), TargetVfo::B);
    tap(in, '2');
    tap(in, 'D');
    CHECK_LOG(f, std::string("digit ") + modeName(c.mode) + " 2 2",
              std::string("commit ") + modeName(c.mode) + " [2] vfo2");
    CHECK_EQ(in.mode(), InputMode::Normal);
  }
}

TEST(input_clear_with_nothing_to_cancel_is_unassigned) {
  Fake f;
  KeypadInput in(f);
  tap(in, '#');
  CHECK_LOG(f, "rejected CLEAR");
}

TEST(input_clear_cancels_staged_command_or_waiting_short) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  KeypadInput in(f);
  in.stageCommand("SWR?");
  tap(in, '#');
  CHECK(!in.hasStagedCommand());
  tap(in, '0', 1000);
  tap(in, '#', 1100);
  in.poll(2000);
  tap(in, 'D');
  CHECK_LOG(f, "clear", "clear", "rejected ENTER");
}

// --- Clear and Enter routing ----------------------------------------------------------

TEST(input_clear_leaves_every_mode) {
  const InputMode modes[] = {InputMode::ProfileSelect,  InputMode::FreqEntry,
                             InputMode::RfPowerEntry,   InputMode::CivAddrEntry,
                             InputMode::RptOffsetEntry, InputMode::CtcssEntry,
                             InputMode::DcsEntry};
  for (InputMode m : modes) {
    Fake f;
    KeypadInput in(f);
    in.beginEntry(m, TargetVfo::A);
    typeKeys(in, "1#");
    CHECK_EQ(in.mode(), InputMode::Normal);
    CHECK_EQ(in.digits(), "");
    CHECK_EQ(in.entryTargetVfo(), TargetVfo::Current);
    const Log log = f.take();
    CHECK(!log.empty() && log.back() == "clear");
  }
}

TEST(input_clear_cancels_staged_mode_and_waiting_short) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  KeypadInput in(f);
  in.beginModeSelect(TargetVfo::A);
  tap(in, '3');
  CHECK(in.stagedModeActive());
  tap(in, '#');
  CHECK_EQ(in.mode(), InputMode::Normal);
  tap(in, '0', 1000);
  tap(in, '#', 1100);
  in.poll(2000);
  tap(in, 'D');
  CHECK_LOG(f, "mode digit 3", "clear", "clear", "rejected ENTER");
}

TEST(input_enter_in_normal_mode_routes_staged_mode_then_staged_command) {
  Fake f;
  KeypadInput in(f);
  in.stageCommand("MODE?");
  in.beginModeSelect(TargetVfo::B);
  tap(in, '4');
  tap(in, 'D');
  tap(in, 'D');
  tap(in, 'D');
  CHECK_LOG(f, "mode digit 4", "mode commit 4 vfo2", "send staged MODE?",
                        "rejected ENTER");
}

// F3: a staged mode is modal until Enter applies it or '#' cancels it. Keys
// run no short or long action; a mode digit replaces the staged mode, and any
// other key beeps.
TEST(input_staged_mode_is_modal) {
  Fake f;
  f.validModeDigits = "12";
  f.shorts = {"1 7"};
  f.holds = {"1 0"};
  KeypadInput in(f);
  in.beginModeSelect(TargetVfo::B);
  tap(in, '2');
  tap(in, '7');
  hold(in, '0');
  hold(in, '*');
  CHECK_EQ(in.mode(), InputMode::ModeSelect);
  CHECK(in.stagedModeActive());
  tap(in, '1');
  tap(in, 'D');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "mode digit 2", "rejected MODE SELECT 7", "rejected MODE SELECT 0",
            "rejected MODE SELECT *", "mode digit 1", "mode commit 1 vfo2");
}

// --- Mode select -----------------------------------------------------------------------

TEST(input_mode_select_stages_mode_for_target_vfo) {
  Fake f;
  f.waits = {"3 1"};
  KeypadInput in(f);
  in.setBank(3);
  in.beginModeSelect(TargetVfo::A);
  tap(in, '1', 1000);  // no double-click wait during mode select
  CHECK(in.stagedModeActive());
  CHECK(!in.doubleClickPending());
  tap(in, 'D');
  CHECK(!in.stagedModeActive());
  CHECK_LOG(f, "mode digit 1", "mode commit 1 vfo1");
}

// F2: mode select is modal. A key that picks no mode beeps (the listener gives
// the feedback) and mode select stays active; only '#' cancels it.
TEST(input_mode_select_invalid_key_keeps_it_active) {
  Fake f;
  f.validModeDigits = "12";
  KeypadInput in(f);
  in.beginModeSelect(TargetVfo::B);
  typeKeys(in, "7A*");
  CHECK_EQ(in.mode(), InputMode::ModeSelect);
  tap(in, '2');
  tap(in, 'D');
  CHECK_LOG(f, "rejected MODE SELECT 7", "rejected MODE SELECT A", "rejected MODE SELECT *",
            "mode digit 2", "mode commit 2 vfo2");
}

TEST(input_mode_select_enter_beeps_and_keeps_it_active) {
  Fake f;
  KeypadInput in(f);
  in.stageCommand("PO?");
  in.beginModeSelect(TargetVfo::Current);
  tap(in, 'D');
  CHECK_EQ(in.mode(), InputMode::ModeSelect);
  CHECK(!in.stagedModeActive());
  CHECK(in.hasStagedCommand());
  CHECK_LOG(f, "rejected MODE SELECT D");
}

TEST(input_mode_select_clear_cancels_it) {
  Fake f;
  KeypadInput in(f);
  in.beginModeSelect(TargetVfo::A);
  tap(in, '#');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK(!in.stagedModeActive());
  CHECK_LOG(f, "clear");
}

// Fixed by construction: a mode select started without a VFO used to reuse the
// VFO of an earlier mode select that ended on an invalid digit.
TEST(input_mode_select_target_vfo_is_not_reused) {
  Fake f;
  f.validModeDigits = "5";
  KeypadInput in(f);
  in.beginModeSelect(TargetVfo::A);
  typeKeys(in, "0#");
  in.beginModeSelect(TargetVfo::Current);
  tap(in, '5');
  tap(in, 'D');
  CHECK_LOG(f, "rejected MODE SELECT 0", "clear", "mode digit 5", "mode commit 5 vfo0");
}

// F1: every key is the mode digit, even one with a short action on the bank
// (Bank 3 '7' is a band-stack query).
TEST(input_mode_select_key_is_mode_digit_not_short_action) {
  Fake f;
  f.shorts = {"3 7"};
  KeypadInput in(f);
  in.setBank(3);
  in.beginModeSelect(TargetVfo::A);
  tap(in, '7');
  CHECK(in.stagedModeActive());
  CHECK_LOG(f, "mode digit 7");
}

// F2: holds during mode select run no long action and do not open bank select;
// the press is the mode digit.
TEST(input_mode_select_ignores_holds) {
  Fake f;
  f.holds = {"1 0", "1 2"};
  KeypadInput in(f);
  in.beginModeSelect(TargetVfo::Current);
  hold(in, '0');
  hold(in, '*');
  CHECK_EQ(in.mode(), InputMode::ModeSelect);
  hold(in, '2');
  CHECK(in.stagedModeActive());
  CHECK_EQ(in.bank(), 1);
  CHECK_LOG(f, "rejected MODE SELECT 0", "rejected MODE SELECT *", "mode digit 2");
}

// Fixed by construction: the picked mode is part of mode select, so a new mode
// select starts with none.
TEST(input_mode_select_does_not_keep_an_earlier_pick) {
  Fake f;
  KeypadInput in(f);
  in.beginModeSelect(TargetVfo::A);
  typeKeys(in, "3#");
  in.beginModeSelect(TargetVfo::A);
  CHECK(!in.stagedModeActive());
  tap(in, 'D');
  CHECK_EQ(in.mode(), InputMode::ModeSelect);
  CHECK_LOG(f, "mode digit 3", "clear", "rejected MODE SELECT D");
}

// --- Entry table -----------------------------------------------------------------------

TEST(input_every_entry_mode_has_its_rules) {
  const InputMode entries[] = {InputMode::BankSelect,     InputMode::ProfileSelect,
                               InputMode::FreqEntry,      InputMode::RfPowerEntry,
                               InputMode::CivAddrEntry,   InputMode::RptOffsetEntry,
                               InputMode::CtcssEntry,     InputMode::DcsEntry};
  for (InputMode m : entries) {
    const EntrySpec *e = keypadEntrySpec(m);
    CHECK(e != nullptr);
    if (!e) continue;
    CHECK_EQ(e->mode, m);
    // The digit buffer holds the longest entry plus the frequency point.
    CHECK(e->maxLen >= 1 && e->maxLen <= 12);
  }
  CHECK(keypadEntrySpec(InputMode::Normal) == nullptr);
  CHECK(keypadEntrySpec(InputMode::ModeSelect) == nullptr);
}

// --- Staged command --------------------------------------------------------------------

TEST(input_staged_command_is_replaced_and_sent_once) {
  Fake f;
  KeypadInput in(f);
  in.stageCommand("PO?");
  in.stageCommand("SWR?");
  CHECK_EQ(in.stagedCommand(), "SWR?");
  tap(in, 'D');
  CHECK(!in.hasStagedCommand());
  tap(in, 'D');
  CHECK_LOG(f, "send staged SWR?", "rejected ENTER");
}

// An entry keeps the staged command: Enter commits the entry first.
TEST(input_staged_command_waits_through_an_entry) {
  Fake f;
  KeypadInput in(f);
  in.stageCommand("PO?");
  in.beginEntry(InputMode::RfPowerEntry);
  typeKeys(in, "5DD");
  CHECK_LOG(f, "digit RfPowerEntry 5 5", "commit RfPowerEntry [5] vfo0", "send staged PO?");
}

TEST(input_staged_command_is_cut_to_its_maximum) {
  Fake f;
  KeypadInput in(f);
  in.stageCommand("0123456789012345678901234567");
  CHECK_EQ(strlen(in.stagedCommand()), KeypadInput::kMaxStagedCommand);
}
