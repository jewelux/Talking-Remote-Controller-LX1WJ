// Tests for the keypad input state machine (firmware/keypad_input.*) and
// DigitBuffer. A recording fake stands in for the listener and the keymap.
#include "keypad_input.h"
#include "test_runner.h"

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
    case InputMode::ModeStaged: return "ModeStaged";
    case InputMode::FreqEntry: return "FreqEntry";
    case InputMode::RfPowerEntry: return "RfPowerEntry";
    case InputMode::CivAddrEntry: return "CivAddrEntry";
    case InputMode::RptOffsetEntry: return "RptOffsetEntry";
    case InputMode::CtcssEntry: return "CtcssEntry";
    case InputMode::DcsEntry: return "DcsEntry";
  }
  return "?";
}

// Records every listener call as one line. The keymap is a set of
// "bank key" ids per gesture; an action may also run a hook, e.g. to begin
// an entry.
struct Fake : KeypadInputListener {
  std::vector<std::string> log;
  int activity = 0;
  int pressedActivity = 0;

  std::set<std::string> holds, shorts, doubles, waits;
  std::map<std::string, std::function<void()>> hooks;  // "hold 1 0" -> hook
  std::string validModeDigits = "123456789";
  bool hasStagedCommand = false;

  bool run(const char *gesture, const std::set<std::string> &keys, uint8_t bank, char key) {
    const std::string id = keyId(bank, key);
    if (!keys.count(id)) return false;
    const std::string entry = std::string(gesture) + " " + id;
    log.push_back(entry);
    auto hook = hooks.find(entry);
    if (hook != hooks.end()) hook->second();
    return true;
  }

  void onActivity(bool pressed) override {
    ++activity;
    if (pressed) ++pressedActivity;
  }
  bool runHold(uint8_t bank, char key) override { return run("hold", holds, bank, key); }
  bool runShort(uint8_t bank, char key) override { return run("short", shorts, bank, key); }
  bool runDoubleClick(uint8_t bank, char key) override {
    return run("double", doubles, bank, key);
  }
  bool wantsDoubleClick(uint8_t bank, char key) override { return waits.count(keyId(bank, key)); }
  void onBankQuery(uint8_t bank) override { log.push_back("bank? " + std::to_string(bank)); }
  void onBankSelectStart() override { log.push_back("bank please"); }
  void onDigitAccepted(InputMode mode, char key, const char *digits) override {
    log.push_back(std::string("digit ") + modeName(mode) + " " + key + " " + digits);
  }
  void onUnassigned(const char *label) override {
    log.push_back(std::string("unassigned ") + label);
  }
  void onCommit(InputMode mode, const char *digits, uint8_t targetVfo) override {
    log.push_back(std::string("commit ") + modeName(mode) + " [" + digits + "] vfo" +
                  std::to_string(targetVfo));
  }
  bool onModeDigit(char key, uint8_t &mode) override {
    const bool ok = key != '\0' && validModeDigits.find(key) != std::string::npos;
    log.push_back(std::string("mode digit ") + key + (ok ? "" : " invalid"));
    if (ok) mode = (uint8_t)(key - '0');
    return ok;
  }
  void onModeCommit(uint8_t mode, uint8_t targetVfo) override {
    log.push_back("mode commit " + std::to_string(mode) + " vfo" + std::to_string(targetVfo));
  }
  bool sendStagedCommand() override {
    if (!hasStagedCommand) return false;
    hasStagedCommand = false;
    log.push_back("staged command");
    return true;
  }
  void onClear() override { log.push_back("clear"); }

  // The log since the last call, then cleared.
  std::vector<std::string> take() {
    std::vector<std::string> out;
    out.swap(log);
    return out;
  }
};

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
  CHECK_LOG(f, "unassigned BANK4 B");
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

TEST(input_hold_without_long_action_runs_short_on_release) {
  Fake f;
  f.shorts = {"1 7"};
  KeypadInput in(f);
  hold(in, '7');
  CHECK_LOG(f, "short 1 7");
}

TEST(input_hold_without_any_action_is_unassigned_on_release) {
  Fake f;
  KeypadInput in(f);
  hold(in, 'C');
  CHECK_LOG(f, "unassigned BANK1 C");
}

// Fixed by construction: FT-817 Bank 3 '1' long used to type its own release
// into the frequency entry it began.
TEST(input_release_of_hold_that_began_entry_is_not_typed) {
  Fake f;
  KeypadInput in(f);
  in.setBank(3);
  f.holds = {"3 1"};
  f.hooks["hold 3 1"] = [&] { in.beginEntry(InputMode::FreqEntry, KEYPAD_VFO_CURRENT); };
  hold(in, '1');
  CHECK_LOG(f, "hold 3 1");
  CHECK_EQ(in.mode(), InputMode::FreqEntry);
  CHECK_EQ(in.digits(), "");
  tap(in, '1');
  CHECK_LOG(f, "digit FreqEntry 1 1");
}

// Fixed by construction: '#' used to clear the hold flags, so a key held
// through '#' ran its short action on release.
TEST(input_clear_keeps_hold_of_key_still_down) {
  Fake f;
  f.holds = {"1 3"};
  f.shorts = {"1 3"};
  KeypadInput in(f);
  in.onKey('3', KeyGesture::Pressed, 0);
  in.onKey('3', KeyGesture::Held, 0);
  tap(in, '#');
  in.onKey('3', KeyGesture::Released, 0);
  CHECK_LOG(f, "hold 1 3", "clear");
}

TEST(input_hold_of_d_and_hash_does_nothing_and_release_still_acts) {
  Fake f;
  KeypadInput in(f);
  hold(in, 'D');
  hold(in, '#');
  CHECK_LOG(f, "unassigned ENTER", "clear");
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

TEST(input_handled_hold_cancels_waiting_short) {
  Fake f;
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  f.holds = {"4 2"};
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  hold(in, '2', 1100);
  in.poll(2000);
  CHECK_LOG(f, "hold 4 2");
}

TEST(input_unhandled_hold_keeps_waiting_short) {
  Fake f;
  f.waits = {"4 0"};
  f.shorts = {"4 0"};
  KeypadInput in(f);
  in.setBank(4);
  tap(in, '0', 1000);
  in.onKey('7', KeyGesture::Pressed, 1100);
  in.onKey('7', KeyGesture::Held, 1100);
  in.poll(2000);
  CHECK_LOG(f, "short 4 0");
}

TEST(input_other_key_during_wait_runs_first) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0", "1 7"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  tap(in, '7', 1100);
  in.poll(1300);
  CHECK_LOG(f, "short 1 7", "short 1 0");
}

// Kept from the old dispatcher: a second waiting key replaces the first, which
// is dropped.
TEST(input_other_waiting_key_replaces_waiting_key) {
  Fake f;
  f.waits = {"1 0", "1 1"};
  f.shorts = {"1 0", "1 1"};
  KeypadInput in(f);
  tap(in, '0', 1000);
  tap(in, '1', 1100);
  in.poll(1400);
  CHECK_LOG(f, "short 1 1");
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
  CHECK_LOG(f, "short 3 0");
}

TEST(input_deferred_short_without_action_is_unassigned) {
  Fake f;
  f.waits = {"2 9"};
  KeypadInput in(f);
  in.setBank(2);
  tap(in, '9', 1000);
  in.poll(1300);
  CHECK_LOG(f, "unassigned BANK2 9");
}

// --- Bank ('*') ----------------------------------------------------------------

TEST(input_star_short_says_bank) {
  Fake f;
  KeypadInput in(f);
  in.setBank(6);
  tap(in, '*');
  CHECK_LOG(f, "bank? 6");
}

TEST(input_bank_select_takes_last_digit_and_commits) {
  Fake f;
  KeypadInput in(f);
  hold(in, '*');
  CHECK_EQ(in.mode(), InputMode::BankSelect);
  typeKeys(in, "35");
  CHECK_EQ(in.bank(), 1);
  tap(in, 'D');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_EQ(in.bank(), 5);
  CHECK_LOG(f, "bank please", "digit BankSelect 3 3", "digit BankSelect 5 5",
                        "commit BankSelect [5] vfo0");
  // The next short '*' is not mistaken for the release of the hold.
  tap(in, '*');
  CHECK_LOG(f, "bank? 5");
}

TEST(input_bank_select_without_digit_keeps_bank) {
  Fake f;
  KeypadInput in(f);
  in.setBank(2);
  hold(in, '*');
  tap(in, 'D');
  CHECK_EQ(in.bank(), 2);
  CHECK_LOG(f, "bank please", "commit BankSelect [] vfo0");
}

TEST(input_bank_select_rejects_other_keys) {
  Fake f;
  f.holds = {"1 3"};
  KeypadInput in(f);
  hold(in, '*');
  typeKeys(in, "0A");
  tap(in, '*');  // silent in bank select
  hold(in, '3');  // holds are ignored, the release is a digit
  CHECK_EQ(in.mode(), InputMode::BankSelect);
  CHECK_LOG(f, "bank please", "unassigned BANK SELECT 0", "unassigned BANK SELECT A",
                        "digit BankSelect 3 3");
}

TEST(input_bank_select_clear) {
  Fake f;
  KeypadInput in(f);
  hold(in, '*');
  typeKeys(in, "4#");
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_EQ(in.bank(), 1);
  CHECK_LOG(f, "bank please", "digit BankSelect 4 4", "clear");
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
  CHECK_LOG(f, "hold 9 A", "unassigned PROFILE 0", "digit ProfileSelect 1 1",
                        "digit ProfileSelect 2 12", "unassigned PROFILE 3",
                        "unassigned PROFILE B", "unassigned PROFILE *", "unassigned PROFILE A",
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
  in.beginEntry(InputMode::FreqEntry, KEYPAD_VFO_B);
  typeKeys(in, "*14**");
  CHECK_LOG(f, "unassigned FREQ POINT", "digit FreqEntry 1 1", "digit FreqEntry 4 14",
                        "digit FreqEntry * 14*", "unassigned FREQ POINT");
  typeKeys(in, "123456");
  CHECK_LOG(f, "digit FreqEntry 1 14*1", "digit FreqEntry 2 14*12",
                        "digit FreqEntry 3 14*123", "digit FreqEntry 4 14*1234",
                        "digit FreqEntry 5 14*12345", "unassigned FREQ 6");
  typeKeys(in, "AD");
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "unassigned ENTRY A", "commit FreqEntry [14*12345] vfo2");
}

TEST(input_frequency_entry_length_limit) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::FreqEntry);
  typeKeys(in, "123456789012");
  f.take();
  tap(in, '3');
  CHECK_LOG(f, "unassigned FREQ 3");
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
  CHECK_LOG(f, "digit FreqEntry 4 1234567*1234", "unassigned FREQ 5");
}

TEST(input_rf_power_entry_limits) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::RfPowerEntry);
  typeKeys(in, "1004*BD");
  CHECK_LOG(f, "digit RfPowerEntry 1 1", "digit RfPowerEntry 0 10",
                        "digit RfPowerEntry 0 100", "unassigned RFPOWER 4",
                        "unassigned ENTRY *", "unassigned ENTRY B",
                        "commit RfPowerEntry [100] vfo0");
}

TEST(input_civ_address_entry_limits) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::CivAddrEntry);
  typeKeys(in, "1488*D");
  CHECK_LOG(f, "digit CivAddrEntry 1 1", "digit CivAddrEntry 4 14",
                        "digit CivAddrEntry 8 148", "unassigned CIVADDR 8",
                        "unassigned ENTRY *", "commit CivAddrEntry [148] vfo0");
}

TEST(input_bank6_entry_limits) {
  struct Case {
    InputMode mode;
    const char *accepted;
  };
  const Case cases[] = {
      {InputMode::RptOffsetEntry, "5000"},
      {InputMode::CtcssEntry, "1318"},
      {InputMode::DcsEntry, "023"},
  };
  for (const Case &c : cases) {
    Fake f;
    KeypadInput in(f);
    in.beginEntry(c.mode);
    typeKeys(in, c.accepted);
    f.take();
    typeKeys(in, "9*");
    CHECK_LOG(f, "unassigned BANK6 ENTRY 9", "unassigned ENTRY *");
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

TEST(input_entry_starts_empty_each_time) {
  Fake f;
  KeypadInput in(f);
  in.beginEntry(InputMode::RfPowerEntry);
  typeKeys(in, "5#");
  in.beginEntry(InputMode::RfPowerEntry);
  tap(in, 'D');
  CHECK_LOG(f, "digit RfPowerEntry 5 5", "clear", "commit RfPowerEntry [] vfo0");
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
    in.beginEntry(m, KEYPAD_VFO_A);
    typeKeys(in, "1#");
    CHECK_EQ(in.mode(), InputMode::Normal);
    CHECK_EQ(in.digits(), "");
    CHECK_EQ(in.entryTargetVfo(), KEYPAD_VFO_CURRENT);
    const Log log = f.take();
    CHECK(!log.empty() && log.back() == "clear");
  }
}

TEST(input_clear_cancels_staged_mode_and_waiting_short) {
  Fake f;
  f.waits = {"1 0"};
  f.shorts = {"1 0"};
  KeypadInput in(f);
  in.beginModeSelect(KEYPAD_VFO_A);
  tap(in, '3');
  CHECK(in.stagedModeActive());
  tap(in, '#');
  CHECK_EQ(in.mode(), InputMode::Normal);
  tap(in, '0', 1000);
  tap(in, '#', 1100);
  in.poll(2000);
  tap(in, 'D');
  CHECK_LOG(f, "mode digit 3", "clear", "clear", "unassigned ENTER");
}

TEST(input_enter_in_normal_mode_routes_staged_mode_then_staged_command) {
  Fake f;
  KeypadInput in(f);
  f.hasStagedCommand = true;
  in.beginModeSelect(KEYPAD_VFO_B);
  tap(in, '4');
  tap(in, 'D');
  tap(in, 'D');
  tap(in, 'D');
  CHECK_LOG(f, "mode digit 4", "mode commit 4 vfo2", "staged command",
                        "unassigned ENTER");
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
  in.beginModeSelect(KEYPAD_VFO_B);
  tap(in, '2');
  tap(in, '7');
  hold(in, '0');
  hold(in, '*');
  CHECK_EQ(in.mode(), InputMode::ModeStaged);
  tap(in, '1');
  tap(in, 'D');
  CHECK_EQ(in.mode(), InputMode::Normal);
  CHECK_LOG(f, "mode digit 2", "mode digit 7 invalid", "mode digit 0 invalid",
            "mode digit * invalid", "mode digit 1", "mode commit 1 vfo2");
}

// --- Mode select -----------------------------------------------------------------------

TEST(input_mode_select_stages_mode_for_target_vfo) {
  Fake f;
  f.waits = {"3 1"};
  KeypadInput in(f);
  in.setBank(3);
  in.beginModeSelect(KEYPAD_VFO_A);
  tap(in, '1', 1000);  // no double-click wait during mode select
  CHECK(!in.modeSelectActive());
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
  in.beginModeSelect(KEYPAD_VFO_B);
  typeKeys(in, "7A*");
  CHECK_EQ(in.mode(), InputMode::ModeSelect);
  tap(in, '2');
  CHECK(!in.modeSelectActive());
  tap(in, 'D');
  CHECK_LOG(f, "mode digit 7 invalid", "mode digit A invalid", "mode digit * invalid",
            "mode digit 2", "mode commit 2 vfo2");
}

TEST(input_mode_select_enter_beeps_and_keeps_it_active) {
  Fake f;
  f.hasStagedCommand = true;
  KeypadInput in(f);
  in.beginModeSelect(KEYPAD_VFO_CURRENT);
  tap(in, 'D');
  CHECK(in.modeSelectActive());
  CHECK(f.hasStagedCommand);
  CHECK_LOG(f, "unassigned MODE SELECT D");
}

TEST(input_mode_select_clear_cancels_it) {
  Fake f;
  KeypadInput in(f);
  in.beginModeSelect(KEYPAD_VFO_A);
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
  in.beginModeSelect(KEYPAD_VFO_A);
  typeKeys(in, "0#");
  in.beginModeSelect(KEYPAD_VFO_CURRENT);
  tap(in, '5');
  tap(in, 'D');
  CHECK_LOG(f, "mode digit 0 invalid", "clear", "mode digit 5", "mode commit 5 vfo0");
}

// F1: every key is the mode digit, even one with a short action on the bank
// (Bank 3 '7' is a band-stack query).
TEST(input_mode_select_key_is_mode_digit_not_short_action) {
  Fake f;
  f.shorts = {"3 7"};
  KeypadInput in(f);
  in.setBank(3);
  in.beginModeSelect(KEYPAD_VFO_A);
  tap(in, '7');
  CHECK(!in.modeSelectActive());
  CHECK(in.stagedModeActive());
  CHECK_LOG(f, "mode digit 7");
}

// F2: holds during mode select run no long action and do not open bank select;
// the release is the mode digit.
TEST(input_mode_select_ignores_holds) {
  Fake f;
  f.holds = {"1 0", "1 2"};
  KeypadInput in(f);
  in.beginModeSelect(KEYPAD_VFO_CURRENT);
  hold(in, '0');
  hold(in, '*');
  CHECK_EQ(in.mode(), InputMode::ModeSelect);
  hold(in, '2');
  CHECK(!in.modeSelectActive());
  CHECK(in.stagedModeActive());
  CHECK_EQ(in.bank(), 1);
  CHECK_LOG(f, "mode digit 0 invalid", "mode digit * invalid", "mode digit 2");
}
