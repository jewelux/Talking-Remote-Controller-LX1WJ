// keypad_keymap against the characterization matrix, for every radio family.
// A recording stub implements keypad_actions.h: each action appends its name
// and arguments, spelled as in keymap_expectations.inc.
#include <cstdarg>
#include <initializer_list>
#include <string>

#include "keymap_expectations.h"
#include "keypad_actions.h"
#include "keypad_keymap.h"
#include "keypad_labels.h"
#include "test_runner.h"

using namespace keymap_expect;

namespace {

std::string g_calls;
// The key and gesture outcome() runs, as KeypadInput names it for
// keypadActiveKey(): "BANK2 8 SHORT".
char g_key[24] = "";

void record(const char *fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (!g_calls.empty()) g_calls += ", ";
  g_calls += buf;
}

KeypadTraits traitsFor(uint16_t family) {
  KeypadTraits t;
  if (family & CIV) t.layout = KeypadLayout::Civ;
  else if (family == FTDX10) t.layout = KeypadLayout::Ftdx10;
  else if (family == FT817) t.layout = KeypadLayout::Ft817;
  else if (family == FT857) t.layout = KeypadLayout::Ft857;
  else if (family == FT8X7_OTHER) t.layout = KeypadLayout::Ft8x7;
  // Monitor and transceive are PROTO_CIV only (radio_protocol.cpp).
  t.supportsMonitor = (family & CIV) != 0;
  t.supportsTransceive = (family & CIV) != 0;
  t.canGetRfPower = family == IC7300 || family == TS480 || family == FTDX10 || family == FT891;
  // The Bank 2, 4 and 7 settings, as the profiles' caps give them.
  using S = Setting;
  auto add = [&t](std::initializer_list<Setting> settings) {
    for (Setting s : settings) t.settings.add(s);
  };
  if (family == IC7300) {
    add({S::NrLevel, S::NbLevel, S::Notch, S::PbtInner, S::PbtOuter, S::FilterSlot, S::Agc, S::Preamp, S::Att});
  }
  if (family == TS480) add({S::Agc, S::Preamp, S::Att});
  if (family == FTDX10 || family == FT891) add({S::NrLevel, S::NbLevel, S::Agc, S::Preamp, S::Att});
  if (family == FT817) add({S::Agc});
  if (family == FT857) add({S::NrLevel, S::NbLevel, S::RxLowCut, S::RxHighCut, S::BpfWidth, S::Agc, S::Att});
  if (family == IC7300) add({S::Monitor, S::MonitorLevel});
  if (family == IC7300 || family == TS480 || family == FTDX10 || family == FT891 || family == FT857) {
    add({S::Proc, S::ProcLevel});
  }
  if (family != IC706 && family != FT8X7_OTHER) add({S::Vox, S::VoxGain, S::MicGain, S::VoxDelay});
  if (family == FT857) add({S::TxEq, S::TxLowCut, S::TxHighCut});
  // Bank 7.
  if (family == IC7300) add({S::BreakIn, S::CwPitch});
  if (family == TS480) add({S::CwSpeed, S::CwDelay});
  if (family == FTDX10 || family == FT891) add({S::Keyer, S::CwSpeed, S::BreakIn, S::CwPitch, S::CwDelay});
  if (family == FT817 || family == FT857) {
    add({S::Keyer, S::CwSpeed, S::BreakIn, S::Sidetone, S::CwPitch, S::CwWeight, S::KeyReverse, S::CwDelay});
  }
  if (family == FT857) add({S::Qsk, S::Paddle});
  // Bank 6.
  if (family & (FT8X7 | FTDX10 | FT891)) add({S::RptShift, S::ToneType, S::Ctcss});
  if (family == IC7300) add({S::ToneType, S::Ctcss});
  if (family & FT8X7) add({S::Dcs});
  return t;
}

// Runs one bound action and returns what it did: the recorded actions, or
// sentinel when the key has no action for the gesture.
std::string outcome(KeyAction action, uint8_t bank, char key, const char *gesture,
                    const char *sentinel) {
  g_calls.clear();
  if (!action) return sentinel;
  snprintf(g_key, sizeof(g_key), "BANK%u %c %s", (unsigned)bank, key, gesture);
  action();
  g_key[0] = '\0';
  return g_calls.empty() ? "(bound, nothing ran)" : g_calls;
}

bool isEmpty(const KeyBinding &b) {
  return !b.shortAction && !b.holdAction && !b.doubleAction && !b.doubleHoldAction &&
         !b.waitsForDouble && b.focus == Setting::None;
}

void checkOutcome(const Family &f, uint8_t bank, char key, const char *gesture,
                  const std::string &actual, const char *expected) {
  if (actual == expected) return;
  fprintf(stderr, "  %s bank %u key %c %s: got \"%s\", expected \"%s\"\n", f.name, bank, key,
          gesture, actual.c_str(), expected);
  ++::test::g_checkFailures;
}

template <typename Fn>
void forEachKey(Fn fn) {
  for (const Family &f : kFamilies) {
    const KeypadTraits t = traitsFor(f.bit);
    for (uint8_t bank = 1; bank <= 9; ++bank) {
      for (const char *k = kBankKeys; *k; ++k) fn(f, t, bank, *k);
    }
  }
}

}  // namespace

// ---- Recording stub for keypad_actions.h ----

void sendKeypadCommand(const char *cmd) { record("send(%s)", cmd); }

void settingSay(Setting s) { record("settingSay(%s)", settingInfo(s).label); }
void settingEntry(Setting s) { record("settingEntry(%s)", settingInfo(s).label); }
void settingNext(Setting s) { record("settingNext(%s)", settingInfo(s).label); }

void focusSay() { record("focusSay"); }
void focusEntry() { record("focusEntry"); }
void focusCentre() { record("focusCentre"); }
void focusStep(int8_t d) { record("focusStep(%d)", (int)d); }
void focusBigStep(int8_t d) { record("focusBigStep(%d)", (int)d); }

void queryBank1Frequency() { record("queryBank1Frequency"); }
void beginBank1FrequencySet() { record("beginBank1FrequencySet"); }
void roundActiveFrequency(uint32_t hz) { record("roundActiveFrequency(%u)", (unsigned)hz); }
void queryBank1RxTx() { record("queryBank1RxTx"); }
void queryBank1TxFrequency() { record("queryBank1TxFrequency"); }
void queryBank1Ft857TxFrequency() { record("queryBank1Ft857TxFrequency"); }
void queryBank1Lock() { record("queryBank1Lock"); }
void toggleBank1Lock() { record("toggleBank1Lock"); }
void queryBank1Power() { record("queryBank1Power"); }
void queryBank1RfPower() { record("queryBank1RfPower"); }
void beginBank1RfPowerSet() { record("beginBank1RfPowerSet"); }
void queryBank1Smeter() { record("queryBank1Smeter"); }
void queryBank1Swr() { record("queryBank1Swr"); }
void queryBank1Mode() { record("queryBank1Mode"); }
void beginBank1ModeSelect() { record("beginBank1ModeSelect"); }

void queryBank2Nr() { record("queryBank2Nr"); }
void toggleBank2Nr() { record("toggleBank2Nr"); }
void queryBank2Nb() { record("queryBank2Nb"); }
void toggleBank2Nb() { record("toggleBank2Nb"); }
void queryBank2Notch() { record("queryBank2Notch"); }
void toggleBank2Notch() { record("toggleBank2Notch"); }
void toggleBank2FilterShape() { record("toggleBank2FilterShape"); }
void queryBank2Ft8x7Dbf() { record("queryBank2Ft8x7Dbf"); }
void queryBank2Ft8x7Ipo() { record("queryBank2Ft8x7Ipo"); }
void queryBank1Alc() { record("queryBank1Alc"); }
void queryBank2AutoNotch() { record("queryBank2AutoNotch"); }
void toggleBank2AutoNotch() { record("toggleBank2AutoNotch"); }
void queryBank2Att() { record("queryBank2Att"); }
void stepBank2Att() { record("stepBank2Att"); }
void queryBank2Preamp() { record("queryBank2Preamp"); }
void stepBank2Preamp() { record("stepBank2Preamp"); }
void queryBank2Agc() { record("queryBank2Agc"); }
void queryBank2Ft817Antenna() { record("queryBank2Ft817Antenna"); }
void toggleBank2Ft8x7Dbf() { record("toggleBank2Ft8x7Dbf"); }
void toggleBank2Ft817Antenna() { record("toggleBank2Ft817Antenna"); }
void beginBank2AgcSelect() { record("beginBank2AgcSelect"); }

void queryBank3Split() { record("queryBank3Split"); }
void toggleBank3Split() { record("toggleBank3Split"); }
void queryBank3TxFrequency() { record("queryBank3TxFrequency"); }
void queryBank3VfoA() { record("queryBank3VfoA"); }
void selectBank3VfoA() { record("selectBank3VfoA"); }
void beginBank3VfoAFrequencySet() { record("beginBank3VfoAFrequencySet"); }
void queryBank3VfoB() { record("queryBank3VfoB"); }
void selectBank3VfoB() { record("selectBank3VfoB"); }
void beginBank3VfoBFrequencySet() { record("beginBank3VfoBFrequencySet"); }
void queryBank3Ft8x7CurrentVfo() { record("queryBank3Ft8x7CurrentVfo"); }
void beginBank3Ft8x7CurrentVfoFrequencySet() { record("beginBank3Ft8x7CurrentVfoFrequencySet"); }
void beginBank3Ft8x7OtherVfoFrequencySet() { record("beginBank3Ft8x7OtherVfoFrequencySet"); }
void queryBank3Ft8x7OtherVfo() { record("queryBank3Ft8x7OtherVfo"); }
void toggleBank3Ft8x7Vfo() { record("toggleBank3Ft8x7Vfo"); }
void copyBank3Ft817VfoToOther() { record("copyBank3Ft817VfoToOther"); }
void reportBank3Ft857VfoBUnsupported() { record("reportBank3Ft857VfoBUnsupported"); }
void queryBank3VfoAMode() { record("queryBank3VfoAMode"); }
void beginBank3VfoAModeSet() { record("beginBank3VfoAModeSet"); }
void queryBank3VfoBMode() { record("queryBank3VfoBMode"); }
void beginBank3VfoBModeSet() { record("beginBank3VfoBModeSet"); }
void syncBank3VfoA() { record("syncBank3VfoA"); }
void syncBank3VfoB() { record("syncBank3VfoB"); }
void queryBank3Ft8x7Rit() { record("queryBank3Ft8x7Rit"); }
void toggleBank3Ft8x7Rit() { record("toggleBank3Ft8x7Rit"); }
void selectBank3Ft817ActiveVfoA() { record("selectBank3Ft817ActiveVfoA"); }
void selectBank3Ft817ActiveVfoB() { record("selectBank3Ft817ActiveVfoB"); }
void queryBank3RxTx() { record("queryBank3RxTx"); }
void setBank3Ft857Ptt(bool on) { record("setBank3Ft857Ptt(%s)", on ? "true" : "false"); }
void queryBank3BandStack(uint8_t r) { record("queryBank3BandStack(%u)", r); }
void recallBank3BandStack(uint8_t r) { record("recallBank3BandStack(%u)", r); }

void queryBank4Tuner() { record("queryBank4Tuner"); }
void toggleBank4Tuner() { record("toggleBank4Tuner"); }
void triggerBank4Tune() { record("triggerBank4Tune"); }

void queryBank5Rit() { record("queryBank5Rit"); }
void toggleBank5Rit() { record("toggleBank5Rit"); }
void setBank5RitOffset(int32_t hz) { record("setBank5RitOffset(%d)", (int)hz); }
void adjustBank5Rit(int32_t d) { record("adjustBank5Rit(%d)", (int)d); }
void setBank5RitOff() { record("setBank5RitOff"); }

void setBank6RepeaterOff() { record("setBank6RepeaterOff"); }
void setBank6RepeaterMinus() { record("setBank6RepeaterMinus"); }
void setBank6RepeaterPlus() { record("setBank6RepeaterPlus"); }
void setBank6RepeaterOffsetPreset(uint8_t p) { record("setBank6RepeaterOffsetPreset(%u)", (unsigned)p); }
void beginBank6RepeaterOffsetEntry() { record("beginBank6RepeaterOffsetEntry"); }
void setBank6ToneOff() { record("setBank6ToneOff"); }
void setBank6ToneModeCtcss() { record("setBank6ToneModeCtcss"); }
void setBank6ToneModeDcs() { record("setBank6ToneModeDcs"); }
void queryBank6Tone() { record("queryBank6Tone"); }
void queryBank6ToneDir() { record("queryBank6ToneDir"); }
void stepBank6ToneDir() { record("stepBank6ToneDir"); }
void queryBank6SplitTone() { record("queryBank6SplitTone"); }

void queryBank8CivAddress() { record("queryBank8CivAddress"); }
void beginBank8CivAddressEntry() { record("beginBank8CivAddressEntry"); }
void queryBank8Baud() { record("queryBank8Baud"); }
void cycleBank8Baud(int d) { record("cycleBank8Baud(%d)", d); }
void toggleBank8Transceive() { record("toggleBank8Transceive"); }
void queryBank8Ft8x7Row() { record("queryBank8Ft8x7Row"); }
void queryBank8Ft8x7Menu() { record("queryBank8Ft8x7Menu"); }
void beginBank8Ft8x7RowEntry() { record("beginBank8Ft8x7RowEntry"); }
void beginBank8Ft8x7MenuEntry() { record("beginBank8Ft8x7MenuEntry"); }

void queryBank9TuningSpeech() { record("queryBank9TuningSpeech"); }
void toggleBank9TuningSpeech() { record("toggleBank9TuningSpeech"); }
void queryBank9Verbose() { record("queryBank9Verbose"); }
void toggleBank9Verbose() { record("toggleBank9Verbose"); }
void queryBank9Experimental() { record("queryBank9Experimental"); }
void toggleBank9Experimental() { record("toggleBank9Experimental"); }
void adjustBank9Volume(int d) { record("adjustBank9Volume(%d)", d); }
void queryBank9Volume() { record("queryBank9Volume"); }
void queryBank9Speed() { record("queryBank9Speed"); }
void stepBank9Speed() { record("stepBank9Speed"); }
void queryBank9Profile() { record("queryBank9Profile"); }
void beginBank9ProfileSelect() { record("beginBank9ProfileSelect"); }
void resetBank9Profile() { record("resetBank9Profile"); }
void selectNextProfile() { record("selectNextProfile"); }
void selectPrevProfile() { record("selectPrevProfile"); }

// Records the key the action is bound to.
void reportFtdx10HiddenKey() { record("reportFtdx10HiddenKey(%s)", g_key); }

// ---- Tests ----

TEST(keymap_short_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    const KeyAction action = keymapLookup(t, bank, key).shortAction;
    checkOutcome(f, bank, key, "short", outcome(action, bank, key, "SHORT", "unassigned"),
                 lookup(f.bit, bank, key).shortAction);
  });
}

TEST(keymap_hold_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    const KeyAction action = keymapLookup(t, bank, key).holdAction;
    checkOutcome(f, bank, key, "long", outcome(action, bank, key, "LONG", "none"),
                 lookup(f.bit, bank, key).longAction);
  });
}

TEST(keymap_double_click_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    const char *expected = lookup(f.bit, bank, key).doubleAction;
    const bool waits = strcmp(expected, "-") != 0;
    g_calls.clear();
    const KeyBinding b = keymapLookup(t, bank, key);
    CHECK(g_calls.empty());
    if (b.waitsForDouble != waits) {
      fprintf(stderr, "  %s bank %u key %c: waitsForDouble %d, expected %d\n", f.name, bank,
              key, b.waitsForDouble, waits);
      ++::test::g_checkFailures;
    }
    // A key that does not wait never gets a double click, so it has no action.
    if (b.doubleAction && !b.waitsForDouble) {
      fprintf(stderr, "  %s bank %u key %c: double action without the wait\n", f.name, bank, key);
      ++::test::g_checkFailures;
    }
    if (b.doubleHoldAction && !b.waitsForDouble) {
      fprintf(stderr, "  %s bank %u key %c: double hold action without the wait\n", f.name, bank,
              key);
      ++::test::g_checkFailures;
    }
    checkOutcome(f, bank, key, "double",
                 outcome(b.doubleAction, bank, key, "DOUBLE", waits ? "none" : "-"), expected);
  });
}

TEST(keymap_double_hold_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    const KeyAction action = keymapLookup(t, bank, key).doubleHoldAction;
    checkOutcome(f, bank, key, "double hold", outcome(action, bank, key, "DOUBLE LONG", "none"),
                 lookup(f.bit, bank, key).doubleHoldAction);
  });
}

TEST(keymap_focus_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    const Setting focus = keymapLookup(t, bank, key).focus;
    checkOutcome(f, bank, key, "focus", focus == Setting::None ? "-" : settingInfo(focus).label,
                 lookup(f.bit, bank, key).focus);
  });
}

// A waits for a double only when the focus has a centre; the bank's own focus is passed in.
TEST(keymap_focus_keys_follow_the_focus) {
  const KeypadTraits t = traitsFor(IC7300);
  CHECK(keymapLookup(t, 9, 'A').waitsForDouble);
  CHECK(keymapLookup(t, 9, 'A', Setting::SpeechSpeed).waitsForDouble);
  CHECK(!keymapLookup(t, 9, 'A', Setting::Volume).waitsForDouble);
  CHECK(!keymapLookup(t, 9, 'A', Setting::None).waitsForDouble);
  CHECK(!keymapLookup(t, 1, 'A', Setting::RfPower).waitsForDouble);
  CHECK(keymapLookup(t, 1, 'B', Setting::Mode).shortAction != nullptr);
  CHECK(keymapHomeFocus(t, 5) == Setting::RitOffset);
  CHECK(keymapHomeFocus(t, 9) == Setting::Profile);
  CHECK(keymapHomeFocus(t, 1) == Setting::None);
  CHECK(keymapLookup(t, 2, 'A', Setting::PbtInner).waitsForDouble);
  CHECK(keymapLookup(t, 7, 'A', Setting::BreakIn).shortAction != nullptr);
  CHECK(keymapLookup(t, 4, 'A', Setting::MicGain).shortAction != nullptr);
}

TEST(keymap_ignores_state_machine_keys_and_other_banks) {
  const KeypadTraits t = traitsFor(IC7300);
  g_calls.clear();
  for (const char *k = "*#D"; *k; ++k) {
    for (uint8_t bank = 0; bank <= 10; ++bank) CHECK(isEmpty(keymapLookup(t, bank, *k)));
  }
  CHECK(isEmpty(keymapLookup(t, 0, '1')));
  CHECK(isEmpty(keymapLookup(t, 8, '9')));
  CHECK(isEmpty(keymapLookup(t, 10, '1')));
  CHECK(g_calls.empty());
}

// Help mode says every gesture that has an action, by a name kSpokenLabels has; a gesture with
// no action has no help. "not available" may stand alone (A, B and C with no focus).
void checkHelp(const Family &f, uint8_t bank, char key, Setting focus, const char *gesture, KeyAction action,
               const GestureHelp &help) {
  const char *problem = nullptr;
  if (!action) {
    if (help.verb != HelpVerb::None) problem = "help without an action";
  } else if (help.verb == HelpVerb::None) {
    problem = "no help";
  } else if (!help.label) {
    if (help.verb != HelpVerb::NotAvailable) problem = "help without a label";
  } else if (!spokenLabelWords(help.label)) {
    problem = "help label with no spoken name";
  }
  if (!problem) return;
  fprintf(stderr, "  %s bank %u key %c focus %s %s: %s (%s)\n", f.name, bank, key,
          settingInfo(focus).label, gesture, problem, help.label ? help.label : "-");
  ++::test::g_checkFailures;
}

void checkBindingHelp(const Family &f, uint8_t bank, char key, Setting focus, const KeyBinding &b) {
  checkHelp(f, bank, key, focus, "short", b.shortAction, b.shortHelp);
  checkHelp(f, bank, key, focus, "long", b.holdAction, b.holdHelp);
  checkHelp(f, bank, key, focus, "double", b.doubleAction, b.doubleHelp);
  checkHelp(f, bank, key, focus, "double hold", b.doubleHoldAction, b.doubleHoldHelp);
}

TEST(keymap_every_gesture_has_help) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    checkBindingHelp(f, bank, key, Setting::Home, keymapLookup(t, bank, key));
    if (key != 'A' && key != 'B' && key != 'C') return;
    // A, B and C for every focus a key could give them.
    for (uint8_t s = 0; s < (uint8_t)Setting::Count; ++s) {
      if ((Setting)s == Setting::Home) continue;
      checkBindingHelp(f, bank, key, (Setting)s, keymapLookup(t, bank, key, (Setting)s));
    }
  });
}

TEST(keymap_focus_keys_help_names_the_focus) {
  const KeypadTraits t = traitsFor(IC7300);
  const KeyBinding a = keymapLookup(t, 9, 'A');
  CHECK(strcmp(a.shortHelp.label, "PROFILE") == 0);
  CHECK(a.shortHelp.verb == HelpVerb::Say);
  CHECK(a.holdHelp.verb == HelpVerb::Pick);
  CHECK(a.doubleHelp.verb == HelpVerb::Centre);
  const KeyBinding b = keymapLookup(t, 1, 'B', Setting::RfPower);
  CHECK(strcmp(b.shortHelp.label, "RFPOWER") == 0);
  CHECK(b.shortHelp.verb == HelpVerb::Up);
  CHECK(b.holdHelp.verb == HelpVerb::UpFast);
  CHECK(keymapLookup(t, 1, 'A', Setting::RfPower).holdHelp.verb == HelpVerb::Entry);
  const KeyBinding none = keymapLookup(t, 1, 'C', Setting::None);
  CHECK(none.shortHelp.label == nullptr);
  CHECK(none.shortHelp.verb == HelpVerb::NotAvailable);
}
