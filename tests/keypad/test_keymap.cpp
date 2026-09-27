// keypad_keymap against the characterization matrix, for every radio family.
// A recording stub implements keypad_actions.h: each action appends its name
// and arguments, spelled as in keymap_expectations.inc.
#include <cstdarg>
#include <string>

#include "keymap_expectations.h"
#include "keypad_actions.h"
#include "keypad_keymap.h"
#include "test_runner.h"

using namespace keymap_expect;

namespace {

std::string g_calls;

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
  t.civ = (family & CIV) != 0;
  t.ftdx10 = family == FTDX10;
  t.ft817 = family == FT817;
  t.ft857Family = family == FT857;
  t.lightIcomFallback = family == LIGHT;
  // Monitor and transceive are PROTO_CIV only (radio_protocol.cpp).
  t.supportsMonitor = t.civ;
  t.supportsTransceive = t.civ;
  t.canGetRfPower = family == IC7300 || family == LIGHT;
  return t;
}

// Runs one keymap call and returns what it did: the recorded actions, or
// sentinel when it reported that the key has no action.
std::string outcome(bool (*fn)(const KeypadTraits &, uint8_t, char), const KeypadTraits &t,
                    uint8_t bank, char key, const char *sentinel) {
  g_calls.clear();
  const bool ran = fn(t, bank, key);
  if (!ran) return g_calls.empty() ? sentinel : "(false after " + g_calls + ")";
  return g_calls.empty() ? "(true, nothing ran)" : g_calls;
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

void queryBank1Frequency() { record("queryBank1Frequency"); }
void beginBank1FrequencySet() { record("beginBank1FrequencySet"); }
void roundActiveFrequency500() { record("roundActiveFrequency500"); }
void queryBank1RxTx() { record("queryBank1RxTx"); }
void queryBank1TxFrequency() { record("queryBank1TxFrequency"); }
void queryBank1Lock() { record("queryBank1Lock"); }
void toggleBank1Lock() { record("toggleBank1Lock"); }
void queryBank1Power() { record("queryBank1Power"); }
void queryBank1RfPower() { record("queryBank1RfPower"); }
void beginBank1RfPowerSet() { record("beginBank1RfPowerSet"); }
void queryBank1Smeter() { record("queryBank1Smeter"); }
void queryBank1Swr() { record("queryBank1Swr"); }
void queryBank1Mode() { record("queryBank1Mode"); }
void beginBank1ModeSelect() { record("beginBank1ModeSelect"); }
void ftdx10QueryTuner() { record("ftdx10QueryTuner"); }
void ftdx10ToggleTuner() { record("ftdx10ToggleTuner"); }
void ftdx10Tune() { record("ftdx10Tune"); }
void ftdx10QueryPreamp() { record("ftdx10QueryPreamp"); }
void ftdx10TogglePreamp() { record("ftdx10TogglePreamp"); }

void queryBank2Nr() { record("queryBank2Nr"); }
void toggleBank2Nr() { record("toggleBank2Nr"); }
void queryBank2Nb() { record("queryBank2Nb"); }
void toggleBank2Nb() { record("toggleBank2Nb"); }
void queryBank2Notch() { record("queryBank2Notch"); }
void toggleBank2Notch() { record("toggleBank2Notch"); }
void queryBank2NrLevel() { record("queryBank2NrLevel"); }
void adjustBank2NrLevel(int d) { record("adjustBank2NrLevel(%d)", d); }
void queryBank2NbLevel() { record("queryBank2NbLevel"); }
void adjustBank2NbLevel(int d) { record("adjustBank2NbLevel(%d)", d); }
void queryBank2PbtInner() { record("queryBank2PbtInner"); }
void adjustBank2PbtInner(int d) { record("adjustBank2PbtInner(%d)", d); }
void queryBank2PbtOuter() { record("queryBank2PbtOuter"); }
void adjustBank2PbtOuter(int d) { record("adjustBank2PbtOuter(%d)", d); }
void sendBank2FilterShapeQuery() { record("sendBank2FilterShapeQuery"); }
void toggleBank2FilterShape() { record("toggleBank2FilterShape"); }
void queryBank2FilterWidth() { record("queryBank2FilterWidth"); }
void cycleBank2FilterWidth(int d) { record("cycleBank2FilterWidth(%d)", d); }
void ftdx10QueryAgc() { record("ftdx10QueryAgc"); }
void ftdx10AgcFast() { record("ftdx10AgcFast"); }
void ftdx10AgcSlow() { record("ftdx10AgcSlow"); }
void ftdx10QueryPowerState() { record("ftdx10QueryPowerState"); }
void ftdx10PowerOff() { record("ftdx10PowerOff"); }
void ftdx10PowerOn() { record("ftdx10PowerOn"); }
void ftdx10QueryInfo() { record("ftdx10QueryInfo"); }
void ftdx10QueryId() { record("ftdx10QueryId"); }

void queryBank3Split() { record("queryBank3Split"); }
void toggleBank3Split() { record("toggleBank3Split"); }
void queryBank3TxFrequency() { record("queryBank3TxFrequency"); }
void calibrateBank3Ft857Split() { record("calibrateBank3Ft857Split"); }
void queryBank3VfoA() { record("queryBank3VfoA"); }
void selectBank3VfoA() { record("selectBank3VfoA"); }
void beginBank3VfoAFrequencySet() { record("beginBank3VfoAFrequencySet"); }
void queryBank3VfoB() { record("queryBank3VfoB"); }
void selectBank3VfoB() { record("selectBank3VfoB"); }
void beginBank3VfoBFrequencySet() { record("beginBank3VfoBFrequencySet"); }
void queryBank4VfoAMode(uint8_t b) { record("queryBank4VfoAMode(%u)", b); }
void beginBank4VfoAModeSet(uint8_t b) { record("beginBank4VfoAModeSet(%u)", b); }
void queryBank4VfoBMode(uint8_t b) { record("queryBank4VfoBMode(%u)", b); }
void beginBank4VfoBModeSet(uint8_t b) { record("beginBank4VfoBModeSet(%u)", b); }
void syncBank3Ft817VfoA() { record("syncBank3Ft817VfoA"); }
void syncBank3Ft817VfoB() { record("syncBank3Ft817VfoB"); }
void syncBank3Ft857VfoA() { record("syncBank3Ft857VfoA"); }
void syncBank3Ft857VfoB() { record("syncBank3Ft857VfoB"); }
void setBank3Ft857Clar(bool on) { record("setBank3Ft857Clar(%s)", on ? "true" : "false"); }
void selectBank3Ft817ActiveVfoA() { record("selectBank3Ft817ActiveVfoA"); }
void selectBank3Ft817ActiveVfoB() { record("selectBank3Ft817ActiveVfoB"); }
void queryBank3RxTx() { record("queryBank3RxTx"); }
void setBank3Ft857Ptt(bool on) { record("setBank3Ft857Ptt(%s)", on ? "true" : "false"); }
void queryBank3BandStack(uint8_t r) { record("queryBank3BandStack(%u)", r); }
void recallBank3BandStack(uint8_t r) { record("recallBank3BandStack(%u)", r); }

void queryBank4Tuner() { record("queryBank4Tuner"); }
void toggleBank4Tuner() { record("toggleBank4Tuner"); }
void triggerBank4Tune() { record("triggerBank4Tune"); }
void sendBank4MonitorQuery() { record("sendBank4MonitorQuery"); }
void toggleBank4Monitor() { record("toggleBank4Monitor"); }
void queryBank4MonitorLevel() { record("queryBank4MonitorLevel"); }
void adjustBank4MonitorLevel(int d) { record("adjustBank4MonitorLevel(%d)", d); }
void sendBank4TransceiveQuery() { record("sendBank4TransceiveQuery"); }
void toggleBank4Transceive() { record("toggleBank4Transceive"); }

void queryBank5Rit() { record("queryBank5Rit"); }
void toggleBank5Rit() { record("toggleBank5Rit"); }
void setBank5RitOffset(int32_t hz) { record("setBank5RitOffset(%d)", (int)hz); }
void adjustBank5Rit(int32_t d) { record("adjustBank5Rit(%d)", (int)d); }
void setBank5RitOff() { record("setBank5RitOff"); }

void queryBank6Repeater() { record("queryBank6Repeater"); }
void setBank6RepeaterMinus() { record("setBank6RepeaterMinus"); }
void setBank6RepeaterPlus() { record("setBank6RepeaterPlus"); }
void queryBank6RepeaterOffset() { record("queryBank6RepeaterOffset"); }
void setBank6RepeaterOffset70cm() { record("setBank6RepeaterOffset70cm"); }
void setBank6RepeaterOffset10m() { record("setBank6RepeaterOffset10m"); }
void queryBank6ToneMode() { record("queryBank6ToneMode"); }
void setBank6ToneModeCtcss() { record("setBank6ToneModeCtcss"); }
void setBank6ToneModeDcs() { record("setBank6ToneModeDcs"); }
void queryBank6CtcssDefault() { record("queryBank6CtcssDefault"); }
void beginBank6CtcssEntry() { record("beginBank6CtcssEntry"); }
void queryBank6DcsDefault() { record("queryBank6DcsDefault"); }
void beginBank6DcsEntry() { record("beginBank6DcsEntry"); }

void queryBank8CivAddress() { record("queryBank8CivAddress"); }
void beginBank8CivAddressEntry() { record("beginBank8CivAddressEntry"); }
void cycleBank8Baud(int d) { record("cycleBank8Baud(%d)", d); }

bool selectBank9DirectProfile(char key) {
  record("selectBank9DirectProfile(%c)", key);
  return true;
}
void queryBank9TuningSpeech() { record("queryBank9TuningSpeech"); }
void toggleBank9TuningSpeech() { record("toggleBank9TuningSpeech"); }
void adjustBank9Volume(int d) { record("adjustBank9Volume(%d)", d); }
void queryBank9Volume() { record("queryBank9Volume"); }
void queryBank9Profile() { record("queryBank9Profile"); }
void beginBank9ProfileSelect() { record("beginBank9ProfileSelect"); }
void selectNextProfile() { record("selectNextProfile"); }
void selectPrevProfile() { record("selectPrevProfile"); }

void reportFtdx10HiddenKey(const char *label) { record("reportFtdx10HiddenKey(%s)", label); }

// ---- Tests ----

TEST(keymap_short_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    checkOutcome(f, bank, key, "short", outcome(keymapShort, t, bank, key, "unassigned"),
                 lookup(f.bit, bank, key).shortAction);
  });
}

TEST(keymap_hold_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    checkOutcome(f, bank, key, "long", outcome(keymapHold, t, bank, key, "none"),
                 lookup(f.bit, bank, key).longAction);
  });
}

TEST(keymap_double_click_matches_expectations) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    const char *expected = lookup(f.bit, bank, key).doubleAction;
    const bool waits = strcmp(expected, "-") != 0;
    g_calls.clear();
    const bool wants = keymapWantsDoubleClick(t, bank, key);
    CHECK(g_calls.empty());
    if (wants != waits) {
      fprintf(stderr, "  %s bank %u key %c: wantsDoubleClick %d, expected %d\n", f.name, bank,
              key, wants, waits);
      ++::test::g_checkFailures;
    }
    // A key that does not wait never gets a double click, so it has no action.
    checkOutcome(f, bank, key, "double",
                 outcome(keymapDoubleClick, t, bank, key, waits ? "none" : "-"), expected);
  });
}

namespace {

// Holds the old keypadEvent() guarded with !g_modeSetActive.
bool legacyModeSelectGuardedHold(uint16_t family, uint8_t bank, char key) {
  return bank == 3 && ((key >= '1' && key <= '5') || (key == '6' && family == FT817));
}

}  // namespace

TEST(keymap_mode_select_hold_runs_unguarded_holds) {
  forEachKey([](const Family &f, const KeypadTraits &t, uint8_t bank, char key) {
    const char *expected = legacyModeSelectGuardedHold(f.bit, bank, key)
                               ? "none"
                               : lookup(f.bit, bank, key).longAction;
    checkOutcome(f, bank, key, "mode-select long",
                 outcome(keymapModeSelectHold, t, bank, key, "none"), expected);
  });
}

TEST(keymap_mode_select_hold_examples) {
  // F2: Bank 1 '0' long still starts a frequency entry.
  g_calls.clear();
  CHECK(keymapModeSelectHold(traitsFor(FT817), 1, '0'));
  CHECK_EQ(g_calls, "beginBank1FrequencySet");
  // FT-857 Bank 3 '6' long (PTT) is not guarded; FT-817's is.
  g_calls.clear();
  CHECK(keymapModeSelectHold(traitsFor(FT857), 3, '6'));
  CHECK_EQ(g_calls, "setBank3Ft857Ptt(true)");
  g_calls.clear();
  CHECK(!keymapModeSelectHold(traitsFor(FT817), 3, '6'));
  CHECK(g_calls.empty());
}

TEST(keymap_ignores_state_machine_keys_and_other_banks) {
  const KeypadTraits t = traitsFor(IC7300);
  for (const char *k = "*#D"; *k; ++k) {
    for (uint8_t bank = 0; bank <= 10; ++bank) {
      g_calls.clear();
      CHECK(!keymapShort(t, bank, *k));
      CHECK(!keymapHold(t, bank, *k));
      CHECK(!keymapDoubleClick(t, bank, *k));
      CHECK(!keymapWantsDoubleClick(t, bank, *k));
      CHECK(g_calls.empty());
    }
  }
  g_calls.clear();
  CHECK(!keymapShort(t, 0, '1'));
  CHECK(!keymapShort(t, 7, '1'));
  CHECK(!keymapShort(t, 10, '1'));
  CHECK(g_calls.empty());
}
