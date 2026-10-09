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
  t.canGetRfPower = family == IC7300;
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
         !b.waitsForDouble;
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
void queryBank2NrLevel() { record("queryBank2NrLevel"); }
void adjustBank2NrLevel(int d) { record("adjustBank2NrLevel(%d)", d); }
void queryBank2NbLevel() { record("queryBank2NbLevel"); }
void adjustBank2NbLevel(int d) { record("adjustBank2NbLevel(%d)", d); }
void queryBank2PbtInner() { record("queryBank2PbtInner"); }
void adjustBank2PbtInner(int d) { record("adjustBank2PbtInner(%d)", d); }
void queryBank2PbtOuter() { record("queryBank2PbtOuter"); }
void adjustBank2PbtOuter(int d) { record("adjustBank2PbtOuter(%d)", d); }
void toggleBank2FilterShape() { record("toggleBank2FilterShape"); }
void queryBank2FilterWidth() { record("queryBank2FilterWidth"); }
void cycleBank2FilterWidth(int d) { record("cycleBank2FilterWidth(%d)", d); }
void queryBank2Ft8x7NrLevel() { record("queryBank2Ft8x7NrLevel"); }
void queryBank2Ft8x7NbLevel() { record("queryBank2Ft8x7NbLevel"); }
void queryBank2Ft8x7Dbf() { record("queryBank2Ft8x7Dbf"); }
void queryBank2Ft8x7LowCut() { record("queryBank2Ft8x7LowCut"); }
void queryBank2Ft8x7HighCut() { record("queryBank2Ft8x7HighCut"); }
void queryBank2Ft8x7MicEq() { record("queryBank2Ft8x7MicEq"); }
void queryBank2Ft8x7Ipo() { record("queryBank2Ft8x7Ipo"); }
void queryBank2Ft8x7Att() { record("queryBank2Ft8x7Att"); }
void queryBank2Ft8x7Agc() { record("queryBank2Ft8x7Agc"); }
void queryBank2Ft817Antenna() { record("queryBank2Ft817Antenna"); }

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
void toggleBank4Monitor() { record("toggleBank4Monitor"); }
void queryBank4MonitorLevel() { record("queryBank4MonitorLevel"); }
void adjustBank4MonitorLevel(int d) { record("adjustBank4MonitorLevel(%d)", d); }
void toggleBank4Transceive() { record("toggleBank4Transceive"); }

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
void queryBank6CtcssDefault() { record("queryBank6CtcssDefault"); }
void beginBank6CtcssEntry() { record("beginBank6CtcssEntry"); }
void queryBank6DcsDefault() { record("queryBank6DcsDefault"); }
void beginBank6DcsEntry() { record("beginBank6DcsEntry"); }

void queryBank8CivAddress() { record("queryBank8CivAddress"); }
void beginBank8CivAddressEntry() { record("beginBank8CivAddressEntry"); }
void queryBank8Baud() { record("queryBank8Baud"); }
void cycleBank8Baud(int d) { record("cycleBank8Baud(%d)", d); }
void queryBank8Ft8x7Row() { record("queryBank8Ft8x7Row"); }
void queryBank8Ft8x7Menu() { record("queryBank8Ft8x7Menu"); }

void queryBank9TuningSpeech() { record("queryBank9TuningSpeech"); }
void toggleBank9TuningSpeech() { record("toggleBank9TuningSpeech"); }
void queryBank9Verbose() { record("queryBank9Verbose"); }
void toggleBank9Verbose() { record("toggleBank9Verbose"); }
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

TEST(keymap_ignores_state_machine_keys_and_other_banks) {
  const KeypadTraits t = traitsFor(IC7300);
  g_calls.clear();
  for (const char *k = "*#D"; *k; ++k) {
    for (uint8_t bank = 0; bank <= 10; ++bank) CHECK(isEmpty(keymapLookup(t, bank, *k)));
  }
  CHECK(isEmpty(keymapLookup(t, 0, '1')));
  CHECK(isEmpty(keymapLookup(t, 7, '1')));
  CHECK(isEmpty(keymapLookup(t, 10, '1')));
  CHECK(g_calls.empty());
}
