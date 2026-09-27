#include "keypad_keymap.h"

#include "keypad_actions.h"

namespace {

enum class Gesture : uint8_t { Short, Hold, DoubleClick, WantsDoubleClick };

using Action = void (*)();

// Double-click slot of a key that waits for a double click but has no double
// action: a quick second press restarts the wait, and the short action then
// runs once.
void waitOnly() {}

// One key: its short, hold and double-click actions. nullptr = none; a key
// with no double action does not wait for a double click.
bool run(Gesture g, Action shortAction, Action holdAction = nullptr,
         Action doubleAction = nullptr) {
  Action action = nullptr;
  switch (g) {
    case Gesture::Short:
      action = shortAction;
      break;
    case Gesture::Hold:
      action = holdAction;
      break;
    case Gesture::DoubleClick:
      action = doubleAction == waitOnly ? nullptr : doubleAction;
      break;
    case Gesture::WantsDoubleClick:
      return doubleAction != nullptr;
  }
  if (!action) return false;
  action();
  return true;
}

void ftdx10HiddenBank2Key8() { reportFtdx10HiddenKey("BANK2 8"); }
void ftdx10HiddenBank2Key9() { reportFtdx10HiddenKey("BANK2 9"); }
void ftdx10HiddenBank4() { reportFtdx10HiddenKey("BANK4"); }

bool bank1(const KeypadTraits& t, Gesture g, char key) {
  switch (key) {
    case '0': return run(g, queryBank1Frequency, beginBank1FrequencySet, roundActiveFrequency500);
    case '1': return run(g, queryBank1RxTx, nullptr, waitOnly);
    case '2': return run(g, queryBank1TxFrequency, nullptr, waitOnly);
    case '3': return run(g, queryBank1Lock, toggleBank1Lock);
    case '4': return run(g, queryBank1Power);
    case '5':
      if (t.ftdx10) return run(g, ftdx10QueryTuner, ftdx10ToggleTuner, ftdx10Tune);
      return false;
    case '6':
      if (t.ftdx10) return run(g, ftdx10QueryPreamp, ftdx10TogglePreamp);
      if (t.civ) return run(g, t.canGetRfPower ? queryBank1RfPower : nullptr, beginBank1RfPowerSet);
      return false;
    case '7': return run(g, queryBank1Smeter);
    case '8': return run(g, queryBank1Swr);
    case '9': return run(g, queryBank1Mode, beginBank1ModeSelect);
    default: return false;
  }
}

bool bank2(const KeypadTraits& t, Gesture g, char key) {
  switch (key) {
    case '1': return run(g, queryBank2Nr, toggleBank2Nr);
    case '2': return run(g, queryBank2Nb, toggleBank2Nb);
    case '3': return run(g, queryBank2Notch, toggleBank2Notch);
    case '4':
      if (t.civ) {
        return run(g, queryBank2NrLevel, [] { adjustBank2NrLevel(10); },
                   [] { adjustBank2NrLevel(-10); });
      }
      if (t.ftdx10) return run(g, ftdx10QueryAgc, ftdx10AgcFast, ftdx10AgcSlow);
      return false;
    case '5':
      if (t.civ) {
        return run(g, queryBank2NbLevel, [] { adjustBank2NbLevel(10); },
                   [] { adjustBank2NbLevel(-10); });
      }
      if (t.ftdx10) return run(g, ftdx10QueryPowerState, ftdx10PowerOff, ftdx10PowerOn);
      return false;
    case '6':
      if (t.civ) {
        return run(g, queryBank2PbtInner, [] { adjustBank2PbtInner(10); },
                   [] { adjustBank2PbtInner(-10); });
      }
      if (t.ftdx10) return run(g, ftdx10QueryInfo);
      return false;
    case '7':
      if (t.civ) {
        return run(g, queryBank2PbtOuter, [] { adjustBank2PbtOuter(10); },
                   [] { adjustBank2PbtOuter(-10); });
      }
      if (t.ftdx10) return run(g, ftdx10QueryId);
      return false;
    case '8':
      if (t.civ) return run(g, sendBank2FilterShapeQuery, toggleBank2FilterShape);
      if (t.ftdx10) return run(g, ftdx10HiddenBank2Key8);
      return false;
    case '9':
      if (t.civ) {
        return run(g, queryBank2FilterWidth, [] { cycleBank2FilterWidth(1); },
                   [] { cycleBank2FilterWidth(-1); });
      }
      if (t.ftdx10) return run(g, ftdx10HiddenBank2Key9);
      return false;
    default: return false;
  }
}

// Bank 3 '7'-'9': band stack register reg.
bool bank3BandStack(const KeypadTraits& t, Gesture g, uint8_t reg) {
  if (g == Gesture::Short) {
    queryBank3BandStack(reg);
    return true;
  }
  if (g == Gesture::Hold) {
    if (t.ftdx10) {
      reportFtdx10HiddenKey("BANK3 BSTACK");
    } else {
      recallBank3BandStack(reg);
    }
    return true;
  }
  return false;
}

bool bank3(const KeypadTraits& t, Gesture g, char key) {
  switch (key) {
    case '0':
      if (t.ft857Family) return run(g, queryBank3Split, toggleBank3Split, calibrateBank3Ft857Split);
      return run(g, queryBank3Split, toggleBank3Split, queryBank3TxFrequency);
    case '1':
      if (t.ft817) {
        return run(g, queryBank3VfoA, beginBank3VfoAFrequencySet, beginBank3VfoAFrequencySet);
      }
      return run(g, queryBank3VfoA, selectBank3VfoA, beginBank3VfoAFrequencySet);
    case '2':
      if (t.ft817) {
        return run(g, queryBank3VfoB, beginBank3VfoBFrequencySet, beginBank3VfoBFrequencySet);
      }
      return run(g, queryBank3VfoB, selectBank3VfoB, beginBank3VfoBFrequencySet);
    case '3':
      if (t.ft817) {
        return run(g, queryBank3VfoAMode, beginBank3VfoAModeSet);
      }
      return false;
    case '4':
      if (t.ft817 || t.ft857Family) return run(g, syncBank3VfoA, syncBank3VfoB);
      return run(g, queryBank3VfoAMode, beginBank3VfoAModeSet);
    case '5':
      if (t.ft857Family) {
        return run(g, [] { setBank3Ft857Clar(true); }, [] { setBank3Ft857Clar(false); });
      }
      return run(g, queryBank3VfoBMode, beginBank3VfoBModeSet);
    case '6':
      if (t.ft817) return run(g, selectBank3Ft817ActiveVfoA, selectBank3Ft817ActiveVfoB);
      if (t.ft857Family) return run(g, queryBank3RxTx, [] { setBank3Ft857Ptt(true); });
      return run(g, queryBank3RxTx);
    case '7': return bank3BandStack(t, g, 1);
    case '8': return bank3BandStack(t, g, 2);
    case '9': return bank3BandStack(t, g, 3);
    default: return false;
  }
}

bool bank4(const KeypadTraits& t, Gesture g, char key) {
  switch (key) {
    case '0': return run(g, queryBank4Tuner, toggleBank4Tuner, triggerBank4Tune);
    case '1':
      if (t.ftdx10) return run(g, ftdx10HiddenBank4, ftdx10HiddenBank4);
      return run(g, t.supportsMonitor ? sendBank4MonitorQuery : nullptr, toggleBank4Monitor);
    case '2':
      if (t.ftdx10) return run(g, ftdx10HiddenBank4, ftdx10HiddenBank4, ftdx10HiddenBank4);
      return run(g, queryBank4MonitorLevel, [] { adjustBank4MonitorLevel(10); },
                 [] { adjustBank4MonitorLevel(-10); });
    case '3':
      if (t.ftdx10) return run(g, ftdx10HiddenBank4, ftdx10HiddenBank4);
      return run(g, t.supportsTransceive ? sendBank4TransceiveQuery : nullptr,
                 toggleBank4Transceive);
    default: return false;
  }
}

bool bank5(Gesture g, char key) {
  switch (key) {
    case '0': return run(g, queryBank5Rit, toggleBank5Rit, [] { setBank5RitOffset(0); });
    case '1': return run(g, [] { adjustBank5Rit(-10); }, [] { adjustBank5Rit(-100); });
    case '2': return run(g, [] { adjustBank5Rit(10); }, [] { adjustBank5Rit(100); });
    case '3': return run(g, [] { setBank5RitOffset(0); }, setBank5RitOff);
    case '4': return run(g, [] { adjustBank5Rit(-1); }, [] { adjustBank5Rit(-500); });
    case '5': return run(g, [] { adjustBank5Rit(1); }, [] { adjustBank5Rit(500); });
    default: return false;
  }
}

bool bank6(Gesture g, char key) {
  switch (key) {
    case '0': return run(g, setBank6RepeaterOff, setBank6RepeaterMinus, setBank6RepeaterPlus);
    case '1':
      return run(g, setBank6RepeaterOffset1, setBank6RepeaterOffset2,
                 beginBank6RepeaterOffsetEntry);
    case '2': return run(g, setBank6ToneOff, setBank6ToneModeCtcss, setBank6ToneModeDcs);
    case '3': return run(g, queryBank6CtcssDefault, beginBank6CtcssEntry);
    case '4': return run(g, queryBank6DcsDefault, beginBank6DcsEntry);
    default: return false;
  }
}

bool bank8(Gesture g, char key) {
  switch (key) {
    case '1': return run(g, queryBank8CivAddress, beginBank8CivAddressEntry);
    case '2': return run(g, [] { cycleBank8Baud(1); }, [] { cycleBank8Baud(-1); }, waitOnly);
    default: return false;
  }
}

bool bank9(const KeypadTraits& t, Gesture g, char key) {
  // Light-Icom fallback: the digits pick a built-in profile, ahead of their
  // other short actions.
  if (g == Gesture::Short && t.lightIcomFallback && key >= '1' && key <= '9' &&
      selectBank9DirectProfile(key)) {
    return true;
  }
  switch (key) {
    case '4': return run(g, queryBank9TuningSpeech, toggleBank9TuningSpeech);
    case '7': return run(g, [] { adjustBank9Volume(-1); }, [] { adjustBank9Volume(-2); });
    case '8': return run(g, [] { adjustBank9Volume(1); }, [] { adjustBank9Volume(2); });
    case '9': return run(g, queryBank9Volume);
    case 'A': return run(g, queryBank9Profile, beginBank9ProfileSelect);
    case 'B': return run(g, selectNextProfile);
    case 'C': return run(g, selectPrevProfile);
    default: return false;
  }
}

bool dispatch(const KeypadTraits& t, uint8_t bank, char key, Gesture g) {
  switch (bank) {
    case 1: return bank1(t, g, key);
    case 2: return bank2(t, g, key);
    case 3: return bank3(t, g, key);
    case 4: return bank4(t, g, key);
    case 5: return bank5(g, key);
    case 6: return bank6(g, key);
    case 8: return bank8(g, key);
    case 9: return bank9(t, g, key);
    default: return false;
  }
}

}  // namespace

bool keymapShort(const KeypadTraits& traits, uint8_t bank, char key) {
  return dispatch(traits, bank, key, Gesture::Short);
}

bool keymapHold(const KeypadTraits& traits, uint8_t bank, char key) {
  return dispatch(traits, bank, key, Gesture::Hold);
}

bool keymapDoubleClick(const KeypadTraits& traits, uint8_t bank, char key) {
  return dispatch(traits, bank, key, Gesture::DoubleClick);
}

bool keymapWantsDoubleClick(const KeypadTraits& traits, uint8_t bank, char key) {
  return dispatch(traits, bank, key, Gesture::WantsDoubleClick);
}
