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

// A key that runs a console command: "CMD <label>", then cmd. The '+' turns
// the lambda into an Action, so it also works in a conditional.
#define SEND(label, cmd) +[] { sendKeypadCommand(label, cmd); }

using L = KeypadLayout;

bool isFt8x7(const KeypadTraits& t) {
  return t.layout == L::Ft8x7 || t.layout == L::Ft817 || t.layout == L::Ft857;
}

void ftdx10HiddenBank2Key8() { reportFtdx10HiddenKey("BANK2 8"); }
void ftdx10HiddenBank2Key9() { reportFtdx10HiddenKey("BANK2 9"); }
void ftdx10HiddenBank4() { reportFtdx10HiddenKey("BANK4"); }

bool bank1(const KeypadTraits& t, Gesture g, char key) {
  switch (key) {
    case '0':
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK1 0 SHORT -> FREQ?", "FREQ?"), beginBank1FrequencySet,
                   [] { roundActiveFrequency(500); });
      }
      return run(g, queryBank1Frequency, beginBank1FrequencySet, [] { roundActiveFrequency(500); });
    case '1':
      if (t.layout == L::Ftdx10) return run(g, SEND("BANK1 1 SHORT -> RXTX?", "RXTX?"), nullptr, waitOnly);
      if (t.layout == L::Ft817) return run(g, reportBank1Ft817RxTxUnreliable, nullptr, waitOnly);
      return run(g, queryBank1RxTx, nullptr, waitOnly);
    case '2':
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK1 2 SHORT -> TXFREQ?", "TXFREQ?"), nullptr, waitOnly);
      }
      if (t.layout == L::Ft857) return run(g, queryBank1Ft857TxFrequency, nullptr, waitOnly);
      return run(g, queryBank1TxFrequency, nullptr, waitOnly);
    case '3':
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK1 3 SHORT -> LOCK?", "LOCK?"), SEND("BANK1 3 LONG -> LOCK", "LOCK TOGGLE"));
      }
      return run(g, queryBank1Lock, toggleBank1Lock);
    case '4': return run(g, queryBank1Power);
    case '5':
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK1 5 SHORT -> TUNER?", "TUNER?"),
                   SEND("BANK1 5 LONG -> TUNER TOGGLE", "TUNER TOGGLE"), SEND("BANK1 5 DOUBLE -> TUNE", "TUNE"));
      }
      return false;
    case '6':
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK1 6 SHORT -> PA?", "PA?"), SEND("BANK1 6 LONG -> PA TOGGLE", "PA TOGGLE"));
      }
      if (t.layout == L::Civ) return run(g, t.canGetRfPower ? queryBank1RfPower : nullptr, beginBank1RfPowerSet);
      return false;
    case '7': return run(g, queryBank1Smeter);
    case '8': return run(g, queryBank1Swr);
    case '9': return run(g, queryBank1Mode, beginBank1ModeSelect);
    default: return false;
  }
}

bool bank2(const KeypadTraits& t, Gesture g, char key) {
  const bool civ = t.layout == L::Civ;
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '1':
      if (ftdx10) return run(g, SEND("BANK2 1 SHORT -> NR?", "NR?"), SEND("BANK2 1 LONG -> NR", "NR TOGGLE"));
      return run(g, queryBank2Nr, toggleBank2Nr);
    case '2':
      if (ftdx10) return run(g, SEND("BANK2 2 SHORT -> NB?", "NB?"), SEND("BANK2 2 LONG -> NB", "NB TOGGLE"));
      return run(g, queryBank2Nb, toggleBank2Nb);
    case '3':
      if (ftdx10) {
        return run(g, SEND("BANK2 3 SHORT -> NOTCH?", "NOTCH?"), SEND("BANK2 3 LONG -> NOTCH", "NOTCH TOGGLE"));
      }
      return run(g, queryBank2Notch, toggleBank2Notch);
    case '4':
      if (civ) {
        return run(g, queryBank2NrLevel, [] { adjustBank2NrLevel(10); },
                   [] { adjustBank2NrLevel(-10); });
      }
      if (ftdx10) {
        return run(g, SEND("BANK2 4 SHORT -> GT?", "GT?"), SEND("BANK2 4 LONG -> GT FAST", "GT FAST"),
                   SEND("BANK2 4 DOUBLE -> GT SLOW", "GT SLOW"));
      }
      return false;
    case '5':
      if (civ) {
        return run(g, queryBank2NbLevel, [] { adjustBank2NbLevel(10); },
                   [] { adjustBank2NbLevel(-10); });
      }
      if (ftdx10) {
        return run(g, SEND("BANK2 5 SHORT -> PS?", "PS?"), SEND("BANK2 5 LONG -> PS OFF", "PS OFF"),
                   SEND("BANK2 5 DOUBLE -> PS ON", "PS ON"));
      }
      return false;
    case '6':
      if (civ) {
        return run(g, queryBank2PbtInner, [] { adjustBank2PbtInner(10); },
                   [] { adjustBank2PbtInner(-10); });
      }
      if (ftdx10) return run(g, SEND("BANK2 6 SHORT -> IF?", "IF?"));
      return false;
    case '7':
      if (civ) {
        return run(g, queryBank2PbtOuter, [] { adjustBank2PbtOuter(10); },
                   [] { adjustBank2PbtOuter(-10); });
      }
      if (ftdx10) return run(g, SEND("BANK2 7 SHORT -> ID?", "ID?"));
      return false;
    case '8':
      if (civ) return run(g, SEND("BANK2 8 SHORT -> FILSHAPE?", "FILSHAPE?"), toggleBank2FilterShape);
      if (ftdx10) return run(g, ftdx10HiddenBank2Key8);
      return false;
    case '9':
      if (civ) {
        return run(g, queryBank2FilterWidth, [] { cycleBank2FilterWidth(1); },
                   [] { cycleBank2FilterWidth(-1); });
      }
      if (ftdx10) return run(g, ftdx10HiddenBank2Key9);
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
    if (t.layout == L::Ftdx10) {
      reportFtdx10HiddenKey("BANK3 BSTACK");
    } else {
      recallBank3BandStack(reg);
    }
    return true;
  }
  return false;
}

// FT-817 and FT-857/897 track the active VFO themselves: '1' is the current
// VFO and '2' the other one.
bool bank3(const KeypadTraits& t, Gesture g, char key) {
  switch (key) {
    case '0':
      if (t.layout == L::Ft857) {
        return run(g, [] { setBank3Ft857Split(false); }, [] { setBank3Ft857Split(true); },
                   calibrateBank3Ft857Split);
      }
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK3 0 SHORT -> SPLIT?", "SPLIT?"), SEND("BANK3 0 LONG -> SPLIT", "SPLIT TOGGLE"),
                   SEND("BANK3 0 DOUBLE -> TXFREQ?", "TXFREQ?"));
      }
      return run(g, queryBank3Split, toggleBank3Split, queryBank3TxFrequency);
    case '1':
      if (t.layout == L::Ft817 || t.layout == L::Ft857) {
        return run(g, queryBank3Ft8x7CurrentVfo, toggleBank3Ft8x7Vfo, beginBank3Ft8x7CurrentVfoFrequencySet);
      }
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK3 1 SHORT -> VFOA?", "VFOA?"), SEND("BANK3 1 LONG -> VFO A", "VFO A"),
                   beginBank3VfoAFrequencySet);
      }
      return run(g, queryBank3VfoA, selectBank3VfoA, beginBank3VfoAFrequencySet);
    case '2':
      if (t.layout == L::Ft817) {
        return run(g, queryBank3Ft817OtherVfo, copyBank3Ft817VfoToOther, beginBank3Ft8x7OtherVfoFrequencySet);
      }
      if (t.layout == L::Ft857) {
        return run(g, queryBank3Ft857OtherVfo, reportBank3Ft857VfoBUnsupported,
                   beginBank3Ft8x7OtherVfoFrequencySet);
      }
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK3 2 SHORT -> VFOB?", "VFOB?"), SEND("BANK3 2 LONG -> VFO B", "VFO B"),
                   beginBank3VfoBFrequencySet);
      }
      return run(g, queryBank3VfoB, selectBank3VfoB, beginBank3VfoBFrequencySet);
    case '3':
      if (t.layout == L::Ft817) {
        return run(g, [] { queryBank3VfoAMode('3'); }, [] { beginBank3VfoAModeSet('3'); });
      }
      return false;
    case '4':
      if (t.layout == L::Ft817 || t.layout == L::Ft857) return run(g, syncBank3VfoA, syncBank3VfoB);
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK3 4 SHORT -> VFOA MODE?", "VFOA MODE?"), [] { beginBank3VfoAModeSet('4'); });
      }
      return run(g, [] { queryBank3VfoAMode('4'); }, [] { beginBank3VfoAModeSet('4'); });
    case '5':
      if (t.layout == L::Ft857) {
        return run(g, [] { setBank3Ft857Clar(true); }, [] { setBank3Ft857Clar(false); });
      }
      if (t.layout == L::Ftdx10) {
        return run(g, SEND("BANK3 5 SHORT -> VFOB MODE?", "VFOB MODE?"), beginBank3VfoBModeSet);
      }
      return run(g, queryBank3VfoBMode, beginBank3VfoBModeSet);
    case '6':
      if (t.layout == L::Ft817) return run(g, selectBank3Ft817ActiveVfoA, selectBank3Ft817ActiveVfoB);
      if (t.layout == L::Ft857) {
        return run(g, [] { setBank3Ft857Ptt(false); }, [] { setBank3Ft857Ptt(true); });
      }
      if (t.layout == L::Ftdx10) return run(g, SEND("BANK3 6 SHORT -> RXTX?", "RXTX?"));
      return run(g, queryBank3RxTx);
    case '7': return bank3BandStack(t, g, 1);
    case '8': return bank3BandStack(t, g, 2);
    case '9': return bank3BandStack(t, g, 3);
    default: return false;
  }
}

bool bank4(const KeypadTraits& t, Gesture g, char key) {
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '0':
      if (ftdx10) {
        return run(g, SEND("BANK4 0 SHORT -> TUNER?", "TUNER?"), SEND("BANK4 0 LONG -> TUNER", "TUNER TOGGLE"),
                   SEND("BANK4 0 DOUBLE -> TUNE", "TUNE"));
      }
      return run(g, queryBank4Tuner, toggleBank4Tuner, triggerBank4Tune);
    case '1':
      if (ftdx10) return run(g, ftdx10HiddenBank4, ftdx10HiddenBank4);
      return run(g, t.supportsMonitor ? SEND("BANK4 1 SHORT -> MONITOR?", "MONITOR?") : nullptr,
                 toggleBank4Monitor);
    case '2':
      if (ftdx10) return run(g, ftdx10HiddenBank4, ftdx10HiddenBank4, ftdx10HiddenBank4);
      return run(g, queryBank4MonitorLevel, [] { adjustBank4MonitorLevel(10); },
                 [] { adjustBank4MonitorLevel(-10); });
    case '3':
      if (ftdx10) return run(g, ftdx10HiddenBank4, ftdx10HiddenBank4);
      return run(g, t.supportsTransceive ? SEND("BANK4 3 SHORT -> TRANSCEIVE?", "TRANSCEIVE?") : nullptr,
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

// FT-8x7 repeater shift, offset and tones. Empty on the other radios.
bool bank6(const KeypadTraits& t, Gesture g, char key) {
  if (!isFt8x7(t)) return false;
  switch (key) {
    case '0': return run(g, setBank6RepeaterOff, setBank6RepeaterMinus, setBank6RepeaterPlus);
    case '1':
      return run(g, [] { setBank6RepeaterOffsetPreset(1); },
                 [] { setBank6RepeaterOffsetPreset(2); }, beginBank6RepeaterOffsetEntry);
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
    case 6: return bank6(t, g, key);
    case 8: return bank8(g, key);
    case 9: return bank9(t, g, key);
    default: return false;
  }
}

#undef SEND

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
