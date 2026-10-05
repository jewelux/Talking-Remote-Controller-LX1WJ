#include "keypad_keymap.h"

#include "keypad_actions.h"

namespace {

using Action = KeyAction;

// Double-click slot of a key that waits for a double click but has no double
// action: a quick second press restarts the wait, and the short action then
// runs once.
void waitOnly() {}

// One key: its short, hold, double-click and double-hold (a short press, then a
// hold) actions. nullptr = none; a key with neither a double nor a double-hold
// action does not wait for a double click. A key with only a double-hold action
// passes waitOnly as its double.
KeyBinding bind(Action shortAction, Action holdAction = nullptr, Action doubleAction = nullptr,
                Action doubleHoldAction = nullptr) {
  KeyBinding b;
  b.shortAction = shortAction;
  b.holdAction = holdAction;
  b.doubleAction = doubleAction == waitOnly ? nullptr : doubleAction;
  b.doubleHoldAction = doubleHoldAction;
  b.waitsForDouble = doubleAction != nullptr || doubleHoldAction != nullptr;
  return b;
}

// A key that runs the console command cmd, traced "CMD <key> -> <cmd>". The
// '+' turns the lambda into an Action, so it also works in a conditional.
#define SEND(cmd) +[] { sendKeypadCommand(cmd); }

using L = KeypadLayout;

bool isFt8x7(const KeypadTraits& t) {
  return t.layout == L::Ft8x7 || t.layout == L::Ft817 || t.layout == L::Ft857;
}

KeyBinding bank1(const KeypadTraits& t, char key) {
  switch (key) {
    case '0':
      if (t.layout == L::Ftdx10) {
        return bind(SEND("FREQ?"), beginBank1FrequencySet, [] { roundActiveFrequency(500); });
      }
      return bind(queryBank1Frequency, beginBank1FrequencySet, [] { roundActiveFrequency(500); });
    case '1':
      if (t.layout == L::Ftdx10) return bind(SEND("RXTX?"), nullptr, waitOnly);
      return bind(queryBank1RxTx, nullptr, waitOnly);
    case '2':
      if (t.layout == L::Ftdx10) return bind(SEND("TXFREQ?"), nullptr, waitOnly);
      if (t.layout == L::Ft857) return bind(queryBank1Ft857TxFrequency, nullptr, waitOnly);
      return bind(queryBank1TxFrequency, nullptr, waitOnly);
    case '3':
      if (t.layout == L::Ftdx10) return bind(SEND("LOCK?"), SEND("LOCK TOGGLE"));
      return bind(queryBank1Lock, toggleBank1Lock);
    case '4': return bind(queryBank1Power);
    case '5':
      if (t.layout == L::Ftdx10) return bind(SEND("TUNER?"), SEND("TUNER TOGGLE"), SEND("TUNE"));
      return {};
    case '6':
      if (t.layout == L::Ftdx10) return bind(SEND("PA?"), SEND("PA TOGGLE"));
      if (t.layout == L::Civ) return bind(t.canGetRfPower ? queryBank1RfPower : nullptr, beginBank1RfPowerSet);
      if (t.layout == L::Ft817 || t.layout == L::Ft857) return bind(queryBank1RfPower);
      return {};
    case '7': return bind(queryBank1Smeter);
    case '8': return bind(queryBank1Swr);
    case '9': return bind(queryBank1Mode, beginBank1ModeSelect);
    default: return {};
  }
}

KeyBinding bank2(const KeypadTraits& t, char key) {
  const bool civ = t.layout == L::Civ;
  const bool ftdx10 = t.layout == L::Ftdx10;
  // FT-817 and FT-857/897: settings the radio keeps in its EEPROM, read only. The FT-817
  // has no DSP and no readable IPO/ATT, so of these it answers only AGC. Its '0' is the
  // antenna jack of the current band.
  if (t.layout == L::Ft817 && key == '0') return bind(queryBank2Ft817Antenna);
  if (t.layout == L::Ft817 || t.layout == L::Ft857) {
    switch (key) {
      case '4': return bind(queryBank2Ft8x7NrLevel);
      case '5': return bind(queryBank2Ft8x7NbLevel);
      case '6': return bind(queryBank2Ft8x7Dbf);
      case '7': return bind(queryBank2Ft8x7LowCut);
      case '8': return bind(queryBank2Ft8x7HighCut);
      case '9': return bind(queryBank2Ft8x7MicEq);
      case 'A': return bind(queryBank2Ft8x7Ipo, queryBank2Ft8x7Att, queryBank2Ft8x7Agc);
      default: break;
    }
  }
  switch (key) {
    case '1': return bind(queryBank2Nr, toggleBank2Nr);
    case '2': return bind(queryBank2Nb, toggleBank2Nb);
    case '3': return bind(queryBank2Notch, toggleBank2Notch);
    case '4':
      if (civ) {
        return bind(queryBank2NrLevel, [] { adjustBank2NrLevel(10); },
                   [] { adjustBank2NrLevel(-10); });
      }
      if (ftdx10) return bind(SEND("GT?"), SEND("GT FAST"), SEND("GT SLOW"));
      return {};
    case '5':
      if (civ) {
        return bind(queryBank2NbLevel, [] { adjustBank2NbLevel(10); },
                   [] { adjustBank2NbLevel(-10); });
      }
      if (ftdx10) return bind(SEND("PS?"), SEND("PS OFF"), SEND("PS ON"));
      return {};
    case '6':
      if (civ) {
        return bind(queryBank2PbtInner, [] { adjustBank2PbtInner(10); },
                   [] { adjustBank2PbtInner(-10); });
      }
      if (ftdx10) return bind(SEND("IF?"));
      return {};
    case '7':
      if (civ) {
        return bind(queryBank2PbtOuter, [] { adjustBank2PbtOuter(10); },
                   [] { adjustBank2PbtOuter(-10); });
      }
      if (ftdx10) return bind(SEND("ID?"));
      return {};
    case '8':
      if (civ) return bind(SEND("FILSHAPE?"), toggleBank2FilterShape);
      if (ftdx10) return bind(reportFtdx10HiddenKey);
      return {};
    case '9':
      if (civ) {
        return bind(queryBank2FilterWidth, [] { cycleBank2FilterWidth(1); },
                   [] { cycleBank2FilterWidth(-1); });
      }
      if (ftdx10) return bind(reportFtdx10HiddenKey);
      return {};
    default: return {};
  }
}

// Bank 3 '7'-'9': band stack register Reg.
template <uint8_t Reg>
KeyBinding bank3BandStack(const KeypadTraits& t) {
  return bind([] { queryBank3BandStack(Reg); },
              t.layout == L::Ftdx10 ? reportFtdx10HiddenKey : +[] { recallBank3BandStack(Reg); });
}

// FT-817 and FT-857/897 track the active VFO themselves: '1' is the current
// VFO and '2' the other one.
KeyBinding bank3(const KeypadTraits& t, char key) {
  switch (key) {
    case '0':
      if (t.layout == L::Ftdx10) {
        return bind(SEND("SPLIT?"), SEND("SPLIT TOGGLE"), SEND("TXFREQ?"));
      }
      return bind(queryBank3Split, toggleBank3Split, queryBank3TxFrequency);
    case '1':
      if (t.layout == L::Ft817 || t.layout == L::Ft857) {
        return bind(queryBank3Ft8x7CurrentVfo, toggleBank3Ft8x7Vfo, beginBank3Ft8x7CurrentVfoFrequencySet);
      }
      if (t.layout == L::Ftdx10) {
        return bind(SEND("VFOA?"), SEND("VFO A"), beginBank3VfoAFrequencySet);
      }
      return bind(queryBank3VfoA, selectBank3VfoA, beginBank3VfoAFrequencySet);
    case '2':
      if (t.layout == L::Ft817) {
        return bind(queryBank3Ft8x7OtherVfo, copyBank3Ft817VfoToOther, beginBank3Ft8x7OtherVfoFrequencySet);
      }
      if (t.layout == L::Ft857) {
        return bind(queryBank3Ft8x7OtherVfo, reportBank3Ft857VfoBUnsupported,
                   beginBank3Ft8x7OtherVfoFrequencySet);
      }
      if (t.layout == L::Ftdx10) {
        return bind(SEND("VFOB?"), SEND("VFO B"), beginBank3VfoBFrequencySet);
      }
      return bind(queryBank3VfoB, selectBank3VfoB, beginBank3VfoBFrequencySet);
    case '3':
      if (t.layout == L::Ft817) return bind(queryBank3VfoAMode, beginBank3VfoAModeSet);
      return {};
    case '4':
      // The FT-857/897 reads its VFO from the radio, so only the FT-817 needs the sync keys.
      if (t.layout == L::Ft817) return bind(syncBank3VfoA, syncBank3VfoB);
      if (t.layout == L::Ft857) return {};
      if (t.layout == L::Ftdx10) return bind(SEND("VFOA MODE?"), beginBank3VfoAModeSet);
      return bind(queryBank3VfoAMode, beginBank3VfoAModeSet);
    case '5':
      if (t.layout == L::Ft817 || t.layout == L::Ft857) {
        return bind(queryBank3Ft8x7Rit, toggleBank3Ft8x7Rit);
      }
      if (t.layout == L::Ftdx10) return bind(SEND("VFOB MODE?"), beginBank3VfoBModeSet);
      return bind(queryBank3VfoBMode, beginBank3VfoBModeSet);
    case '6':
      if (t.layout == L::Ft817) return bind(selectBank3Ft817ActiveVfoA, selectBank3Ft817ActiveVfoB);
      if (t.layout == L::Ft857) {
        return bind([] { setBank3Ft857Ptt(false); }, [] { setBank3Ft857Ptt(true); });
      }
      if (t.layout == L::Ftdx10) return bind(SEND("RXTX?"));
      return bind(queryBank3RxTx);
    case '7': return bank3BandStack<1>(t);
    case '8': return bank3BandStack<2>(t);
    case '9': return bank3BandStack<3>(t);
    default: return {};
  }
}

KeyBinding bank4(const KeypadTraits& t, char key) {
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '0':
      if (ftdx10) return bind(SEND("TUNER?"), SEND("TUNER TOGGLE"), SEND("TUNE"));
      return bind(queryBank4Tuner, toggleBank4Tuner, triggerBank4Tune);
    case '1':
      if (ftdx10) return bind(reportFtdx10HiddenKey, reportFtdx10HiddenKey);
      return bind(t.supportsMonitor ? SEND("MONITOR?") : nullptr, toggleBank4Monitor);
    case '2':
      if (ftdx10) return bind(reportFtdx10HiddenKey, reportFtdx10HiddenKey, reportFtdx10HiddenKey);
      return bind(queryBank4MonitorLevel, [] { adjustBank4MonitorLevel(10); },
                 [] { adjustBank4MonitorLevel(-10); });
    case '3':
      if (ftdx10) return bind(reportFtdx10HiddenKey, reportFtdx10HiddenKey);
      return bind(t.supportsTransceive ? SEND("TRANSCEIVE?") : nullptr, toggleBank4Transceive);
    default: return {};
  }
}

KeyBinding bank5(char key) {
  switch (key) {
    case '0': return bind(queryBank5Rit, toggleBank5Rit, [] { setBank5RitOffset(0); });
    case '1': return bind([] { adjustBank5Rit(-10); }, [] { adjustBank5Rit(-100); });
    case '2': return bind([] { adjustBank5Rit(10); }, [] { adjustBank5Rit(100); });
    case '3': return bind([] { setBank5RitOffset(0); }, setBank5RitOff);
    case '4': return bind([] { adjustBank5Rit(-1); }, [] { adjustBank5Rit(-500); });
    case '5': return bind([] { adjustBank5Rit(1); }, [] { adjustBank5Rit(500); });
    default: return {};
  }
}

// FT-8x7 repeater shift, offset and tones. Empty on the other radios.
KeyBinding bank6(const KeypadTraits& t, char key) {
  if (!isFt8x7(t)) return {};
  switch (key) {
    case '0': return bind(setBank6RepeaterOff, setBank6RepeaterMinus, setBank6RepeaterPlus);
    case '1':
      return bind([] { setBank6RepeaterOffsetPreset(1); },
                 [] { setBank6RepeaterOffsetPreset(2); }, beginBank6RepeaterOffsetEntry);
    case '2': return bind(setBank6ToneOff, setBank6ToneModeCtcss, setBank6ToneModeDcs);
    case '3': return bind(queryBank6CtcssDefault, beginBank6CtcssEntry);
    case '4': return bind(queryBank6DcsDefault, beginBank6DcsEntry);
    default: return {};
  }
}

KeyBinding bank8(const KeypadTraits& t, char key) {
  // FT-817 and FT-857/897: the radio's menu position.
  const bool ft8x7Menu = t.layout == L::Ft817 || t.layout == L::Ft857;
  switch (key) {
    case '1': return bind(queryBank8CivAddress, beginBank8CivAddressEntry);
    case '2': return bind(queryBank8Baud, [] { cycleBank8Baud(-1); }, [] { cycleBank8Baud(1); });
    case '7':
      if (ft8x7Menu) return bind(queryBank8Ft8x7Row);
      return {};
    case '8':
      if (ft8x7Menu) return bind(queryBank8Ft8x7Menu);
      return {};
    default: return {};
  }
}

KeyBinding bank9(char key) {
  switch (key) {
    case '4': return bind(queryBank9TuningSpeech, toggleBank9TuningSpeech);
    case '5': return bind(queryBank9Verbose, toggleBank9Verbose);
    case '7': return bind([] { adjustBank9Volume(-1); }, [] { adjustBank9Volume(-2); });
    case '8': return bind([] { adjustBank9Volume(1); }, [] { adjustBank9Volume(2); });
    case '9': return bind(queryBank9Volume);
    case 'A': return bind(queryBank9Profile, beginBank9ProfileSelect, resetBank9Profile);
    case 'B': return bind(selectNextProfile);
    case 'C': return bind(selectPrevProfile);
    default: return {};
  }
}

#undef SEND

}  // namespace

KeyBinding keymapLookup(const KeypadTraits& traits, uint8_t bank, char key) {
  switch (bank) {
    case 1: return bank1(traits, key);
    case 2: return bank2(traits, key);
    case 3: return bank3(traits, key);
    case 4: return bank4(traits, key);
    case 5: return bank5(key);
    case 6: return bank6(traits, key);
    case 8: return bank8(traits, key);
    case 9: return bank9(key);
    default: return {};
  }
}
