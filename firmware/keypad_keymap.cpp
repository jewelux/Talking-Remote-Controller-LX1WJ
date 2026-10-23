#include "keypad_keymap.h"

#include "keypad_actions.h"

namespace {

using Action = KeyAction;

// Double-click slot of a key that waits for a double click but has no double
// action: a quick second press restarts the wait, and the short action then
// runs once.
void waitOnly() {}

using V = HelpVerb;

// A binding as bind() builds it; help() and withFocus() add to it.
struct Key : KeyBinding {
  // What help mode says: label with each gesture's verb. Only a gesture that has an action
  // gets help; tests/keypad checks that every one does.
  Key help(const char* label, V shortVerb, V holdVerb = V::None, V doubleVerb = V::None,
           V doubleHoldVerb = V::None) const {
    Key k = *this;
    k.shortHelp = gesture(shortAction, label, shortVerb);
    k.holdHelp = gesture(holdAction, label, holdVerb);
    k.doubleHelp = gesture(doubleAction, label, doubleVerb);
    k.doubleHoldHelp = gesture(doubleHoldAction, label, doubleHoldVerb);
    return k;
  }
  // One gesture under a label of its own (the FT-8x7 IPO key says the attenuator on a double).
  Key helpShort(const char* label, V verb) const {
    Key k = *this;
    k.shortHelp = gesture(shortAction, label, verb);
    return k;
  }
  Key helpHold(const char* label, V verb) const {
    Key k = *this;
    k.holdHelp = gesture(holdAction, label, verb);
    return k;
  }
  Key helpDouble(const char* label, V verb) const {
    Key k = *this;
    k.doubleHelp = gesture(doubleAction, label, verb);
    return k;
  }
  Key helpDoubleHold(const char* label, V verb) const {
    Key k = *this;
    k.doubleHoldHelp = gesture(doubleHoldAction, label, verb);
    return k;
  }
  // A digit key that also sets its bank's focus: what A, B and C act on after it.
  Key withFocus(Setting setting) const {
    Key k = *this;
    k.focus = setting;
    return k;
  }

 private:
  static GestureHelp gesture(KeyAction action, const char* label, V verb) {
    return action ? GestureHelp{label, verb} : GestureHelp{};
  }
};

// One key: its short, hold, double-click and double-hold (a short press, then a
// hold) actions. nullptr = none; a key with neither a double nor a double-hold
// action does not wait for a double click. A key with only a double-hold action
// passes waitOnly as its double.
Key bind(Action shortAction, Action holdAction = nullptr, Action doubleAction = nullptr,
         Action doubleHoldAction = nullptr) {
  Key b;
  b.shortAction = shortAction;
  b.holdAction = holdAction;
  b.doubleAction = doubleAction == waitOnly ? nullptr : doubleAction;
  b.doubleHoldAction = doubleHoldAction;
  b.waitsForDouble = doubleAction != nullptr || doubleHoldAction != nullptr;
  return b;
}

// A, B and C: say, entry, centre and step on the bank's focus. A waits for a
// double only when the focus has a centre. With no focus they say "not
// available".
KeyBinding focusKey(char key, Setting focus) {
  const SettingInfo& info = settingInfo(focus);
  if (focus == Setting::None) {
    switch (key) {
      case 'A': return bind(focusSay, focusEntry).help(nullptr, V::NotAvailable, V::NotAvailable);
      case 'B':
        return bind([] { focusStep(1); }, [] { focusBigStep(1); }).help(nullptr, V::NotAvailable, V::NotAvailable);
      case 'C':
        return bind([] { focusStep(-1); }, [] { focusBigStep(-1); }).help(nullptr, V::NotAvailable, V::NotAvailable);
      default: return {};
    }
  }
  switch (key) {
    case 'A':
      return bind(focusSay, focusEntry, info.hasCentre ? focusCentre : nullptr)
          .help(info.label, V::Say, info.kind == SettingKind::Number ? V::Entry : V::Pick, V::Centre);
    case 'B': return bind([] { focusStep(1); }, [] { focusBigStep(1); }).help(info.label, V::Up, V::UpFast);
    case 'C': return bind([] { focusStep(-1); }, [] { focusBigStep(-1); }).help(info.label, V::Down, V::DownFast);
    default: return {};
  }
}

bool isFocusKey(char key) { return key == 'A' || key == 'B' || key == 'C'; }

// A key that runs the console command cmd, traced "CMD <key> -> <cmd>". The
// '+' turns the lambda into an Action, so it also works in a conditional.
#define SEND(cmd) +[] { sendKeypadCommand(cmd); }

using L = KeypadLayout;

bool isFt8x7(const KeypadTraits& t) {
  return t.layout == L::Ft8x7 || t.layout == L::Ft817 || t.layout == L::Ft857;
}

// An FTDX10 key whose function the radio hides: label, then "not available", on each gesture.
Key ftdx10Hidden(const char* label, bool hold, bool dbl = false) {
  return bind(reportFtdx10HiddenKey, hold ? reportFtdx10HiddenKey : nullptr, dbl ? reportFtdx10HiddenKey : nullptr)
      .help(label, V::NotAvailable, V::NotAvailable, V::NotAvailable);
}

KeyBinding bank1(const KeypadTraits& t, char key, Setting focus) {
  if (isFocusKey(key)) return focusKey(key, focus);
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '0':
      return bind(ftdx10 ? SEND("FREQ?") : queryBank1Frequency, beginBank1FrequencySet,
                  [] { roundActiveFrequency(500); })
          .help("FREQ", V::Say, V::Entry)
          .helpDouble("ROUND", V::Centre);
    case '1': return bind(ftdx10 ? SEND("RXTX?") : queryBank1RxTx, nullptr, waitOnly).help("RXTX", V::Say);
    case '2':
      if (ftdx10) return bind(SEND("TXFREQ?"), nullptr, waitOnly).help("TXFREQ", V::Say);
      if (t.layout == L::Ft857) return bind(queryBank1Ft857TxFrequency, nullptr, waitOnly).help("TXFREQ", V::Say);
      return bind(queryBank1TxFrequency, nullptr, waitOnly).help("TXFREQ", V::Say);
    case '3':
      if (ftdx10) return bind(SEND("LOCK?"), SEND("LOCK TOGGLE")).help("LOCK", V::Say, V::Toggle);
      return bind(queryBank1Lock, toggleBank1Lock).help("LOCK", V::Say, V::Toggle);
    case '4': return bind(queryBank1Power).help("PO", V::Say);
    // The FT-8x7 reads its ALC from a 0..15 TX meter, console only (ALC?).
    case '5': return isFt8x7(t) ? KeyBinding{} : bind(queryBank1Alc).help("ALC", V::Say);
    case '6': {
      // FT-8x7: experimental; the FT-817/818 picks a level.
      if (t.layout == L::Ft8x7) return {};
      const bool reads = t.layout == L::Ft857 || t.layout == L::Ft817 || t.canGetRfPower;
      return bind(reads ? queryBank1RfPower : nullptr, beginBank1RfPowerSet)
          .withFocus(Setting::RfPower)
          .help("RFPOWER", V::Say, t.layout == L::Ft817 ? V::Pick : V::Entry);
    }
    case '7': return bind(queryBank1Smeter).help("SM", V::Say);
    case '8': return bind(queryBank1Swr).help("SWR", V::Say);
    case '9': return bind(queryBank1Mode, beginBank1ModeSelect).withFocus(Setting::Mode).help("MODE", V::Say, V::Pick);
    default: return {};
  }
}

KeyBinding bank2(const KeypadTraits& t, char key) {
  const bool civ = t.layout == L::Civ;
  const bool ftdx10 = t.layout == L::Ftdx10;
  // FT-817 and FT-857/897: settings the radio keeps in its EEPROM, read only but for AGC ('A'
  // below), NR, NB and notch ('1'-'3' below) and the FT-857/897 DSP keys '4'-'8', which experimental mode can
  // change. The FT-817 has no DSP and no readable IPO/ATT, so of these it answers only AGC. Its
  // '0' is the antenna jack of the current band.
  if (t.layout == L::Ft817 && key == '0') return bind(queryBank2Ft817Antenna).help("ANT", V::Say);
  if (t.layout == L::Ft857) {
    switch (key) {
      case '4':
        return bind(queryBank2Ft8x7NrLevel, [] { stepBank2Ft8x7NrLevel(1); }, [] { stepBank2Ft8x7NrLevel(-1); })
            .help("NRLEVEL", V::Say, V::Up, V::Down);
      case '5':
        return bind(queryBank2Ft8x7NbLevel, [] { stepBank2Ft8x7NbLevel(10); }, [] { stepBank2Ft8x7NbLevel(-10); })
            .help("NBLEVEL", V::Say, V::Up, V::Down);
      case '6': return bind(queryBank2Ft8x7Dbf, toggleBank2Ft8x7Dbf).help("DBF", V::Say, V::Toggle);
      case '7':
        return bind(queryBank2Ft8x7LowCut, [] { stepBank2Ft8x7LowCut(1); }, [] { stepBank2Ft8x7LowCut(-1); })
            .help("HPF", V::Say, V::Up, V::Down);
      case '8':
        return bind(queryBank2Ft8x7HighCut, [] { stepBank2Ft8x7HighCut(1); }, [] { stepBank2Ft8x7HighCut(-1); })
            .help("LPF", V::Say, V::Up, V::Down);
      default: break;
    }
  }
  if (t.layout == L::Ft817 || t.layout == L::Ft857) {
    switch (key) {
      case '4': return bind(queryBank2Ft8x7NrLevel).help("NRLEVEL", V::Say);
      case '5': return bind(queryBank2Ft8x7NbLevel).help("NBLEVEL", V::Say);
      case '6': return bind(queryBank2Ft8x7Dbf).help("DBF", V::Say);
      case '7': return bind(queryBank2Ft8x7LowCut).help("HPF", V::Say);
      case '8': return bind(queryBank2Ft8x7HighCut).help("LPF", V::Say);
      case '9': return bind(queryBank2Ft8x7MicEq).help("MICEQ", V::Say);
      case 'B': return bind(queryBank2Ft8x7Ipo, nullptr, queryBank2Att).help("IPO", V::Say).helpDouble("ATT", V::Say);
      default: break;
    }
  }
  switch (key) {
    // Every radio but the FT-817 ('0' above): the auto notch.
    case '0': return bind(queryBank2AutoNotch, toggleBank2AutoNotch).help("ANOTCH", V::Say, V::Toggle);
    case '1': return bind(queryBank2Nr, toggleBank2Nr).help("NR", V::Say, V::Toggle);
    case '2': return bind(queryBank2Nb, toggleBank2Nb).help("NB", V::Say, V::Toggle);
    case '3': return bind(queryBank2Notch, toggleBank2Notch).help("NOTCH", V::Say, V::Toggle);
    case '4':
      if (civ) {
        return bind(queryBank2NrLevel, [] { adjustBank2NrLevel(10); }, [] { adjustBank2NrLevel(-10); })
            .help("NRLEVEL", V::Say, V::Up, V::Down);
      }
      // The Yaesu ASCII radios: RL0 1..15 (other radios say "not available").
      return bind([] { queryBank2YaesuRxLevel(RxLevel::Nr); }, [] { stepBank2YaesuRxLevel(RxLevel::Nr, 1); },
                  [] { stepBank2YaesuRxLevel(RxLevel::Nr, -1); })
          .help("NRLEVEL", V::Say, V::Up, V::Down);
    case '5':
      if (civ) {
        return bind(queryBank2NbLevel, [] { adjustBank2NbLevel(10); }, [] { adjustBank2NbLevel(-10); })
            .help("NBLEVEL", V::Say, V::Up, V::Down);
      }
      if (ftdx10) {
        return bind(SEND("PS?"), SEND("PS OFF"), SEND("PS ON"))
            .help("PS", V::Say)
            .helpHold("PS OFF", V::Run)
            .helpDouble("PS ON", V::Run);
      }
      // The Yaesu ASCII radios: NL0 0..10. The FTDX10 keeps the power switch here.
      return bind([] { queryBank2YaesuRxLevel(RxLevel::Nb); }, [] { stepBank2YaesuRxLevel(RxLevel::Nb, 1); },
                  [] { stepBank2YaesuRxLevel(RxLevel::Nb, -1); })
          .help("NBLEVEL", V::Say, V::Up, V::Down);
    case '6':
      if (civ) {
        return bind(queryBank2PbtInner, [] { adjustBank2PbtInner(10); }, [] { adjustBank2PbtInner(-10); })
            .help("PBT1", V::Say, V::Up, V::Down);
      }
      if (ftdx10) return bind(SEND("IF?")).help("IF", V::Say);
      return {};
    case '7':
      if (civ) {
        return bind(queryBank2PbtOuter, [] { adjustBank2PbtOuter(10); }, [] { adjustBank2PbtOuter(-10); })
            .help("PBT2", V::Say, V::Up, V::Down);
      }
      if (ftdx10) return bind(SEND("ID?")).help("ID", V::Say);
      return {};
    case '8':
      if (civ) return bind(SEND("FILSHAPE?"), toggleBank2FilterShape).help("FILSHAPE", V::Say, V::Toggle);
      if (ftdx10) return ftdx10Hidden("FILSHAPE", false);
      return {};
    case '9':
      if (civ) {
        return bind(queryBank2FilterWidth, [] { cycleBank2FilterWidth(1); }, [] { cycleBank2FilterWidth(-1); })
            .help("FILWIDTH", V::Say, V::Up, V::Down);
      }
      if (ftdx10) return ftdx10Hidden("FILWIDTH", false);
      return {};
    // Every radio: what agcQuery reads, and the select of the speeds it can be set to.
    case 'A': return bind(queryBank2Agc, beginBank2AgcSelect).help("AGC", V::Say, V::Pick);
    // Every radio but the FT-8x7 ('B' above): the preamp and the attenuator, hold for the next
    // level.
    case 'B': return bind(queryBank2Preamp, stepBank2Preamp).help("PREAMP", V::Say, V::Next);
    case 'C': return bind(queryBank2Att, stepBank2Att).help("ATT", V::Say, V::Next);
    default: return {};
  }
}

// Bank 3 '7'-'9': band stack register Reg, said by label.
template <uint8_t Reg>
KeyBinding bank3BandStack(const KeypadTraits& t, const char* label) {
  const bool ftdx10 = t.layout == L::Ftdx10;
  return bind([] { queryBank3BandStack(Reg); }, ftdx10 ? reportFtdx10HiddenKey : +[] { recallBank3BandStack(Reg); })
      .help(label, V::Say, ftdx10 ? V::NotAvailable : V::Pick);
}

// FT-817 and FT-857/897 track the active VFO themselves: '1' is the current
// VFO and '2' the other one.
KeyBinding bank3(const KeypadTraits& t, char key, Setting focus) {
  if (isFocusKey(key)) return focusKey(key, focus);
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '0':
      return bind(ftdx10 ? SEND("SPLIT?") : queryBank3Split, ftdx10 ? SEND("SPLIT TOGGLE") : toggleBank3Split,
                  ftdx10 ? SEND("TXFREQ?") : queryBank3TxFrequency)
          .help("SPLIT", V::Say, V::Toggle)
          .helpDouble("TXFREQ", V::Say);
    case '1':
      if (t.layout == L::Ft817 || t.layout == L::Ft857) {
        return bind(queryBank3Ft8x7CurrentVfo, toggleBank3Ft8x7Vfo, beginBank3Ft8x7CurrentVfoFrequencySet)
            .help("VFO", V::Say)
            .helpHold("A/B", V::Run)
            .helpDouble("FREQ", V::Entry);
      }
      return bind(ftdx10 ? SEND("VFOA?") : queryBank3VfoA, ftdx10 ? SEND("VFO A") : selectBank3VfoA,
                  beginBank3VfoAFrequencySet)
          .help("VFOA", V::Say)
          .helpHold("VFO A", V::Pick)
          .helpDouble("VFOA FREQ", V::Entry);
    case '2':
      if (t.layout == L::Ft817) {
        return bind(queryBank3Ft8x7OtherVfo, copyBank3Ft817VfoToOther, beginBank3Ft8x7OtherVfoFrequencySet)
            .help("VFO", V::Say)
            .helpHold("A=B", V::Run)
            .helpDouble("FREQ", V::Entry);
      }
      if (t.layout == L::Ft857) {
        return bind(queryBank3Ft8x7OtherVfo, reportBank3Ft857VfoBUnsupported, beginBank3Ft8x7OtherVfoFrequencySet)
            .help("VFO", V::Say)
            .helpHold("A=B", V::NotAvailable)
            .helpDouble("FREQ", V::Entry);
      }
      return bind(ftdx10 ? SEND("VFOB?") : queryBank3VfoB, ftdx10 ? SEND("VFO B") : selectBank3VfoB,
                  beginBank3VfoBFrequencySet)
          .help("VFOB", V::Say)
          .helpHold("VFO B", V::Pick)
          .helpDouble("VFOB FREQ", V::Entry);
    case '3':
      if (t.layout == L::Ft817) return bind(queryBank3VfoAMode, beginBank3VfoAModeSet).help("VFOA MODE", V::Say, V::Pick);
      return {};
    case '4':
      // The FT-857/897 reads its VFO from the radio, so only the FT-817 needs the sync keys.
      if (t.layout == L::Ft817) {
        return bind(syncBank3VfoA, syncBank3VfoB).helpShort("SYNC A", V::Run).helpHold("SYNC B", V::Run);
      }
      if (t.layout == L::Ft857) return {};
      return bind(ftdx10 ? SEND("VFOA MODE?") : queryBank3VfoAMode, beginBank3VfoAModeSet)
          .help("VFOA MODE", V::Say, V::Pick);
    case '5':
      if (t.layout == L::Ft817 || t.layout == L::Ft857) {
        return bind(queryBank3Ft8x7Rit, toggleBank3Ft8x7Rit).help("RIT", V::Say, V::Toggle);
      }
      return bind(ftdx10 ? SEND("VFOB MODE?") : queryBank3VfoBMode, beginBank3VfoBModeSet)
          .help("VFOB MODE", V::Say, V::Pick);
    case '6':
      if (t.layout == L::Ft817) {
        return bind(selectBank3Ft817ActiveVfoA, selectBank3Ft817ActiveVfoB)
            .helpShort("VFO A", V::Pick)
            .helpHold("VFO B", V::Pick);
      }
      if (t.layout == L::Ft857) {
        return bind([] { setBank3Ft857Ptt(false); }, [] { setBank3Ft857Ptt(true); })
            .helpShort("PTT OFF", V::Run)
            .helpHold("PTT ON", V::Run);
      }
      return bind(ftdx10 ? SEND("RXTX?") : queryBank3RxTx).help("RXTX", V::Say);
    case '7': return bank3BandStack<1>(t, "BSTACK1");
    case '8': return bank3BandStack<2>(t, "BSTACK2");
    case '9': return bank3BandStack<3>(t, "BSTACK3");
    default: return {};
  }
}

KeyBinding bank4(const KeypadTraits& t, char key) {
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '0':
      return bind(ftdx10 ? SEND("TUNER?") : queryBank4Tuner, ftdx10 ? SEND("TUNER TOGGLE") : toggleBank4Tuner,
                  ftdx10 ? SEND("TUNE") : triggerBank4Tune)
          .help("TUNER", V::Say, V::Toggle)
          .helpDouble("TUNE", V::Run);
    case '1':
      if (ftdx10) return ftdx10Hidden("MONITOR", true);
      return bind(t.supportsMonitor ? SEND("MONITOR?") : nullptr, toggleBank4Monitor).help("MONITOR", V::Say, V::Toggle);
    case '2':
      if (ftdx10) return ftdx10Hidden("MONLEVEL", true, true);
      return bind(queryBank4MonitorLevel, [] { adjustBank4MonitorLevel(10); }, [] { adjustBank4MonitorLevel(-10); })
          .help("MONLEVEL", V::Say, V::Up, V::Down);
    case '3':
      if (ftdx10) return ftdx10Hidden("TRANSCEIVE", true);
      return bind(t.supportsTransceive ? SEND("TRANSCEIVE?") : nullptr, toggleBank4Transceive)
          .help("TRANSCEIVE", V::Say, V::Toggle);
    default: return {};
  }
}

// The RIT offset is the home focus, and every key but the on/off switch moves it.
KeyBinding bank5(char key, Setting focus) {
  if (isFocusKey(key)) return focusKey(key, focus);
  constexpr Setting kOffset = Setting::RitOffset;
  constexpr const char* kLabel = "RITOFFSET";
  switch (key) {
    case '0':
      return bind(queryBank5Rit, toggleBank5Rit, [] { setBank5RitOffset(0); })
          .help("RIT", V::Say, V::Toggle)
          .helpDouble(kLabel, V::Centre);
    case '1':
      return bind([] { adjustBank5Rit(-10); }, [] { adjustBank5Rit(-100); })
          .withFocus(kOffset)
          .help(kLabel, V::Down, V::DownFast);
    case '2':
      return bind([] { adjustBank5Rit(10); }, [] { adjustBank5Rit(100); })
          .withFocus(kOffset)
          .help(kLabel, V::Up, V::UpFast);
    case '3':
      return bind([] { setBank5RitOffset(0); }, setBank5RitOff)
          .withFocus(kOffset)
          .help(kLabel, V::Centre)
          .helpHold("RIT OFF", V::Run);
    case '4':
      return bind([] { adjustBank5Rit(-1); }, [] { adjustBank5Rit(-500); })
          .withFocus(kOffset)
          .help(kLabel, V::Down, V::DownFast);
    case '5':
      return bind([] { adjustBank5Rit(1); }, [] { adjustBank5Rit(500); })
          .withFocus(kOffset)
          .help(kLabel, V::Up, V::UpFast);
    default: return {};
  }
}

// FT-8x7 repeater shift, offset and tones. Empty on the other radios.
KeyBinding bank6(const KeypadTraits& t, char key, Setting focus) {
  if (!isFt8x7(t)) return {};
  if (isFocusKey(key)) return focusKey(key, focus);
  switch (key) {
    case '0':
      return bind(setBank6RepeaterOff, setBank6RepeaterMinus, setBank6RepeaterPlus)
          .helpShort("RPT OFF", V::Run)
          .helpHold("RPT MINUS", V::Run)
          .helpDouble("RPT PLUS", V::Run);
    case '1':
      return bind([] { setBank6RepeaterOffsetPreset(1); }, [] { setBank6RepeaterOffsetPreset(2); },
                  beginBank6RepeaterOffsetEntry)
          .helpShort("RPTSHIFT1", V::Run)
          .helpHold("RPTSHIFT2", V::Run)
          .helpDouble("RPTSHIFT", V::Entry);
    case '2':
      return bind(queryBank6Tone, setBank6ToneModeCtcss, setBank6ToneOff, setBank6ToneModeDcs)
          .help("TONE", V::Say)
          .helpHold("TONE CTCSS", V::Run)
          .helpDouble("TONE OFF", V::Run)
          .helpDoubleHold("TONE DCS", V::Run);
    case '3': return bind(queryBank6CtcssDefault, beginBank6CtcssEntry).help("CTCSS", V::Say, V::Entry);
    case '4': return bind(queryBank6DcsDefault, beginBank6DcsEntry).help("DCS", V::Say, V::Entry);
    case '5': return bind(queryBank6ToneDir, stepBank6ToneDir).help("TONEDIR", V::Say, V::Next);
    case '6': return bind(queryBank6SplitTone).help("SPLITTONE", V::Say);
    default: return {};
  }
}

KeyBinding bank8(const KeypadTraits& t, char key, Setting focus) {
  if (isFocusKey(key)) return focusKey(key, focus);
  // FT-817 and FT-857/897: the radio's menu position.
  const bool ft8x7Menu = t.layout == L::Ft817 || t.layout == L::Ft857;
  switch (key) {
    case '1': return bind(queryBank8CivAddress, beginBank8CivAddressEntry).help("CIVADDR", V::Say, V::Entry);
    case '2':
      return bind(queryBank8Baud, [] { cycleBank8Baud(-1); }, [] { cycleBank8Baud(1); })
          .help("BAUD", V::Say, V::Down, V::Up);
    case '7':
      if (ft8x7Menu) return bind(queryBank8Ft8x7Row, beginBank8Ft8x7RowEntry).help("ROW", V::Say, V::Entry);
      return {};
    case '8':
      if (ft8x7Menu) return bind(queryBank8Ft8x7Menu, beginBank8Ft8x7MenuEntry).help("MENU", V::Say, V::Entry);
      return {};
    default: return {};
  }
}

// Bank 7: CW. Every radio; the FT-8x7 sets experimentally.
KeyBinding bank7(char key) {
  switch (key) {
    case '1': return bind(queryBank7BreakIn, toggleBank7BreakIn).help("BK", V::Say, V::Toggle);
    case '2': return bind(queryBank7Keyer, toggleBank7Keyer).help("KYR", V::Say, V::Toggle);
    case '3':
      return bind([] { queryBank7CwLevel(CwLevel::Speed); }, [] { stepBank7CwLevel(CwLevel::Speed, 1); },
                  [] { stepBank7CwLevel(CwLevel::Speed, -1); })
          .help("CWSPEED", V::Say, V::Up, V::Down);
    case '4':
      return bind([] { queryBank7CwLevel(CwLevel::Pitch); }, [] { stepBank7CwLevel(CwLevel::Pitch, 1); },
                  [] { stepBank7CwLevel(CwLevel::Pitch, -1); })
          .help("CWPITCH", V::Say, V::Up, V::Down);
    case '5':
      return bind([] { queryBank7CwLevel(CwLevel::Delay); }, [] { stepBank7CwLevel(CwLevel::Delay, 1); },
                  [] { stepBank7CwLevel(CwLevel::Delay, -1); })
          .help("CWDELAY", V::Say, V::Up, V::Down);
    default: return {};
  }
}

// The profile is the home focus: A, B and C select, step and reset it until the speed or
// volume keys move the focus.
KeyBinding bank9(char key, Setting focus) {
  if (isFocusKey(key)) return focusKey(key, focus);
  constexpr Setting kVolume = Setting::Volume;
  switch (key) {
    case '0':
      return bind(queryBank9Experimental, toggleBank9Experimental).help("EXPERIMENTAL", V::Say, V::Toggle);
    case '4':
      return bind(queryBank9TuningSpeech, toggleBank9TuningSpeech).help("TUNINGSPEECH", V::Say, V::Toggle);
    case '5': return bind(queryBank9Verbose, toggleBank9Verbose).help("VERBOSE", V::Say, V::Toggle);
    case '6':
      return bind(queryBank9Speed, stepBank9Speed).withFocus(Setting::SpeechSpeed).help("SPEED", V::Say, V::Next);
    case '7':
      return bind([] { adjustBank9Volume(-1); }, [] { adjustBank9Volume(-2); })
          .withFocus(kVolume)
          .help("VOLUME", V::Down, V::DownFast);
    case '8':
      return bind([] { adjustBank9Volume(1); }, [] { adjustBank9Volume(2); })
          .withFocus(kVolume)
          .help("VOLUME", V::Up, V::UpFast);
    case '9': return bind(queryBank9Volume).withFocus(kVolume).help("VOLUME", V::Say);
    default: return {};
  }
}

#undef SEND

}  // namespace

Setting keymapHomeFocus(const KeypadTraits&, uint8_t bank) {
  switch (bank) {
    case 5: return Setting::RitOffset;
    case 9: return Setting::Profile;
    default: return Setting::None;
  }
}

KeyBinding keymapLookup(const KeypadTraits& traits, uint8_t bank, char key, Setting focus) {
  if (focus == Setting::Home) focus = keymapHomeFocus(traits, bank);
  switch (bank) {
    case 1: return bank1(traits, key, focus);
    case 2: return bank2(traits, key);
    case 3: return bank3(traits, key, focus);
    case 4: return bank4(traits, key);
    case 5: return bank5(key, focus);
    case 6: return bank6(traits, key, focus);
    case 7: return bank7(key);
    case 8: return bank8(traits, key, focus);
    case 9: return bank9(key, focus);
    default: return {};
  }
}
