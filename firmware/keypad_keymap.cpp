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
  const bool none = focus == Setting::None;
  const char* label = none ? nullptr : info.label;
  const auto verb = [none](V v) { return none ? V::NotAvailable : v; };
  switch (key) {
    case 'A':
      return bind(focusSay, focusEntry, info.hasCentre ? focusCentre : nullptr)
          .help(label, verb(V::Say), verb(info.kind == SettingKind::Number ? V::Entry : V::Pick), V::Centre);
    case 'B': return bind([] { focusStep(1); }, [] { focusBigStep(1); }).help(label, verb(V::Up), verb(V::UpFast));
    case 'C':
      return bind([] { focusStep(-1); }, [] { focusBigStep(-1); }).help(label, verb(V::Down), verb(V::DownFast));
    default: return {};
  }
}

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

// RF power, on Bank 1 and the transmit banks: its entry waits for Enter on every radio. The
// FT-8x7 sets it experimentally; the FT-817/818 picks a level.
KeyBinding rfPowerKey(const KeypadTraits& t) {
  if (t.layout == L::Ft8x7) return {};
  const bool reads = t.layout == L::Ft857 || t.layout == L::Ft817 || t.canGetRfPower;
  return bind(reads ? queryBank1RfPower : nullptr, beginBank1RfPowerSet)
      .withFocus(Setting::RfPower)
      .help("RFPOWER", V::Say, t.layout == L::Ft817 ? V::Pick : V::Entry);
}

KeyBinding bank1(const KeypadTraits& t, char key) {
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '0':
      return bind(ftdx10 ? SEND("FREQ?") : queryBank1Frequency, beginBank1FrequencySet,
                  [] { roundActiveFrequency(500); })
          .help("FREQ", V::Say, V::Entry)
          .helpDouble("ROUND", V::Run);
    case '1': return bind(ftdx10 ? SEND("RXTX?") : queryBank1RxTx, nullptr, waitOnly).help("RXTX", V::Say);
    case '2':
      if (ftdx10) return bind(SEND("TXFREQ?"), nullptr, waitOnly).help("TXFREQ", V::Say);
      if (t.layout == L::Ft857) return bind(queryBank1Ft857TxFrequency, nullptr, waitOnly).help("TXFREQ", V::Say);
      return bind(queryBank1TxFrequency, nullptr, waitOnly).help("TXFREQ", V::Say);
    case '3':
      if (ftdx10) return bind(SEND("LOCK?"), SEND("LOCK TOGGLE")).help("LOCK", V::Say, V::Toggle);
      return bind(queryBank1Lock, toggleBank1Lock).help("LOCK", V::Say, V::Toggle);
    case '4': return bind(queryBank1Power).help("PO", V::Say);
    case '5': return bind(queryBank1Alc).help("ALC", V::Say);
    case '6': return rfPowerKey(t);
    case '7': return bind(queryBank1Smeter).help("SM", V::Say);
    case '8': return bind(queryBank1Swr).help("SWR", V::Say);
    case '9': return bind(queryBank1Mode, beginBank1ModeSelect).withFocus(Setting::Mode).help("MODE", V::Say, V::Pick);
    default: return {};
  }
}

// Actions on the Setting S, for a key to bind.
template <Setting S>
void saySetting() {
  settingSay(S);
}
template <Setting S>
void enterSetting() {
  settingEntry(S);
}
template <Setting S>
void nextSetting() {
  settingNext(S);
}

// k, with s as its focus where the radio has s.
Key withFocusIfHas(const KeypadTraits& t, const Key& k, Setting s) {
  return t.settings.has(s) ? k.withFocus(s) : k;
}

// A key of its own for S: short says it, long opens its entry (a number) or pick (a list), and
// it is the focus where the radio has it.
template <Setting S>
Key settingKey(const KeypadTraits& t) {
  const SettingInfo& info = settingInfo(S);
  const V hold = info.kind == SettingKind::Number ? V::Entry : V::Pick;
  return withFocusIfHas(t, bind(saySetting<S>, enterSetting<S>).help(info.label, V::Say, hold), S);
}

// A switch with a level (NR, NB): short says it, long toggles it, a double says the level and a
// short then hold opens the level's entry, which is the focus. Where the radio has no level,
// just the switch, with no wait for a double.
template <Setting Level>
Key switchWithLevel(const KeypadTraits& t, Action say, Action toggle, const char* label) {
  if (!t.settings.has(Level)) return bind(say, toggle).help(label, V::Say, V::Toggle);
  const char* levelLabel = settingInfo(Level).label;
  return bind(say, toggle, saySetting<Level>, enterSetting<Level>)
      .withFocus(Level)
      .help(label, V::Say, V::Toggle)
      .helpDouble(levelLabel, V::Say)
      .helpDoubleHold(levelLabel, V::Entry);
}

// Receive. The top row the switches with a level, the middle row the passband (low side, filter,
// high side), the bottom row AGC, preamp and attenuator, '0' the auto notch (FT-817: the antenna
// jack). The FT-857/897 DSP has only an auto notch, on '3', so its '0' is free. The FT-8x7
// keeps these in its EEPROM: read always, set only experimentally.
KeyBinding bank2(const KeypadTraits& t, char key) {
  const bool ft8x7 = isFt8x7(t);
  switch (key) {
    case '0':
      if (t.layout == L::Ft817) {
        return bind(queryBank2Ft817Antenna, toggleBank2Ft817Antenna).help("ANT", V::Say, V::Toggle);
      }
      if (ft8x7) return {};
      return bind(queryBank2AutoNotch, toggleBank2AutoNotch).help("ANOTCH", V::Say, V::Toggle);
    case '1': return switchWithLevel<Setting::NrLevel>(t, queryBank2Nr, toggleBank2Nr, "NR");
    case '2': return switchWithLevel<Setting::NbLevel>(t, queryBank2Nb, toggleBank2Nb, "NB");
    // The Icom notch toggle walks its widths, which B and C step too.
    case '3':
      if (ft8x7) return bind(queryBank2Ft8x7AutoNotch, toggleBank2Ft8x7AutoNotch).help("ANOTCH", V::Say, V::Toggle);
      return withFocusIfHas(t, bind(queryBank2Notch, toggleBank2Notch).help("NOTCH", V::Say, V::Toggle),
                            Setting::Notch);
    case '4': return ft8x7 ? settingKey<Setting::RxLowCut>(t) : settingKey<Setting::PbtInner>(t);
    case '5':
      // FT-857/897: DBF, its bandpass width second. Icom: the filter, its shape second.
      if (ft8x7) {
        if (!t.settings.has(Setting::BpfWidth)) return bind(queryBank2Ft8x7Dbf).help("DBF", V::Say);
        return bind(queryBank2Ft8x7Dbf, toggleBank2Ft8x7Dbf, saySetting<Setting::BpfWidth>,
                    enterSetting<Setting::BpfWidth>)
            .withFocus(Setting::BpfWidth)
            .help("DBF", V::Say, V::Toggle)
            .helpDouble("BPF", V::Say)
            .helpDoubleHold("BPF", V::Pick);
      }
      if (!t.settings.has(Setting::FilterSlot)) {
        return bind(saySetting<Setting::FilterSlot>).help("FILWIDTH", V::Say);
      }
      return bind(saySetting<Setting::FilterSlot>, nextSetting<Setting::FilterSlot>, SEND("FILSHAPE?"),
                  toggleBank2FilterShape)
          .withFocus(Setting::FilterSlot)
          .help("FILWIDTH", V::Say, V::Next)
          .helpDouble("FILSHAPE", V::Say)
          .helpDoubleHold("FILSHAPE", V::Toggle);
    case '6': return ft8x7 ? settingKey<Setting::RxHighCut>(t) : settingKey<Setting::PbtOuter>(t);
    // Every radio: what agcQuery reads, and the select of the speeds it can be set to.
    case '7':
      return withFocusIfHas(t, bind(queryBank2Agc, beginBank2AgcSelect).help("AGC", V::Say, V::Pick), Setting::Agc);
    // The FT-8x7 reads its IPO and attenuator only (the FT-817/818 not even those); the other
    // radios step to the next level on a hold.
    case '8':
      if (ft8x7) return bind(queryBank2Ft8x7Ipo).help("IPO", V::Say);
      return withFocusIfHas(t, bind(queryBank2Preamp, stepBank2Preamp).help("PREAMP", V::Say, V::Next),
                            Setting::Preamp);
    case '9':
      if (ft8x7) return bind(queryBank2Att).help("ATT", V::Say);
      return withFocusIfHas(t, bind(queryBank2Att, stepBank2Att).help("ATT", V::Say, V::Next), Setting::Att);
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
KeyBinding bank3(const KeypadTraits& t, char key) {
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

// A key for a setting S the radio may lack: where it has it, its say and toggle (a switch) or its
// key of its own; else "not available" on a short.
template <Setting S>
Key settingOrNotAvailable(const KeypadTraits& t) {
  const SettingInfo& info = settingInfo(S);
  if (!t.settings.has(S)) return bind(saySetting<S>).help(info.label, V::NotAvailable);
  if (info.kind != SettingKind::Switch) return settingKey<S>(t);
  return bind(saySetting<S>, nextSetting<S>).help(info.label, V::Say, V::Toggle);
}

// A switch S with its level, as switchWithLevel. Where the radio lacks S but has the level (the
// TS-480 keyer speed), the level's own key; where it has neither, "not available".
template <Setting S, Setting Level>
Key switchSettingWithLevel(const KeypadTraits& t) {
  if (t.settings.has(S)) return switchWithLevel<Level>(t, saySetting<S>, nextSetting<S>, settingInfo(S).label);
  if (t.settings.has(Level)) return settingKey<Level>(t);
  return settingOrNotAvailable<S>(t);
}

// A short list S (two or three named states): short says it, long steps to the next; "not
// available" where the radio lacks it.
template <Setting S>
Key shortListOrNotAvailable(const KeypadTraits& t) {
  const char* label = settingInfo(S).label;
  if (!t.settings.has(S)) return bind(saySetting<S>).help(label, V::NotAvailable);
  return bind(saySetting<S>, nextSetting<S>).help(label, V::Say, V::Next);
}

// The tuner, '0' on both transmit banks: says it, toggles it, a double tunes.
KeyBinding tunerKey(const KeypadTraits& t) {
  const bool ftdx10 = t.layout == L::Ftdx10;
  return bind(ftdx10 ? SEND("TUNER?") : queryBank4Tuner, ftdx10 ? SEND("TUNER TOGGLE") : toggleBank4Tuner,
              ftdx10 ? SEND("TUNE") : triggerBank4Tune)
      .help("TUNER", V::Say, V::Toggle)
      .helpDouble("TUNE", V::Run);
}

// Phone transmit, the twin of Bank 2: the top row the switches with a level, the middle row the
// TX passband (low side, equalizer, high side), the bottom row mic gain, RF power and VOX delay,
// '0' the tuner. The FT-8x7 keeps these in its EEPROM: read always, set only experimentally.
KeyBinding bank4(const KeypadTraits& t, char key) {
  switch (key) {
    case '0': return tunerKey(t);
    case '1': return switchSettingWithLevel<Setting::Proc, Setting::ProcLevel>(t);
    case '2': return switchSettingWithLevel<Setting::Vox, Setting::VoxGain>(t);
    case '3': return switchSettingWithLevel<Setting::Monitor, Setting::MonitorLevel>(t);
    // FT-857/897: the TX equalizer's HPF, the whole equalizer, its LPF.
    case '4': return settingOrNotAvailable<Setting::TxLowCut>(t);
    case '5': return settingOrNotAvailable<Setting::TxEq>(t);
    case '6': return settingOrNotAvailable<Setting::TxHighCut>(t);
    case '7': return settingOrNotAvailable<Setting::MicGain>(t);
    case '8': return rfPowerKey(t);
    case '9': return settingOrNotAvailable<Setting::VoxDelay>(t);
    default: return {};
  }
}

// The RIT offset is the home focus, and every key but the on/off switch moves it.
KeyBinding bank5(char key) {
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

// Repeater and tone: '0' the shift, '1' the FT-8x7 offset, '2' the tone type, '3' the CTCSS
// tone, '4' the DCS code, '5' encode or decode, '6' the FT-857/897 split tone. The CAT radios
// have the shift, CTCSS and its tone where their profile does; a CTCSS or DCS step is set at
// once, as these are CAT commands on every radio.
KeyBinding bank6(const KeypadTraits& t, char key) {
  const bool ft8x7 = isFt8x7(t);
  const bool tone = t.settings.has(Setting::ToneType);
  switch (key) {
    case '0':
      if (!t.settings.has(Setting::RptShift)) return bind(saySetting<Setting::RptShift>).help("RPT", V::NotAvailable);
      return bind(setBank6RepeaterOff, setBank6RepeaterMinus, setBank6RepeaterPlus)
          .helpShort("RPT OFF", V::Run)
          .helpHold("RPT MINUS", V::Run)
          .helpDouble("RPT PLUS", V::Run);
    case '1':
      // No CAT radio sets its offset: the preset says "not available".
      if (!ft8x7) return bind([] { setBank6RepeaterOffsetPreset(1); }).help("RPTSHIFT", V::NotAvailable);
      return bind([] { setBank6RepeaterOffsetPreset(1); }, [] { setBank6RepeaterOffsetPreset(2); },
                  beginBank6RepeaterOffsetEntry)
          .helpShort("RPTSHIFT1", V::Run)
          .helpHold("RPTSHIFT2", V::Run)
          .helpDouble("RPTSHIFT", V::Entry);
    case '2':
      if (!tone) return bind(queryBank6Tone).help("TONE", V::NotAvailable);
      return bind(queryBank6Tone, setBank6ToneModeCtcss, setBank6ToneOff,
                  t.settings.has(Setting::Dcs) ? setBank6ToneModeDcs : nullptr)
          .help("TONE", V::Say)
          .helpHold("TONE CTCSS", V::Run)
          .helpDouble("TONE OFF", V::Run)
          .helpDoubleHold("TONE DCS", V::Run);
    case '3': return settingOrNotAvailable<Setting::Ctcss>(t);
    case '4': return settingOrNotAvailable<Setting::Dcs>(t);
    case '5':
      if (!tone) return bind(queryBank6ToneDir).help("TONEDIR", V::NotAvailable);
      return bind(queryBank6ToneDir, stepBank6ToneDir).help("TONEDIR", V::Say, V::Next);
    case '6': return ft8x7 ? bind(queryBank6SplitTone).help("SPLITTONE", V::Say) : KeyBinding{};
    default: return {};
  }
}

KeyBinding bank8(const KeypadTraits& t, char key) {
  // FT-817 and FT-857/897: the radio's menu position.
  const bool ft8x7Menu = t.layout == L::Ft817 || t.layout == L::Ft857;
  switch (key) {
    case '1': return bind(queryBank8CivAddress, beginBank8CivAddressEntry).help("CIVADDR", V::Say, V::Entry);
    case '2':
      return bind(queryBank8Baud, [] { cycleBank8Baud(-1); }, [] { cycleBank8Baud(1); })
          .help("BAUD", V::Say, V::Down, V::Up);
    case '3':
      if (t.layout == L::Ftdx10) return ftdx10Hidden("TRANSCEIVE", true);
      return bind(t.supportsTransceive ? SEND("TRANSCEIVE?") : nullptr, toggleBank8Transceive)
          .help("TRANSCEIVE", V::Say, V::Toggle);
    // FTDX10: the radio's power switch, its ID and its status line.
    case '4':
      if (t.layout != L::Ftdx10) return {};
      return bind(SEND("PS?"), SEND("PS OFF"), SEND("PS ON"))
          .help("PS", V::Say)
          .helpHold("PS OFF", V::Run)
          .helpDouble("PS ON", V::Run);
    case '5': return t.layout == L::Ftdx10 ? bind(SEND("ID?")).help("ID", V::Say) : KeyBinding{};
    case '6': return t.layout == L::Ftdx10 ? bind(SEND("IF?")).help("IF", V::Say) : KeyBinding{};
    case '7':
      if (ft8x7Menu) return bind(queryBank8Ft8x7Row, beginBank8Ft8x7RowEntry).help("ROW", V::Say, V::Entry);
      return {};
    case '8':
      if (ft8x7Menu) return bind(queryBank8Ft8x7Menu, beginBank8Ft8x7MenuEntry).help("MENU", V::Say, V::Entry);
      return {};
    default: return {};
  }
}

// Break-in: a switch, with the FT-857/897 QSK time second. The Icom off, semi and full are a
// short list the hold steps through, and B and C walk.
KeyBinding breakInKey(const KeypadTraits& t) {
  constexpr Setting kBreakIn = Setting::BreakIn;
  if (t.settings.has(Setting::Qsk)) return switchSettingWithLevel<kBreakIn, Setting::Qsk>(t);
  if (!t.settings.has(kBreakIn)) return bind(saySetting<kBreakIn>).help("BK", V::NotAvailable);
  const Key key = bind(saySetting<kBreakIn>, nextSetting<kBreakIn>);
  if (t.layout == L::Civ) return key.withFocus(kBreakIn).help("BK", V::Say, V::Next);
  return key.help("BK", V::Say, V::Toggle);
}

// CW, the twin of Bank 4: the top row the keyer with its speed, break-in and the sidetone, the
// middle row the keyer type, pitch and weight, the bottom row key reverse, RF power and the
// break-in delay, '0' the tuner. The FT-8x7 keeps these in its EEPROM: read always, set only
// experimentally.
KeyBinding bank7(const KeypadTraits& t, char key) {
  switch (key) {
    case '0': return tunerKey(t);
    case '1': return switchSettingWithLevel<Setting::Keyer, Setting::CwSpeed>(t);
    case '2': return breakInKey(t);
    case '3': return settingOrNotAvailable<Setting::Sidetone>(t);
    // FT-857/897: the paddle as the keyer (ELEKEY) or the mic keys (MICKEY).
    case '4': return shortListOrNotAvailable<Setting::Paddle>(t);
    case '5': return settingOrNotAvailable<Setting::CwPitch>(t);
    case '6': return settingOrNotAvailable<Setting::CwWeight>(t);
    case '7': return settingOrNotAvailable<Setting::KeyReverse>(t);
    case '8': return rfPowerKey(t);
    case '9': return settingOrNotAvailable<Setting::CwDelay>(t);
    default: return {};
  }
}

// The profile is the home focus: A, B and C select, step and reset it until the volume ('7')
// or speed ('8') key moves the focus, and '1' brings it back.
KeyBinding bank9(char key) {
  constexpr Setting kVolume = Setting::Volume;
  switch (key) {
    case '0':
      return bind(queryBank9Experimental, toggleBank9Experimental).help("EXPERIMENTAL", V::Say, V::Toggle);
    case '1':
      return bind(queryBank9Profile, beginBank9ProfileSelect, resetBank9Profile)
          .withFocus(Setting::Profile)
          .help("PROFILE", V::Say, V::Pick, V::Centre);
    case '4':
      return bind(queryBank9TuningSpeech, toggleBank9TuningSpeech).help("TUNINGSPEECH", V::Say, V::Toggle);
    case '5': return bind(queryBank9Verbose, toggleBank9Verbose).help("VERBOSE", V::Say, V::Toggle);
    case '7':
      return bind(queryBank9Volume, enterSetting<kVolume>).withFocus(kVolume).help("VOLUME", V::Say, V::Entry);
    case '8':
      return bind(queryBank9Speed, stepBank9Speed).withFocus(Setting::SpeechSpeed).help("SPEED", V::Say, V::Next);
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
  if (bank < 1 || bank > 9) return {};
  if (isFocusKey(key)) return focusKey(key, focus == Setting::Home ? keymapHomeFocus(traits, bank) : focus);
  switch (bank) {
    case 1: return bank1(traits, key);
    case 2: return bank2(traits, key);
    case 3: return bank3(traits, key);
    case 4: return bank4(traits, key);
    case 5: return bank5(key);
    case 6: return bank6(traits, key);
    case 7: return bank7(traits, key);
    case 8: return bank8(traits, key);
    default: return bank9(key);
  }
}
