#pragma once

#include <stdint.h>

// The radio and speech settings the keypad names: what a key's A, B and C move
// (its focus), and what help mode calls it. Pure C++ with no Arduino.h, so the
// keymap and the host tests (tests/keypad) use it.
//
// One Setting covers a value on every radio: NrLevel is the Icom level in
// percent, the Yaesu RL0 step and the FT-857 menu 49 alike. How a radio reads,
// sets and steps it is not here.

enum class Setting : uint8_t {
  None,  // nothing to move
  Home,  // the bank's home focus, before a key has acted
  // Bank 1
  RfPower,
  Mode,
  // Bank 2
  Nr,
  NrLevel,
  Nb,
  NbLevel,
  Notch,
  AutoNotch,
  RxLowCut,
  RxHighCut,
  PbtInner,
  PbtOuter,
  FilterSlot,
  FilterShape,
  Dbf,
  BpfWidth,
  Agc,
  Preamp,
  Att,
  Antenna,
  // Bank 4
  Proc,
  ProcLevel,
  Vox,
  VoxGain,
  Monitor,
  MonitorLevel,
  TxLowCut,
  TxEq,
  TxHighCut,
  MicGain,
  VoxDelay,
  // Bank 5
  Rit,
  RitOffset,
  // Bank 6
  RptShift,
  ToneType,
  Ctcss,
  Dcs,
  // Bank 7
  Keyer,
  CwSpeed,
  BreakIn,
  Qsk,
  Sidetone,
  Paddle,
  CwPitch,
  CwWeight,
  KeyReverse,
  CwDelay,
  // Bank 9
  Profile,
  SpeechSpeed,
  Volume,
  Count
};

// What a setting is on most radios. A radio may differ (the TS-480 NR is a list
// of NR1, NR2 and off), so this only picks a key's focus and help wording.
enum class SettingKind : uint8_t {
  Switch,  // on or off: nothing for B and C to move
  List,    // named states or a table of values
  Number,  // a value in a range
};

struct SettingInfo {
  Setting id;
  SettingKind kind;
  // A double press of A brings it back to a centre or default (PBT centre,
  // RIT offset 0, normal speech speed, the profile reset).
  bool hasCentre;
  // Its trace label, as the console command names it ("NRLEVEL"). Also the key
  // of its spoken name in kSpokenLabels (keypad_labels.cpp).
  const char* label;
};

// The row of setting; None and Home have rows too.
const SettingInfo& settingInfo(Setting setting);
