#pragma once

#include <Arduino.h>

#include "keypad_value_entry.h"
#include "radio_features.h"

// How the keypad reads, sets and says one Setting on the current radio. What
// differs per radio stays inside: one RfPower covers the Icom level, the ASCII
// PC and the FT-857/897 menu 75.
struct SettingOps {
  Setting setting;
  const char* prompt;  // the word its entry asks with: "power" -> "power please"
  // The radio has it, by its caps; no radio traffic.
  bool (*available)();
  // Fills spec for an entry: range, steps and how it is typed. May read the
  // radio (the band group of the FT-857/897 power). Unsupported where it
  // cannot be set.
  FeatureStatus (*limits)(ValueSpec& spec);
  // The value now, in spec's units.
  FeatureStatus (*read)(int32_t& value);
  // Sets value; readBack holds what the radio confirmed, or value where it
  // cannot be read back.
  FeatureStatus (*write)(int32_t value, int32_t& readBack);
  // Trace text ("RFPOWER 45 W") and the spoken value, its name as LabelMode
  // has it ("power 45 watts").
  String (*text)(int32_t value);
  void (*speak)(int32_t value);
  // True where B and C open the entry one step away, staged for Enter, rather
  // than set the value at once: RF power, the FT-8x7 EEPROM settings. nullptr:
  // never.
  bool (*confirm)() = nullptr;
  // What A double sets, where settingInfo says the setting has a centre.
  int32_t centre = 0;
};

// nullptr for a setting with no ops.
const SettingOps* settingOps(Setting setting);
// The Bank 2 receive settings (keypad_setting_ops_rx.cpp), which settingOps also
// finds.
const SettingOps* rxSettingOps(Setting setting);
// The Bank 4 phone transmit settings (keypad_setting_ops_tx.cpp), likewise.
const SettingOps* txSettingOps(Setting setting);
