#pragma once

#include <stdint.h>

// What each bank key does, per radio: the keypad layout as code. Pure C++ with
// no Arduino.h; the actions it runs are declared in keypad_actions.h. The host
// tests (tests/keypad) check it against the characterization matrix.
//
// Bank keys are '0'-'9' and 'A'-'C'. '*', 'D' and '#' belong to KeypadInput.

// The radio traits the keymap depends on. The glue fills this in once per key
// event from the active profile.
struct KeypadTraits {
  bool civ = false;                 // PROTO_CIV
  bool ftdx10 = false;              // isFtdx10KeypadProfile()
  bool ft817 = false;               // isFt8x7Ft817Keypad()
  bool ft857Family = false;         // isFt8x7Ft857FamilyKeypad()
  bool lightIcomFallback = false;   // lightIcomFallbackActive()
  bool supportsMonitor = false;     // protocolSupportsMonitor()
  bool supportsTransceive = false;  // protocolSupportsTransceive()
  bool canGetRfPower = false;       // currentStoredProfile().caps.getRfPower
};

// Each returns true when the key has an action for the gesture and it ran.
bool keymapShort(const KeypadTraits& traits, uint8_t bank, char key);
bool keymapHold(const KeypadTraits& traits, uint8_t bank, char key);
bool keymapDoubleClick(const KeypadTraits& traits, uint8_t bank, char key);
// True when a short press of the key waits for a possible double click.
bool keymapWantsDoubleClick(const KeypadTraits& traits, uint8_t bank, char key);

// LEGACY(F1): during mode select, these keys run their short action instead of
// being taken as the mode digit, because the old dispatcher handled them
// first. Returns true when the key is one of them and its action ran.
bool keymapModeSelectShort(const KeypadTraits& traits, uint8_t bank, char key);
// LEGACY(F2): during mode select, holds run their long actions, except Bank 3
// keys the old dispatcher guarded ('1'-'5', and '6' on the FT-817).
bool keymapModeSelectHold(const KeypadTraits& traits, uint8_t bank, char key);
