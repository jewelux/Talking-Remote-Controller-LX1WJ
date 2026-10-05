#pragma once

#include <stdint.h>

#include "keypad_input.h"

// What each bank key does, per radio: the keypad layout as code. Pure C++ with
// no Arduino.h; the actions it binds are declared in keypad_actions.h. The host
// tests (tests/keypad) check it against the characterization matrix.
//
// Bank keys are '0'-'9' and 'A'-'C'. '*', 'D' and '#' belong to KeypadInput.

// The keypad layout a radio gets. Exactly one applies; the keymap picks each
// key's action from it, and the actions do not check the layout again (see
// keypad_actions.h).
enum class KeypadLayout : uint8_t {
  Generic,  // no radio-specific keys: Kenwood ASCII, FTDX ASCII other than FTDX10
  Civ,      // PROTO_CIV
  Ftdx10,   // isFtdx10KeypadProfile(): most keys run a console command
  Ft8x7,    // PROTO_YAESU_FT8X7 with no known model: Generic plus Bank 6
  Ft817,    // RadioModel Ft817 and Ft818
  Ft857,    // RadioModel Ft857 (FT-857, FT-897)
};

// What the keymap depends on. The glue fills this in from the active profile
// each time it looks up a key.
struct KeypadTraits {
  KeypadLayout layout = KeypadLayout::Generic;
  bool supportsMonitor = false;     // protocolSupportsMonitor()
  bool supportsTransceive = false;  // protocolSupportsTransceive()
  bool canGetRfPower = false;       // currentProfile().caps.getRfPower
};

// What key does on bank for this radio. An empty binding for keys the keymap
// does not handle, including '*', 'D' and '#'.
KeyBinding keymapLookup(const KeypadTraits& traits, uint8_t bank, char key);
