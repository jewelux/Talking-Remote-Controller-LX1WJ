#pragma once

#include <stdint.h>

// What each bank key does, per radio: the keypad layout as code. Pure C++ with
// no Arduino.h; the actions it runs are declared in keypad_actions.h. The host
// tests (tests/keypad) check it against the characterization matrix.
//
// Bank keys are '0'-'9' and 'A'-'C'. '*', 'D' and '#' belong to KeypadInput.

// The keypad layout a radio gets. Exactly one applies; the keymap picks each
// key's action from it, and the actions do not check the radio again.
enum class KeypadLayout : uint8_t {
  Generic,  // no radio-specific keys: Kenwood ASCII, FTDX ASCII other than FTDX10
  Civ,      // PROTO_CIV
  Ftdx10,   // isFtdx10KeypadProfile(): most keys run a console command
  Ft8x7,    // PROTO_YAESU_FT8X7 with no known variant: Generic plus Bank 6
  Ft817,    // FT8X7 variant "ft817" (FT-817, FT-818)
  Ft857,    // FT8X7 variant "ft857_897" (FT-857, FT-897)
};

// What the keymap depends on. The glue fills this in once per key event from
// the active profile.
struct KeypadTraits {
  KeypadLayout layout = KeypadLayout::Generic;
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

