#pragma once

#include <stdint.h>

// What help mode says for a key gesture instead of running it. Pure C++ with no
// Arduino.h, so the keymap and the host tests (tests/keypad) use it.

// What the gesture would do, said with its name (ui_keypad_common.cpp).
enum class HelpVerb : uint8_t {
  None,      // nothing to say: the gesture has no action
  Say,       // says the value: just the name
  Toggle,    // switches it: the name, then "on off"
  Next,      // steps a short list: "next", then the name
  Entry,     // opens a typed entry: "choose", then the name
  Pick,      // opens a pick: "choose", then the name
  Run,       // does it (tune, swap the VFOs): just the name
  Centre,    // back to the centre or default: "reset", then the name
  Up,        // "up", then the name
  Down,      // "down", then the name
  UpFast,    // "up fast", then the name
  DownFast,  // "down fast", then the name
};

struct GestureHelp {
  // A trace label ("NRLEVEL"), spoken through kSpokenLabels (keypad_labels.cpp).
  const char* label = nullptr;
  HelpVerb verb = HelpVerb::None;
};
