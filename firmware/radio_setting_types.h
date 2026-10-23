#pragma once

// Radio settings as HamTRC names them, whatever the protocol. No Arduino.h, so the host tests
// can build what uses them.

#include <stdint.h>

// AGC speeds. Not every radio has each: the Icoms have FAST, MID and SLOW, the FT-8x7 OFF, FAST,
// SLOW and AUTO.
enum class AgcMode : uint8_t { Off, Fast, Mid, Slow, Auto };
