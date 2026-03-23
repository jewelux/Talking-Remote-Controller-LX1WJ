#pragma once

#include <Arduino.h>

// Standalone operation is more important than USB debug output.
// Set to 1 if you explicitly want live USB logging while connected.
static const bool ENABLE_DEBUG_LOG = false;

inline bool debugLogEnabled() {
  return ENABLE_DEBUG_LOG && (bool)Serial;
}

#define DBG_PRINT(x) do { if (debugLogEnabled()) Serial.print(x); } while (0)
#define DBG_PRINTLN(x) do { if (debugLogEnabled()) Serial.println(x); } while (0)
