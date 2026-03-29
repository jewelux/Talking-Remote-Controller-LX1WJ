#pragma once

#include "radio_types.h"

extern StoredProfile g_slotProfiles[MAX_PROFILE_SLOTS];
extern uint8_t g_profileId;
extern uint8_t g_lastSavedProfile;
extern uint8_t g_civRadioAddr;
extern HardwareSerial civUart1;
extern HardwareSerial civUart2;
extern HardwareSerial* g_civSerial;
extern Keypad keypad;
extern bool g_quiet;
extern bool g_speechEnabled;
extern bool g_tuningSpeakEnabled;
extern uint32_t g_suppressFreqSpeakUntilMs;
extern LiveState live;
extern bool g_ft8x7SplitKnown;
extern bool g_ft8x7SplitOn;
extern volatile bool g_audioPlaying;
extern uint8_t g_volumeLevel;
extern bool g_keypadExecuting;
extern bool g_suppressModePrefixOnce;

#define CIVSER (*g_civSerial)
