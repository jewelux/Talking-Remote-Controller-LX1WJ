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
extern uint32_t g_suspendPollingUntilMs;
extern LiveState live;
extern bool g_yaesuCatTrace;
// EXPERIMENTAL ON: every capability of the active profile counts as on. Not saved.
extern bool g_experimentalCaps;
// True when the most recent wait for a radio reply gave up without an answer.
extern bool g_radioReplyTimedOut;
extern volatile bool g_audioPlaying;
extern uint8_t g_volumeLevel;
extern bool g_keypadExecuting;

#define CIVSER (*g_civSerial)
