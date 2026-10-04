#pragma once

#include "radio_profile_table.h"
#include "radio_types.h"

// The slot of the active profile.
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
// Off: answers say only the value, not its name ("50 watts" for "power 50 watts").
extern bool g_verboseSpeech;
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
