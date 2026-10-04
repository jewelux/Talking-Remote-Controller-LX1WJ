#include "radio_globals.h"

uint8_t g_profileId = kDefaultProfileSlot;
uint8_t g_lastSavedProfile = 0xFF;
uint8_t g_civRadioAddr = 0;
HardwareSerial civUart1(1);
HardwareSerial civUart2(2);
HardwareSerial* g_civSerial = &civUart1;
bool g_quiet = false;
bool g_speechEnabled = true;
bool g_tuningSpeakEnabled = true;
bool g_verboseSpeech = true;
uint32_t g_suppressFreqSpeakUntilMs = 0;
uint32_t g_suspendPollingUntilMs = 0;
LiveState live;
bool g_yaesuCatTrace = false;
bool g_experimentalCaps = false;
bool g_radioReplyTimedOut = false;
