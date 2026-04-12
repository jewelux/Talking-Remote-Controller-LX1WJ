#include "radio_globals.h"

static constexpr CivProfile PROFILE_7300_CIV = {0x94, "IC-7300", CIV_BAUD, 1, CIV_RX_PIN, CIV_TX_PIN, true, false};

StoredProfile g_slotProfiles[MAX_PROFILE_SLOTS];
uint8_t g_profileId = (ICOM_MODEL == ICOM_IC_706MKIIG) ? PROFILE_ID_706_CIV : PROFILE_ID_7300;
uint8_t g_lastSavedProfile = 0xFF;
uint8_t g_civRadioAddr = PROFILE_7300_CIV.civAddr;
HardwareSerial civUart1(1);
HardwareSerial civUart2(2);
HardwareSerial* g_civSerial = &civUart1;
bool g_quiet = false;
bool g_speechEnabled = true;
bool g_tuningSpeakEnabled = true;
uint32_t g_suppressFreqSpeakUntilMs = 0;
uint32_t g_suspendPollingUntilMs = 0;
LiveState live;
bool g_ft8x7SplitKnown = false;
bool g_ft8x7SplitOn = false;
bool g_yaesuCatTrace = false;
