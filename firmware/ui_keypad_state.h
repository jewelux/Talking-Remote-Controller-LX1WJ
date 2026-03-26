#pragma once

#include <Arduino.h>

extern uint8_t g_bank;
extern bool g_bankSelectActive;
extern uint8_t g_bankStage;
extern bool g_modeSetActive;
extern bool g_modeStageActive;
extern uint8_t g_modeStageMode;
extern uint8_t g_modeStageTargetVfo;
extern char g_modeStageKey;
extern bool g_profileSelectActive;
extern uint8_t g_profileStage;
extern bool g_nineHoldConsumed;
extern String g_kpStagedCmd;
extern bool g_kpHasStagedCmd;
extern bool g_starHoldConsumed;
extern bool g_zeroHoldConsumed;
extern bool g_aHoldConsumed;
extern bool g_oneHoldConsumed;
extern bool g_twoHoldConsumed;
extern bool g_threeHoldConsumed;
extern bool g_fourHoldConsumed;
extern bool g_fiveHoldConsumed;
extern bool g_sixHoldConsumed;
extern bool g_sevenHoldConsumed;
extern bool g_eightHoldConsumed;
extern bool g_freqEntryActive;
extern String g_freqEntryDigits;
extern bool g_freqEntryIsMHz;
extern uint8_t g_freqEntryTargetVfo;
extern bool g_volStageActive;
extern uint8_t g_volStageLevel;
extern bool g_pendingClickActive;
extern uint8_t g_pendingClickBank;
extern char g_pendingClickKey;
extern uint32_t g_pendingClickAtMs;
