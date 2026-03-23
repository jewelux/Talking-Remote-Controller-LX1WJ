#pragma once

#include "radio_globals.h"

void updateFreqSpeechDebounce(uint64_t newHz);
void speakPendingFreqIfIdle();
void pollFrequencyIfDue();
void pollSMeterIfDue();
void handleSMeterRaw(int32_t raw);
void handleObservedFrequency(uint64_t hz, bool verbose);
void handleObservedMode(uint8_t mode, bool verbose);
