#pragma once

#include "radio_globals.h"

void updateFreqSpeechDebounce(uint64_t newHz);
void speakPendingFreqIfIdle();
// Drops a tuning announcement that is waiting for the dial to go idle.
void cancelPendingFreqAnnouncement();
// The background frequency poll only feeds the tuning announcements, so it runs
// only while they are on. While it is off live.freqHz is not kept current: code
// that needs the radio's frequency reads it from the radio.
bool frequencyPollEnabled();
void pollFrequencyIfDue();
void pollSMeterIfDue();
void handleSMeterRaw(int32_t raw);
void handleObservedFrequency(uint64_t hz, bool verbose);
void handleObservedMode(uint8_t mode, bool verbose);
