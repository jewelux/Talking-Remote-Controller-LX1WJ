#pragma once

#include "radio_globals.h"

bool refreshLiveFrequency();
bool refreshLiveMode();
bool refreshLiveSmeter();
bool refreshLivePower();
bool refreshLiveSwr();
bool refreshLiveNr();
bool refreshLiveNb();
bool refreshLiveNotch();
bool refreshLiveNotchWidth();
bool applyFrequencyAndTrack(uint64_t hz, bool suppressSpeechWindow);
bool applyModeAndTrack(uint8_t mode, uint8_t filter);
bool applyNrAndTrack(bool on);
bool applyNbAndTrack(bool on);
bool applyNotchAndTrack(bool on);
bool applyNotchWidthAndTrack(NotchWidth width);
