#pragma once

#include "radio_globals.h"

void resetLiveRadioState();
void rememberLiveFrequency(uint64_t hz, uint32_t nowMs);
void rememberLiveMode(uint8_t mode, uint32_t nowMs);
void rememberLiveSmeter(int32_t raw, uint32_t nowMs);
void rememberLivePower(int32_t raw, uint32_t nowMs);
void rememberLiveSwr(int32_t raw, uint32_t nowMs);
void rememberLiveNr(bool on, uint32_t nowMs);
void rememberLiveNb(bool on, uint32_t nowMs);
void rememberLiveNotch(bool on, uint32_t nowMs, bool widthValid = false, NotchWidth width = NOTCH_WIDTH_UNKNOWN);
void rememberActiveVfo(bool vfoA);
