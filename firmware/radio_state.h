#pragma once

#include "radio_globals.h"

void resetLiveRadioState();
void rememberLiveFrequency(uint64_t hz, uint32_t nowMs);
// Marks hz as the current-VFO frequency the user last heard or entered; tuning
// is announced again only once the radio moves FREQ_SPEAK_MIN_STEP_HZ away from it.
void rememberAnnouncedFrequency(uint64_t hz);
// The device itself just changed the frequency or VFO: the poll that sees the
// change must not announce it as tuning.
void muteTuningSpeechAfterOwnChange();
void rememberLiveMode(uint8_t mode, uint32_t nowMs);
void rememberLiveSmeter(int32_t raw, const SMeterReading& reading, uint32_t nowMs);
void rememberLivePower(int32_t raw, uint32_t nowMs);
void rememberLiveSwr(int32_t raw, uint32_t nowMs);
void rememberLiveNr(bool on, uint32_t nowMs);
void rememberLiveNb(bool on, uint32_t nowMs);
void rememberLiveNotch(bool on, uint32_t nowMs, bool widthValid = false, NotchWidth width = NOTCH_WIDTH_UNKNOWN);
void rememberActiveVfo(bool vfoA);
void rememberSplitState(bool on);
void rememberDialLockState(bool on);
