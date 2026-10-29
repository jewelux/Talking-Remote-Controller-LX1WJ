#pragma once

#include "radio_globals.h"
#include "engine_civ.h"

// Runs query, a radio read that is true on a good reply, up to 3 times: again after a garbled or
// unexpected reply, never after a timeout, so a radio that does not answer costs one wait.
template <typename Query>
bool retryBadReply(Query query) {
  for (uint8_t attempt = 0;; ++attempt) {
    if (query()) return true;
    if (g_radioReplyTimedOut || attempt == 2) return false;
    pumpIncoming(20);
    delay(25);
  }
}

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
