#pragma once

#include "radio_features.h"
#include "radio_globals.h"

// How the keypad and the console show and say a feature's state
// (radio_features.h): the same status line and the same words from both.

// "unsupported", "timeout", "no reply" or "failed"; "" for Ok.
const char* featureStatusText(FeatureStatus status);

// "NR ON", "NR OFF"; "NR 1", "NR 2" on the TS-480.
String nrStateText(const NrState& state);
void speakNrState(const NrState& state);

String nbStateText(bool on);
void speakNbState(bool on);

// "NOTCH OFF", "NOTCH ON", or the width: "NOTCH NAR", "NOTCH MID", "NOTCH WIDE".
String notchStateText(const NotchState& state);
void speakNotchState(const NotchState& state);
