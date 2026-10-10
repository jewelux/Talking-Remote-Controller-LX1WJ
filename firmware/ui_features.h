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

// "PROC ON", "PROC OFF"; "processor on".
String procStateText(bool on);
void speakProcState(bool on);
// "PROCLEVEL 50"; "processor level 50".
String procLevelText(uint8_t level);
void speakProcLevel(uint8_t level);
// "MICGAIN 50"; "mic gain 50".
String micGainText(uint8_t level);
void speakMicGain(uint8_t level);
// "VOX ON", "VOX OFF"; "vox on".
String voxStateText(bool on);
void speakVoxState(bool on);
// "VOXGAIN 50"; "vox gain 50".
String voxGainText(uint8_t level);
void speakVoxGain(uint8_t level);
// "VOXDELAY 500 ms"; "vox delay 500 m s".
String voxDelayText(uint16_t ms);
void speakVoxDelay(uint16_t ms);

// FT-857/897 EEPROM settings: "AGC FAST", "IPO ON", "RFPOWER 100 W", "MENU 76".
String ft8x7SettingText(const Ft8x7SettingState& state);
// The radio's soft key label spelled, then the state: "a g c fast", "i p o on", "power 100 watts".
void speakFt8x7Setting(const Ft8x7SettingState& state);
