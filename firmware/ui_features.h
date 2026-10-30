#pragma once

#include "radio_features.h"
#include "radio_globals.h"

// How the keypad and the console show and say a feature's state
// (radio_features.h): the same status line and the same words from both.

// "unsupported", "experimental", "timeout", "no reply" or "failed"; "" for Ok.
const char* featureStatusText(FeatureStatus status);

// "NR ON", "NR OFF"; "NR 1", "NR 2" on the TS-480.
String nrStateText(const NrState& state);
void speakNrState(const NrState& state);

String nbStateText(bool on);
void speakNbState(bool on);

// "NOTCH OFF", "NOTCH ON", or the width: "NOTCH NAR", "NOTCH MID", "NOTCH WIDE".
String notchStateText(const NotchState& state);
void speakNotchState(const NotchState& state);

// "OFF", "FAST", "MID", "SLOW", "AUTO".
const char* agcModeName(AgcMode mode);
// "AGC FAST"; "agc fast".
String agcText(AgcMode mode);
void speakAgc(AgcMode mode);

// "NRLEVEL 8", "NBLEVEL 5": a level in the radio's own steps; "noise reduction 8".
String rxLevelText(RxLevel level, uint16_t value);
void speakRxLevel(RxLevel level, uint16_t value);

// "CTCSS 88.5 Hz"; "c t c s s 88.5".
String ctcssText(uint16_t tenths);
void speakCtcss(uint16_t tenths);
// "RPT PLUS", "RPT MINUS", "RPT SIMPLEX"; "repeater plus", "repeater off" (as Bank 6).
const char* rptShiftName(RptShift shift);
String rptShiftText(RptShift shift);
void speakRptShift(RptShift shift);

// "ANOTCH ON"; "auto notch on".
String autoNotchText(bool on);
void speakAutoNotch(bool on);

// "ATT OFF", "ATT ON", or with its dB: "ATT 12 DB"; "attenuator 12".
String attText(const AttState& state);
void speakAtt(const AttState& state);

// "PREAMP OFF", "PREAMP ON" where it has one level, else "PREAMP 1", "PREAMP 2"; "preamplifier 1".
String preampText(uint8_t level);
void speakPreamp(uint8_t level);

// "RFPOWER 50 W", "RFPOWER 0.5 W"; "power 50 watts".
String rfPowerText(uint16_t tenths);
void speakRfPower(uint16_t tenths);

// "OFF", "ON", "SEMI", "FULL".
const char* breakInModeName(BreakInMode mode);
// "BK SEMI"; "b k semi".
String breakInText(BreakInMode mode);
void speakBreakIn(BreakInMode mode);
// "KYR ON"; "k y r on".
String keyerText(bool on);
void speakKeyer(bool on);

// "CWSPEED 20 WPM", "CWPITCH 700 Hz", "CWDELAY 250 ms", "CWDELAY FULL"; spoken as the FT-8x7
// CW menus are.
const char* cwLevelName(CwLevel level);
String cwLevelText(CwLevel level, uint16_t value);
void speakCwLevel(CwLevel level, uint16_t value);

// "ALC 45%"; "a.l.c. 45 percent".
String alcText(uint8_t percent);
void speakAlc(uint8_t percent);

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

// FT-8x7 tone squelch. The type: "TONE OFF", "TONE CTCSS", "TONE DCS", or "TONE CTCSS RX DCS TX"
// when one type is awaited and the other sent (split tone); "tone ctcss".
String toneTypeText(const YaesuToneMode& mode);
void speakToneType(const YaesuToneMode& mode);
// Encode only, decode only or both: "TONEDIR TX", "TONEDIR RX", "TONEDIR BOTH", "TONEDIR SPLIT"
// for a mixed mode; "TONE OFF" when off. "tone tx".
String toneDirText(const YaesuToneMode& mode);
void speakToneDir(const YaesuToneMode& mode);
// Both, after a change: "TONE CTCSS TX", "TONE OFF", a mixed mode as the type; "tone ctcss tx".
String toneModeText(const YaesuToneMode& mode);
void speakToneMode(const YaesuToneMode& mode);
// FT-857/897 split tone (menu 79): "SPLITTONE ON"; "split tone on".
String splitToneText(bool on);
void speakSplitTone(bool on);

// FT-857/897 EEPROM settings: "AGC FAST", "IPO ON", "RFPOWER 100 W", "MENU 76"; FT-817/818
// "RFPOWER L2 (1 W)".
String ft8x7SettingText(const Ft8x7SettingState& state);
// The radio's soft key label spelled, then the state: "a g c fast", "i p o on", "power 100 watts";
// FT-817/818 "power l two", "power high".
void speakFt8x7Setting(const Ft8x7SettingState& state);
// "AUTO", "FAST", "SLOW", "OFF": the console word, which speakToken also says.
const char* ft8x7AgcName(YaesuAgc agc);
// "OFF", "LPF", "HPF", "BOTH": menu 48 as the radio shows it, and the console word.
const char* ft8x7MicEqName(YaesuFt857MicEq eq);
// "USB", "LSB", "AUTO": menu 23 as the radio shows it, and the console word.
const char* ft857CwBfoName(Ft857CwBfo bfo);
// 30 -> "3.0": the CW dash/dot ratio from tenths.
String cwWeightText(uint16_t tenths);
// "HIGH", "L3", "L2", "L1": the console word.
const char* ft817PowerLevelName(Ft817PowerLevel level);
// The level alone: "l two", "high".
void speakFt817PowerLevel(Ft817PowerLevel level);
