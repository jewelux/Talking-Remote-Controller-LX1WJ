#pragma once

#include "radio_globals.h"

bool asciiQueryFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool asciiSetFrequency(const RadioProfile& sp, uint64_t hz);
bool asciiQueryMode(const RadioProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool asciiSetMode(const RadioProfile& sp, uint8_t mode);
bool asciiQuerySMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQueryPoMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQuerySWRRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQueryStatusLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryIdLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryOmLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryPreamp(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetPreamp(const RadioProfile& sp, bool on);
bool asciiQueryAgcLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiSetAgcCommand(const RadioProfile& sp, const char* cmd);
bool asciiQueryPowerState(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetPowerState(const RadioProfile& sp, bool on);
bool asciiQueryTuner(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetTuner(const RadioProfile& sp, bool on);
bool asciiStartTune(const RadioProfile& sp);
bool asciiQuerySplit(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetSplit(const RadioProfile& sp, bool on);
bool asciiQueryNr(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNr(const RadioProfile& sp, bool on);
bool asciiQueryNb(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNb(const RadioProfile& sp, bool on);
bool asciiQueryNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNotch(const RadioProfile& sp, bool on);
bool asciiQueryLock(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetLock(const RadioProfile& sp, bool on);
bool asciiQueryYaesuRadioInfoFlag(const RadioProfile& sp, const char* code, bool& onOut, uint32_t timeoutMs);
// Yaesu FTDX speech processor (PR0) and its level (PL, 0..100).
bool asciiQueryYaesuProc(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiQueryYaesuProcLevel(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
// Yaesu FTDX mic gain (MG, 0..100).
bool asciiQueryYaesuMicGain(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
// Yaesu FTDX VOX (VX) and its gain (VG, 0..100).
bool asciiQueryYaesuVox(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiQueryYaesuVoxGain(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
// Yaesu FTDX VOX delay (VD), in ms.
bool asciiQueryYaesuVoxDelayMs(const RadioProfile& sp, uint16_t& msOut, uint32_t timeoutMs);
// The writes of the above, each read back: caps.setProc, setMicGain, setVox. Levels 0..100; the
// VOX delay goes to the VD step nearest ms, 30..3000. Not yet tested on a radio.
bool asciiSetYaesuProc(const RadioProfile& sp, bool on);
bool asciiSetYaesuProcLevel(const RadioProfile& sp, uint8_t level);
bool asciiSetYaesuMicGain(const RadioProfile& sp, uint8_t level);
bool asciiSetYaesuVox(const RadioProfile& sp, bool on);
bool asciiSetYaesuVoxGain(const RadioProfile& sp, uint8_t level);
bool asciiSetYaesuVoxDelayMs(const RadioProfile& sp, uint16_t ms);
// The VD delay steps of its steps from the one nearest ms, kept within 30..3000.
uint16_t asciiYaesuVoxDelayStep(uint16_t ms, int steps);
bool asciiQueryActiveVfoA(const RadioProfile& sp, bool& vfoAOut, uint32_t timeoutMs);
bool asciiSelectVfoA(const RadioProfile& sp);
bool asciiSelectVfoB(const RadioProfile& sp);
bool asciiSwapVfo(const RadioProfile& sp);
bool asciiQueryVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs);
bool asciiSetVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t hz);
