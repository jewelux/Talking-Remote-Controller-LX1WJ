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
bool asciiQueryActiveVfoA(const RadioProfile& sp, bool& vfoAOut, uint32_t timeoutMs);
bool asciiSelectVfoA(const RadioProfile& sp);
bool asciiSelectVfoB(const RadioProfile& sp);
bool asciiSwapVfo(const RadioProfile& sp);
bool asciiQueryVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs);
bool asciiSetVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t hz);
