#pragma once

#include "radio_globals.h"

bool asciiQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool asciiSetFrequency(const StoredProfile& sp, uint64_t hz);
bool asciiQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool asciiSetMode(const StoredProfile& sp, uint8_t mode);
bool asciiQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQueryStatusLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryIdLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryOmLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryPreamp(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetPreamp(const StoredProfile& sp, bool on);
bool asciiQueryAgcLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiSetAgcCommand(const StoredProfile& sp, const char* cmd);
bool asciiQueryPowerState(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetPowerState(const StoredProfile& sp, bool on);
bool asciiQueryTuner(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetTuner(const StoredProfile& sp, bool on);
bool asciiStartTune(const StoredProfile& sp);
bool asciiQueryNr(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNr(const StoredProfile& sp, bool on);
bool asciiQueryNb(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNb(const StoredProfile& sp, bool on);
bool asciiQueryNotch(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNotch(const StoredProfile& sp, bool on);
bool asciiQueryLock(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetLock(const StoredProfile& sp, bool on);
