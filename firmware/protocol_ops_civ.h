#pragma once

#include "radio_globals.h"

bool civQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool civSetFrequency(const StoredProfile& sp, uint64_t hz);
bool civQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool civSetMode(const StoredProfile& sp, uint8_t mode, uint8_t filter);
bool civQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool civQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool civQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool civQueryNr(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetNr(const StoredProfile& sp, bool on);
bool civQueryNb(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetNb(const StoredProfile& sp, bool on);
bool civQueryNotch(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetNotch(const StoredProfile& sp, bool on);
bool civQueryNotchWidth(const StoredProfile& sp, NotchWidth& widthOut, uint32_t timeoutMs);
bool civSetNotchWidth(const StoredProfile& sp, NotchWidth width);
