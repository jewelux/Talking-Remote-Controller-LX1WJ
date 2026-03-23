#pragma once

#include "radio_globals.h"

bool yaesuCatQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool yaesuCatSetFrequency(const StoredProfile& sp, uint64_t hz);
bool yaesuCatQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool yaesuCatSetMode(const StoredProfile& sp, uint8_t mode);
bool yaesuCatQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryVolumeRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQuerySquelchRaw(int32_t& rawOut, uint32_t timeoutMs);
bool yaesuCatQueryStatusRaw(uint8_t& rawOut, uint32_t timeoutMs);
bool yaesuCatToggleVfo();
bool yaesuCatSelectVfoA();
bool yaesuCatSelectVfoB();
bool yaesuCatSetPtt(bool on);
bool yaesuCatSetClarifier(bool on);
bool yaesuCatSetSplit(bool on);
bool yaesuCatSetLockDocumentedRaw(bool on);
bool yaesuCatMemoryWrite();
bool yaesuCatMemoryReadRaw(uint8_t rsp[5], uint32_t timeoutMs);
bool yaesuCatSetAgcMode(uint8_t modeByte);
bool yaesuCatSetClarifierOffsetRaw(const uint8_t data[4]);
