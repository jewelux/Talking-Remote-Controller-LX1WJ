#pragma once

#include "radio_globals.h"

bool queryFrequency(uint64_t& hzOut, uint32_t timeoutMs = 800);
bool setFrequency(uint64_t hz);
bool queryMode(uint8_t& modeOut, uint32_t timeoutMs = 800);
bool setMode(uint8_t mode, uint8_t filter = 1);
bool querySMeterRaw(int32_t& rawOut, uint32_t timeoutMs = 800);
bool queryPoMeterRaw(int32_t& rawOut, uint32_t timeoutMs = 800);
bool querySWRRaw(int32_t& rawOut, uint32_t timeoutMs = 800);
bool queryNr(bool& onOut, uint32_t timeoutMs = 800);
bool setNr(bool on);
bool queryNb(bool& onOut, uint32_t timeoutMs = 800);
bool setNb(bool on);
bool queryNotch(bool& onOut, uint32_t timeoutMs = 800);
bool setNotch(bool on);
bool queryNotchWidth(NotchWidth& widthOut, uint32_t timeoutMs = 800);
bool setNotchWidth(NotchWidth width);
