#pragma once

#include <Arduino.h>

#include "radio_globals.h"

bool readAsciiLine(String& out, uint32_t timeoutMs);
bool transactAsciiCommand(const char* cmd, String& out, const char* expectPrefix, uint32_t timeoutMs);
bool parseAsciiUnsignedResponse(const String& line, const char* prefix, uint64_t& valueOut);
bool parseAsciiSignedResponse(const String& line, const char* prefix, int32_t& valueOut);
bool profileModeCodeForInternal(const StoredProfile& sp, uint8_t mode, String& codeOut);
bool profileInternalModeForCode(const StoredProfile& sp, const String& code, uint8_t& modeOut);
