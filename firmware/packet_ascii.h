#pragma once

#include "radio_globals.h"

bool asciiPacketReadLine(String& out, uint32_t timeoutMs);
bool asciiPacketSendCommand(const char* cmd);
