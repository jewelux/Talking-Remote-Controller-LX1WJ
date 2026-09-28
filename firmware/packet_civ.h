#pragma once

#include "radio_globals.h"

size_t civPacketReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs);
void civPacketSendFrame(uint8_t to, uint8_t from, uint8_t cmd, const uint8_t* data, size_t dataLen);
