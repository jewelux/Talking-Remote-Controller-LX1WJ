#pragma once

#include <Arduino.h>

#include "radio_globals.h"

size_t civReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs);
CivDecoded civDecode(const uint8_t* buf, size_t n);
void civFlushInput();
void civSend(uint8_t cmd, const uint8_t* data, size_t dataLen);
bool waitReply(uint8_t expectCmd, CivDecoded& out, uint32_t timeoutMs);
