#pragma once

#include <Arduino.h>

#include "radio_globals.h"

size_t civReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs);
std::optional<CivFrame> civDecode(const uint8_t* buf, size_t n);
void civFlushInput();
void civSend(uint8_t cmd, const uint8_t* data, size_t dataLen);
// The radio's next reply with command expectCmd, or empty after timeoutMs.
std::optional<CivFrame> waitReply(uint8_t expectCmd, uint32_t timeoutMs);
