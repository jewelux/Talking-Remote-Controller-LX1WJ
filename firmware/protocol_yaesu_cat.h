#pragma once

#include <Arduino.h>

#include "ft8x7_codec.h"
#include "radio_globals.h"

void yaesuCatFlushInput();
// Makes the next transaction wait out stray bytes (after a timeout or a misaligned frame).
void yaesuCatMarkLineDirty();
// Call after (re)opening the UART: the first command then keeps the same minimum gap as between
// commands, so it never follows a line change immediately.
void yaesuCatNoteLineOpened();
void yaesuCatSend5(const uint8_t data[5]);
bool yaesuCatRead1(uint8_t& out, uint32_t timeoutMs);
bool yaesuCatRead5(uint8_t out[5], uint32_t timeoutMs);
bool yaesuCatTransact1(const uint8_t cmd[5], uint8_t& rsp, uint32_t timeoutMs);
bool yaesuCatTransact5(const uint8_t cmd[5], uint8_t rsp[5], uint32_t timeoutMs);
void yaesuCatSniff(uint32_t windowMs);
void yaesuCatPrintFrame(const uint8_t data[5]);
bool parseHexByteString(const String& s, uint8_t& valueOut);
String byteToUpperHex(uint8_t v);
