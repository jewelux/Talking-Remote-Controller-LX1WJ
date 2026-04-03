#pragma once

#include <Arduino.h>

#include "radio_globals.h"

void yaesuCatFlushInput();
void yaesuCatSend5(const uint8_t data[5]);
bool yaesuCatRead1(uint8_t& out, uint32_t timeoutMs);
bool yaesuCatRead5(uint8_t out[5], uint32_t timeoutMs);
bool yaesuCatTransact1(const uint8_t cmd[5], uint8_t& rsp, uint32_t timeoutMs);
bool yaesuCatTransact5(const uint8_t cmd[5], uint8_t rsp[5], uint32_t timeoutMs);
void yaesuCatPrintFrame(const uint8_t data[5]);
uint64_t yaesuCatDecodeFreqHz(const uint8_t data[4]);
void yaesuCatEncodeFreqHz(uint64_t hz, uint8_t out[4]);
void yaesuCatEncodeRepeaterOffsetHz(uint64_t hz, uint8_t out[4]);
bool parseHexByteString(const String& s, uint8_t& valueOut);
String byteToUpperHex(uint8_t v);
