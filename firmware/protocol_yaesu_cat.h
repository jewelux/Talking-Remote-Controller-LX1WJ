#pragma once

#include <Arduino.h>

#include "radio_globals.h"

// FT-817/857/897 coverage bounds; decoded frequencies outside indicate a misaligned frame.
static constexpr uint64_t YAESU_CAT_MIN_FREQ_HZ = 100000ULL;
static constexpr uint64_t YAESU_CAT_MAX_FREQ_HZ = 470000000ULL;

void yaesuCatFlushInput();
// Makes the next transaction wait out stray bytes (after a timeout or a misaligned frame).
void yaesuCatMarkLineDirty();
// Call after (re)opening the UART: the first command then keeps the same minimum gap as between
// commands, so it never follows a line change immediately.
void yaesuCatNoteLineOpened();
void yaesuCatSend5(const uint8_t data[5]);
bool yaesuCatRead1(uint8_t& out, uint32_t timeoutMs);
// For a reply whose length varies: a byte that does not come within windowMs is not a timeout.
// The line is still marked dirty in case it arrives late.
bool yaesuCatReadOptional1(uint8_t& out, uint32_t windowMs);
bool yaesuCatRead5(uint8_t out[5], uint32_t timeoutMs);
bool yaesuCatTransact1(const uint8_t cmd[5], uint8_t& rsp, uint32_t timeoutMs);
bool yaesuCatTransact5(const uint8_t cmd[5], uint8_t rsp[5], uint32_t timeoutMs);
void yaesuCatSniff(uint32_t windowMs);
void yaesuCatPrintFrame(const uint8_t data[5]);
uint64_t yaesuCatDecodeFreqHz(const uint8_t data[4]);
bool yaesuCatFreqFieldValid(const uint8_t data[4]);
void yaesuCatEncodeFreqHz(uint64_t hz, uint8_t out[4]);
void yaesuCatEncodeRepeaterOffsetHz(uint64_t hz, uint8_t out[4]);
bool parseHexByteString(const String& s, uint8_t& valueOut);
String byteToUpperHex(uint8_t v);
