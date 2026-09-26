#pragma once

#include "radio_globals.h"

// Drives a UART TX pin to its idle level (high, or low when inverted) without a pulse to the
// opposite level, which the radio would read as a start bit or a break.
void serialTransportDriveTxIdle(int pin, bool invert);
void serialTransportApplyProfile(const CivProfile& profile);
void serialTransportFlushInput();
size_t serialTransportAvailable();
int serialTransportRead();
size_t serialTransportWrite(const uint8_t* data, size_t len);
size_t serialTransportWriteByte(uint8_t value);
size_t serialTransportPrint(const char* text);
void serialTransportFlushOutput();
