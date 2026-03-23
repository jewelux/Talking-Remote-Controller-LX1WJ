#pragma once

#include "radio_globals.h"

void serialTransportApplyProfile(const CivProfile& profile);
void serialTransportFlushInput();
size_t serialTransportAvailable();
int serialTransportRead();
size_t serialTransportWrite(const uint8_t* data, size_t len);
size_t serialTransportWriteByte(uint8_t value);
size_t serialTransportPrint(const char* text);
void serialTransportFlushOutput();
