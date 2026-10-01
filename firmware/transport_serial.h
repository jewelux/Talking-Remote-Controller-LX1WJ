#pragma once

#include "radio_globals.h"

// Drives a UART TX pin to its idle level (high, or low when inverted) without a pulse to the
// opposite level, which the radio would read as a start bit or a break.
void serialTransportDriveTxIdle(int pin, bool invert);
void serialTransportApplyProfile(const ConnectionProfile& profile);
void serialTransportFlushInput();
size_t serialTransportAvailable();
int serialTransportRead();
size_t serialTransportWrite(const uint8_t* data, size_t len);
size_t serialTransportWriteByte(uint8_t value);
size_t serialTransportPrint(const char* text);
void serialTransportFlushOutput();

// While an FT-847 profile is active, only the FT-847 CAT layer may write to the radio. Its
// frames have no start marker, so stray bytes from an ASCII, CI-V or FT-8x7 command (e.g. a
// console command written for another radio) could be read as an FT-847 command: 0x08 is
// PTT ON, 0x80 CAT OFF. The FT-847 layer opens the gate around each frame; other writes are
// dropped and reported on the console.
void serialTransportSetFt847WriteGate(bool open);
