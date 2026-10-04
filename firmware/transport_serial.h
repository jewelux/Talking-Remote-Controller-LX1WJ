#pragma once

#include "radio_globals.h"

// Drives a UART TX pin to its idle level (high, or low when inverted) without a pulse to the
// opposite level, which the radio would read as a start bit or a break.
void serialTransportDriveTxIdle(int pin, bool invert);

// The UART, pins and line inversion behind a RadioPort.
struct SerialPortPins {
  uint8_t uartNum;
  int8_t rxPin;
  int8_t txPin;
  bool txInvert;
  bool rxInvert;
};
const SerialPortPins& serialPortPins(RadioPort port);

// Reopens the radio UART for the active link.
void serialTransportApplyProfile(const ConnectionProfile& profile);
void serialTransportFlushInput();
size_t serialTransportAvailable();
int serialTransportRead();
size_t serialTransportWrite(const uint8_t* data, size_t len);
size_t serialTransportWriteByte(uint8_t value);
size_t serialTransportPrint(const char* text);
// How many writes went to the radio since boot. Equal before and after an action: it sent nothing.
uint32_t serialTransportWriteCount();
void serialTransportFlushOutput();
