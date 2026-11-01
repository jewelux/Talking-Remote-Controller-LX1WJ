#pragma once

// Finds a CDC-ACM serial port in a USB configuration descriptor. No ESP-IDF or Arduino.h, so the
// host tests can build it (tests/usb).

#include <stddef.h>
#include <stdint.h>

constexpr uint8_t kCdcNoInterface = 0xFF;

struct CdcPortDesc {
  uint8_t dataIntf = 0;
  uint8_t commIntf = kCdcNoInterface;  // the ACM control interface before it, which takes the line setup
  uint8_t epIn = 0;
  uint8_t epOut = 0;
};

// The serial port numbered index (0 = the first) among the CDC data interfaces that have a bulk IN
// and a bulk OUT endpoint, in their first alternate setting, with the ACM control interface before
// it. A radio with two ports (the IC-705's CI-V and GPS, the X6100's CH342 port A and B) lists them
// in that order. False when the device has fewer ports.
bool findCdcSerialPort(const uint8_t* config, size_t length, uint8_t index, CdcPortDesc& out);
