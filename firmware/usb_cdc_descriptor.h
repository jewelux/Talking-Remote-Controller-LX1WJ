#pragma once

// Finds a CDC-ACM serial port in a USB configuration descriptor, or the port of a Silicon Labs
// CP210x USB-UART bridge (the IC-7300's USB port). No ESP-IDF or Arduino.h, so the host tests can
// build it (tests/usb).

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

// Silicon Labs CP210x USB-UART bridges, by VID:PID: 10C4:EA60 (CP2102, CP2102N, CP2103, CP2104,
// CP2109, also the bridge in the IC-7300 and IC-9700), 10C4:EA70 (CP2105, two ports) and
// 10C4:EA71 (CP2108, four ports). They are not CDC-ACM: one vendor-class interface per UART, set up
// with vendor requests (transport_usb_host.cpp).
bool isCp210x(uint16_t vid, uint16_t pid);

// The UART numbered index (0 = the first) of a CP210x: the vendor-class (FF) interfaces with a bulk
// IN and a bulk OUT endpoint, in their first alternate setting. The vendor requests go to that
// interface, so commIntf is the same as dataIntf. False when the device has fewer.
bool findCp210xSerialPort(const uint8_t* config, size_t length, uint8_t index, CdcPortDesc& out);
