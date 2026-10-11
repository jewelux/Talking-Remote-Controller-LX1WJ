#pragma once

#include <Arduino.h>

// A radio on the ESP32-S3 native USB socket (RadioPort::UsbHost). HamTRC is the USB host and talks
// to the first CDC-ACM serial port the radio offers, the CAT port of a QRP Labs QMX. Starting the
// host takes the socket from the PC console until the next restart; the console on the UART0
// socket (Serial0) still answers HAMTRC? and USB?.

// What happened on the socket since the main loop last asked (radio_monitor.cpp reports it).
enum class UsbLinkEvent : uint8_t {
  None,
  SerialReady,   // a radio's serial port is open
  SerialGone,    // that radio was unplugged or switched off
  NoSerialPort,  // a device without a CDC-ACM serial port came, e.g. a sound card or a CP210x
  Held,          // the host was kept off after a crash restart (usbHostHoldAfterCrash)
};

// Starts the USB host the first time; later calls do nothing. False when it could not start or
// was held back.
bool usbHostStart();
bool usbHostStarted();
// Keeps the next usbHostStart() from starting the host. The restart came from a crash, maybe in
// the USB host, and the keypad must still be able to pick another radio.
void usbHostHoldAfterCrash();
bool usbHostSerialReady();

size_t usbHostAvailable();
int usbHostRead();
// Queues the bytes for the radio; 0 while no serial port is open.
size_t usbHostWrite(const uint8_t* data, size_t len);
// Waits until the queued bytes went out, at most timeoutMs.
void usbHostFlushOutput(uint32_t timeoutMs);
void usbHostFlushInput();

UsbLinkEvent usbHostTakeEvent();

// For the Bank 9 USB key: the same facts as USB?, as numbers.
struct UsbHostSummary {
  bool started;         // the host runs
  bool failed;          // it could not start, or was held after a crash restart
  bool serialOpen;      // a radio's serial port is open
  uint32_t devicesSeen; // USB devices plugged in since the host started, hubs included
  uint32_t txBytes;     // sent to the radio
  uint32_t rxBytes;     // received from the radio
};
UsbHostSummary usbHostSummary();

// USB?: host state, the device, byte counts and the recent events.
void usbHostPrintStatus(Print& out);
