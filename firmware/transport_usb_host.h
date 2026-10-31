#pragma once

#include <Arduino.h>

// A radio on the ESP32-S3 native USB socket (RadioPort::UsbHost). HamTRC is the USB host and talks
// to the first CDC-ACM serial port the radio offers: the CAT port of a QRP Labs QMX, or the shell
// of a NanoVNA (which also answers VNA commands on Serial0, see nanovna_client.h). Starting the
// host takes the socket from the PC console until the next restart; the console on the UART0
// socket (Serial0) still answers HAMTRC?, USB?, SM?, and PO?, SWR? and PAVOLT for the QMX.

// What happened on the socket since the main loop last asked (radio_monitor.cpp reports it).
enum class UsbLinkEvent : uint8_t {
  None,
  SerialReady,   // a radio's serial port is open
  SerialGone,    // that radio was unplugged or switched off
  NoSerialPort,  // a device without a CDC-ACM serial port came, e.g. a sound card or a CP210x, or
                 // one the USB Host Library could not enumerate
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
// Up to len bytes that already came, without waiting; how many.
size_t usbHostReadBytes(uint8_t* buf, size_t len);
// Queues the bytes for the radio; 0 while no serial port is open.
size_t usbHostWrite(const uint8_t* data, size_t len);
// Waits until the queued bytes went out, at most timeoutMs.
void usbHostFlushOutput(uint32_t timeoutMs);
void usbHostFlushInput();

UsbLinkEvent usbHostTakeEvent();

// For the Bank 8 USB key: the same facts as USB?, as numbers.
struct UsbHostSummary {
  bool started;         // the host runs
  bool failed;          // it could not start, or was held after a crash restart
  bool serialOpen;      // a radio's serial port is open
  uint32_t devicesSeen; // USB devices that answered since the host started, hubs included
  uint32_t txBytes;     // sent to the radio
  uint32_t rxBytes;     // received from the radio
};
UsbHostSummary usbHostSummary();

// USB?: host state, the device, byte counts and the recent events.
void usbHostPrintStatus(Print& out);
