#include "transport_serial.h"

#include "radio_catalog.h"
#include "transport_usb_host.h"

static uint32_t s_writeCount = 0;

static constexpr SerialPortPins kCivJackPins = {1, CIV_RX_PIN, CIV_TX_PIN, true, false};
static constexpr SerialPortPins kRs232Pins = {2, RS232_RX_PIN, RS232_TX_PIN, false, false};
static constexpr SerialPortPins kCatTtlPins = {2, CAT_RX_PIN, CAT_TX_PIN, CAT_TX_INVERT, CAT_RX_INVERT};
static constexpr SerialPortPins kNoPins = {0, -1, -1, false, false};

const SerialPortPins& serialPortPins(RadioPort port) {
  switch (port) {
    case RadioPort::CivJack: return kCivJackPins;
    case RadioPort::Rs232: return kRs232Pins;
    case RadioPort::CatTtl: return kCatTtlPins;
    case RadioPort::UsbHost: return kNoPins;
  }
  return kRs232Pins;
}

static bool usbLink() {
  return currentConnectionProfile().port == RadioPort::UsbHost;
}

// ASCII CAT (Kenwood, Elecraft, Yaesu FTDX) ends its commands with ';' and never needs CR or LF.
static bool asciiCatProtocol() {
  const ProtocolType pt = currentProtocolType();
  return pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII;
}

// A CR switches a QMX serial port into its terminal mode, which ends CAT until the radio
// restarts. Drops CR and LF before they reach a USB radio that speaks ASCII CAT.
static size_t usbWriteWithoutLineEnds(const uint8_t* data, size_t len) {
  uint8_t buf[64];
  size_t accepted = 0;
  while (len) {
    size_t n = 0;
    size_t used = 0;
    while (used < len && n < sizeof(buf)) {
      const uint8_t b = data[used++];
      if (b != '\r' && b != '\n') buf[n++] = b;
    }
    if (n && usbHostWrite(buf, n) < n) return accepted;
    accepted += used;
    data += used;
    len -= used;
  }
  return accepted;
}

void serialTransportDriveTxIdle(int pin, bool invert) {
  // digitalWrite() is ignored until pinMode() has claimed the pin as GPIO, and pinMode(OUTPUT)
  // drives the output register's current level, which is 0 after reset. Claim the pin as an input
  // pulled to the idle level first, so the level can be set before the output is enabled.
  pinMode(pin, invert ? INPUT_PULLDOWN : INPUT_PULLUP);
  digitalWrite(pin, invert ? LOW : HIGH);
  pinMode(pin, OUTPUT);
}

void serialTransportApplyProfile(const ConnectionProfile& profile) {
  civUart1.end();
  civUart2.end();
  pinMode(CIV_TX_PIN, INPUT);
  serialTransportDriveTxIdle(RS232_TX_PIN, false);
  pinMode(CAT_TX_PIN, INPUT_PULLUP);
  pinMode(CAT_RX_PIN, INPUT);

  if (profile.port == RadioPort::UsbHost) {
    (void)usbHostStart();
    return;
  }

  const SerialPortPins& pins = serialPortPins(profile.port);
  g_civSerial = (pins.uartNum == 2) ? &civUart2 : &civUart1;
  g_civSerial->end();
  const bool ft8x7Cat = profile.framing == SerialFraming::Ft8x7Cat;
  if (ft8x7Cat) {
    serialTransportDriveTxIdle(pins.txPin, pins.txInvert);
    delay(5);
  }
  g_civSerial->begin(profile.baud, ft8x7Cat ? SERIAL_8N2 : SERIAL_8N1, pins.rxPin, pins.txPin);

  uart_port_t up = (pins.uartNum == 2) ? UART_NUM_2 : UART_NUM_1;
  uint32_t invMask = 0;
  if (pins.txInvert) invMask |= UART_SIGNAL_TXD_INV;
  if (pins.rxInvert) invMask |= UART_SIGNAL_RXD_INV;
  uart_set_line_inverse(up, UART_SIGNAL_INV_DISABLE);
  if (invMask) uart_set_line_inverse(up, invMask);
}

void serialTransportFlushInput() {
  if (usbLink()) {
    usbHostFlushInput();
    return;
  }
  while (g_civSerial->available()) (void)g_civSerial->read();
}

size_t serialTransportAvailable() {
  if (usbLink()) return usbHostAvailable();
  return g_civSerial->available();
}

int serialTransportRead() {
  if (usbLink()) return usbHostRead();
  return g_civSerial->read();
}

size_t serialTransportReadBytes(uint8_t* buf, size_t len) {
  if (usbLink()) return usbHostReadBytes(buf, len);
  const size_t n = min(len, (size_t)g_civSerial->available());
  return n ? g_civSerial->read(buf, n) : 0;
}

static bool s_ft847WriteGateOpen = false;

void serialTransportSetFt847WriteGate(bool open) { s_ft847WriteGateOpen = open; }

// See serialTransportSetFt847WriteGate. A dropped write is not counted: it never reached the radio.
static bool serialTransportWriteAllowed() {
  if (currentProtocolType() != PROTO_YAESU_FT847 || s_ft847WriteGateOpen) return true;
  if (Serial) Serial.println("[F847] blocked a write that is not an FT-847 CAT frame");
  return false;
}

size_t serialTransportWrite(const uint8_t* data, size_t len) {
  if (!serialTransportWriteAllowed()) return 0;
  ++s_writeCount;
  if (usbLink()) return asciiCatProtocol() ? usbWriteWithoutLineEnds(data, len) : usbHostWrite(data, len);
  return g_civSerial->write(data, len);
}

size_t serialTransportWriteByte(uint8_t value) {
  return serialTransportWrite(&value, 1);
}

size_t serialTransportPrint(const char* text) {
  return serialTransportWrite(reinterpret_cast<const uint8_t*>(text), strlen(text));
}

uint32_t serialTransportWriteCount() {
  return s_writeCount;
}

void serialTransportFlushOutput() {
  if (usbLink()) {
    usbHostFlushOutput(200);
    return;
  }
  g_civSerial->flush();
}
