#include "transport_serial.h"

#include "radio_catalog.h"

static uint32_t s_writeCount = 0;

static constexpr SerialPortPins kCivJackPins = {1, CIV_RX_PIN, CIV_TX_PIN, true, false};
static constexpr SerialPortPins kRs232Pins = {2, RS232_RX_PIN, RS232_TX_PIN, false, false};
static constexpr SerialPortPins kCatTtlPins = {2, CAT_RX_PIN, CAT_TX_PIN, CAT_TX_INVERT, CAT_RX_INVERT};

const SerialPortPins& serialPortPins(RadioPort port) {
  switch (port) {
    case RadioPort::CivJack: return kCivJackPins;
    case RadioPort::Rs232: return kRs232Pins;
    case RadioPort::CatTtl: return kCatTtlPins;
  }
  return kRs232Pins;
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
  while (g_civSerial->available()) (void)g_civSerial->read();
}

size_t serialTransportAvailable() {
  return g_civSerial->available();
}

int serialTransportRead() {
  return g_civSerial->read();
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
  return g_civSerial->write(data, len);
}

size_t serialTransportWriteByte(uint8_t value) {
  if (!serialTransportWriteAllowed()) return 0;
  ++s_writeCount;
  return g_civSerial->write(value);
}

size_t serialTransportPrint(const char* text) {
  if (!serialTransportWriteAllowed()) return 0;
  ++s_writeCount;
  return g_civSerial->print(text);
}

uint32_t serialTransportWriteCount() {
  return s_writeCount;
}

void serialTransportFlushOutput() {
  g_civSerial->flush();
}
