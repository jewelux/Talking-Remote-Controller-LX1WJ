#include "transport_serial.h"

#include "radio_catalog.h"

static uint32_t s_writeCount = 0;

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

  g_civSerial = (profile.uartNum == 2) ? &civUart2 : &civUart1;
  g_civSerial->end();
  const uint32_t serialConfig = (currentProtocolType() == PROTO_YAESU_FT8X7) ? SERIAL_8N2 : SERIAL_8N1;
  if (currentProtocolType() == PROTO_YAESU_FT8X7) {
    serialTransportDriveTxIdle(profile.txPin, profile.txInvert);
    delay(5);
  }
  g_civSerial->begin(profile.baud, serialConfig, profile.rxPin, profile.txPin);

  uart_port_t up = (profile.uartNum == 2) ? UART_NUM_2 : UART_NUM_1;
  uint32_t invMask = 0;
  if (profile.txInvert) invMask |= UART_SIGNAL_TXD_INV;
  if (profile.rxInvert) invMask |= UART_SIGNAL_RXD_INV;
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

size_t serialTransportWrite(const uint8_t* data, size_t len) {
  ++s_writeCount;
  return g_civSerial->write(data, len);
}

size_t serialTransportWriteByte(uint8_t value) {
  ++s_writeCount;
  return g_civSerial->write(value);
}

size_t serialTransportPrint(const char* text) {
  ++s_writeCount;
  return g_civSerial->print(text);
}

uint32_t serialTransportWriteCount() {
  return s_writeCount;
}

void serialTransportFlushOutput() {
  g_civSerial->flush();
}
