#include "transport_serial.h"

void serialTransportApplyProfile(const CivProfile& profile) {
  civUart1.end();
  civUart2.end();
  pinMode(CIV_TX_PIN, INPUT);
  pinMode(RS232_TX_PIN, INPUT);
  pinMode(CAT_TX_PIN, INPUT);
  pinMode(CAT_RX_PIN, INPUT);

  g_civSerial = (profile.uartNum == 2) ? &civUart2 : &civUart1;
  g_civSerial->end();
  g_civSerial->begin(profile.baud, SERIAL_8N1, profile.rxPin, profile.txPin);

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
  return g_civSerial->write(data, len);
}

size_t serialTransportWriteByte(uint8_t value) {
  return g_civSerial->write(value);
}

size_t serialTransportPrint(const char* text) {
  return g_civSerial->print(text);
}

void serialTransportFlushOutput() {
  g_civSerial->flush();
}
