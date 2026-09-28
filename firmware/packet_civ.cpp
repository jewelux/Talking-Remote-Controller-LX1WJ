#include "packet_civ.h"

#include "transport_serial.h"

size_t civPacketReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t n = 0;
  uint8_t feCount = 0;
  // A frame longer than buf is dropped whole, never returned cut short.
  bool overflow = false;
  while (millis() - start < timeoutMs) {
    while (serialTransportAvailable()) {
      uint8_t b = (uint8_t)serialTransportRead();
      if (n == 0) {
        if (b == 0xFE) {
          buf[n++] = b;
          feCount = 1;
        }
        continue;
      }
      if (n == 1) {
        if (b == 0xFE && feCount == 1) buf[n++] = b;
        else {
          n = 0;
          feCount = 0;
        }
        continue;
      }
      if (n < bufMax) buf[n++] = b;
      else overflow = true;
      if (b == 0xFD) {
        if (!overflow) return n;
        n = 0;
        feCount = 0;
        overflow = false;
      }
    }
    delay(1);
  }
  return 0;
}

void civPacketSendFrame(uint8_t to, uint8_t from, uint8_t cmd, const uint8_t* data, size_t dataLen) {
  const uint8_t pre[4] = {0xFE, 0xFE, to, from};
  serialTransportWrite(pre, sizeof(pre));
  serialTransportWriteByte(cmd);
  if (data && dataLen) serialTransportWrite(data, dataLen);
  serialTransportWriteByte(0xFD);
  serialTransportFlushOutput();
}
