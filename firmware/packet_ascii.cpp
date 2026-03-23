#include "packet_ascii.h"

#include "transport_serial.h"

bool asciiPacketReadLine(String& out, uint32_t timeoutMs) {
  out = "";
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (serialTransportAvailable()) {
      char c = (char)serialTransportRead();
      if (c == ';') {
        out += c;
        return true;
      }
      if (c == '\r' || c == '\n') {
        if (out.length()) return true;
        continue;
      }
      if ((uint8_t)c < 32 || (uint8_t)c > 126) continue;
      out += c;
    }
    delay(1);
  }
  return out.length() > 0;
}

bool asciiPacketSendCommand(const char* cmd) {
  if (!cmd || !cmd[0]) return false;
  serialTransportFlushInput();
  serialTransportPrint(cmd);
  serialTransportFlushOutput();
  return true;
}
