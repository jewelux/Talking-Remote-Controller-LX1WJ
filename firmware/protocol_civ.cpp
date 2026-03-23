#include "protocol_civ.h"

#include "packet_civ.h"
#include "radio_globals.h"
#include "transport_serial.h"

size_t civReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs) {
  return civPacketReadFrame(buf, bufMax, timeoutMs);
}

CivDecoded civDecode(const uint8_t* buf, size_t n) {
  return civPacketDecode(buf, n);
}

void civFlushInput() {
  serialTransportFlushInput();
}

void civSend(uint8_t cmd, const uint8_t* data, size_t dataLen) {
  civPacketSendFrame(g_civRadioAddr, CIV_CTRL_ADDR, cmd, data, dataLen);
}

bool waitReply(uint8_t expectCmd, CivDecoded& out, uint32_t timeoutMs) {
  uint32_t start = millis();
  uint8_t buf[96];
  while (millis() - start < timeoutMs) {
    size_t n = civReadFrame(buf, sizeof(buf), 60);
    if (!n) continue;
    CivDecoded d = civDecode(buf, n);
    if (!d.ok) continue;
    if (d.from != g_civRadioAddr) continue;
    if (!(d.to == CIV_CTRL_ADDR || d.to == 0x00)) continue;
    if (d.cmd != expectCmd) continue;
    out = d;
    return true;
  }
  return false;
}
