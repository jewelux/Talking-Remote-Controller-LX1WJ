#include "protocol_civ.h"

#include "packet_civ.h"
#include "radio_globals.h"
#include "transport_serial.h"

size_t civReadFrame(uint8_t* buf, size_t bufMax, uint32_t timeoutMs) {
  return civPacketReadFrame(buf, bufMax, timeoutMs);
}

std::optional<CivFrame> civDecode(const uint8_t* buf, size_t n) {
  return civDecodeFrame(buf, n);
}

void civFlushInput() {
  serialTransportFlushInput();
}

void civSend(uint8_t cmd, const uint8_t* data, size_t dataLen) {
  civPacketSendFrame(g_civRadioAddr, CIV_CTRL_ADDR, cmd, data, dataLen);
}

std::optional<CivFrame> waitReply(uint8_t expectCmd, uint32_t timeoutMs) {
  g_radioReplyTimedOut = false;
  // The radio answering "NG" means it rejected the command; that is not a timeout.
  bool rejected = false;
  uint32_t start = millis();
  uint8_t buf[kCivMaxFrame];
  while (millis() - start < timeoutMs) {
    size_t n = civReadFrame(buf, sizeof(buf), 60);
    if (!n) continue;
    std::optional<CivFrame> d = civDecode(buf, n);
    if (!d) continue;
    if (d->from != g_civRadioAddr) continue;
    if (!(d->to == CIV_CTRL_ADDR || d->to == 0x00)) continue;
    if (d->cmd == CIV_REPLY_NG) rejected = true;
    if (d->cmd != expectCmd) continue;
    return d;
  }
  g_radioReplyTimedOut = !rejected;
  return std::nullopt;
}
