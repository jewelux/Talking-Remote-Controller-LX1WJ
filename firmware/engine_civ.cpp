#include "engine_civ.h"
#include "radio_catalog.h"
#include "protocol_civ.h"
#include "radio_monitor.h"
#include "radio_mode.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "transport_serial.h"

void handleIncomingFrame(const CivDecoded& d) {
  if (currentProtocolType() != PROTO_CIV || d.from != g_civRadioAddr) return;
  if (d.cmd == 0x00 && d.payloadLen >= 5) {
    uint64_t hz = decodeBcdFrequencyHz(d.payload, 5);
    handleObservedFrequency(hz, true);
    return;
  }
  if ((d.cmd == 0x01 || d.cmd == 0x04) && d.payloadLen >= 1) {
    uint8_t m = d.payload[0];
    handleObservedMode(m, true);
  }
}

void pumpIncoming(uint32_t maxMs) {
  if (currentProtocolType() != PROTO_CIV) return;
  uint32_t start = millis();
  uint8_t buf[96];
  while (millis() - start < maxMs) {
    if (!serialTransportAvailable()) break;
    size_t n = civReadFrame(buf, sizeof(buf), 20);
    if (!n) break;
    CivDecoded d = civDecode(buf, n);
    if (d.ok) handleIncomingFrame(d);
  }
}
