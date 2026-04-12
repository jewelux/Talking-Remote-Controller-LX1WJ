#include "protocol_ops_yaesu.h"

#include "protocol_ascii.h"
#include "protocol_yaesu_cat.h"
#include "radio_protocol.h"
#include "radio_state.h"

static bool yaesuCatQueryMeterByte(uint8_t cmdByte, int32_t& rawOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, cmdByte};
  uint8_t rsp = 0;
  if (!yaesuCatTransact1(cmd, rsp, timeoutMs)) return false;
  rawOut = rsp;
  return true;
}

static bool yaesuCatSendWriteOnly(const uint8_t cmd[5]) {
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  delay(60);
  return true;
}

bool yaesuCatQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
  hzOut = yaesuCatDecodeFreqHz(rsp);
  return true;
}

bool yaesuCatQueryModeRawByte(uint8_t& modeByteOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
  modeByteOut = (uint8_t)(rsp[4] & 0x7F);
  return true;
}

bool yaesuCatSetFrequency(const StoredProfile& sp, uint64_t hz) {
  if (!sp.caps.setFreq) return false;
  uint8_t cmd[5] = {0, 0, 0, 0, 0x01};
  yaesuCatEncodeFreqHz(hz, cmd);
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  // FT-817/857 frequency writes behave like write-only commands in practice.
  // Avoid querying immediately afterward because the follow-up CAT traffic can
  // steal the bus before the radio settles the new value.
  delay(140);
  return true;
}

bool yaesuCatQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t rsp[5] = {0};
  if (!yaesuCatTransact5(cmd, rsp, timeoutMs)) return false;
  String code = byteToUpperHex((uint8_t)(rsp[4] & 0x7F));
  return profileInternalModeForCode(sp, code, modeOut);
}

bool yaesuCatSetMode(const StoredProfile& sp, uint8_t mode) {
  if (!sp.caps.setMode) return false;
  String code;
  uint8_t modeByte = 0;
  if (!profileModeCodeForInternal(sp, mode, code)) return false;
  if (!parseHexByteString(code, modeByte)) return false;
  if (!yaesuCatSetModeRawByte(modeByte)) return false;
  // FT-817/857 mode writes also need quiet time after the raw write command.
  // A direct readback right here is more likely to interfere than to help.
  delay(140);
  return true;
}

bool yaesuCatSetModeRawByte(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x07};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter) return false;
  return yaesuCatQueryMeterByte(0xE7, rawOut, timeoutMs);
}

bool yaesuCatQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower) return false;
  return yaesuCatQueryMeterByte(0xBD, rawOut, timeoutMs);
}

bool yaesuCatQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr) return false;
  return yaesuCatQueryMeterByte(0xBC, rawOut, timeoutMs);
}

bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs) {
  // BUGFIX V3.5.1: Opcode 0xBB = EEPROM Read auf FT-817/857. Das Radio antwortet mit
  // 2 Bytes (nicht 1). Der alte Code las nur 1 Byte, das zweite blieb im RX-Puffer
  // und korrumpierte die naechste Frequenzabfrage (Frame-Shift um 1 Byte).
  // Fix: beide Bytes lesen und das zweite verwerfen.
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xBB};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  uint8_t b0 = 0;
  uint8_t b1 = 0;
  if (!yaesuCatRead1(b0, timeoutMs)) return false;
  yaesuCatRead1(b1, 50);  // zweites Byte lesen und verwerfen
  rawOut = b0;
  return true;
}

bool yaesuCatQueryVolumeRaw(int32_t& rawOut, uint32_t timeoutMs) {
  return yaesuCatQueryMeterByte(0x13, rawOut, timeoutMs);
}

bool yaesuCatQuerySquelchRaw(int32_t& rawOut, uint32_t timeoutMs) {
  return yaesuCatQueryMeterByte(0x14, rawOut, timeoutMs);
}

bool yaesuCatQueryRxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xE7};
  return yaesuCatTransact1(cmd, rawOut, timeoutMs);
}

bool yaesuCatQueryTxStatusRaw(uint8_t& rawOut, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xF7};
  return yaesuCatTransact1(cmd, rawOut, timeoutMs);
}

bool yaesuCatQueryStatusRaw(uint8_t& rawOut, uint32_t timeoutMs) {
  return yaesuCatQueryTxStatusRaw(rawOut, timeoutMs);
}

bool yaesuCatToggleVfo() {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x81};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSelectVfoA() {
  // BUGFIX V3.5.1: War ein Stub (return false). Alle Aufrufer (selectVfoA,
  // queryVfoFrequency, setVfoFrequency, queryVfoMode, setVfoMode) schlugen dadurch
  // lautlos fehl. Frequenzschreiben auf VFO A/B meldete fälschlich "Error".
  //
  // FT-817 hat keinen direkten "Gehe zu VFO A"-Befehl. Einzige Moeglichkeit:
  // Toggle (0x81) wenn wir wissen dass gerade VFO B aktiv ist.
  // Ist der aktive VFO unbekannt oder bereits A -> nichts senden, als OK melden.
  if (!live.activeVfoKnown || live.activeVfoA) {
    rememberActiveVfo(true);
    return true;
  }
  // Aktuell auf VFO B -> einmal toggeln um auf A zu wechseln
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x81};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  delay(60);
  rememberActiveVfo(true);
  return true;
}

bool yaesuCatSelectVfoB() {
  // BUGFIX V3.5.1: War ein Stub (return false). Siehe yaesuCatSelectVfoA().
  if (!live.activeVfoKnown || !live.activeVfoA) {
    rememberActiveVfo(false);
    return true;
  }
  // Aktuell auf VFO A -> einmal toggeln um auf B zu wechseln
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x81};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  delay(60);
  rememberActiveVfo(false);
  return true;
}

bool yaesuCatSetPtt(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x08 : 0x88};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetClarifier(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x05 : 0x85};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetSplit(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x02 : 0x82};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetLockDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x00 : 0x80};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetRepeaterShiftRaw(uint8_t shiftByte) {
  const uint8_t cmd[5] = {shiftByte, 0x00, 0x00, 0x00, 0x09};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetRepeaterOffsetHzRaw(uint64_t hz) {
  uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xF9};
  yaesuCatEncodeRepeaterOffsetHz(hz, cmd);
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetPowerDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, on ? 0x0F : 0x8F};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatMemoryWrite() {
  // BUGFIX V3.5.1: Opcode 0x0A ist auf FT-817/857 "Set CTCSS/DCS Mode" (NICHT
  // "Memory Write"). [0x00,0x00,0x00,0x00,0x0A] = Mode 0x00 = CTCSS/DCS ausschalten.
  // Aufruf dieses Befehls hat unbeabsichtigt CTCSS auf dem Radio deaktiviert!
  // Einen "Memory Write"-CAT-Befehl gibt es beim FT8x7 nicht.
  // Funktion deaktiviert um Radio-Einstellungen zu schuetzen.
  return false;
}

bool yaesuCatMemoryReadRaw(uint8_t rsp[5], uint32_t timeoutMs) {
  // HINWEIS: Opcode 0x0B ist auf FT-817/857 "Set CTCSS Tone" (Write-Only).
  // Ein "Memory Read"-Befehl existiert beim FT8x7 nicht via CAT.
  // Diese Funktion ist nicht implementierbar und gibt immer false zurueck.
  (void)rsp;
  (void)timeoutMs;
  return false;
}

bool yaesuCatSetAgcMode(uint8_t modeByte) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, modeByte, 0xF3};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetClarifierOffsetRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0xF5};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetToneDcsModeRaw(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x0A};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetCtcssToneRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0B};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetDcsCodeRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0C};
  return yaesuCatSendWriteOnly(cmd);
}
