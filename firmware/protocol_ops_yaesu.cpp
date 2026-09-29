#include "protocol_ops_yaesu.h"

#include "protocol_ascii.h"
#include "protocol_yaesu_cat.h"
#include "radio_catalog.h"
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
  if (!yaesuCatFreqFieldValid(rsp)) {
    yaesuCatMarkLineDirty();
    return false;
  }
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
  // The mode byte shares its frame with the frequency; a bad frequency field means the frame is misaligned.
  if (!yaesuCatFreqFieldValid(rsp)) {
    yaesuCatMarkLineDirty();
    return false;
  }
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
  uint8_t rxStatus = 0;
  if (!yaesuCatQueryRxStatusRaw(rxStatus, timeoutMs)) return false;
  rawOut = rxStatus;
  return true;
}

SMeterReading yaesuCatDecodeSMeter(uint8_t rxStatus) {
  // RX status (0xE7): bits 7..4 are squelch/tone/discriminator flags, bits 3..0 the
  // meter: 0x0..0x9 = S0..S9, 0xA..0xF = S9+10..S9+60 dB.
  const uint8_t level = rxStatus & 0x0F;
  SMeterReading reading;
  if (level <= 9) {
    reading.sUnits = level;
  } else {
    reading.sUnits = 9;
    reading.dbOverS9 = (uint8_t)((level - 9) * 10);
  }
  return reading;
}

// Power and the high-SWR flag come from the TX status. ALC and SWR need the undocumented 0xBD,
// answered by the FT-817/818 and FT-857/897 while transmitting: two bytes of 0..15 meters,
// byte 0 = PWR (high nibble) | ALC (low), byte 1 = SWR (high) | MOD (low), as Hamlib's ft817.c
// reads them. In receive the FT-897 does not answer 0xBD and misses the next polls, so it is sent
// only after the TX status shows PTT on.
bool yaesuCatQueryTxMeters(YaesuTxMeters& out, bool withBdMeters, uint32_t timeoutMs) {
  out = YaesuTxMeters();
  uint8_t txStatus = 0;
  if (!yaesuCatQueryTxStatusRaw(txStatus, timeoutMs)) return false;
  if (!yaesuCatTxStatusTransmitting(txStatus)) return true;
  out.transmitting = true;
  out.highSwr = (txStatus & 0x40) != 0;
  out.po = txStatus & 0x0F;
  if (!withBdMeters) return true;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xBD};
  uint8_t meters[2] = {0};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  if (!yaesuCatRead1(meters[0], timeoutMs) || !yaesuCatRead1(meters[1], timeoutMs)) return false;
  out.alc = meters[0] & 0x0F;
  out.swr = (meters[1] >> 4) & 0x0F;
  return true;
}

// SWR per meter bar, measured on an FT-817 by WA4YA/DL4YA (Hamlib's FT817_SWR_CAL).
float yaesuSwrFromMeter(uint8_t bars) {
  static constexpr float kSwr[] = {1.0f, 1.4f, 1.8f, 2.13f, 2.25f, 3.7f, 6.0f, 7.0f, 8.0f, 9.0f};
  static constexpr size_t kCount = sizeof(kSwr) / sizeof(kSwr[0]);
  return bars < kCount ? kSwr[bars] : 10.0f;
}

bool yaesuCatQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower) return false;
  YaesuTxMeters meters;
  if (!yaesuCatQueryTxMeters(meters, false, timeoutMs)) return false;
  rawOut = meters.po;
  return true;
}

bool yaesuCatQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr) return false;
  YaesuTxMeters meters;
  if (!yaesuCatQueryTxMeters(meters, true, timeoutMs)) return false;
  rawOut = meters.swr;
  return true;
}

bool yaesuCatQueryAlcRaw(int32_t& rawOut, uint32_t timeoutMs) {
  YaesuTxMeters meters;
  if (!yaesuCatQueryTxMeters(meters, true, timeoutMs)) return false;
  rawOut = meters.alc;
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

bool yaesuCatTxStatusTransmitting(uint8_t txStatus) {
  return (txStatus & 0x80) == 0;
}

// Undocumented 0xBB reads the EEPROM word at an even address; the byte at an odd address is
// the second of the pair. Read-only: the write opcode (0xBC) is never sent, since a bad write
// can wipe the radio's memories and calibration.
bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs) {
  const uint8_t cmd[5] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFE), 0x00, 0x00, 0xBB};
  uint8_t word[2] = {0};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  if (!yaesuCatRead1(word[0], timeoutMs) || !yaesuCatRead1(word[1], timeoutMs)) return false;
  out = word[addr & 0x01];
  return true;
}

// The TX status split bit is valid only while transmitting; the FT-857/897 answer 0xFF in
// receive. Otherwise split is bit 7 of an EEPROM byte, at the addresses Hamlib reads (0x8D
// confirmed on an FT-897: 0x03 with split off, 0x83 with split on).
bool yaesuCatQuerySplit(bool& onOut, uint32_t timeoutMs) {
  uint8_t txStatus = 0;
  if (!yaesuCatQueryTxStatusRaw(txStatus, timeoutMs)) return false;
  if (yaesuCatTxStatusTransmitting(txStatus)) {
    onOut = (txStatus & 0x20) != 0;
    return true;
  }
  const uint16_t addr = currentProfileVariantIs("ft817") ? 0x007A : 0x008D;
  uint8_t flags = 0;
  if (!yaesuCatReadEepromByte(addr, flags, timeoutMs)) return false;
  onOut = (flags & 0x80) != 0;
  return true;
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

static constexpr uint16_t kValidCtcssTenths[] = {
  670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
  948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
  1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
  1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
  2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541
};

static constexpr uint16_t kValidDcsCodes[] = {
  23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72, 73,
  74, 114, 115, 116, 122, 125, 131, 132, 134, 143, 145, 152, 155, 156, 162,
  165, 172, 174, 205, 212, 223, 225, 226, 243, 244, 245, 246, 251, 252, 255,
  261, 263, 265, 266, 271, 274, 306, 311, 315, 325, 331, 332, 343, 346, 351,
  356, 364, 365, 371, 411, 412, 413, 423, 431, 432, 445, 446, 452, 454, 455,
  462, 464, 465, 466, 503, 506, 516, 523, 526, 532, 546, 565, 606, 612, 624,
  627, 631, 632, 654, 662, 664, 703, 712, 723, 731, 732, 734, 743, 754
};

template <size_t N>
static bool containsU16(const uint16_t (&values)[N], uint16_t needle) {
  for (size_t i = 0; i < N; ++i) {
    if (values[i] == needle) return true;
  }
  return false;
}

bool yaesuCtcssTenthsValid(uint16_t toneTenths) {
  return containsU16(kValidCtcssTenths, toneTenths);
}

bool yaesuDcsCodeValid(uint16_t dcsCode) {
  return containsU16(kValidDcsCodes, dcsCode);
}

// BCD pair: CTCSS 88.5 Hz (885) -> 08 85; DCS 023 -> 00 23.
static void encodeToneBcd(uint16_t value, uint8_t& b0, uint8_t& b1) {
  const uint8_t d1 = (uint8_t)(value % 10); value /= 10;
  const uint8_t d10 = (uint8_t)(value % 10); value /= 10;
  const uint8_t d100 = (uint8_t)(value % 10); value /= 10;
  const uint8_t d1000 = (uint8_t)(value % 10);
  b0 = (uint8_t)((d1000 << 4) | d100);
  b1 = (uint8_t)((d10 << 4) | d1);
}

// FT-857/897 take separate TX and RX values; the FT-817 takes one.
static void fillToneData(uint16_t value, uint8_t data[4]) {
  encodeToneBcd(value, data[0], data[1]);
  data[2] = 0x00;
  data[3] = 0x00;
  if (currentProfileVariantIs("ft857_897")) {
    data[2] = data[0];
    data[3] = data[1];
  }
}

bool yaesuCatSetCtcssTenths(uint16_t toneTenths) {
  if (!yaesuCtcssTenthsValid(toneTenths)) return false;
  uint8_t data[4];
  fillToneData(toneTenths, data);
  if (!yaesuCatSetCtcssToneRaw(data)) return false;
  live.ctcssValid = true;
  live.ctcssTenths = toneTenths;
  return true;
}

bool yaesuCatSetDcsCode(uint16_t dcsCode) {
  if (!yaesuDcsCodeValid(dcsCode)) return false;
  uint8_t data[4];
  fillToneData(dcsCode, data);
  if (!yaesuCatSetDcsCodeRaw(data)) return false;
  live.dcsValid = true;
  live.dcsCode = dcsCode;
  return true;
}
