#include "protocol_ops_yaesu.h"

#include "ft8x7_eeprom_map.h"
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

static void yaesuCatSendWriteOnly(const uint8_t cmd[5]) {
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  delay(YAESU_CAT_WRITE_SETTLE_MS);
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
  delay(YAESU_CAT_FREQ_MODE_SETTLE_MS);
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
  yaesuCatSetModeRawByte(modeByte);
  delay(YAESU_CAT_FREQ_MODE_SETTLE_MS);
  return true;
}

void yaesuCatSetModeRawByte(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x07};
  yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter) return false;
  uint8_t rxStatus = 0;
  if (!yaesuCatQueryRxStatusRaw(rxStatus, timeoutMs)) return false;
  rawOut = rxStatus;
  return true;
}

SMeterReading yaesuCatDecodeSMeter(uint8_t rxStatus) {
  SMeterReading reading;
  yaesuCatDecodeSMeterLevel(rxStatus, reading.sUnits, reading.dbOverS9);
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

// Undocumented 0xBB reads the EEPROM word at an even address; the byte at an odd address is
// the second of the pair.
bool yaesuCatReadEepromWord(uint16_t addr, uint8_t out[2], uint32_t timeoutMs) {
  const uint8_t cmd[5] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFE), 0x00, 0x00, 0xBB};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  return yaesuCatRead1(out[0], timeoutMs) && yaesuCatRead1(out[1], timeoutMs);
}

bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs) {
  uint8_t word[2] = {0};
  if (!yaesuCatReadEepromWord(addr, word, timeoutMs)) return false;
  out = word[addr & 0x01];
  return true;
}

// Undocumented 0xBC writes two EEPROM bytes, at addr and addr + 1; odd addresses work too
// (0xBB at 0069 returned the bytes at 0069 and 006A on an FT-897). It takes effect at once.
// CAUTION: a bad write can wipe the radio's memories and calibration. Any reply byte is left
// for the next flush to drain.
void yaesuCatWriteEeprom2(uint16_t addr, const uint8_t data[2]) {
  const uint8_t cmd[5] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), data[0], data[1], 0xBC};
  yaesuCatSendWriteOnly(cmd);
  yaesuCatMarkLineDirty();
}

// FT-8x7 settings that CAT can only read from the EEPROM (ft8x7_eeprom_map.h).
bool yaesuFt8x7QueryFlag(Ft8x7Flag flag, bool& onOut, uint32_t timeoutMs) {
  Ft8x7FlagField field;
  if (!ft8x7FlagField(currentFt8x7Model(), flag, field)) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(field.addr, b, timeoutMs)) return false;
  onOut = ft8x7FlagValue(field, b);
  return true;
}

bool yaesuFt8x7HasFlag(Ft8x7Flag flag) {
  Ft8x7FlagField field;
  return ft8x7FlagField(currentFt8x7Model(), flag, field);
}

// The TX status split bit is valid only while transmitting; the FT-857/897 answer 0xFF in
// receive. Otherwise the EEPROM has it.
bool yaesuCatQuerySplit(bool& onOut, uint32_t timeoutMs) {
  uint8_t txStatus = 0;
  if (!yaesuCatQueryTxStatusRaw(txStatus, timeoutMs)) return false;
  if (yaesuCatTxStatusTransmitting(txStatus)) {
    onOut = (txStatus & 0x20) != 0;
    return true;
  }
  return yaesuFt8x7QueryFlag(Ft8x7Flag::Split, onOut, timeoutMs);
}

bool yaesuFt8x7QueryAgc(YaesuAgc& out, uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  uint8_t b = 0;
  if (ft8x7IsFt817Family(model)) {
    if (!yaesuCatReadEepromByte(FT817_AGC_ADDR, b, timeoutMs)) return false;
    out = ft817AgcFromByte(b);
    return true;
  }
  if (model != Ft8x7Model::Ft857) return false;
  bool on = false;
  if (!yaesuFt8x7QueryFlag(Ft8x7Flag::AgcOn, on, timeoutMs)) return false;
  if (!on) {
    out = YaesuAgc::Off;
    return true;
  }
  if (!yaesuCatReadEepromByte(FT857_AGC_SPEED_ADDR, b, timeoutMs)) return false;
  out = ft857AgcFromSpeed(b);
  return true;
}

bool yaesuFt8x7QueryMenuAndRow(uint8_t& menuOut, uint8_t& rowOut, uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  if (ft8x7IsFt817Family(model)) {
    uint8_t menu = 0;
    uint8_t row = 0;
    if (!yaesuCatReadEepromByte(FT817_MENU_ADDR, menu, timeoutMs)) return false;
    if (!yaesuCatReadEepromByte(FT817_ROW_ADDR, row, timeoutMs)) return false;
    menuOut = ft817MenuFromByte(menu);
    rowOut = ft817RowFromByte(row);
    return true;
  }
  if (model != Ft8x7Model::Ft857) return false;
  uint8_t word[2] = {0};
  if (!yaesuCatReadEepromWord(FT857_MENU_ROW_ADDR, word, timeoutMs)) return false;
  menuOut = (uint8_t)(word[0] + 1);
  rowOut = (uint8_t)(word[1] + 1);
  return true;
}

// The band slot of hz (null outside the amateur bands) and its block on the active VFO.
// False when the active VFO could not be read.
static bool ft857ActiveBandBlock(uint64_t hz, const Ft857BandSlot*& slotOut, uint16_t& blockOut,
                                 uint32_t timeoutMs) {
  slotOut = ft857BandSlotForHz(hz);
  if (!slotOut) return true;
  bool vfoB = false;
  if (!yaesuFt8x7QueryFlag(Ft8x7Flag::VfoB, vfoB, timeoutMs)) return false;
  blockOut = ft857BandBlock(*slotOut, vfoB);
  return true;
}

bool yaesuFt857QueryBandFlags(uint64_t hz, YaesuFt857BandFlags& out, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  out = YaesuFt857BandFlags();
  const Ft857BandSlot* slot = nullptr;
  uint16_t block = 0;
  if (!ft857ActiveBandBlock(hz, slot, block, timeoutMs)) return false;
  if (!slot) return true;
  uint8_t nar = 0;
  uint8_t ipoAtt = 0;
  if (!yaesuCatReadEepromByte(block + FT857_BAND_NAR_OFFSET, nar, timeoutMs)) return false;
  if (slot->hasIpoAtt && !yaesuCatReadEepromByte(block + FT857_BAND_IPO_ATT_OFFSET, ipoAtt, timeoutMs)) {
    return false;
  }
  out.bandKnown = true;
  out.hasIpoAtt = slot->hasIpoAtt;
  out.nar = (nar & FT857_BAND_NAR_MASK) != 0;
  out.ipo = slot->hasIpoAtt && (ipoAtt & FT857_BAND_IPO_MASK) != 0;
  out.att = slot->hasIpoAtt && (ipoAtt & FT857_BAND_ATT_MASK) != 0;
  return true;
}

bool yaesuFt857QueryRfPowerWatts(uint64_t hz, uint8_t& wattsOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(ft857RfPowerAddr(ft8x7BandGroupForHz(hz)), b, timeoutMs)) return false;
  wattsOut = ft857RfPowerWatts(b);
  return true;
}

bool yaesuFt817QueryRfPowerTenths(uint16_t& tenthsOut, uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  if (!ft8x7IsFt817Family(model)) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT817_RF_POWER_ADDR, b, timeoutMs)) return false;
  tenthsOut = ft817RfPowerTenths(b, model == Ft8x7Model::Ft818);
  return true;
}

bool yaesuFt817QueryRearAntenna(uint64_t hz, bool& rearOut, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT817_ANTENNA_ADDR, b, timeoutMs)) return false;
  rearOut = (b & ft817RearAntennaMask(ft8x7BandGroupForHz(hz))) != 0;
  return true;
}

bool yaesuFt857QueryLevel(YaesuFt857Level level, uint16_t& valueOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  uint16_t addr = 0;
  if (!ft857LevelAddr(level, addr)) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(addr, b, timeoutMs)) return false;
  valueOut = ft857LevelValue(level, b);
  return true;
}

bool yaesuFt857QueryRitOffsetHz(uint64_t hz, int32_t& offsetOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  const Ft857BandSlot* slot = nullptr;
  uint16_t block = 0;
  if (!ft857ActiveBandBlock(hz, slot, block, timeoutMs) || !slot) return false;
  uint8_t word[2] = {0};
  if (!yaesuCatReadEepromWord(block + FT857_BAND_RIT_OFFSET, word, timeoutMs)) return false;
  offsetOut = ft857RitOffsetHz(word);
  return true;
}

bool yaesuFt857QueryMicEq(YaesuFt857MicEq& out, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT857_MIC_EQ_ADDR, b, timeoutMs)) return false;
  out = ft857MicEqFromByte(b);
  return true;
}

// The CAT clarifier commands (05 on, 85 off) switch RIT, the short press of the CLAR key, which
// the manuals also call the clarifier. The radio answers 00 when it switched and F0 when RIT was
// already in that state, also after a front panel change.
static bool ft8x7SetRitReply(bool on, bool& changedOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family() && !currentIsFt817Family()) return false;
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x05 : 0x85)};
  uint8_t rsp = 0;
  if (!yaesuCatTransact1(cmd, rsp, timeoutMs)) return false;
  if (rsp != 0x00 && rsp != 0xF0) return false;
  changedOut = rsp == 0x00;
  return true;
}

// RIT on/off is not in the EEPROM (on the FT-857/897 0x6A bit 3 is whether the knob tunes RIT or
// IF shift), so switching RIT off asks: refused means it is off, and if it was on it is switched
// back at once. That hands the knob to RIT if IF shift had it.
bool yaesuFt8x7QueryRit(bool& onOut, uint32_t timeoutMs) {
  bool changed = false;
  if (!ft8x7SetRitReply(false, changed, timeoutMs)) return false;
  if (changed && !ft8x7SetRitReply(true, changed, timeoutMs)) return false;
  onOut = changed;
  return true;
}

bool yaesuFt8x7SetRit(bool on, uint32_t timeoutMs) {
  bool changed = false;
  return ft8x7SetRitReply(on, changed, timeoutMs);
}

bool yaesuFt8x7ToggleRit(bool& onOut, uint32_t timeoutMs) {
  bool changed = false;
  if (!ft8x7SetRitReply(true, changed, timeoutMs)) return false;
  if (changed) {
    onOut = true;
    return true;
  }
  if (!ft8x7SetRitReply(false, changed, timeoutMs) || !changed) return false;
  onOut = false;
  return true;
}

void yaesuCatToggleVfo() {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0x81};
  yaesuCatSendWriteOnly(cmd);
}

// The FT-8x7 has no command for VFO A or B, only the A/B toggle, so these toggle when the
// tracked VFO is the other one. An unknown VFO counts as the target.
void yaesuCatSelectVfoA() {
  if (live.activeVfoKnown && !live.activeVfoA) yaesuCatToggleVfo();
  rememberActiveVfo(true);
}

void yaesuCatSelectVfoB() {
  if (live.activeVfoKnown && live.activeVfoA) yaesuCatToggleVfo();
  rememberActiveVfo(false);
}

void yaesuCatSetPtt(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x08 : 0x88)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetClarifier(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x05 : 0x85)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetSplit(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x02 : 0x82)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetLockDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x00 : 0x80)};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetRepeaterShiftRaw(uint8_t shiftByte) {
  const uint8_t cmd[5] = {shiftByte, 0x00, 0x00, 0x00, 0x09};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetRepeaterOffsetHzRaw(uint64_t hz) {
  uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, 0xF9};
  yaesuCatEncodeRepeaterOffsetHz(hz, cmd);
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetPowerDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x0F : 0x8F)};
  yaesuCatSendWriteOnly(cmd);
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

void yaesuCatSetAgcMode(uint8_t modeByte) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, modeByte, 0xF3};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetClarifierOffsetRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0xF5};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetToneDcsModeRaw(uint8_t modeByte) {
  const uint8_t cmd[5] = {modeByte, 0x00, 0x00, 0x00, 0x0A};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetCtcssToneRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0B};
  yaesuCatSendWriteOnly(cmd);
}

void yaesuCatSetDcsCodeRaw(const uint8_t data[4]) {
  const uint8_t cmd[5] = {data[0], data[1], data[2], data[3], 0x0C};
  yaesuCatSendWriteOnly(cmd);
}

// FT-857/897 take separate TX and RX values; the FT-817 takes one.
static void fillToneData(uint16_t value, uint8_t data[4]) {
  yaesuEncodeToneData(value, currentIsFt857Family(), data);
}

bool yaesuCatSetCtcssTenths(uint16_t toneTenths) {
  if (!yaesuCtcssTenthsValid(toneTenths)) return false;
  uint8_t data[4];
  fillToneData(toneTenths, data);
  yaesuCatSetCtcssToneRaw(data);
  live.ctcssValid = true;
  live.ctcssTenths = toneTenths;
  return true;
}

bool yaesuCatSetDcsCode(uint16_t dcsCode) {
  if (!yaesuDcsCodeValid(dcsCode)) return false;
  uint8_t data[4];
  fillToneData(dcsCode, data);
  yaesuCatSetDcsCodeRaw(data);
  live.dcsValid = true;
  live.dcsCode = dcsCode;
  return true;
}
