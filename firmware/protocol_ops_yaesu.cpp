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
bool yaesuCatWriteEeprom2(uint16_t addr, const uint8_t data[2]) {
  const uint8_t cmd[5] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), data[0], data[1], 0xBC};
  yaesuCatSendWriteOnly(cmd);
  yaesuCatMarkLineDirty();
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
  const uint16_t addr = currentIsFt817Family() ? 0x007A : 0x008D;
  uint8_t flags = 0;
  if (!yaesuCatReadEepromByte(addr, flags, timeoutMs)) return false;
  onOut = (flags & 0x80) != 0;
  return true;
}

// FT-857/897 settings that CAT can only read from the EEPROM (ft8x7_eeprom_map.h).
static bool ft857ReadBit(uint16_t addr, uint8_t mask, bool& onOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(addr, b, timeoutMs)) return false;
  onOut = (b & mask) != 0;
  return true;
}

bool yaesuFt857QueryNb(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x006A, 0x20, onOut, timeoutMs); }
bool yaesuFt857QueryBreakIn(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x006B, 0x20, onOut, timeoutMs); }
bool yaesuFt857QueryKeyer(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x006B, 0x10, onOut, timeoutMs); }
bool yaesuFt857QueryDnr(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x00A8, 0x02, onOut, timeoutMs); }
bool yaesuFt857QueryDnf(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x00A8, 0x01, onOut, timeoutMs); }
bool yaesuFt857QueryDbf(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x00A8, 0x0C, onOut, timeoutMs); }

bool yaesuFt857QueryAgc(YaesuAgc& out, uint32_t timeoutMs) {
  bool on = false;
  if (!ft857ReadBit(FT857_AGC_ON_ADDR, FT857_AGC_ON_MASK, on, timeoutMs)) return false;
  if (!on) {
    out = YaesuAgc::Off;
    return true;
  }
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT857_AGC_SPEED_ADDR, b, timeoutMs)) return false;
  out = ft857AgcFromSpeed(b);
  return true;
}

// 0x68 bit 0 is the active VFO (1 = B). It follows both the front panel A/B key and the CAT
// toggle.
bool yaesuFt857QueryVfoB(bool& vfoBOut, uint32_t timeoutMs) {
  return ft857ReadBit(0x0068, 0x01, vfoBOut, timeoutMs);
}

// The band slot of hz (null outside the amateur bands) and its block on the active VFO.
// False when the active VFO could not be read.
static bool ft857ActiveBandBlock(uint64_t hz, const Ft857BandSlot*& slotOut, uint16_t& blockOut,
                                 uint32_t timeoutMs) {
  slotOut = ft857BandSlotForHz(hz);
  if (!slotOut) return true;
  bool vfoB = false;
  if (!yaesuFt857QueryVfoB(vfoB, timeoutMs)) return false;
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

bool yaesuFt857QueryMenuAndRow(uint8_t& menuOut, uint8_t& rowOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  uint8_t word[2] = {0};
  if (!yaesuCatReadEepromWord(FT857_MENU_ROW_ADDR, word, timeoutMs)) return false;
  menuOut = (uint8_t)(word[0] + 1);
  rowOut = (uint8_t)(word[1] + 1);
  return true;
}

bool yaesuFt817QueryRfPowerTenths(uint16_t& tenthsOut, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return false;
  const bool ft818 = currentFt8x7Model() == Ft8x7Model::Ft818;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT817_RF_POWER_ADDR, b, timeoutMs)) return false;
  tenthsOut = ft817RfPowerTenths(b, ft818);
  return true;
}

// Lock and fast tuning are stored inverted, like on the FT-857/897, though the KA7OEI map says
// 1 = on. All follow the front panel at once.
static bool ft817ReadBit(uint16_t addr, uint8_t mask, bool& onOut, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(addr, b, timeoutMs)) return false;
  onOut = (b & mask) != 0;
  return true;
}

bool yaesuFt817QueryLock(bool& onOut, uint32_t timeoutMs) {
  bool unlocked = false;
  if (!ft817ReadBit(0x0057, 0x40, unlocked, timeoutMs)) return false;
  onOut = !unlocked;
  return true;
}

bool yaesuFt817QueryFastTuning(bool& onOut, uint32_t timeoutMs) {
  bool slow = false;
  if (!ft817ReadBit(0x0057, 0x80, slow, timeoutMs)) return false;
  onOut = !slow;
  return true;
}

bool yaesuFt817QueryNb(bool& onOut, uint32_t timeoutMs) { return ft817ReadBit(0x0057, 0x20, onOut, timeoutMs); }
// IF shift (a long press of CLAR; the map's "PBT").
bool yaesuFt817QueryIfShift(bool& onOut, uint32_t timeoutMs) { return ft817ReadBit(0x0057, 0x10, onOut, timeoutMs); }
bool yaesuFt817QueryVox(bool& onOut, uint32_t timeoutMs) { return ft817ReadBit(0x0058, 0x80, onOut, timeoutMs); }
bool yaesuFt817QueryBreakIn(bool& onOut, uint32_t timeoutMs) { return ft817ReadBit(0x0058, 0x20, onOut, timeoutMs); }
bool yaesuFt817QueryKeyer(bool& onOut, uint32_t timeoutMs) { return ft817ReadBit(0x0058, 0x10, onOut, timeoutMs); }

bool yaesuFt817QueryAgc(YaesuAgc& out, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT817_AGC_ADDR, b, timeoutMs)) return false;
  out = ft817AgcFromByte(b);
  return true;
}

bool yaesuFt817QueryMenuAndRow(uint8_t& menuOut, uint8_t& rowOut, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return false;
  uint8_t menu = 0;
  uint8_t row = 0;
  if (!yaesuCatReadEepromByte(FT817_MENU_ADDR, menu, timeoutMs)) return false;
  if (!yaesuCatReadEepromByte(FT817_ROW_ADDR, row, timeoutMs)) return false;
  menuOut = ft817MenuFromByte(menu);
  rowOut = ft817RowFromByte(row);
  return true;
}

bool yaesuFt817QueryRearAntenna(uint64_t hz, bool& rearOut, uint32_t timeoutMs) {
  return ft817ReadBit(FT817_ANTENNA_ADDR, ft817RearAntennaMask(ft8x7BandGroupForHz(hz)), rearOut, timeoutMs);
}

// Lock and fast tuning are stored inverted.
bool yaesuFt857QueryVox(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x006B, 0x80, onOut, timeoutMs); }
bool yaesuFt857QueryProc(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x00A9, 0x02, onOut, timeoutMs); }
bool yaesuFt857QueryDspRow(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x00A8, 0x80, onOut, timeoutMs); }

bool yaesuFt857QueryLock(bool& onOut, uint32_t timeoutMs) {
  bool unlocked = false;
  if (!ft857ReadBit(0x006A, 0x40, unlocked, timeoutMs)) return false;
  onOut = !unlocked;
  return true;
}

bool yaesuFt857QueryFastTuning(bool& onOut, uint32_t timeoutMs) {
  bool slow = false;
  if (!ft857ReadBit(0x006A, 0x80, slow, timeoutMs)) return false;
  onOut = !slow;
  return true;
}

// 0xA7 bit 7 = filter 2. Bit 1 was set when the built-in filter was chosen in CW but not in USB.
bool yaesuFt857QueryFilter(YaesuFt857Filter& out, uint32_t timeoutMs) {
  bool filter2 = false;
  if (!ft857ReadBit(0x00A7, 0x80, filter2, timeoutMs)) return false;
  out = filter2 ? YaesuFt857Filter::Filter2 : YaesuFt857Filter::BuiltIn;
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

bool yaesuFt857QueryIfShift(bool& onOut, uint32_t timeoutMs) { return ft857ReadBit(0x006A, 0x10, onOut, timeoutMs); }

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

bool yaesuFt857QueryKnobIsSquelch(bool& squelchOut, uint32_t timeoutMs) {
  return ft857ReadBit(0x0072, 0x80, squelchOut, timeoutMs);
}

bool yaesuFt857QueryMicEq(YaesuFt857MicEq& out, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT857_MIC_EQ_ADDR, b, timeoutMs)) return false;
  out = ft857MicEqFromByte(b);
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
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x08 : 0x88)};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetClarifier(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x05 : 0x85)};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetSplit(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x02 : 0x82)};
  return yaesuCatSendWriteOnly(cmd);
}

bool yaesuCatSetLockDocumentedRaw(bool on) {
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x00 : 0x80)};
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
  const uint8_t cmd[5] = {0x00, 0x00, 0x00, 0x00, (uint8_t)(on ? 0x0F : 0x8F)};
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

// FT-857/897 take separate TX and RX values; the FT-817 takes one.
static void fillToneData(uint16_t value, uint8_t data[4]) {
  yaesuEncodeToneData(value, currentIsFt857Family(), data);
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
