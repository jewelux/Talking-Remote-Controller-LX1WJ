#include "protocol_ft8x7_eeprom.h"

#include "protocol_yaesu_cat.h"
#include "radio_catalog.h"

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

bool yaesuFt8x7HasMicGain(Ft8x7MicGain gain) {
  YaesuFt857Level level = YaesuFt857Level::SsbMicGain;
  return currentIsFt817Family() || (currentIsFt857Family() && ft857MicGainLevel(gain, level));
}

bool yaesuFt8x7QueryMicGain(Ft8x7MicGain gain, uint8_t& valueOut, uint32_t timeoutMs) {
  if (currentIsFt857Family()) {
    YaesuFt857Level level = YaesuFt857Level::SsbMicGain;
    uint16_t value = 0;
    if (!ft857MicGainLevel(gain, level) || !yaesuFt857QueryLevel(level, value, timeoutMs)) return false;
    valueOut = value > 100 ? 100 : (uint8_t)value;
    return true;
  }
  if (!currentIsFt817Family()) return false;
  bool pkt9600 = false;
  if (gain == Ft8x7MicGain::Pkt) {
    uint8_t rate = 0;
    if (!yaesuCatReadEepromByte(FT817_PKT_RATE_ADDR, rate, timeoutMs)) return false;
    pkt9600 = (rate & FT817_PKT_RATE_9600_MASK) != 0;
  }
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(ft817MicGainAddr(gain, pkt9600), b, timeoutMs)) return false;
  valueOut = ft817LevelFromByte(b);
  return true;
}

bool yaesuFt8x7QueryVoxGain(uint8_t& valueOut, uint32_t timeoutMs) {
  if (currentIsFt857Family()) {
    uint16_t value = 0;
    if (!yaesuFt857QueryLevel(YaesuFt857Level::VoxGain, value, timeoutMs)) return false;
    valueOut = value > 100 ? 100 : (uint8_t)value;
    return true;
  }
  if (!currentIsFt817Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT817_VOX_GAIN_ADDR, b, timeoutMs)) return false;
  valueOut = ft817LevelFromByte(b);
  return true;
}

bool yaesuFt8x7QueryVoxDelayMs(uint16_t& msOut, uint32_t timeoutMs) {
  if (currentIsFt857Family()) return yaesuFt857QueryLevel(YaesuFt857Level::VoxDelay, msOut, timeoutMs);
  if (!currentIsFt817Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT817_VOX_DELAY_ADDR, b, timeoutMs)) return false;
  msOut = ft817VoxDelayMs(b);
  return true;
}
