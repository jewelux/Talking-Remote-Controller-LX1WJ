#include "protocol_ft8x7_eeprom.h"

#include "protocol_yaesu_cat.h"
#include "radio_catalog.h"

// Undocumented 0xBB reads two EEPROM bytes, at addr and addr + 1; odd addresses work too (0xBB
// at 0069 returned the bytes at 0069 and 006A on an FT-897, and FT8x7Com reads the FT-817's
// 0075 and 0076 so).
static bool readEepromPair(uint16_t addr, uint8_t out[2], uint32_t timeoutMs) {
  const uint8_t cmd[5] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), 0x00, 0x00, 0xBB};
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  return yaesuCatRead1(out[0], timeoutMs) && yaesuCatRead1(out[1], timeoutMs);
}

bool yaesuCatReadEepromWord(uint16_t addr, uint8_t out[2], uint32_t timeoutMs) {
  return readEepromPair((uint16_t)(addr & 0xFFFE), out, timeoutMs);
}

bool yaesuCatReadEepromByte(uint16_t addr, uint8_t& out, uint32_t timeoutMs) {
  uint8_t word[2] = {0};
  if (!yaesuCatReadEepromWord(addr, word, timeoutMs)) return false;
  out = word[addr & 0x01];
  return true;
}

// Undocumented 0xBC writes two EEPROM bytes, at addr and addr + 1; odd addresses work too, as
// for 0xBB. It takes effect at once.
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
  Ft8x7MenuRowMap map;
  if (!ft8x7MenuRowMap(currentFt8x7Model(), map)) return false;
  uint8_t pair[2] = {0};
  if (!readEepromPair(map.addr, pair, timeoutMs)) return false;
  menuOut = ft8x7MenuFromPair(map, pair);
  rowOut = ft8x7RowFromPair(map, pair);
  return true;
}

YaesuMenuRowWrite yaesuFt8x7SetMenuOrRow(bool row, uint8_t number, uint8_t& numberOut,
                                         uint32_t timeoutMs) {
  Ft8x7MenuRowMap map;
  if (!ft8x7MenuRowMap(currentFt8x7Model(), map)) return YaesuMenuRowWrite::Unsupported;
  if (number < 1 || number > ft8x7MenuRowLast(map, row, true)) return YaesuMenuRowWrite::OutOfRange;
  bool extOn = true;
  if (!row) {
    uint8_t ext = 0;
    if (!yaesuCatReadEepromByte(map.extAddr, ext, timeoutMs)) return YaesuMenuRowWrite::NoReply;
    extOn = (ext & map.extMask) != 0;
    if (number > ft8x7MenuRowLast(map, row, extOn)) return YaesuMenuRowWrite::OutOfRange;
  }
  uint8_t old[2] = {0};
  if (!readEepromPair(map.addr, old, timeoutMs)) return YaesuMenuRowWrite::NoReply;
  uint8_t pair[2] = {0};
  if (!ft8x7MenuRowPair(map, row, number, extOn, old, pair)) return YaesuMenuRowWrite::UnexpectedBytes;
  yaesuCatWriteEeprom2(map.addr, pair);
  uint8_t check[2] = {0};
  if (!readEepromPair(map.addr, check, timeoutMs)) return YaesuMenuRowWrite::NoReply;
  if (check[0] != pair[0] || check[1] != pair[1]) return YaesuMenuRowWrite::NotVerified;
  numberOut = row ? ft8x7RowFromPair(map, check) : ft8x7MenuFromPair(map, check);
  return YaesuMenuRowWrite::Ok;
}

// The block of the band the active VFO is on (0x6C), on any frequency. onMemoryOut: a memory
// channel or one being tuned is in use, and the radio uses its own record instead.
static bool ft857ActiveBandBlock(uint16_t& blockOut, bool& onMemoryOut, uint32_t timeoutMs) {
  Ft8x7FlagField vfoB;
  if (!ft8x7FlagField(currentFt8x7Model(), Ft8x7Flag::VfoB, vfoB)) return false;
  uint8_t vfoMem = 0;
  uint8_t bands = 0;
  if (!yaesuCatReadEepromByte(vfoB.addr, vfoMem, timeoutMs)) return false;
  if (!yaesuCatReadEepromByte(FT857_VFO_BANDS_ADDR, bands, timeoutMs)) return false;
  onMemoryOut = (vfoMem & FT857_MEMORY_MASK) != 0;
  blockOut = ft857VfoBandBlock(bands, ft8x7FlagValue(vfoB, vfoMem));
  return true;
}

bool yaesuFt857QueryBandFlags(uint64_t hz, YaesuFt857BandFlags& out, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  out = YaesuFt857BandFlags();
  uint16_t block = 0;
  bool onMemory = false;
  if (!ft857ActiveBandBlock(block, onMemory, timeoutMs)) return false;
  const bool hasIpoAtt = ft857HasIpoAtt(hz);
  uint8_t nar = 0;
  uint8_t ipoAtt = 0;
  if (!yaesuCatReadEepromByte(block + FT857_BAND_NAR_OFFSET, nar, timeoutMs)) return false;
  if (hasIpoAtt && !yaesuCatReadEepromByte(block + FT857_BAND_IPO_ATT_OFFSET, ipoAtt, timeoutMs)) {
    return false;
  }
  out.hasIpoAtt = hasIpoAtt;
  out.nar = (nar & FT857_BAND_NAR_MASK) != 0;
  out.ipo = hasIpoAtt && (ipoAtt & FT857_BAND_IPO_MASK) != 0;
  out.att = hasIpoAtt && (ipoAtt & FT857_BAND_ATT_MASK) != 0;
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

bool yaesuFt857QueryRitOffsetHz(int32_t& offsetOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  uint16_t block = 0;
  bool onMemory = false;
  if (!ft857ActiveBandBlock(block, onMemory, timeoutMs)) return false;
  uint8_t word[2] = {0};
  if (!yaesuCatReadEepromWord(block + FT857_BAND_RIT_OFFSET, word, timeoutMs)) return false;
  offsetOut = ft857RitOffsetHz(word);
  return true;
}

// The tone byte of the band block the active VFO is on; 0, and knownOut false, on a memory.
static bool ft857QueryToneByte(bool& knownOut, uint8_t& out, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return false;
  out = 0;
  knownOut = false;
  uint16_t block = 0;
  bool onMemory = false;
  if (!ft857ActiveBandBlock(block, onMemory, timeoutMs)) return false;
  if (onMemory) return true;
  if (!yaesuCatReadEepromByte(block + FT857_BAND_TONE_OFFSET, out, timeoutMs)) return false;
  knownOut = true;
  return true;
}

bool yaesuFt857QueryToneMode(bool& knownOut, YaesuToneMode& out, uint32_t timeoutMs) {
  uint8_t b = 0;
  if (!ft857QueryToneByte(knownOut, b, timeoutMs)) return false;
  out = ft857ToneModeFromByte(b);
  return true;
}

bool yaesuFt857QuerySplitTone(bool& knownOut, bool& onOut, uint32_t timeoutMs) {
  uint8_t b = 0;
  if (!ft857QueryToneByte(knownOut, b, timeoutMs)) return false;
  onOut = (b & FT857_BAND_SPLIT_TONE_MASK) != 0;
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
