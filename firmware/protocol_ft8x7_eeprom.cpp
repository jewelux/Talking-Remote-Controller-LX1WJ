#include "protocol_ft8x7_eeprom.h"

#include "protocol_ops_yaesu.h"
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
static constexpr uint32_t FT8X7_EEPROM_WRITE_SETTLE_MS = 120;
static_assert(FT8X7_EEPROM_WRITE_SETTLE_MS >= YAESU_CAT_WRITE_SETTLE_MS,
              "the delay after a write would wrap around");

void yaesuCatWriteEeprom2(uint16_t addr, const uint8_t data[2]) {
  const uint8_t cmd[5] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), data[0], data[1], 0xBC};
  // The FT-817 answers a write with one byte about 110 ms later (measured), and misses a
  // command sent before it. The FT-897 does not answer; it gets as long.
  if (currentIsFt817Family()) {
    uint8_t ack = 0;
    (void)yaesuCatTransact1(cmd, ack, YAESU_CAT_REPLY_TIMEOUT_MS);
    return;
  }
  yaesuCatSendWriteOnly(cmd);
  delay(FT8X7_EEPROM_WRITE_SETTLE_MS - YAESU_CAT_WRITE_SETTLE_MS);
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

// Ok when the radio is receiving, so a settings write may go ahead.
static YaesuEepromWrite checkReceiving(uint32_t timeoutMs) {
  uint8_t txStatus = 0;
  if (!yaesuCatQueryTxStatusRaw(txStatus, timeoutMs)) return YaesuEepromWrite::NoReply;
  return yaesuCatTxStatusTransmitting(txStatus) ? YaesuEepromWrite::Transmitting : YaesuEepromWrite::Ok;
}

// Writes pair at addr and reads it back into check. The FT-817 may miss the first read after
// a write that changed its menu item (measured), so the read is tried twice.
static YaesuEepromWrite writeEepromPairChecked(uint16_t addr, const uint8_t pair[2], uint8_t check[2],
                                               uint32_t timeoutMs) {
  yaesuCatWriteEeprom2(addr, pair);
  if (!readEepromPair(addr, check, timeoutMs) && !readEepromPair(addr, check, timeoutMs)) {
    return YaesuEepromWrite::NoReply;
  }
  if (check[0] != pair[0] || check[1] != pair[1]) return YaesuEepromWrite::NotVerified;
  return YaesuEepromWrite::Ok;
}

YaesuEepromWrite yaesuFt8x7SetMenuOrRow(bool row, uint8_t number, uint8_t& numberOut,
                                        uint32_t timeoutMs) {
  Ft8x7MenuRowMap map;
  if (!ft8x7MenuRowMap(currentFt8x7Model(), map)) return YaesuEepromWrite::Unsupported;
  if (number < 1 || number > ft8x7MenuRowLast(map, row, true)) return YaesuEepromWrite::OutOfRange;
  const YaesuEepromWrite rx = checkReceiving(timeoutMs);
  if (rx != YaesuEepromWrite::Ok) return rx;
  bool extOn = true;
  if (!row) {
    uint8_t ext = 0;
    if (!yaesuCatReadEepromByte(map.extAddr, ext, timeoutMs)) return YaesuEepromWrite::NoReply;
    extOn = (ext & map.extMask) != 0;
    if (number > ft8x7MenuRowLast(map, row, extOn)) return YaesuEepromWrite::OutOfRange;
  }
  uint8_t old[2] = {0};
  if (!readEepromPair(map.addr, old, timeoutMs)) return YaesuEepromWrite::NoReply;
  uint8_t pair[2] = {0};
  if (!ft8x7MenuRowPair(map, row, number, extOn, old, pair)) return YaesuEepromWrite::UnexpectedBytes;
  uint8_t check[2] = {0};
  const YaesuEepromWrite written = writeEepromPairChecked(map.addr, pair, check, timeoutMs);
  if (written != YaesuEepromWrite::Ok) return written;
  numberOut = row ? ft8x7RowFromPair(map, check) : ft8x7MenuFromPair(map, check);
  return YaesuEepromWrite::Ok;
}

// Reads the setting's byte (pair[0]) and the one after it, while the radio is receiving.
static YaesuEepromWrite readForWrite(uint16_t addr, uint8_t pair[2], uint32_t timeoutMs) {
  const YaesuEepromWrite rx = checkReceiving(timeoutMs);
  if (rx != YaesuEepromWrite::Ok) return rx;
  return readEepromPair(addr, pair, timeoutMs) ? YaesuEepromWrite::Ok : YaesuEepromWrite::NoReply;
}

// Writes b at addr unless pair[0], as read, already is b. 0xBC writes two bytes, so the byte
// after it is written back as read.
static YaesuEepromWrite writeByteChecked(uint16_t addr, uint8_t pair[2], uint8_t b, uint32_t timeoutMs) {
  if (b == pair[0]) return YaesuEepromWrite::Ok;
  pair[0] = b;
  uint8_t check[2] = {0};
  return writeEepromPairChecked(addr, pair, check, timeoutMs);
}

// Writes pair[0] at addr as make(old, b) builds it from the byte just read, so a step or a
// toggle starts from what the radio holds; make returns false when old does not fit the map.
// byteOut is the byte the radio now holds.
template <typename MakeByte>
static YaesuEepromWrite writeFromRead(uint16_t addr, uint8_t pair[2], MakeByte make, uint8_t& byteOut,
                                      uint32_t timeoutMs) {
  uint8_t b = 0;
  if (!make(pair[0], b)) return YaesuEepromWrite::UnexpectedBytes;
  const YaesuEepromWrite written = writeByteChecked(addr, pair, b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) byteOut = b;
  return written;
}

// The same, reading the byte first while the radio is receiving.
template <typename MakeByte>
static YaesuEepromWrite rewriteByte(uint16_t addr, MakeByte make, uint8_t& byteOut, uint32_t timeoutMs) {
  uint8_t pair[2] = {0};
  const YaesuEepromWrite read = readForWrite(addr, pair, timeoutMs);
  if (read != YaesuEepromWrite::Ok) return read;
  return writeFromRead(addr, pair, make, byteOut, timeoutMs);
}

// Sets flag to next(the flag as read); onOut is the flag as written.
template <typename Next>
static YaesuEepromWrite rewriteFlag(Ft8x7Flag flag, Next next, bool& onOut, uint32_t timeoutMs) {
  Ft8x7FlagField field;
  if (!ft8x7FlagField(currentFt8x7Model(), flag, field)) return YaesuEepromWrite::Unsupported;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      field.addr,
      [&](uint8_t old, uint8_t& out) {
        out = ft8x7FlagByte(field, next(ft8x7FlagValue(field, old)), old);
        return true;
      },
      b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) onOut = ft8x7FlagValue(field, b);
  return written;
}

YaesuEepromWrite yaesuFt8x7SetFlag(Ft8x7Flag flag, bool on, uint32_t timeoutMs) {
  bool onWritten = on;
  return rewriteFlag(flag, [on](bool) { return on; }, onWritten, timeoutMs);
}

YaesuEepromWrite yaesuFt8x7ToggleFlag(Ft8x7Flag flag, bool& onOut, uint32_t timeoutMs) {
  return rewriteFlag(flag, [](bool on) { return !on; }, onOut, timeoutMs);
}

// FT-857/897: sets level to target(the value stored); valueOut is the value as written.
template <typename Target>
static YaesuEepromWrite ft857RewriteLevel(YaesuFt857Level level, Target target, uint16_t& valueOut,
                                          uint32_t timeoutMs) {
  uint16_t addr = 0;
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!currentIsFt857Family() || !ft857LevelAddr(level, addr) || !ft857LevelRange(level, lo, hi)) {
    return YaesuEepromWrite::Unsupported;
  }
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      addr,
      [&](uint8_t old, uint8_t& out) { return ft857LevelByte(level, target(ft857LevelValue(level, old)), old, out); },
      b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) valueOut = ft857LevelValue(level, b);
  return written;
}

YaesuEepromWrite yaesuFt857SetLevel(YaesuFt857Level level, uint16_t value, uint16_t& valueOut,
                                    uint32_t timeoutMs) {
  uint16_t lo = 0;
  uint16_t hi = 0;
  if (!currentIsFt857Family() || !ft857LevelRange(level, lo, hi)) return YaesuEepromWrite::Unsupported;
  if (value < lo || value > hi) return YaesuEepromWrite::OutOfRange;
  return ft857RewriteLevel(level, [value](uint16_t) { return value; }, valueOut, timeoutMs);
}

YaesuEepromWrite yaesuFt857StepLevel(YaesuFt857Level level, int steps, uint16_t& valueOut,
                                     uint32_t timeoutMs) {
  return ft857RewriteLevel(
      level, [level, steps](uint16_t stored) { return ft857LevelStep(level, stored, steps); }, valueOut, timeoutMs);
}

YaesuEepromWrite yaesuFt857SetRfPowerWatts(uint64_t hz, uint8_t watts, uint8_t& wattsOut,
                                           uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return YaesuEepromWrite::Unsupported;
  const Ft8x7BandGroup group = ft8x7BandGroupForHz(hz);
  uint8_t lo = 0;
  uint8_t hi = 0;
  ft857RfPowerRange(group, lo, hi);
  if (watts < lo || watts > hi) return YaesuEepromWrite::OutOfRange;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      ft857RfPowerAddr(group), [&](uint8_t old, uint8_t& out) { return ft857RfPowerByte(group, watts, old, out); },
      b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) wattsOut = ft857RfPowerWatts(b);
  return written;
}

YaesuEepromWrite yaesuFt817SetPowerLevel(Ft817PowerLevel level, Ft817PowerLevel& levelOut,
                                         uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return YaesuEepromWrite::Unsupported;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      FT817_RF_POWER_ADDR,
      [level](uint8_t old, uint8_t& out) {
        out = ft817PowerLevelByte(level, old);
        return true;
      },
      b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) levelOut = ft817PowerLevel(b);
  return written;
}

// FT-857/897: the speed at 0x6A, then the AgcOn flag at 0xA8. They are not neighbours, so one
// 0xBC cannot write both; the radio is checked for receiving once, before either. A failure
// before the flag is written leaves AGC as it was, and when it was off, only the speed it will
// use once switched on has changed.
static YaesuEepromWrite ft857SetAgcSpeed(YaesuAgc agc, YaesuAgc& agcOut, uint32_t timeoutMs) {
  Ft8x7FlagField onField;
  if (!ft8x7FlagField(Ft8x7Model::Ft857, Ft8x7Flag::AgcOn, onField)) return YaesuEepromWrite::Unsupported;
  uint8_t pair[2] = {0};
  const YaesuEepromWrite read = readForWrite(FT857_AGC_SPEED_ADDR, pair, timeoutMs);
  if (read != YaesuEepromWrite::Ok) return read;
  uint8_t speed = 0;
  const YaesuEepromWrite speedWritten = writeFromRead(
      FT857_AGC_SPEED_ADDR, pair,
      [agc](uint8_t old, uint8_t& out) {
        out = ft857AgcSpeedByte(agc, old);
        return true;
      },
      speed, timeoutMs);
  if (speedWritten != YaesuEepromWrite::Ok) return speedWritten;
  if (!readEepromPair(onField.addr, pair, timeoutMs)) return YaesuEepromWrite::NoReply;
  uint8_t flag = 0;
  const YaesuEepromWrite onWritten = writeFromRead(
      onField.addr, pair,
      [&](uint8_t old, uint8_t& out) {
        out = ft8x7FlagByte(onField, true, old);
        return true;
      },
      flag, timeoutMs);
  if (onWritten == YaesuEepromWrite::Ok) agcOut = ft857AgcFromSpeed(speed);
  return onWritten;
}

YaesuEepromWrite yaesuFt8x7SetAgc(YaesuAgc agc, YaesuAgc& agcOut, uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  if (ft8x7IsFt817Family(model)) {
    uint8_t b = 0;
    const YaesuEepromWrite written = rewriteByte(
        FT817_AGC_ADDR,
        [agc](uint8_t old, uint8_t& out) {
          out = ft817AgcByte(agc, old);
          return true;
        },
        b, timeoutMs);
    if (written == YaesuEepromWrite::Ok) agcOut = ft817AgcFromByte(b);
    return written;
  }
  if (model != Ft8x7Model::Ft857) return YaesuEepromWrite::Unsupported;
  if (agc != YaesuAgc::Off) return ft857SetAgcSpeed(agc, agcOut, timeoutMs);
  const YaesuEepromWrite off = yaesuFt8x7SetFlag(Ft8x7Flag::AgcOn, false, timeoutMs);
  if (off == YaesuEepromWrite::Ok) agcOut = YaesuAgc::Off;
  return off;
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

bool yaesuFt817QueryPowerLevel(Ft817PowerLevel& levelOut, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT817_RF_POWER_ADDR, b, timeoutMs)) return false;
  levelOut = ft817PowerLevel(b);
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

YaesuEepromWrite yaesuFt857SetMicEq(YaesuFt857MicEq eq, YaesuFt857MicEq& eqOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return YaesuEepromWrite::Unsupported;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      FT857_MIC_EQ_ADDR,
      [eq](uint8_t old, uint8_t& out) {
        out = ft857MicEqByte(eq, old);
        return true;
      },
      b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) eqOut = ft857MicEqFromByte(b);
  return written;
}

bool yaesuFt8x7HasMicGain(Ft8x7MicGain gain) {
  YaesuFt857Level level = YaesuFt857Level::SsbMicGain;
  return currentIsFt817Family() || (currentIsFt857Family() && ft857MicGainLevel(gain, level));
}

// FT-817/818: the address of gain; the PKT one by the packet rate.
static bool ft817MicGainAddrNow(Ft8x7MicGain gain, uint16_t& addrOut, uint32_t timeoutMs) {
  bool pkt9600 = false;
  if (gain == Ft8x7MicGain::Pkt) {
    uint8_t rate = 0;
    if (!yaesuCatReadEepromByte(FT817_PKT_RATE_ADDR, rate, timeoutMs)) return false;
    pkt9600 = (rate & FT817_PKT_RATE_9600_MASK) != 0;
  }
  addrOut = ft817MicGainAddr(gain, pkt9600);
  return true;
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
  uint16_t addr = 0;
  if (!ft817MicGainAddrNow(gain, addr, timeoutMs)) return false;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(addr, b, timeoutMs)) return false;
  valueOut = ft817LevelFromByte(b);
  return true;
}

// Sets gain to target(the gain stored, 0..100); valueOut is the gain as written.
template <typename Target>
static YaesuEepromWrite rewriteMicGain(Ft8x7MicGain gain, Target target, uint8_t& valueOut, uint32_t timeoutMs) {
  if (!yaesuFt8x7HasMicGain(gain)) return YaesuEepromWrite::Unsupported;
  if (currentIsFt857Family()) {
    YaesuFt857Level level = YaesuFt857Level::SsbMicGain;
    (void)ft857MicGainLevel(gain, level);
    uint16_t read = 0;
    const YaesuEepromWrite written = ft857RewriteLevel(
        level, [&](uint16_t stored) { return (uint16_t)target(stored > 100 ? 100 : (uint8_t)stored); }, read,
        timeoutMs);
    if (written == YaesuEepromWrite::Ok) valueOut = (uint8_t)read;
    return written;
  }
  uint16_t addr = 0;
  if (!ft817MicGainAddrNow(gain, addr, timeoutMs)) return YaesuEepromWrite::NoReply;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      addr, [&](uint8_t old, uint8_t& out) { return ft817LevelByte(target(ft817LevelFromByte(old)), old, out); }, b,
      timeoutMs);
  if (written == YaesuEepromWrite::Ok) valueOut = ft817LevelFromByte(b);
  return written;
}

YaesuEepromWrite yaesuFt8x7SetMicGain(Ft8x7MicGain gain, uint8_t value, uint8_t& valueOut,
                                      uint32_t timeoutMs) {
  if (!yaesuFt8x7HasMicGain(gain)) return YaesuEepromWrite::Unsupported;
  if (value > 100) return YaesuEepromWrite::OutOfRange;
  return rewriteMicGain(gain, [value](uint8_t) { return value; }, valueOut, timeoutMs);
}

YaesuEepromWrite yaesuFt8x7StepMicGain(Ft8x7MicGain gain, int steps, uint8_t& valueOut, uint32_t timeoutMs) {
  return rewriteMicGain(gain, [steps](uint8_t stored) { return percentStep(stored, steps); }, valueOut,
                        timeoutMs);
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

// The VOX gain set to target(the gain read for the write).
template <typename Target>
static YaesuEepromWrite rewriteVoxGain(Target target, uint8_t& valueOut, uint32_t timeoutMs) {
  if (currentIsFt857Family()) {
    uint16_t read = 0;
    const YaesuEepromWrite written = ft857RewriteLevel(
        YaesuFt857Level::VoxGain, [&](uint16_t stored) { return (uint16_t)target(stored > 100 ? 100 : (uint8_t)stored); },
        read, timeoutMs);
    if (written == YaesuEepromWrite::Ok) valueOut = (uint8_t)read;
    return written;
  }
  if (!currentIsFt817Family()) return YaesuEepromWrite::Unsupported;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      FT817_VOX_GAIN_ADDR,
      [&](uint8_t old, uint8_t& out) { return ft817VoxGainByte(target(ft817LevelFromByte(old)), old, out); }, b,
      timeoutMs);
  if (written == YaesuEepromWrite::Ok) valueOut = ft817LevelFromByte(b);
  return written;
}

YaesuEepromWrite yaesuFt8x7SetVoxGain(uint8_t value, uint8_t& valueOut, uint32_t timeoutMs) {
  if (value < 1 || value > 100) return YaesuEepromWrite::OutOfRange;
  return rewriteVoxGain([value](uint8_t) { return value; }, valueOut, timeoutMs);
}

YaesuEepromWrite yaesuFt8x7StepVoxGain(int steps, uint8_t& valueOut, uint32_t timeoutMs) {
  return rewriteVoxGain(
      [steps](uint8_t stored) {
        const uint8_t stepped = percentStep(stored, steps);
        return stepped < 1 ? (uint8_t)1 : stepped;
      },
      valueOut, timeoutMs);
}

// The VOX delay in ms set to target(the delay read for the write).
template <typename Target>
static YaesuEepromWrite rewriteFt817VoxDelay(Target target, uint16_t& msOut, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return YaesuEepromWrite::Unsupported;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      FT817_VOX_DELAY_ADDR,
      [&](uint8_t old, uint8_t& out) { return ft817VoxDelayByte(target(ft817VoxDelayMs(old)), old, out); }, b,
      timeoutMs);
  if (written == YaesuEepromWrite::Ok) msOut = ft817VoxDelayMs(b);
  return written;
}

YaesuEepromWrite yaesuFt8x7SetVoxDelayMs(uint16_t ms, uint16_t& msOut, uint32_t timeoutMs) {
  if (currentIsFt857Family()) return yaesuFt857SetLevel(YaesuFt857Level::VoxDelay, ms, msOut, timeoutMs);
  if (!currentIsFt817Family()) return YaesuEepromWrite::Unsupported;
  if (ms < 100 || ms > 2500) return YaesuEepromWrite::OutOfRange;
  return rewriteFt817VoxDelay([ms](uint16_t) { return ms; }, msOut, timeoutMs);
}

YaesuEepromWrite yaesuFt8x7StepVoxDelay(int steps, uint16_t& msOut, uint32_t timeoutMs) {
  if (currentIsFt857Family()) return yaesuFt857StepLevel(YaesuFt857Level::VoxDelay, steps, msOut, timeoutMs);
  return rewriteFt817VoxDelay([steps](uint16_t stored) { return ft817VoxDelayStep(stored, steps); }, msOut,
                              timeoutMs);
}

// The jack of the band group of hz set to rear(the jack read for the write).
template <typename Rear>
static YaesuEepromWrite rewriteRearAntenna(uint64_t hz, Rear rear, bool& rearOut, uint32_t timeoutMs) {
  if (!currentIsFt817Family()) return YaesuEepromWrite::Unsupported;
  const Ft8x7BandGroup group = ft8x7BandGroupForHz(hz);
  const uint8_t mask = ft817RearAntennaMask(group);
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      FT817_ANTENNA_ADDR,
      [&](uint8_t old, uint8_t& out) {
        out = ft817RearAntennaByte(group, rear((old & mask) != 0), old);
        return true;
      },
      b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) rearOut = (b & mask) != 0;
  return written;
}

YaesuEepromWrite yaesuFt817SetRearAntenna(uint64_t hz, bool rear, bool& rearOut, uint32_t timeoutMs) {
  return rewriteRearAntenna(hz, [rear](bool) { return rear; }, rearOut, timeoutMs);
}

YaesuEepromWrite yaesuFt817ToggleRearAntenna(uint64_t hz, bool& rearOut, uint32_t timeoutMs) {
  return rewriteRearAntenna(hz, [](bool rear) { return !rear; }, rearOut, timeoutMs);
}

YaesuEepromRead yaesuFt8x7QueryCwLevel(Ft8x7CwLevel level, uint16_t& valueOut, uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  uint16_t addr = 0;
  if (!ft8x7CwLevelAddr(model, level, addr)) return YaesuEepromRead::Unsupported;
  uint8_t pair[2] = {0};
  if (!readEepromPair(addr, pair, timeoutMs)) return YaesuEepromRead::NoReply;
  return ft8x7CwLevelValue(model, level, pair, valueOut) ? YaesuEepromRead::Ok : YaesuEepromRead::UnexpectedBytes;
}

// Sets level to target(the value stored); valueOut is the value read back. The whole pair is
// built from the one read, since the FT-857/897 delay spans both bytes.
template <typename Target>
static YaesuEepromWrite rewriteCwLevel(Ft8x7CwLevel level, Target target, uint16_t& valueOut, uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  uint16_t addr = 0;
  if (!ft8x7CwLevelAddr(model, level, addr)) return YaesuEepromWrite::Unsupported;
  uint8_t old[2] = {0};
  const YaesuEepromWrite read = readForWrite(addr, old, timeoutMs);
  if (read != YaesuEepromWrite::Ok) return read;
  uint16_t stored = 0;
  uint8_t pair[2] = {0};
  if (!ft8x7CwLevelValue(model, level, old, stored) || !ft8x7CwLevelPair(model, level, target(stored), old, pair)) {
    return YaesuEepromWrite::UnexpectedBytes;
  }
  uint8_t check[2] = {pair[0], pair[1]};
  if (pair[0] != old[0] || pair[1] != old[1]) {
    const YaesuEepromWrite written = writeEepromPairChecked(addr, pair, check, timeoutMs);
    if (written != YaesuEepromWrite::Ok) return written;
  }
  return ft8x7CwLevelValue(model, level, check, valueOut) ? YaesuEepromWrite::Ok : YaesuEepromWrite::NotVerified;
}

YaesuEepromWrite yaesuFt8x7SetCwLevel(Ft8x7CwLevel level, uint16_t value, uint16_t& valueOut,
                                      uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  uint16_t addr = 0;
  if (!ft8x7CwLevelAddr(model, level, addr)) return YaesuEepromWrite::Unsupported;
  if (!ft8x7CwLevelAccepts(model, level, value)) return YaesuEepromWrite::OutOfRange;
  return rewriteCwLevel(level, [value](uint16_t) { return value; }, valueOut, timeoutMs);
}

YaesuEepromWrite yaesuFt8x7StepCwLevel(Ft8x7CwLevel level, int steps, uint16_t& valueOut,
                                       uint32_t timeoutMs) {
  const Ft8x7Model model = currentFt8x7Model();
  return rewriteCwLevel(
      level, [model, level, steps](uint16_t stored) { return ft8x7CwLevelStep(model, level, stored, steps); },
      valueOut, timeoutMs);
}

YaesuEepromRead yaesuFt857QueryCwBfo(Ft857CwBfo& out, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return YaesuEepromRead::Unsupported;
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(FT857_CW_BFO_ADDR, b, timeoutMs)) return YaesuEepromRead::NoReply;
  return ft857CwBfoFromByte(b, out) ? YaesuEepromRead::Ok : YaesuEepromRead::UnexpectedBytes;
}

YaesuEepromWrite yaesuFt857SetCwBfo(Ft857CwBfo bfo, Ft857CwBfo& bfoOut, uint32_t timeoutMs) {
  if (!currentIsFt857Family()) return YaesuEepromWrite::Unsupported;
  uint8_t b = 0;
  const YaesuEepromWrite written = rewriteByte(
      FT857_CW_BFO_ADDR,
      [bfo](uint8_t old, uint8_t& out) {
        Ft857CwBfo stored = bfo;
        if (!ft857CwBfoFromByte(old, stored)) return false;
        out = ft857CwBfoByte(bfo, old);
        return true;
      },
      b, timeoutMs);
  if (written == YaesuEepromWrite::Ok) (void)ft857CwBfoFromByte(b, bfoOut);
  return written;
}
