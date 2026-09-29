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
  const uint16_t addr = currentProfileVariantIs("ft817") ? 0x007A : 0x008D;
  uint8_t flags = 0;
  if (!yaesuCatReadEepromByte(addr, flags, timeoutMs)) return false;
  onOut = (flags & 0x80) != 0;
  return true;
}

// FT-857/897 settings that CAT can only read from the EEPROM. Addresses from the yo3ggx FT8x7EE
// map (https://www.yo3ggx.ro/ft8x7ee/eeprom.html), which Hamlib uses too.
static bool ft857ReadBit(uint16_t addr, uint8_t mask, bool& onOut, uint32_t timeoutMs) {
  if (!currentProfileVariantIs("ft857_897")) return false;
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

// AGC off is its own bit (0xA8 bit 5); the speed is 0x6A bits 1..0.
bool yaesuFt857QueryAgc(YaesuAgc& out, uint32_t timeoutMs) {
  bool on = false;
  if (!ft857ReadBit(0x00A8, 0x20, on, timeoutMs)) return false;
  if (!on) {
    out = YaesuAgc::Off;
    return true;
  }
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(0x006A, b, timeoutMs)) return false;
  switch (b & 0x03) {
    case 0x00: out = YaesuAgc::Slow; break;
    case 0x02: out = YaesuAgc::Fast; break;
    default: out = YaesuAgc::Auto; break;
  }
  return true;
}

// Each band keeps a 28-byte block per VFO: +1 bit 3 = NAR, +2 bit 5 = IPO, +2 bit 4 = ATT,
// +12..+15 = the band's last frequency in 10 Hz units, big-endian. VFO B's blocks follow VFO A's.
// Block addresses measured on an FT-897; they differ from the yo3ggx map only for 5 MHz, which
// that map puts at 0x25E (on the FT-897 the general coverage block).
static constexpr uint16_t kFt857VfoBBlockOffset = 0x01C0;

struct Ft857BandSlot {
  uint32_t lowKhz;
  uint32_t highKhz;
  uint16_t block;   // VFO A
  bool hasIpoAtt;   // IPO and ATT exist on HF and 6 m only
};

// Only the amateur bands: the band edges the radio uses outside them are not known.
static constexpr Ft857BandSlot kFt857BandSlots[] = {
  {1800, 2000, 0x00BA, true},     {3500, 4000, 0x00D6, true},     {5250, 5450, 0x00F2, true},
  {7000, 7300, 0x010E, true},     {10100, 10150, 0x012A, true},   {14000, 14350, 0x0146, true},
  {18068, 18168, 0x0162, true},   {21000, 21450, 0x017E, true},   {24890, 24990, 0x019A, true},
  {28000, 29700, 0x01B6, true},   {50000, 54000, 0x01D2, true},   {144000, 148000, 0x0226, false},
  {430000, 450000, 0x0242, false},
};

// 0x68 bit 0 is the active VFO (1 = B). It follows both the front panel A/B key and the CAT
// toggle.
bool yaesuFt857QueryVfoB(bool& vfoBOut, uint32_t timeoutMs) {
  return ft857ReadBit(0x0068, 0x01, vfoBOut, timeoutMs);
}

static bool ft857ActiveBandBlock(const Ft857BandSlot& slot, uint16_t& blockOut, uint32_t timeoutMs) {
  bool vfoB = false;
  if (!yaesuFt857QueryVfoB(vfoB, timeoutMs)) return false;
  blockOut = vfoB ? slot.block + kFt857VfoBBlockOffset : slot.block;
  return true;
}

bool yaesuFt857QueryBandFlags(uint64_t hz, YaesuFt857BandFlags& out, uint32_t timeoutMs) {
  if (!currentProfileVariantIs("ft857_897")) return false;
  out = YaesuFt857BandFlags();
  const uint32_t khz = (uint32_t)(hz / 1000ULL);
  for (const Ft857BandSlot& slot : kFt857BandSlots) {
    if (khz < slot.lowKhz || khz > slot.highKhz) continue;
    uint16_t block = slot.block;
    if (!ft857ActiveBandBlock(slot, block, timeoutMs)) return false;
    uint8_t nar = 0;
    uint8_t ipoAtt = 0;
    if (!yaesuCatReadEepromByte(block + 1, nar, timeoutMs)) return false;
    if (slot.hasIpoAtt && !yaesuCatReadEepromByte(block + 2, ipoAtt, timeoutMs)) return false;
    out.bandKnown = true;
    out.hasIpoAtt = slot.hasIpoAtt;
    out.nar = (nar & 0x08) != 0;
    out.ipo = slot.hasIpoAtt && (ipoAtt & 0x20) != 0;
    out.att = slot.hasIpoAtt && (ipoAtt & 0x10) != 0;
    return true;
  }
  return true;
}

// Menu 75 keeps one maximum power per band group. Bits 6..0 are the watts; bit 7 is set from
// 20 W up (100 W is 0xE4).
bool yaesuFt857QueryRfPowerWatts(uint64_t hz, uint8_t& wattsOut, uint32_t timeoutMs) {
  if (!currentProfileVariantIs("ft857_897")) return false;
  uint16_t addr = 0x009B;                  // HF
  if (hz >= 420000000ULL) addr = 0x00AC;   // UHF
  else if (hz >= 76000000ULL) addr = 0x00AB;  // VHF
  else if (hz >= 33000000ULL) addr = 0x00AA;  // 6 m
  uint8_t b = 0;
  if (!yaesuCatReadEepromByte(addr, b, timeoutMs)) return false;
  wattsOut = b & 0x7F;
  return true;
}

// 0x88 = menu item, 0x89 = soft key row, both from 0. Measured on an FT-897: the radio writes
// them when its menu is exited, not while the rows are stepped.
bool yaesuFt857QueryMenuAndRow(uint8_t& menuOut, uint8_t& rowOut, uint32_t timeoutMs) {
  if (!currentProfileVariantIs("ft857_897")) return false;
  uint8_t word[2] = {0};
  if (!yaesuCatReadEepromWord(0x0088, word, timeoutMs)) return false;
  menuOut = (uint8_t)(word[0] + 1);
  rowOut = (uint8_t)(word[1] + 1);
  return true;
}

// Measured on an FT-897 by changing one setting at a time (menu levels at both ends of their
// range). Lock and fast tuning are stored inverted.
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

struct Ft857LevelField {
  YaesuFt857Level level;
  uint16_t addr;
  uint8_t mask;
  uint8_t shift;
};

static constexpr Ft857LevelField kFt857LevelFields[] = {
  {YaesuFt857Level::CwSpeed, 0x0075, 0x3F, 0},    {YaesuFt857Level::AmMicGain, 0x007B, 0x7F, 0},
  {YaesuFt857Level::DigGain, 0x007D, 0x7F, 0},    {YaesuFt857Level::DigVox, 0x0090, 0x7F, 0},
  {YaesuFt857Level::BpfWidth, 0x0093, 0x0C, 2},   {YaesuFt857Level::HpfCutoff, 0x0095, 0x0F, 0},
  {YaesuFt857Level::LpfCutoff, 0x0094, 0x1F, 0},  {YaesuFt857Level::NrLevel, 0x0093, 0xF0, 4},
  {YaesuFt857Level::FmMicGain, 0x007C, 0x7F, 0},  {YaesuFt857Level::NbLevel, 0x0099, 0x7F, 0},
  {YaesuFt857Level::Pkt1200, 0x007E, 0x7F, 0},    {YaesuFt857Level::Pkt9600, 0x007F, 0x7F, 0},
  {YaesuFt857Level::ProcLevel, 0x009A, 0x7F, 0},  {YaesuFt857Level::SsbMicGain, 0x007A, 0x7F, 0},
  {YaesuFt857Level::VoxDelay, 0x0077, 0xFF, 0},   {YaesuFt857Level::VoxGain, 0x0076, 0x7F, 0},
};

bool yaesuFt857QueryLevel(YaesuFt857Level level, uint16_t& valueOut, uint32_t timeoutMs) {
  if (!currentProfileVariantIs("ft857_897")) return false;
  for (const Ft857LevelField& f : kFt857LevelFields) {
    if (f.level != level) continue;
    uint8_t b = 0;
    if (!yaesuCatReadEepromByte(f.addr, b, timeoutMs)) return false;
    const uint16_t raw = (uint16_t)((b & f.mask) >> f.shift);
    switch (level) {
      case YaesuFt857Level::CwSpeed: valueOut = raw + 4; break;       // 4..60 WPM
      case YaesuFt857Level::BpfWidth: valueOut = 60 << raw; break;    // 60 / 120 / 240 Hz
      case YaesuFt857Level::HpfCutoff: valueOut = 100 + 60 * raw; break;  // 100..1000 Hz
      // 1000..6000 Hz in 32 steps; only the ends were measured, the radio's own steps in
      // between may differ from this straight line by a few Hz.
      case YaesuFt857Level::LpfCutoff: valueOut = (uint16_t)(((1000 + raw * 5000 / 31) + 5) / 10 * 10); break;
      case YaesuFt857Level::NrLevel: valueOut = raw + 1; break;       // 1..16
      case YaesuFt857Level::VoxDelay: valueOut = raw * 100; break;    // 100..3000 ms
      default: valueOut = raw; break;                                 // 0..100
    }
    return true;
  }
  return false;
}

// Band block +10..+11: signed, 10 Hz units (0xFFE1 = -310 Hz).
bool yaesuFt857QueryClarifierOffsetHz(uint64_t hz, int32_t& offsetOut, uint32_t timeoutMs) {
  if (!currentProfileVariantIs("ft857_897")) return false;
  const uint32_t khz = (uint32_t)(hz / 1000ULL);
  for (const Ft857BandSlot& slot : kFt857BandSlots) {
    if (khz < slot.lowKhz || khz > slot.highKhz) continue;
    uint16_t block = slot.block;
    if (!ft857ActiveBandBlock(slot, block, timeoutMs)) return false;
    uint8_t word[2] = {0};
    if (!yaesuCatReadEepromWord(block + 10, word, timeoutMs)) return false;
    offsetOut = (int32_t)(int16_t)(((uint16_t)word[0] << 8) | word[1]) * 10;
    return true;
  }
  return false;
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
