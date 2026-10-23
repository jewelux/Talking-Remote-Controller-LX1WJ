#pragma once

// FT-817/818/857/897 CAT frame fields, without Arduino so the host tests can check them
// (tests/ft8x7).

#include <stddef.h>
#include <stdint.h>

// FT-817/857/897 coverage bounds; decoded frequencies outside indicate a misaligned frame.
static constexpr uint64_t YAESU_CAT_MIN_FREQ_HZ = 100000ULL;
static constexpr uint64_t YAESU_CAT_MAX_FREQ_HZ = 470000000ULL;

// Eight BCD digits of 10 Hz units, most significant first.
uint64_t yaesuCatDecodeFreqHz(const uint8_t data[4]);
// False when a digit is not BCD or the frequency is outside the coverage bounds.
bool yaesuCatFreqFieldValid(const uint8_t data[4]);
void yaesuCatEncodeFreqHz(uint64_t hz, uint8_t out[4]);
void yaesuCatEncodeRepeaterOffsetHz(uint64_t hz, uint8_t out[4]);

// RX status (0xE7) bits 3..0: 0x0..0x9 = S0..S9, 0xA..0xF = S9+10..S9+60 dB.
void yaesuCatDecodeSMeterLevel(uint8_t rxStatus, uint8_t& sUnitsOut, uint8_t& dbOverS9Out);

// TX status (0xF7): bit 7 = PTT (0 = transmitting), bit 6 = high SWR, bit 5 = split (1 = on,
// measured on an FT-897; the manuals say 0 = on), bits 3..0 = PO meter.
bool yaesuCatTxStatusTransmitting(uint8_t txStatus);

// SWR for a 0..15 meter reading of 0xBD.
float yaesuSwrFromMeter(uint8_t bars);

// FT-8x7 CTCSS tone in tenths of Hz (885 = 88.5 Hz) and DCS code (23 = 023).
// Only the standard values are valid.
bool yaesuCtcssTenthsValid(uint16_t toneTenths);
// The 50 tones in order, 0 = 67.0 Hz .. 49 = 254.1 Hz: the Yaesu CN tone number (the FTDX10,
// FTDX101 and FT-891 CAT manuals' table). False outside the table.
bool yaesuCtcssToneIndex(uint16_t toneTenths, uint8_t& indexOut);
bool yaesuCtcssToneAt(uint8_t index, uint16_t& toneTenthsOut);
bool yaesuDcsCodeValid(uint16_t dcsCode);
// The 104 codes in order, 0 = 023 .. 103 = 754. False outside the table.
bool yaesuDcsCodeIndex(uint16_t dcsCode, uint8_t& indexOut);
bool yaesuDcsCodeAt(uint8_t index, uint16_t& dcsCodeOut);
// The standard tone or code next above (dir 1) or below (dir -1) the value, which need not be
// one itself. The value as it is when there is none past it.
uint16_t yaesuCtcssToneStep(uint16_t toneTenths, int8_t dir);
uint16_t yaesuDcsCodeStep(uint16_t dcsCode, int8_t dir);
// The data bytes of a CTCSS tone or DCS code command: the value as four BCD digits (88.5 Hz ->
// 08 85), then the same again when the radio takes separate TX and RX values (FT-857/897), else
// 00 00 (FT-817).
void yaesuEncodeToneData(uint16_t value, bool separateTxRx, uint8_t data[4]);

// Tone squelch: what is sent (tx) and what the squelch waits for (rx). Both the same type is
// the usual tone squelch, tx alone is encode only, rx alone decode only.
enum class YaesuToneType : uint8_t { None, Ctcss, Dcs };
struct YaesuToneMode {
  YaesuToneType tx = YaesuToneType::None;
  YaesuToneType rx = YaesuToneType::None;

  bool isOff() const { return tx == YaesuToneType::None && rx == YaesuToneType::None; }
  bool txOnly() const { return tx != YaesuToneType::None && rx == YaesuToneType::None; }
  bool rxOnly() const { return tx == YaesuToneType::None && rx != YaesuToneType::None; }
  // One type sent, the other awaited (FT-857/897 split tone).
  bool mixed() const { return tx != YaesuToneType::None && rx != YaesuToneType::None && tx != rx; }
  // The sent type, or the awaited one when none is sent.
  YaesuToneType type() const { return tx == YaesuToneType::None ? rx : tx; }
};
// The mode byte of the tone mode command (opcode 0x0A): 8A off, 2A CTCSS, 4A CTCSS encode, 3A
// CTCSS decode, 0A DCS, 0C DCS encode, 0B DCS decode. False for a mode CAT cannot set: tx and rx
// of different types, and without full (the FT-817/818) decode only and DCS encode only.
bool yaesuToneModeByte(YaesuToneMode mode, bool full, uint8_t& out);
// The mode a tone mode command byte sets. False, and out untouched, for any other byte.
bool yaesuToneModeFromByte(uint8_t b, YaesuToneMode& out);
