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
bool yaesuDcsCodeValid(uint16_t dcsCode);
// The data bytes of a CTCSS tone or DCS code command: the value as four BCD digits (88.5 Hz ->
// 08 85), then the same again when the radio takes separate TX and RX values (FT-857/897), else
// 00 00 (FT-817).
void yaesuEncodeToneData(uint16_t value, bool separateTxRx, uint8_t data[4]);
