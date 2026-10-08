#pragma once

// Yaesu FT-847 CAT frame fields, without Arduino so the host tests can check them
// (tests/ft847).
//
// The FT-847 uses the same 5-byte frame as the FT-817/857/897 (four parameter bytes, then the
// opcode), but several opcodes mean something else, so it is its own protocol
// (PROTO_YAESU_FT847) and never runs FT-8x7 code. Notable differences:
//   - 0x00 / 0x80 are CAT ON / CAT OFF (lock on/off on the FT-8x7). The radio ignores every
//     other command until it has seen CAT ON.
//   - 0x03 / 0x13 / 0x23 read frequency and mode of the main, satellite RX and satellite TX VFO
//     (0x13 is a 1-byte meter read on the FT-8x7).
//   - the RX status (0xE7) S-meter is 5 bits (0..31 display dots), not 4.
//   - no lock, VFO A/B, clarifier, power on/off or EEPROM commands.
//   - units built before the 8G05 production run (May 1998) cannot be read at all.
// Sources: FT-847 operating manual pp. 91-93 (CAT System Programming) and Hamlib
// rigs/yaesu/ft847.c. The manual's status bit charts on p. 92 have their titles swapped:
// the chart with S-meter and squelch bits is the receiver status (0xE7), the one with PTT and
// PO/ALC the transmit status (0xF7). Hamlib reads them that way, as does this file.

#include <stddef.h>
#include <stdint.h>

// Opcodes (byte 4 of the frame). The VFO-targeted ones are for the main VFO; add 0x10 for
// satellite RX and 0x20 for satellite TX.
static constexpr uint8_t FT847_OP_CAT_ON = 0x00;
static constexpr uint8_t FT847_OP_CAT_OFF = 0x80;
static constexpr uint8_t FT847_OP_SET_FREQ = 0x01;
static constexpr uint8_t FT847_OP_READ_FREQ_MODE = 0x03;
static constexpr uint8_t FT847_OP_SET_MODE = 0x07;
static constexpr uint8_t FT847_OP_PTT_ON = 0x08;
static constexpr uint8_t FT847_OP_PTT_OFF = 0x88;
static constexpr uint8_t FT847_OP_SAT_ON = 0x4E;
static constexpr uint8_t FT847_OP_SAT_OFF = 0x8E;
static constexpr uint8_t FT847_OP_READ_RX_STATUS = 0xE7;
static constexpr uint8_t FT847_OP_READ_TX_STATUS = 0xF7;

// Mode bytes (byte 0 of a SET_MODE frame, byte 4 of a READ_FREQ_MODE reply). Bit 7 marks the
// narrow filter: 0x82 CW-N, 0x83 CWR-N, 0x84 AM-N, 0x88 FM-N.
static constexpr uint8_t FT847_MODE_LSB = 0x00;
static constexpr uint8_t FT847_MODE_USB = 0x01;
static constexpr uint8_t FT847_MODE_CW = 0x02;
static constexpr uint8_t FT847_MODE_CWR = 0x03;
static constexpr uint8_t FT847_MODE_AM = 0x04;
static constexpr uint8_t FT847_MODE_FM = 0x08;
static constexpr uint8_t FT847_MODE_NARROW_FLAG = 0x80;

// Receive coverage (0.1-30, 36-76, 108-174, 420-512 MHz); a decoded frequency outside it means
// a misaligned frame.
static constexpr uint64_t FT847_MIN_FREQ_HZ = 100000ULL;
static constexpr uint64_t FT847_MAX_FREQ_HZ = 512000000ULL;

// Builds a frame: four parameter bytes, then the opcode.
void ft847BuildFrame(uint8_t opcode, uint8_t frame[5]);

// Eight BCD digits of 10 Hz units, most significant first (same as the FT-8x7).
uint64_t ft847DecodeFreqHz(const uint8_t data[4]);
void ft847EncodeFreqHz(uint64_t hz, uint8_t out[4]);
// False when a digit is not BCD or the frequency is outside the receive coverage.
bool ft847FreqFieldValid(const uint8_t data[4]);

// A SET_FREQ frame for hz (rounded down to 10 Hz).
void ft847BuildSetFreqFrame(uint64_t hz, uint8_t frame[5]);
// A SET_MODE frame for a mode byte.
void ft847BuildSetModeFrame(uint8_t modeByte, uint8_t frame[5]);

// The mode byte without the narrow flag, and whether the flag was set.
uint8_t ft847ModeBase(uint8_t modeByte);
bool ft847ModeNarrow(uint8_t modeByte);
// True for the six mode bases the radio reports (LSB, USB, CW, CWR, AM, FM).
bool ft847ModeBaseKnown(uint8_t modeBase);
// True for the modes HamTRC switches between wide and narrow: CW, CW-R and AM. (FM-N exists too,
// but HamTRC leaves FM alone, and SSB has no narrow mode.) CW-N needs the optional YF-115C filter.
bool ft847ModeCanBeNarrow(uint8_t modeBase);
// The mode byte for modeBase, narrow or wide.
uint8_t ft847ModeByte(uint8_t modeBase, bool narrow);

// RX status (0xE7): bit 7 = squelch closed (1) / open (0), bits 4..0 = S-meter display dots
// 0..31. Dots map to S units the way Hamlib's ft847_get_smeter_level does: 0..3 = S0..S1,
// 3 dots per 6 dB from there to S9 at 19 dots, then 5 dB per dot up to S9+60 at 31.
uint8_t ft847SMeterDots(uint8_t rxStatus);
void ft847DecodeSMeter(uint8_t rxStatus, uint8_t& sUnitsOut, uint8_t& dbOverS9Out);
bool ft847SquelchOpen(uint8_t rxStatus);

// TX status (0xF7): bit 7 = 0 while transmitting, bits 4..0 = PO/ALC meter.
bool ft847TxStatusTransmitting(uint8_t txStatus);
uint8_t ft847TxMeter(uint8_t txStatus);
