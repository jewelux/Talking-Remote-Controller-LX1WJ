#pragma once

// How CAT radios encode setting values: CI-V levels, RF power, the ASCII VOX delay, the
// TS-480 processor levels, AGC, the preamp and attenuator, and the ASCII lock reply. No Arduino.h, so the host tests
// (tests/cat) can build it.

#include <stddef.h>
#include <stdint.h>

#include "ft8x7_eeprom_map.h"
#include "radio_setting_types.h"

// A CI-V 0x14 level (0..255) as 0..100 %, and back.
uint8_t levelRawToPercent(uint16_t raw);
uint16_t levelPercentToRaw(int percent);

// A CI-V RF power level (0..255) as watts on a radio of maxWatts (0 counts as 100), and back.
uint16_t rfPowerRawToWatts(uint16_t raw, uint16_t maxWatts);
uint16_t rfPowerWattsToRaw(int watts, uint16_t maxWatts);

// Yaesu ASCII (newcat) VD answers a step, not ms: 00..05 = 30, 50, 100, 150, 200, 250 ms, then
// 06..33 = 300..3000 ms in 100 ms steps (Hamlib maps it so on the FTDX10, FTDX101 and FT-710).
inline constexpr uint8_t kYaesuAsciiVoxDelaySteps = 34;
uint16_t yaesuAsciiVoxDelayStepMs(uint8_t step);
// The step nearest ms; the lower one on a tie.
uint8_t yaesuAsciiVoxDelayNearestStep(uint16_t ms);
// The ms of the step nearest ms, moved by steps and kept within the table.
uint16_t yaesuAsciiVoxDelayStepBy(uint16_t ms, int steps);

// A VOX delay sent in ms (VDnnnn, FT-891 and TS-480): ms rounded to the nearest multiple of
// stepMs and kept within minMs..maxMs.
uint16_t asciiVoxDelayMsRound(uint16_t ms, uint16_t minMs, uint16_t maxMs, uint16_t stepMs);
// ms moved by steps of stepMs, at least 100 ms each, then rounded and kept as above.
uint16_t asciiVoxDelayMsStepBy(uint16_t ms, int steps, uint16_t minMs, uint16_t maxMs, uint16_t stepMs);

// The TS-480 PL reply after "PL": the processor input level, then its output level, three digits
// and 0..100 each. False on anything else.
bool kenwoodProcLevelsDecode(const char* digits, uint8_t& inputOut, uint8_t& outputOut);

// The FT-8x7's AGC (its EEPROM menu) as an AgcMode, and back; the FT-8x7 has no MID.
AgcMode ft8x7AgcToMode(YaesuAgc agc);
bool ft8x7AgcFromMode(AgcMode mode, YaesuAgc& out);

// Icom 16 12: 01 fast, 02 mid, 03 slow. Hamlib turns AGC off through the time constant
// instead, so OFF is not offered.
bool civAgcDecode(uint8_t raw, AgcMode& out);
bool civAgcEncode(AgcMode mode, uint8_t& rawOut);

// The GT command that reads AGC ("GT0;" on the Yaesus, "GT;" on the others), or nullptr.
const char* asciiAgcReadCommand(AgcEncoding encoding);
// The AGC of a GT reply, given what follows "GT" ("0n;" Yaesu, "nnn;" Kenwood and Elecraft; the
// ';' may be left out). False on anything else.
bool asciiAgcDecode(AgcEncoding encoding, const char* digits, AgcMode& out);
// The GT command that sets mode, into out; false where the radio has no such speed.
bool asciiAgcEncode(AgcEncoding encoding, AgcMode mode, char* out, size_t size);

// Icom 16 02: the preamp level, 00 off, 01 P.AMP1, 02 P.AMP2; false above levels.
bool civPreampDecode(uint8_t raw, uint8_t levels, uint8_t& levelOut);
// Icom 11: the attenuator in dB as two BCD digits, 00 off. False on a byte that is not BCD.
bool civAttDecode(uint8_t raw, uint8_t& dbOut);
// db (0..99) as that byte.
uint8_t civAttEncode(uint8_t db);

// The PA and RA commands that read the preamp and the attenuator, or nullptr.
const char* asciiPreampReadCommand(FrontEndEncoding encoding);
const char* asciiAttReadCommand(FrontEndEncoding encoding);
// The preamp level of a PA reply, given what follows "PA" (the ';' may be left out), 0..levels.
// False on anything else.
bool asciiPreampDecode(FrontEndEncoding encoding, uint8_t levels, const char* digits, uint8_t& levelOut);
// The PA command that sets level (0..levels), into out.
bool asciiPreampEncode(FrontEndEncoding encoding, uint8_t levels, uint8_t level, char* out, size_t size);
// The attenuator step of an RA reply, given what follows "RA", 0..levels.
bool asciiAttDecode(FrontEndEncoding encoding, uint8_t levels, const char* digits, uint8_t& stepOut);
// The RA command that sets step (0..levels), into out.
bool asciiAttEncode(FrontEndEncoding encoding, uint8_t levels, uint8_t step, char* out, size_t size);

// The attenuator step after the one at db (attDb as in SettingEncodings): off goes to step 1, the
// last step to off (0). A db between steps, which a radio's own keys may set, goes to the next
// step above it.
uint8_t attNextStep(const uint8_t* attDb, uint8_t levels, bool on, uint8_t db);
// The step of attDb that is db dB, from 1; 0 where none is.
uint8_t attStepOfDb(const uint8_t* attDb, uint8_t levels, uint8_t db);

// The RF power of a PC reply in tenths of a watt, given what follows "PC": nnn watts, or the
// Elecraft K22 nnnx, x 0 for tenths and 1 for watts (the ';' may be left out). False on anything
// else.
bool asciiRfPowerDecode(const char* digits, uint16_t& tenthsOut);
// "PCnnn;" for watts (0..999), into out.
bool asciiRfPowerEncode(uint16_t watts, char* out, size_t size);

// Icom 16 47: 00 off, 01 semi, 02 full break-in. Encode takes OFF, SEMI and FULL.
bool civBreakInDecode(uint8_t raw, BreakInMode& out);
bool civBreakInEncode(BreakInMode mode, uint8_t& rawOut);
// One digit, 0 off or 1 on, then the end or ';' (Yaesu BI and KR, after the two letters).
bool asciiOnOffDigitDecode(const char* digits, bool& onOut);

// Icom 14 09: 0..255 = 300..900 Hz, linear; the radio keeps 5 Hz steps, so the Hz is rounded
// to 5. The raw value of hz, kept within 300..900.
uint16_t civCwPitchRawToHz(uint16_t raw);
uint16_t civCwPitchHzToRaw(uint16_t hz);
// Yaesu KP: 00..75 = 300..1050 Hz in 10 Hz steps. Decode takes the two digits after "KP"; encode
// the nearest step, kept within the range.
bool yaesuKpDecode(const char* digits, uint16_t& hzOut);
bool yaesuKpEncode(uint16_t hz, char* out, size_t size);
// The value of a reply of exactly n digits after its letters (the ';' may be left out).
bool asciiDigitsDecode(const char* digits, size_t n, uint16_t& valueOut);

// A Yaesu RM reply after "RMn": the meter, three digits 0..255, which the FTDX10 and FTDX101
// follow with three more (a second meter, or 000).
bool yaesuMeterDecode(const char* digits, uint16_t& rawOut);
// raw of fullScale as 0..100 %, rounded; above fullScale is 100.
uint8_t meterPercent(uint16_t raw, uint16_t fullScale);

// An ASCII lock reply digit: on unless it is one of offDigits ("0", or "04" on the FTDX101).
// False when digit is not a digit.
bool asciiLockDigitDecode(char digit, const char* offDigits, bool& onOut);
