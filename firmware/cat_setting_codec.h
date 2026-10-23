#pragma once

// How CAT radios encode setting values: CI-V levels, RF power, the ASCII VOX delay, the
// TS-480 processor levels, AGC and the ASCII lock reply. No Arduino.h, so the host tests
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

// An ASCII lock reply digit: on unless it is one of offDigits ("0", or "04" on the FTDX101).
// False when digit is not a digit.
bool asciiLockDigitDecode(char digit, const char* offDigits, bool& onOut);
