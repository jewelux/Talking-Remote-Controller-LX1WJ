#pragma once

// QRP Labs QMX CAT replies that are not on the TS-480 scale, without Arduino so the host tests
// can check them (tests/qmx).
//
// S-meter: the CAT manual (firmware 1_04_003) says only that SM; "returns the S-meter value in
// dB". The operating manual (firmware 1_02_000a, S-meter display sub-menu) calibrates the meter
// at 6 dB per S unit, from S0 at -127 dBm to S9+36 dB at -37 dBm, so S9 is -73 dBm. The reply
// has not been seen from a radio yet, so both readings are taken: a negative value as dBm, zero
// and above as dB over S0.

#include <stddef.h>
#include <stdint.h>

// S units as whole units below S9, as tens of dB above it (S9+36 is S9+30), up to S9+60.
void qmxDecodeSMeter(int32_t db, uint8_t& sUnitsOut, uint8_t& dbOverS9Out);

// Output power (PC, tenths of a watt: PC45; is 4.5 W) and SWR (SW, hundredths: SW121; is 1.21)
// are measured only while transmitting. In receive the CAT manual says SW; comes back bare, and
// Hamlib's QMX backend takes a bare PC; the same way.
enum class QmxTxMeter : uint8_t { Reading, Receive, Invalid };

// body: the reply after its two-letter prefix, with or without the closing ';'.
QmxTxMeter qmxParseTxMeter(const char* body, int32_t& valueOut);

// value / 10^decimals as text, "4.5" for (45, 1) and "1.21" for (121, 2); decimals at least 1.
void qmxFormatFixed(uint32_t value, uint8_t decimals, char* out, size_t outLen);
