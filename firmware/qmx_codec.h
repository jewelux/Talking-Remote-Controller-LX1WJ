#pragma once

// QRP Labs QMX CAT replies that are not on the TS-480 scale, without Arduino so the host tests
// can check them (tests/qmx).
//
// S-meter: the CAT manual (firmware 1_04_003) says only that SM; "returns the S-meter value in
// dB". The operating manual (firmware 1_02_000a, S-meter display sub-menu) calibrates the meter
// at 6 dB per S unit, from S0 at -127 dBm to S9+36 dB at -37 dBm, so S9 is -73 dBm. The reply
// has not been seen from a radio yet, so both readings are taken: a negative value as dBm, zero
// and above as dB over S0.

#include <stdint.h>

// S units as whole units below S9, as tens of dB above it (S9+36 is S9+30), up to S9+60.
void qmxDecodeSMeter(int32_t db, uint8_t& sUnitsOut, uint8_t& dbOverS9Out);
