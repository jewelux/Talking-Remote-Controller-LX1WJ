#pragma once

#include "radio_globals.h"

bool asciiQueryFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool asciiSetFrequency(const RadioProfile& sp, uint64_t hz);
bool asciiQueryMode(const RadioProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool asciiSetMode(const RadioProfile& sp, uint8_t mode);
bool asciiQuerySMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQueryPoMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQuerySWRRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool asciiQueryStatusLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryIdLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs);
bool asciiQueryOmLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs);
// The preamp level (PA, 0..settings.preampLevels) and the attenuator step (RA,
// 0..settings.attLevels) by settings.frontEnd (caps.getPreamp, setPreamp, getAtt, setAtt); the
// sets are read back.
bool asciiQueryPreampLevel(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
bool asciiSetPreampLevel(const RadioProfile& sp, uint8_t level);
bool asciiQueryAttStep(const RadioProfile& sp, uint8_t& stepOut, uint32_t timeoutMs);
bool asciiSetAttStep(const RadioProfile& sp, uint8_t step);
// The RF power by settings.rfPower (caps.getRfPower, setRfPower), in tenths of a watt; the set,
// in watts, is read back.
bool asciiQueryRfPowerTenths(const RadioProfile& sp, uint16_t& tenthsOut, uint32_t timeoutMs);
bool asciiSetRfPowerWatts(const RadioProfile& sp, uint16_t watts);
// Yaesu BI (break-in, caps.getBreakIn, setBreakIn) and KR (keyer, caps.getKeyer, setKeyer), on
// or off; the sets are read back.
bool asciiQueryBreakIn(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetBreakIn(const RadioProfile& sp, bool on);
bool asciiQueryKeyer(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetKeyer(const RadioProfile& sp, bool on);
// The CW keyer speed (KS, WPM), pitch (Yaesu KP, Hz) and break-in delay (SD, ms) by the
// profile's settings (caps.getCwSpeed ... setCwDelay); the sets are read back, the delay set to
// the radio's step nearest ms.
// Yaesu RM4, the ALC meter, 0..255 (caps.getAlc).
bool asciiQueryAlcRaw(const RadioProfile& sp, uint16_t& rawOut, uint32_t timeoutMs);
bool asciiQueryCwSpeed(const RadioProfile& sp, uint16_t& wpmOut, uint32_t timeoutMs);
bool asciiSetCwSpeed(const RadioProfile& sp, uint16_t wpm);
bool asciiQueryCwPitch(const RadioProfile& sp, uint16_t& hzOut, uint32_t timeoutMs);
bool asciiSetCwPitch(const RadioProfile& sp, uint16_t hz);
bool asciiQueryCwDelayMs(const RadioProfile& sp, uint16_t& msOut, uint32_t timeoutMs);
bool asciiSetCwDelayMs(const RadioProfile& sp, uint16_t ms);
// AGC by settings.agc (caps.getAgc, setAgc); the set is read back. AUTO reads back as AUTO.
bool asciiQueryAgc(const RadioProfile& sp, AgcMode& out, uint32_t timeoutMs);
bool asciiSetAgc(const RadioProfile& sp, AgcMode mode);
bool asciiQueryPowerState(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetPowerState(const RadioProfile& sp, bool on);
bool asciiQueryTuner(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetTuner(const RadioProfile& sp, bool on);
bool asciiStartTune(const RadioProfile& sp);
bool asciiQuerySplit(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetSplit(const RadioProfile& sp, bool on);
bool asciiQueryNr(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNr(const RadioProfile& sp, bool on);
bool asciiQueryNb(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNb(const RadioProfile& sp, bool on);
bool asciiQueryNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetNotch(const RadioProfile& sp, bool on);
bool asciiQueryLock(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiSetLock(const RadioProfile& sp, bool on);
bool asciiQueryYaesuRadioInfoFlag(const RadioProfile& sp, const char* code, bool& onOut, uint32_t timeoutMs);
// Speech processor (Yaesu PR0, TS-480 PR) and its level (PL, 0..100; on the TS-480 the input
// level), mic gain (MG, 0..100), VOX (VX), its gain (VG, 0..settings.voxGainMax) and delay (VD,
// in ms, as settings.voxDelay says): the Yaesu ASCII radios and the TS-480.
bool asciiQueryProc(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiQueryProcLevel(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
bool asciiQueryMicGain(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
bool asciiQueryVox(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool asciiQueryVoxGain(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
bool asciiQueryVoxDelayMs(const RadioProfile& sp, uint16_t& msOut, uint32_t timeoutMs);
// The writes of the above, each read back: caps.setProc, setMicGain, setVox. The VOX delay goes
// to the radio's step nearest ms. Not yet tested on a radio.
bool asciiSetProc(const RadioProfile& sp, bool on);
bool asciiSetProcLevel(const RadioProfile& sp, uint8_t level);
bool asciiSetMicGain(const RadioProfile& sp, uint8_t level);
bool asciiSetVox(const RadioProfile& sp, bool on);
bool asciiSetVoxGain(const RadioProfile& sp, uint8_t level);
bool asciiSetVoxDelayMs(const RadioProfile& sp, uint16_t ms);
bool asciiQueryActiveVfoA(const RadioProfile& sp, bool& vfoAOut, uint32_t timeoutMs);
bool asciiSelectVfoA(const RadioProfile& sp);
bool asciiSelectVfoB(const RadioProfile& sp);
bool asciiSwapVfo(const RadioProfile& sp);
bool asciiQueryVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs);
bool asciiSetVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t hz);
