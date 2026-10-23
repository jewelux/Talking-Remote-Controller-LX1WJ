#pragma once

#include "radio_globals.h"

bool civQueryFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool civSetFrequency(const RadioProfile& sp, uint64_t hz);
bool civQueryMode(const RadioProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool civSetMode(const RadioProfile& sp, uint8_t mode, uint8_t filter);
bool civQuerySMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool civQueryPoMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool civQuerySWRRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
bool civQueryRfPowerLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
bool civSetRfPowerLevel(const RadioProfile& sp, uint16_t value);
bool civQueryNr(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetNr(const RadioProfile& sp, bool on);
bool civQueryNrLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
bool civSetNrLevel(const RadioProfile& sp, uint16_t value);
bool civQueryNb(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetNb(const RadioProfile& sp, bool on);
bool civQueryNbLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
bool civSetNbLevel(const RadioProfile& sp, uint16_t value);
// Speech compressor (COMP) and its level, 0..255.
bool civQueryProc(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civQueryProcLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
// Mic gain, 0..255.
bool civQueryMicGain(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
// VOX and its gain, 0..255.
bool civQueryVox(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civQueryVoxGain(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
// The VOX delay menu address (1A 05) of the model; false where it is not known.
bool civVoxDelayAddr(RadioModel model, uint16_t& addrOut);
// VOX delay (0.0..2.0 s) of the IC-7300, IC-705 and IC-7760, in ms.
bool civQueryVoxDelayMs(const RadioProfile& sp, uint16_t& msOut, uint32_t timeoutMs);
// The writes of the above, each read back: caps.setProc, setMicGain, setVox. The VOX delay goes
// to the nearest 100 ms, 0..2000.
bool civSetProc(const RadioProfile& sp, bool on);
bool civSetProcLevel(const RadioProfile& sp, uint16_t value);
bool civSetMicGain(const RadioProfile& sp, uint16_t value);
bool civSetVox(const RadioProfile& sp, bool on);
bool civSetVoxGain(const RadioProfile& sp, uint16_t value);
bool civSetVoxDelayMs(const RadioProfile& sp, uint16_t ms);
bool civQueryNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetNotch(const RadioProfile& sp, bool on);
bool civQueryNotchWidth(const RadioProfile& sp, NotchWidth& widthOut, uint32_t timeoutMs);
bool civSetNotchWidth(const RadioProfile& sp, NotchWidth width);
// 16 12, the AGC time constant preset: FAST, MID, SLOW (caps.getAgc, setAgc); the set is read back.
bool civQueryAgc(const RadioProfile& sp, AgcMode& out, uint32_t timeoutMs);
bool civSetAgc(const RadioProfile& sp, AgcMode mode);
// 16 02, the preamp level, 0..settings.preampLevels (caps.getPreamp, setPreamp), and 11, the
// attenuator in dB (caps.getAtt, setAtt; the set takes one of settings.attDb or 0). The sets are
// read back.
bool civQueryPreampLevel(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs);
bool civSetPreampLevel(const RadioProfile& sp, uint8_t level);
bool civQueryAttDb(const RadioProfile& sp, uint8_t& dbOut, uint32_t timeoutMs);
bool civSetAttDb(const RadioProfile& sp, uint8_t db);
// 16 47, CW break-in OFF, SEMI or FULL (caps.getBreakIn, setBreakIn); the set is read back.
bool civQueryBreakIn(const RadioProfile& sp, BreakInMode& out, uint32_t timeoutMs);
bool civSetBreakIn(const RadioProfile& sp, BreakInMode mode);
// 14 09, the CW pitch in Hz, 300..900 (caps.getCwPitch, setCwPitch); the set is read back to
// within the radio's 5 Hz step.
bool civQueryCwPitchHz(const RadioProfile& sp, uint16_t& hzOut, uint32_t timeoutMs);
// 16 41, the auto notch (caps.getAutoNotch, setAutoNotch); the set is read back.
bool civQueryAutoNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetAutoNotch(const RadioProfile& sp, bool on);
// 16 42 (repeater tone, CTCSS encode) and 16 43 (tone squelch, encode and decode), caps.getTone,
// setTone; the set turns the other off first and is read back.
bool civQueryTone(const RadioProfile& sp, bool& toneOut, bool& tsqlOut, uint32_t timeoutMs);
bool civSetTone(const RadioProfile& sp, bool tone, bool tsql);
// 1B 00, the repeater tone in tenths of a Hz (caps.getCtcss); the set writes the tone squelch tone
// (1B 01) the same and reads both back.
bool civQueryCtcssTenths(const RadioProfile& sp, uint16_t& tenthsOut, uint32_t timeoutMs);
bool civSetCtcssTenths(const RadioProfile& sp, uint16_t tenths);
// 0F 10 simplex, 11 DUP-, 12 DUP+ (caps.getRptShift, the IC-705); a split reply (00, 01) is simplex.
bool civQueryRptShift(const RadioProfile& sp, RptShift& out, uint32_t timeoutMs);
bool civSetRptShift(const RadioProfile& sp, RptShift shift);
// 15 13, the ALC meter, 0..120 = 0..100 % (caps.getAlc).
bool civQueryAlcRaw(const RadioProfile& sp, uint16_t& rawOut, uint32_t timeoutMs);
bool civSetCwPitchHz(const RadioProfile& sp, uint16_t hz);
bool civQueryPbtInner(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
bool civSetPbtInner(const RadioProfile& sp, uint16_t value);
bool civQueryPbtOuter(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
bool civSetPbtOuter(const RadioProfile& sp, uint16_t value);
bool civQueryDialLock(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetDialLock(const RadioProfile& sp, bool on);
bool civQueryFilterShape(const RadioProfile& sp, bool& softOut, uint32_t timeoutMs);
bool civSetFilterShape(const RadioProfile& sp, bool soft);
bool civQueryMonitorEnabled(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetMonitorEnabled(const RadioProfile& sp, bool on);
bool civQueryMonitorLevel(const RadioProfile& sp, uint16_t& valueOut, uint32_t timeoutMs);
bool civSetMonitorLevel(const RadioProfile& sp, uint16_t value);
bool civQueryTransceiveEnabled(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetTransceiveEnabled(const RadioProfile& sp, bool on);
bool civQueryBandStackEntry(const RadioProfile& sp, uint8_t bandCode, uint8_t registerCode, BandStackEntry& entryOut, uint32_t timeoutMs);
bool civQueryTuner(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetTuner(const RadioProfile& sp, bool on);
bool civStartTune(const RadioProfile& sp);
bool civQueryRxTxStatus(const RadioProfile& sp, bool& txOut, uint32_t timeoutMs);
bool civQueryTxFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool civSelectVfoA(const RadioProfile& sp);
bool civSelectVfoB(const RadioProfile& sp);
bool civQueryVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs);
bool civSetVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t hz);
bool civQueryVfoMode(const RadioProfile& sp, bool targetVfoA, uint8_t& modeOut, uint8_t& filterOut, uint32_t timeoutMs);
bool civSetVfoMode(const RadioProfile& sp, bool targetVfoA, uint8_t mode, uint8_t filter);
bool civQuerySplit(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetSplit(const RadioProfile& sp, bool on);
bool civQueryRitEnabled(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs);
bool civSetRitEnabled(const RadioProfile& sp, bool on);
bool civQueryRitOffsetHz(const RadioProfile& sp, int32_t& hzOut, uint32_t timeoutMs);
bool civSetRitOffsetHz(const RadioProfile& sp, int32_t hz);
