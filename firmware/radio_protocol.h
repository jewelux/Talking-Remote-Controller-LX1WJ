#pragma once

#include "radio_globals.h"

// Whether the current protocol implements the feature at all. When false, the
// matching query/set functions below fail without talking to the radio.
bool protocolSupportsTuner();
bool protocolSupportsMonitor();
bool protocolSupportsTransceive();
bool protocolSupportsBandStack();
bool protocolSupportsRit();

bool queryFrequency(uint64_t& hzOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setFrequency(uint64_t hz);
bool queryMode(uint8_t& modeOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setMode(uint8_t mode, uint8_t filter = 1);
// Whether setMode() can set mode on the current profile, without asking the
// radio.
bool canSetMode(uint8_t mode);
bool querySMeterRaw(int32_t& rawOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
// Converts a querySMeterRaw() value of the current protocol into S units / dB over S9.
SMeterReading sMeterFromRaw(int32_t raw);
bool queryPoMeterRaw(int32_t& rawOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool querySWRRaw(int32_t& rawOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool queryNr(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setNr(bool on);
bool queryNrLevel(uint16_t& valueOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setNrLevel(uint16_t value);
bool queryNb(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setNb(bool on);
bool queryNbLevel(uint16_t& valueOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setNbLevel(uint16_t value);
bool queryNotch(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setNotch(bool on);
bool queryNotchWidth(NotchWidth& widthOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setNotchWidth(NotchWidth width);
bool queryPbtInner(uint16_t& valueOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setPbtInner(uint16_t value);
bool queryPbtOuter(uint16_t& valueOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setPbtOuter(uint16_t value);
// Reads the active VFO from the radio; FT-857/897 only (get_vfo).
bool queryActiveVfo(bool& vfoAOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool queryDialLock(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setDialLock(bool on);
bool queryFilterShape(bool& softOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setFilterShape(bool soft);
bool queryMonitorEnabled(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setMonitorEnabled(bool on);
bool queryMonitorLevel(uint16_t& valueOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setMonitorLevel(uint16_t value);
bool queryTransceiveEnabled(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setTransceiveEnabled(bool on);
bool queryBandStackEntry(uint8_t bandCode, uint8_t registerCode, BandStackEntry& entryOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool queryTuner(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setTuner(bool on);
bool startTune();
bool queryRxTxStatus(bool& txOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool queryTxFrequency(uint64_t& hzOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool selectVfoA();
bool selectVfoB();
bool queryVfoFrequency(bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setVfoFrequency(bool targetVfoA, uint64_t hz);
bool queryVfoMode(bool targetVfoA, uint8_t& modeOut, uint8_t& filterOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setVfoMode(bool targetVfoA, uint8_t mode, uint8_t filter = 1);
// Whether setVfoFrequency / setVfoMode can work at all on this profile.
bool canSetVfoFrequency(bool targetVfoA);
bool canSetVfoMode();
// FT-8x7: the radio's A=B. Copies the active VFO's frequency and mode to the
// other VFO by switching to it and back; the active VFO stays active.
bool ft8x7CopyActiveVfoToOther();
// FT-8x7: reads or sets the other VFO's frequency by switching to it and back.
bool ft8x7QueryOtherVfoFrequency(uint64_t& hzOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool ft8x7SetOtherVfoFrequency(uint64_t hz);
bool querySplit(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setSplit(bool on);
bool queryRitEnabled(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setRitEnabled(bool on);
bool toggleRitEnabled(bool& onOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool queryRitOffsetHz(int32_t& hzOut, uint32_t timeoutMs = kCatReplyTimeoutMs);
bool setRitOffsetHz(int32_t hz);
