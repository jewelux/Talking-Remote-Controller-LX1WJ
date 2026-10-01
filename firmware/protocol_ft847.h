#pragma once

// Yaesu FT-847 CAT (PROTO_YAESU_FT847). Frame fields are in ft847_codec.h; the byte transport
// (flush, read with timeout, late-reply handling) is shared with the FT-8x7 in
// protocol_yaesu_cat.h. Nothing here sends an FT-8x7 opcode.
//
// The radio ignores every command until it has seen CAT ON. CAT ON is sent automatically
// before the first command after the line is opened, and again after a command the radio did
// not answer (it may have been switched off and on).

#include <Arduino.h>

#include "ft847_codec.h"
#include "radio_globals.h"

// Frequency polling (radio_monitor.cpp). Every frame takes 4 byte gaps, so polls are slower
// than on the FT-8x7.
static constexpr uint32_t FREQ_POLL_MS_FT847 = 1000;
static constexpr uint32_t FREQ_POLL_TIMEOUT_MS_FT847 = 500;

// Pause between the five bytes of a frame. The manual allows up to 200 ms; Hamlib uses 50 ms
// (write_delay), so start there. F847GAP on the console changes it for testing.
static constexpr uint32_t FT847_DEFAULT_BYTE_GAP_MS = 50;
// After a command the radio does not answer, before the next one may follow.
static constexpr uint32_t FT847_WRITE_SETTLE_MS = 60;
// After a frequency or mode write, before reading it back.
static constexpr uint32_t FT847_FREQ_MODE_SETTLE_MS = 140;
// After the UART is (re)opened, so the radio drops a stray byte from the line change.
static constexpr uint32_t FT847_LINE_OPEN_HOLDOFF_MS = 100;

// Call after (re)opening the UART for an FT-847 profile.
void ft847NoteLineOpened();

uint32_t ft847ByteGapMs();
void ft847SetByteGapMs(uint32_t ms);

// CAT ON / CAT OFF by hand (console). After CAT OFF no command is sent until CAT ON.
void ft847CatOn();
void ft847CatOff();
bool ft847CatHeldOff();

// Raw frames for the console. The query forms read a 1- or 5-byte reply.
void ft847SendRaw(const uint8_t frame[5]);
bool ft847QueryRaw1(const uint8_t frame[5], uint8_t& rsp, uint32_t timeoutMs);
bool ft847QueryRaw5(const uint8_t frame[5], uint8_t rsp[5], uint32_t timeoutMs);

bool ft847QueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs);
bool ft847SetFrequency(const StoredProfile& sp, uint64_t hz);
bool ft847QueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs);
bool ft847SetMode(const StoredProfile& sp, uint8_t mode);
// The mode byte as the radio reports it, narrow flag included.
bool ft847QueryModeByte(uint8_t& modeByteOut, uint32_t timeoutMs);
bool ft847QueryRxStatus(uint8_t& rxStatusOut, uint32_t timeoutMs);
bool ft847QueryTxStatus(uint8_t& txStatusOut, uint32_t timeoutMs);
bool ft847QuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs);
SMeterReading ft847SMeterFromRaw(uint8_t rxStatus);
bool ft847QueryRxTx(const StoredProfile& sp, bool& txOut, uint32_t timeoutMs);
