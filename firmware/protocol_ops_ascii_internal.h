#pragma once

// Helpers shared by the protocol_ops_ascii*.cpp files; not for other callers.

#include "radio_globals.h"
#include "transport_serial.h"

// Sends cmd as it is; false when there is none.
inline bool asciiSendSimpleCommand(const char* cmd) {
  if (!cmd || !cmd[0]) return false;
  serialTransportFlushInput();
  serialTransportPrint(cmd);
  serialTransportFlushOutput();
  return true;
}

// Sends cmd, then reads the setting back with query: true when the radio has expected.
template <typename T, typename U>
bool asciiSendAndVerify(const RadioProfile& sp, const char* cmd,
                        bool (*query)(const RadioProfile&, T&, uint32_t), const U& expected) {
  if (!asciiSendSimpleCommand(cmd)) return false;
  T readBack{};
  return query(sp, readBack, kCatReplyTimeoutMs) && readBack == expected;
}
