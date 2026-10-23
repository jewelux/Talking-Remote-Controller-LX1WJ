#pragma once

// Helpers shared by the radio_features*.cpp files; not for other callers.

#include "protocol_ft8x7_eeprom.h"
#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_features.h"
#include "radio_globals.h"

// A radio exchange that went wrong: a timeout when the radio said nothing.
inline FeatureStatus failure(FeatureStatus status) {
  return g_radioReplyTimedOut ? FeatureStatus::Timeout : status;
}

inline constexpr uint32_t kFt8x7TimeoutMs = YAESU_CAT_REPLY_TIMEOUT_MS;

inline FeatureStatus eepromWriteStatus(YaesuEepromWrite result) {
  switch (result) {
    case YaesuEepromWrite::Ok: return FeatureStatus::Ok;
    case YaesuEepromWrite::Unsupported: return FeatureStatus::Unsupported;
    case YaesuEepromWrite::OutOfRange: return FeatureStatus::Invalid;
    case YaesuEepromWrite::NoReply: return failure(FeatureStatus::NoReply);
    case YaesuEepromWrite::UnexpectedBytes: return FeatureStatus::Unexpected;
    case YaesuEepromWrite::Transmitting:
    case YaesuEepromWrite::NotVerified: return FeatureStatus::Failed;
  }
  return FeatureStatus::Failed;
}

inline FeatureStatus eepromReadStatus(YaesuEepromRead result) {
  switch (result) {
    case YaesuEepromRead::Ok: return FeatureStatus::Ok;
    case YaesuEepromRead::Unsupported: return FeatureStatus::Unsupported;
    case YaesuEepromRead::NoReply: return failure(FeatureStatus::NoReply);
    case YaesuEepromRead::UnexpectedBytes: return FeatureStatus::Unexpected;
  }
  return FeatureStatus::NoReply;
}

inline bool isFt8x7() {
  return currentProtocolType() == PROTO_YAESU_FT8X7;
}

inline bool isCiv() {
  return currentProtocolType() == PROTO_CIV;
}

// Kenwood, Elecraft or the Yaesu FTDX family; the profile's caps and settings say which commands
// the radio has.
inline bool isAsciiRadio() {
  return isAsciiProtocol(currentProtocolType());
}

// FT-8x7: flips flag from the EEPROM byte read for the write. on is the flag as written.
inline FeatureStatus ft8x7ToggleFlag(Ft8x7Flag flag, bool& on) {
  return eepromWriteStatus(yaesuFt8x7ToggleFlag(flag, on, kFt8x7TimeoutMs));
}
