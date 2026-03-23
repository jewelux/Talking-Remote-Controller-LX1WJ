#include "radio_catalog.h"
#include "radio_globals.h"
#include "protocol_ops_ascii.h"
#include "protocol_ops_civ.h"
#include "protocol_ops_yaesu.h"
#include "radio_protocol.h"

bool queryFrequency(uint64_t& hzOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryFrequency(sp, hzOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryFrequency(sp, hzOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQueryFrequency(sp, hzOut, timeoutMs);
  return false;
}

bool setFrequency(uint64_t hz) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetFrequency(sp, hz);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetFrequency(sp, hz);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatSetFrequency(sp, hz);
  return false;
}

bool queryMode(uint8_t& modeOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryMode(sp, modeOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryMode(sp, modeOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQueryMode(sp, modeOut, timeoutMs);
  return false;
}

bool setMode(uint8_t mode, uint8_t filter) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetMode(sp, mode, filter);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetMode(sp, mode);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatSetMode(sp, mode);
  return false;
}

bool querySMeterRaw(int32_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQuerySMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQuerySMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQuerySMeterRaw(sp, rawOut, timeoutMs);
  return false;
}

bool queryPoMeterRaw(int32_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryPoMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryPoMeterRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQueryPoMeterRaw(sp, rawOut, timeoutMs);
  return false;
}

bool querySWRRaw(int32_t& rawOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQuerySWRRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQuerySWRRaw(sp, rawOut, timeoutMs);
  if (pt == PROTO_YAESU_FT8X7) return yaesuCatQuerySWRRaw(sp, rawOut, timeoutMs);
  return false;
}

bool queryNr(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNr(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryNr(sp, onOut, timeoutMs);
  return false;
}

bool setNr(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNr(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetNr(sp, on);
  return false;
}

bool queryNb(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNb(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryNb(sp, onOut, timeoutMs);
  return false;
}

bool setNb(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNb(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetNb(sp, on);
  return false;
}

bool queryNotch(bool& onOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNotch(sp, onOut, timeoutMs);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiQueryNotch(sp, onOut, timeoutMs);
  return false;
}

bool setNotch(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNotch(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetNotch(sp, on);
  return false;
}

bool queryNotchWidth(NotchWidth& widthOut, uint32_t timeoutMs) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civQueryNotchWidth(sp, widthOut, timeoutMs);
  return false;
}

bool setNotchWidth(NotchWidth width) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetNotchWidth(sp, width);
  return false;
}
