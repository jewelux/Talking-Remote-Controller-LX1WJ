#include "protocol_ascii.h"

#include "packet_ascii.h"
#include "debug_log.h"
#include "ft8x7_model.h"

bool readAsciiLine(String& out, uint32_t timeoutMs) {
  return asciiPacketReadLine(out, timeoutMs);
}

bool transactAsciiCommand(const char* cmd, String& out, const char* expectPrefix, uint32_t timeoutMs) {
  if (!asciiPacketSendCommand(cmd)) return false;
  if (!readAsciiLine(out, timeoutMs)) return false;
  out.trim();
  if (expectPrefix && expectPrefix[0] && !out.startsWith(expectPrefix)) {
    DBG_PRINT("[PROTO] unexpected ASCII reply: ");
    DBG_PRINTLN(out);
    return false;
  }
  return true;
}

bool parseAsciiUnsignedResponse(const String& line, const char* prefix, uint64_t& valueOut) {
  if (!prefix || !prefix[0] || !line.startsWith(prefix)) return false;
  int start = (int)strlen(prefix);
  int end = line.indexOf(';', start);
  if (end < 0) end = line.length();
  String digits = line.substring(start, end);
  digits.trim();
  if (!digits.length()) return false;
  for (size_t i = 0; i < digits.length(); ++i) {
    char c = digits[i];
    if (c < '0' || c > '9') return false;
  }
  valueOut = strtoull(digits.c_str(), nullptr, 10);
  return true;
}

bool parseAsciiSignedResponse(const String& line, const char* prefix, int32_t& valueOut) {
  if (!prefix || !prefix[0] || !line.startsWith(prefix)) return false;
  int start = (int)strlen(prefix);
  int end = line.indexOf(';', start);
  if (end < 0) end = line.length();
  String digits = line.substring(start, end);
  digits.trim();
  if (!digits.length()) return false;
  valueOut = digits.toInt();
  return true;
}

bool profileModeCodeForInternal(const RadioProfile& sp, uint8_t mode, String& codeOut) {
  const char* code = nullptr;
  switch (mode) {
    case 0x00: code = sp.modes->lsb; break;
    case 0x01: code = sp.modes->usb; break;
    case 0x02: code = sp.modes->am; break;
    case 0x03: code = sp.modes->cw; break;
    case 0x04: code = sp.modes->rtty; break;
    case 0x05: code = sp.modes->fm; break;
    case 0x07: code = sp.modes->cwr; break;
    case 0x08: code = sp.modes->rttyR; break;
    case 0x11: code = sp.modes->digi; break;
    case 0x12: code = sp.modes->pkt; break;
    default: break;
  }
  if (!code || !code[0]) return false;
  codeOut = code;
  return true;
}

bool profileInternalModeForCode(const RadioProfile& sp, const String& code, uint8_t& modeOut) {
  if (code.equalsIgnoreCase(sp.modes->lsb)) { modeOut = 0x00; return true; }
  if (code.equalsIgnoreCase(sp.modes->usb)) { modeOut = 0x01; return true; }
  if (code.equalsIgnoreCase(sp.modes->am)) { modeOut = 0x02; return true; }
  if (code.equalsIgnoreCase(sp.modes->cw)) { modeOut = 0x03; return true; }
  if (code.equalsIgnoreCase(sp.modes->rtty) && sp.modes->rtty[0]) { modeOut = 0x04; return true; }
  if (code.equalsIgnoreCase(sp.modes->fm)) { modeOut = 0x05; return true; }
  if (code.equalsIgnoreCase(sp.modes->cwr) && sp.modes->cwr[0]) { modeOut = 0x07; return true; }
  if (code.equalsIgnoreCase(sp.modes->rttyR) && sp.modes->rttyR[0]) { modeOut = 0x08; return true; }
  if (code.equalsIgnoreCase(sp.modes->digi) && sp.modes->digi[0]) { modeOut = 0x11; return true; }
  if (code.equalsIgnoreCase(sp.modes->pkt) && sp.modes->pkt[0]) { modeOut = 0x12; return true; }
  if (code.equalsIgnoreCase(sp.modes->wfm) && sp.modes->wfm[0]) { modeOut = 0x06; return true; }
  if (ft8x7ModelFor(sp.model) == Ft8x7Model::Ft857) {
    // FT-857/897 may report additional undocumented bytes depending on
    // installed filters and packet handling (PKT as 0xFC, read here without the top bit).
    if (code.equalsIgnoreCase("3F")) { modeOut = 0x03; return true; }
    if (code.equalsIgnoreCase("7C")) { modeOut = 0x12; return true; }
  }
  return false;
}
