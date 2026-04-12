#include "protocol_ascii.h"

#include "packet_ascii.h"
#include "debug_log.h"

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

bool profileModeCodeForInternal(const StoredProfile& sp, uint8_t mode, String& codeOut) {
  const char* code = nullptr;
  switch (mode) {
    case 0x00: code = sp.ascii.modeLsb; break;
    case 0x01: code = sp.ascii.modeUsb; break;
    case 0x02: code = sp.ascii.modeAm; break;
    case 0x03: code = sp.ascii.modeCw; break;
    case 0x04: code = sp.ascii.modeRtty; break;
    case 0x05: code = sp.ascii.modeFm; break;
    case 0x07: code = sp.ascii.modeCwr; break;
    case 0x08: code = sp.ascii.modeRttyR; break;
    case 0x11: code = sp.ascii.modeDigi; break;
    default: break;
  }
  if (!code || !code[0]) return false;
  codeOut = code;
  return true;
}

bool profileInternalModeForCode(const StoredProfile& sp, const String& code, uint8_t& modeOut) {
  if (code.equalsIgnoreCase(sp.ascii.modeLsb)) { modeOut = 0x00; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeUsb)) { modeOut = 0x01; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeAm)) { modeOut = 0x02; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeCw)) { modeOut = 0x03; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeRtty) && sp.ascii.modeRtty[0]) { modeOut = 0x04; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeFm)) { modeOut = 0x05; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeCwr) && sp.ascii.modeCwr[0]) { modeOut = 0x07; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeRttyR) && sp.ascii.modeRttyR[0]) { modeOut = 0x08; return true; }
  if (code.equalsIgnoreCase(sp.ascii.modeDigi) && sp.ascii.modeDigi[0]) { modeOut = 0x11; return true; }
  if (!strcmp(sp.variant, "ft857_897")) {
    // FT-857/897 may report additional undocumented bytes depending on
    // installed filters and packet handling.
    if (code.equalsIgnoreCase("0C")) { modeOut = 0x11; return true; }
    if (code.equalsIgnoreCase("3F")) { modeOut = 0x03; return true; }
    if (code.equalsIgnoreCase("7C")) { modeOut = 0x11; return true; }
  }
  return false;
}
