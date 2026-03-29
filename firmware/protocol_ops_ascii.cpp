#include "protocol_ops_ascii.h"

#include "protocol_ascii.h"
#include "radio_protocol.h"
#include "transport_serial.h"

static bool asciiParseOnOffResponse(const String& line, const char* prefix, bool& onOut) {
  int start = (int)strlen(prefix);
  int semi = line.indexOf(';', start);
  if (semi < 0) semi = line.length();
  String value = line.substring(start, semi);
  value.trim();
  if (value == "0" || value == "OFF") { onOut = false; return true; }
  if (value == "1" || value == "ON") { onOut = true; return true; }
  bool digitsOnly = value.length() > 0;
  for (size_t i = 0; i < value.length(); ++i) {
    if (!isDigit(value[i])) {
      digitsOnly = false;
      break;
    }
  }
  if (digitsOnly) {
    onOut = value.toInt() != 0;
    return true;
  }
  return false;
}

static bool asciiSendSimpleCommand(const char* cmd) {
  if (!cmd || !cmd[0]) return false;
  serialTransportFlushInput();
  serialTransportPrint(cmd);
  serialTransportFlushOutput();
  return true;
}

static bool asciiQueryRawLine(const char* cmd, const char* prefix, String& lineOut, uint32_t timeoutMs) {
  if (!cmd || !cmd[0]) return false;
  return transactAsciiCommand(cmd, lineOut, prefix, timeoutMs);
}

bool asciiQueryFrequency(const StoredProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq || !sp.ascii.freqGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.freqGet, line, sp.ascii.freqReplyPrefix, timeoutMs)) return false;
  return parseAsciiUnsignedResponse(line, sp.ascii.freqReplyPrefix, hzOut);
}

bool asciiSetFrequency(const StoredProfile& sp, uint64_t hz) {
  if (!sp.caps.setFreq || !sp.ascii.freqSetFormat[0]) return false;
  char buf[32];
  snprintf(buf, sizeof(buf), sp.ascii.freqSetFormat, (unsigned long long)hz);
  serialTransportFlushInput();
  serialTransportPrint(buf);
  serialTransportFlushOutput();
  delay(40);
  uint64_t readHz = 0;
  return queryFrequency(readHz, 800) && (readHz == hz);
}

bool asciiQueryMode(const StoredProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode || !sp.ascii.modeGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.modeGet, line, sp.ascii.modeReplyPrefix, timeoutMs)) return false;
  int start = (int)strlen(sp.ascii.modeReplyPrefix);
  int semi = line.indexOf(';', start);
  if (semi < 0) semi = line.length();
  String code = line.substring(start, semi);
  code.trim();
  return code.length() && profileInternalModeForCode(sp, code, modeOut);
}

bool asciiSetMode(const StoredProfile& sp, uint8_t mode) {
  if (!sp.caps.setMode || !sp.ascii.modeSetFormat[0]) return false;
  String code;
  if (!profileModeCodeForInternal(sp, mode, code)) return false;
  char cmd[24];
  snprintf(cmd, sizeof(cmd), sp.ascii.modeSetFormat, code.c_str());
  serialTransportFlushInput();
  serialTransportPrint(cmd);
  serialTransportFlushOutput();
  return true;
}

bool asciiQuerySMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter || !sp.ascii.smeterGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.smeterGet, line, sp.ascii.smeterReplyPrefix, timeoutMs)) return false;
  return parseAsciiSignedResponse(line, sp.ascii.smeterReplyPrefix, rawOut);
}

bool asciiQueryPoMeterRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower || !sp.ascii.powerGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.powerGet, line, sp.ascii.powerReplyPrefix, timeoutMs)) return false;
  return parseAsciiSignedResponse(line, sp.ascii.powerReplyPrefix, rawOut);
}

bool asciiQuerySWRRaw(const StoredProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr || !sp.ascii.swrGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.swrGet, line, sp.ascii.swrReplyPrefix, timeoutMs)) return false;
  return parseAsciiSignedResponse(line, sp.ascii.swrReplyPrefix, rawOut);
}

bool asciiQueryStatusLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.ascii.ifGet, sp.ascii.ifReplyPrefix, lineOut, timeoutMs);
}

bool asciiQueryIdLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.ascii.idGet, sp.ascii.idReplyPrefix, lineOut, timeoutMs);
}

bool asciiQueryOmLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.ascii.omGet, sp.ascii.omReplyPrefix, lineOut, timeoutMs);
}

bool asciiQueryPreamp(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.ascii.preampGet[0] || !sp.ascii.preampReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.preampGet, line, sp.ascii.preampReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.ascii.preampReplyPrefix, onOut);
}

bool asciiSetPreamp(const StoredProfile& sp, bool on) {
  return asciiSendSimpleCommand(on ? sp.ascii.preampOnCmd : sp.ascii.preampOffCmd);
}

bool asciiQueryAgcLine(const StoredProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.ascii.agcGet, sp.ascii.agcReplyPrefix, lineOut, timeoutMs);
}

bool asciiSetAgcCommand(const StoredProfile& sp, const char* cmd) {
  (void)sp;
  return asciiSendSimpleCommand(cmd);
}

bool asciiQueryPowerState(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.ascii.powerStateGet[0] || !sp.ascii.powerStateReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.powerStateGet, line, sp.ascii.powerStateReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.ascii.powerStateReplyPrefix, onOut);
}

bool asciiSetPowerState(const StoredProfile& sp, bool on) {
  const char* cmd = on ? sp.ascii.powerStateOnCmd : sp.ascii.powerStateOffCmd;
  if (!cmd || !cmd[0]) return false;
  if (sp.protocolType == PROTO_YAESU_FTDX_ASCII && on) {
    // FTDX10/101 CAT power-on requires the documented double-send timing window.
    if (!asciiSendSimpleCommand(cmd)) return false;
    delay(1100);
    return asciiSendSimpleCommand(cmd);
  }
  return asciiSendSimpleCommand(cmd);
}

bool asciiQueryTuner(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.ascii.tunerGet[0] || !sp.ascii.tunerReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.tunerGet, line, sp.ascii.tunerReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.ascii.tunerReplyPrefix, onOut);
}

bool asciiSetTuner(const StoredProfile& sp, bool on) {
  return asciiSendSimpleCommand(on ? sp.ascii.tunerOnCmd : sp.ascii.tunerOffCmd);
}

bool asciiStartTune(const StoredProfile& sp) {
  return asciiSendSimpleCommand(sp.ascii.tuneStartCmd);
}

bool asciiQueryNr(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNr || !sp.ascii.nrGet[0] || !sp.ascii.nrReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.nrGet, line, sp.ascii.nrReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.ascii.nrReplyPrefix, onOut);
}

bool asciiSetNr(const StoredProfile& sp, bool on) {
  if (!sp.caps.setNr) return false;
  return asciiSendSimpleCommand(on ? sp.ascii.nrOnCmd : sp.ascii.nrOffCmd);
}

bool asciiQueryNb(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNb || !sp.ascii.nbGet[0] || !sp.ascii.nbReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.nbGet, line, sp.ascii.nbReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.ascii.nbReplyPrefix, onOut);
}

bool asciiSetNb(const StoredProfile& sp, bool on) {
  if (!sp.caps.setNb) return false;
  return asciiSendSimpleCommand(on ? sp.ascii.nbOnCmd : sp.ascii.nbOffCmd);
}

bool asciiQueryNotch(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNotch || !sp.ascii.notchGet[0] || !sp.ascii.notchReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.notchGet, line, sp.ascii.notchReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.ascii.notchReplyPrefix, onOut);
}

bool asciiSetNotch(const StoredProfile& sp, bool on) {
  if (!sp.caps.setNotch) return false;
  return asciiSendSimpleCommand(on ? sp.ascii.notchOnCmd : sp.ascii.notchOffCmd);
}

bool asciiQueryLock(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.ascii.lockGet[0] || !sp.ascii.lockReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.ascii.lockGet, line, sp.ascii.lockReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.ascii.lockReplyPrefix, onOut);
}

bool asciiSetLock(const StoredProfile& sp, bool on) {
  return asciiSendSimpleCommand(on ? sp.ascii.lockOnCmd : sp.ascii.lockOffCmd);
}
