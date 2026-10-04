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

bool asciiQueryFrequency(const RadioProfile& sp, uint64_t& hzOut, uint32_t timeoutMs) {
  if (!sp.caps.getFreq || !sp.commands->freqGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->freqGet, line, sp.commands->freqReplyPrefix, timeoutMs)) return false;
  return parseAsciiUnsignedResponse(line, sp.commands->freqReplyPrefix, hzOut);
}

bool asciiSetFrequency(const RadioProfile& sp, uint64_t hz) {
  if (!sp.caps.setFreq || !sp.commands->freqSetFormat[0]) return false;
  char buf[32];
  snprintf(buf, sizeof(buf), sp.commands->freqSetFormat, (unsigned long long)hz);
  serialTransportFlushInput();
  serialTransportPrint(buf);
  serialTransportFlushOutput();
  delay(40);
  uint64_t readHz = 0;
  return queryFrequency(readHz, 800) && (readHz == hz);
}

bool asciiQueryMode(const RadioProfile& sp, uint8_t& modeOut, uint32_t timeoutMs) {
  if (!sp.caps.getMode || !sp.commands->modeGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->modeGet, line, sp.commands->modeReplyPrefix, timeoutMs)) return false;
  int start = (int)strlen(sp.commands->modeReplyPrefix);
  int semi = line.indexOf(';', start);
  if (semi < 0) semi = line.length();
  String code = line.substring(start, semi);
  code.trim();
  return code.length() && profileInternalModeForCode(sp, code, modeOut);
}

bool asciiSetMode(const RadioProfile& sp, uint8_t mode) {
  if (!sp.caps.setMode || !sp.commands->modeSetFormat[0]) return false;
  String code;
  if (!profileModeCodeForInternal(sp, mode, code)) return false;
  char cmd[24];
  snprintf(cmd, sizeof(cmd), sp.commands->modeSetFormat, code.c_str());
  serialTransportFlushInput();
  serialTransportPrint(cmd);
  serialTransportFlushOutput();
  return true;
}

bool asciiQuerySMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSmeter || !sp.commands->smeterGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->smeterGet, line, sp.commands->smeterReplyPrefix, timeoutMs)) return false;
  return parseAsciiSignedResponse(line, sp.commands->smeterReplyPrefix, rawOut);
}

bool asciiQueryPoMeterRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getPower || !sp.commands->powerGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->powerGet, line, sp.commands->powerReplyPrefix, timeoutMs)) return false;
  return parseAsciiSignedResponse(line, sp.commands->powerReplyPrefix, rawOut);
}

bool asciiQuerySWRRaw(const RadioProfile& sp, int32_t& rawOut, uint32_t timeoutMs) {
  if (!sp.caps.getSwr || !sp.commands->swrGet[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->swrGet, line, sp.commands->swrReplyPrefix, timeoutMs)) return false;
  return parseAsciiSignedResponse(line, sp.commands->swrReplyPrefix, rawOut);
}

bool asciiQueryStatusLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.commands->ifGet, sp.commands->ifReplyPrefix, lineOut, timeoutMs);
}

bool asciiQueryIdLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.commands->idGet, sp.commands->idReplyPrefix, lineOut, timeoutMs);
}

bool asciiQueryOmLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.commands->omGet, sp.commands->omReplyPrefix, lineOut, timeoutMs);
}

bool asciiQueryPreamp(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.commands->preampGet[0] || !sp.commands->preampReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->preampGet, line, sp.commands->preampReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.commands->preampReplyPrefix, onOut);
}

bool asciiSetPreamp(const RadioProfile& sp, bool on) {
  return asciiSendSimpleCommand(on ? sp.commands->preampOnCmd : sp.commands->preampOffCmd);
}

bool asciiQueryAgcLine(const RadioProfile& sp, String& lineOut, uint32_t timeoutMs) {
  return asciiQueryRawLine(sp.commands->agcGet, sp.commands->agcReplyPrefix, lineOut, timeoutMs);
}

bool asciiSetAgcCommand(const RadioProfile& sp, const char* cmd) {
  (void)sp;
  return asciiSendSimpleCommand(cmd);
}

bool asciiQueryPowerState(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.commands->powerStateGet[0] || !sp.commands->powerStateReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->powerStateGet, line, sp.commands->powerStateReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.commands->powerStateReplyPrefix, onOut);
}

bool asciiSetPowerState(const RadioProfile& sp, bool on) {
  const char* cmd = on ? sp.commands->powerStateOnCmd : sp.commands->powerStateOffCmd;
  if (!cmd || !cmd[0]) return false;
  if (sp.protocol == PROTO_YAESU_FTDX_ASCII && on) {
    // FTDX10/101 CAT power-on requires the documented double-send timing window.
    if (!asciiSendSimpleCommand(cmd)) return false;
    delay(1100);
    return asciiSendSimpleCommand(cmd);
  }
  return asciiSendSimpleCommand(cmd);
}

bool asciiQueryTuner(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.commands->tunerGet[0] || !sp.commands->tunerReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->tunerGet, line, sp.commands->tunerReplyPrefix, timeoutMs)) {
    delay(40);
    if (!transactAsciiCommand(sp.commands->tunerGet, line, sp.commands->tunerReplyPrefix, timeoutMs)) return false;
  }
  return asciiParseOnOffResponse(line, sp.commands->tunerReplyPrefix, onOut);
}

bool asciiSetTuner(const RadioProfile& sp, bool on) {
  if (!asciiSendSimpleCommand(on ? sp.commands->tunerOnCmd : sp.commands->tunerOffCmd)) return false;
  delay(50);
  return true;
}

bool asciiStartTune(const RadioProfile& sp) {
  return asciiSendSimpleCommand(sp.commands->tuneStartCmd);
}

bool asciiQuerySplit(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.commands->splitGet[0] || !sp.commands->splitReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->splitGet, line, sp.commands->splitReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.commands->splitReplyPrefix, onOut);
}

bool asciiSetSplit(const RadioProfile& sp, bool on) {
  return asciiSendSimpleCommand(on ? sp.commands->splitOnCmd : sp.commands->splitOffCmd);
}

bool asciiQueryNr(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNr || !sp.commands->nrGet[0] || !sp.commands->nrReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->nrGet, line, sp.commands->nrReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.commands->nrReplyPrefix, onOut);
}

bool asciiSetNr(const RadioProfile& sp, bool on) {
  if (!sp.caps.setNr) return false;
  return asciiSendSimpleCommand(on ? sp.commands->nrOnCmd : sp.commands->nrOffCmd);
}

bool asciiQueryNb(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNb || !sp.commands->nbGet[0] || !sp.commands->nbReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->nbGet, line, sp.commands->nbReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.commands->nbReplyPrefix, onOut);
}

bool asciiSetNb(const RadioProfile& sp, bool on) {
  if (!sp.caps.setNb) return false;
  return asciiSendSimpleCommand(on ? sp.commands->nbOnCmd : sp.commands->nbOffCmd);
}

bool asciiQueryNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getNotch || !sp.commands->notchGet[0] || !sp.commands->notchReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->notchGet, line, sp.commands->notchReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.commands->notchReplyPrefix, onOut);
}

bool asciiSetNotch(const RadioProfile& sp, bool on) {
  if (!sp.caps.setNotch) return false;
  return asciiSendSimpleCommand(on ? sp.commands->notchOnCmd : sp.commands->notchOffCmd);
}

bool asciiQueryLock(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.commands->lockGet[0] || !sp.commands->lockReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->lockGet, line, sp.commands->lockReplyPrefix, timeoutMs)) return false;
  return asciiParseOnOffResponse(line, sp.commands->lockReplyPrefix, onOut);
}

bool asciiSetLock(const RadioProfile& sp, bool on) {
  return asciiSendSimpleCommand(on ? sp.commands->lockOnCmd : sp.commands->lockOffCmd);
}

bool asciiQueryYaesuRadioInfoFlag(const RadioProfile& sp, const char* code, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !code || !code[0]) return false;
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "RI%s;", code);
  String line;
  if (!transactAsciiCommand(cmd, line, "RI", timeoutMs)) return false;

  int start = 2;
  int semi = line.indexOf(';', start);
  if (semi < 0) semi = line.length();
  String payload = line.substring(start, semi);
  payload.trim();
  const size_t codeLen = strlen(code);
  if (payload.length() < (int)(codeLen + 1)) return false;
  if (!payload.substring(0, (int)codeLen).equalsIgnoreCase(code)) return false;

  const char state = payload[(int)codeLen];
  if (state == '0') { onOut = false; return true; }
  if (state == '1') { onOut = true; return true; }
  return false;
}

bool asciiQueryActiveVfoA(const RadioProfile& sp, bool& vfoAOut, uint32_t timeoutMs) {
  if (!sp.commands->vfoGet[0] || !sp.commands->vfoReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->vfoGet, line, sp.commands->vfoReplyPrefix, timeoutMs)) return false;
  int start = (int)strlen(sp.commands->vfoReplyPrefix);
  int semi = line.indexOf(';', start);
  if (semi < 0) semi = line.length();
  String value = line.substring(start, semi);
  value.trim();
  if (value == "0") { vfoAOut = true; return true; }
  if (value == "1") { vfoAOut = false; return true; }
  return false;
}

bool asciiSelectVfoA(const RadioProfile& sp) {
  return asciiSendSimpleCommand(sp.commands->vfoACmd);
}

bool asciiSelectVfoB(const RadioProfile& sp) {
  return asciiSendSimpleCommand(sp.commands->vfoBCmd);
}

bool asciiSwapVfo(const RadioProfile& sp) {
  return asciiSendSimpleCommand(sp.commands->vfoSwapCmd);
}

bool asciiQueryVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t& hzOut, uint32_t timeoutMs) {
  const char* cmd = targetVfoA ? sp.commands->vfoAGet : sp.commands->vfoBGet;
  const char* prefix = targetVfoA ? "FA" : "FB";
  if (!cmd[0]) return false;
  String line;
  if (!transactAsciiCommand(cmd, line, prefix, timeoutMs)) return false;
  return parseAsciiUnsignedResponse(line, prefix, hzOut);
}

bool asciiSetVfoFrequency(const RadioProfile& sp, bool targetVfoA, uint64_t hz) {
  const char* format = targetVfoA ? sp.commands->vfoASetFormat : sp.commands->vfoBSetFormat;
  if (!format[0]) return false;
  char buf[32];
  snprintf(buf, sizeof(buf), format, (unsigned long long)hz);
  serialTransportFlushInput();
  serialTransportPrint(buf);
  serialTransportFlushOutput();
  return true;
}
