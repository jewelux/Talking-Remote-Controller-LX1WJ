#include "protocol_ops_ascii.h"

#include "cat_setting_codec.h"
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

// PA and RA as settings.frontEnd encodes them (cat_setting_codec.h).
bool asciiQueryPreampLevel(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs) {
  const char* cmd = asciiPreampReadCommand(sp.settings.frontEnd);
  if (!sp.caps.getPreamp || !cmd) return false;
  String line;
  if (!transactAsciiCommand(cmd, line, "PA", timeoutMs)) return false;
  return asciiPreampDecode(sp.settings.frontEnd, sp.settings.preampLevels, line.c_str() + 2, levelOut);
}

bool asciiSetPreampLevel(const RadioProfile& sp, uint8_t level) {
  char cmd[8];
  if (!sp.caps.setPreamp ||
      !asciiPreampEncode(sp.settings.frontEnd, sp.settings.preampLevels, level, cmd, sizeof(cmd))) {
    return false;
  }
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint8_t readBack = 0xFF;
  return asciiQueryPreampLevel(sp, readBack, 800) && readBack == level;
}

bool asciiQueryAttStep(const RadioProfile& sp, uint8_t& stepOut, uint32_t timeoutMs) {
  const char* cmd = asciiAttReadCommand(sp.settings.frontEnd);
  if (!sp.caps.getAtt || !cmd) return false;
  String line;
  if (!transactAsciiCommand(cmd, line, "RA", timeoutMs)) return false;
  return asciiAttDecode(sp.settings.frontEnd, sp.settings.attLevels, line.c_str() + 2, stepOut);
}

bool asciiSetAttStep(const RadioProfile& sp, uint8_t step) {
  char cmd[8];
  if (!sp.caps.setAtt || !asciiAttEncode(sp.settings.frontEnd, sp.settings.attLevels, step, cmd, sizeof(cmd))) {
    return false;
  }
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint8_t readBack = 0xFF;
  return asciiQueryAttStep(sp, readBack, 800) && readBack == step;
}

// GT, as the profile's settings.agc encodes it (cat_setting_codec.h).
bool asciiQueryAgc(const RadioProfile& sp, AgcMode& out, uint32_t timeoutMs) {
  const char* cmd = asciiAgcReadCommand(sp.settings.agc);
  if (!sp.caps.getAgc || !cmd) return false;
  String line;
  if (!transactAsciiCommand(cmd, line, "GT", timeoutMs)) return false;
  return asciiAgcDecode(sp.settings.agc, line.c_str() + 2, out);
}

bool asciiSetAgc(const RadioProfile& sp, AgcMode mode) {
  char cmd[8];
  if (!sp.caps.setAgc || !asciiAgcEncode(sp.settings.agc, mode, cmd, sizeof(cmd))) return false;
  if (!asciiSendSimpleCommand(cmd)) return false;
  AgcMode readBack = AgcMode::Off;
  return asciiQueryAgc(sp, readBack, 800) && readBack == mode;
}

// PC in watts (the CAT manuals and PC command reference, Hamlib and flrig; Elecraft's reference
// and Hamlib for the KX2).
bool asciiQueryRfPowerTenths(const RadioProfile& sp, uint16_t& tenthsOut, uint32_t timeoutMs) {
  if (!sp.caps.getRfPower || sp.settings.rfPower == RfPowerEncoding::None) return false;
  String line;
  if (!transactAsciiCommand("PC;", line, "PC", timeoutMs)) return false;
  return asciiRfPowerDecode(line.c_str() + 2, tenthsOut);
}

bool asciiSetRfPowerWatts(const RadioProfile& sp, uint16_t watts) {
  char cmd[8];
  if (!sp.caps.setRfPower || sp.settings.rfPower == RfPowerEncoding::None ||
      !asciiRfPowerEncode(watts, cmd, sizeof(cmd))) {
    return false;
  }
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint16_t readBack = 0;
  return asciiQueryRfPowerTenths(sp, readBack, 800) && readBack == watts * 10;
}

// BI and KR, 0 off and 1 on (the FTDX10, FTDX101 and FT-891 CAT manuals, Hamlib newcat).
static bool yaesuQueryOnOff(const char* cmd, const char* prefix, bool& onOut, uint32_t timeoutMs) {
  String line;
  if (!transactAsciiCommand(cmd, line, prefix, timeoutMs)) return false;
  return asciiOnOffDigitDecode(line.c_str() + strlen(prefix), onOut);
}

bool asciiQueryBreakIn(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.getBreakIn) return false;
  return yaesuQueryOnOff("BI;", "BI", onOut, timeoutMs);
}

bool asciiSetBreakIn(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.setBreakIn) return false;
  if (!asciiSendSimpleCommand(on ? "BI1;" : "BI0;")) return false;
  bool readBack = !on;
  return asciiQueryBreakIn(sp, readBack, 800) && readBack == on;
}

// BC0n, 0 off, 1 on (the FTDX10, FTDX101 and FT-891 CAT manuals, Hamlib newcat); the FTDX101
// main band.
bool asciiQueryAutoNotch(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.getAutoNotch) return false;
  return yaesuQueryOnOff("BC0;", "BC0", onOut, timeoutMs);
}

bool asciiSetAutoNotch(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.setAutoNotch) return false;
  if (!asciiSendSimpleCommand(on ? "BC01;" : "BC00;")) return false;
  bool readBack = !on;
  return asciiQueryAutoNotch(sp, readBack, 800) && readBack == on;
}

bool asciiQueryKeyer(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.getKeyer) return false;
  return yaesuQueryOnOff("KR;", "KR", onOut, timeoutMs);
}

bool asciiSetKeyer(const RadioProfile& sp, bool on) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.setKeyer) return false;
  if (!asciiSendSimpleCommand(on ? "KR1;" : "KR0;")) return false;
  bool readBack = !on;
  return asciiQueryKeyer(sp, readBack, 800) && readBack == on;
}

// RM4: the ALC meter, 0..255 (the FTDX10, FTDX101 and FT-891 CAT manuals; Hamlib and flrig read
// RM4 too).
bool asciiQueryAlcRaw(const RadioProfile& sp, uint16_t& rawOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.getAlc) return false;
  String line;
  if (!transactAsciiCommand("RM4;", line, "RM4", timeoutMs)) return false;
  return yaesuMeterDecode(line.c_str() + 3, rawOut);
}

// KS, three digits in WPM (the Yaesu CAT manuals, the TS-480 PC command reference, Elecraft's
// reference; Hamlib).
bool asciiQueryCwSpeed(const RadioProfile& sp, uint16_t& wpmOut, uint32_t timeoutMs) {
  const SettingEncodings& enc = sp.settings;
  if (!sp.caps.getCwSpeed || !enc.cwSpeedMaxWpm) return false;
  String line;
  if (!transactAsciiCommand("KS;", line, "KS", timeoutMs)) return false;
  uint16_t wpm = 0;
  if (!asciiDigitsDecode(line.c_str() + 2, 3, wpm) || wpm > enc.cwSpeedMaxWpm) return false;
  wpmOut = wpm;
  return true;
}

bool asciiSetCwSpeed(const RadioProfile& sp, uint16_t wpm) {
  const SettingEncodings& enc = sp.settings;
  if (!sp.caps.setCwSpeed || !enc.cwSpeedMaxWpm || wpm < enc.cwSpeedMinWpm || wpm > enc.cwSpeedMaxWpm) {
    return false;
  }
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "KS%03u;", (unsigned)wpm);
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint16_t readBack = 0;
  return asciiQueryCwSpeed(sp, readBack, 800) && readBack == wpm;
}

// KP, 00..75 = 300..1050 Hz (the Yaesu CAT manuals, Hamlib newcat).
bool asciiQueryCwPitch(const RadioProfile& sp, uint16_t& hzOut, uint32_t timeoutMs) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.getCwPitch || !sp.settings.cwPitchKp) return false;
  String line;
  if (!transactAsciiCommand("KP;", line, "KP", timeoutMs)) return false;
  return yaesuKpDecode(line.c_str() + 2, hzOut);
}

bool asciiSetCwPitch(const RadioProfile& sp, uint16_t hz) {
  if (sp.protocol != PROTO_YAESU_FTDX_ASCII || !sp.caps.setCwPitch || !sp.settings.cwPitchKp) return false;
  char cmd[8];
  if (hz < 300 || hz > 1050 || hz % 10 || !yaesuKpEncode(hz, cmd, sizeof(cmd))) return false;
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint16_t readBack = 0;
  return asciiQueryCwPitch(sp, readBack, 800) && readBack == hz;
}

// SD: on the FTDX10 a step of the VD table, on the FT-891 and TS-480 ms (their manuals, Hamlib).
bool asciiQueryCwDelayMs(const RadioProfile& sp, uint16_t& msOut, uint32_t timeoutMs) {
  const SettingEncodings& enc = sp.settings;
  if (!sp.caps.getCwDelay || enc.cwDelay == VoxDelayEncoding::None) return false;
  String line;
  if (!transactAsciiCommand("SD;", line, "SD", timeoutMs)) return false;
  uint16_t value = 0;
  if (enc.cwDelay == VoxDelayEncoding::YaesuSteps) {
    if (!asciiDigitsDecode(line.c_str() + 2, 2, value) || value >= kYaesuAsciiVoxDelaySteps) return false;
    msOut = yaesuAsciiVoxDelayStepMs((uint8_t)value);
    return true;
  }
  if (!asciiDigitsDecode(line.c_str() + 2, 4, value) || value > enc.cwDelayMaxMs) return false;
  msOut = value;
  return true;
}

bool asciiSetCwDelayMs(const RadioProfile& sp, uint16_t ms) {
  const SettingEncodings& enc = sp.settings;
  if (!sp.caps.setCwDelay || enc.cwDelay == VoxDelayEncoding::None) return false;
  char cmd[10];
  uint16_t expected = 0;
  if (enc.cwDelay == VoxDelayEncoding::YaesuSteps) {
    const uint8_t step = yaesuAsciiVoxDelayNearestStep(ms);
    snprintf(cmd, sizeof(cmd), "SD%02u;", (unsigned)step);
    expected = yaesuAsciiVoxDelayStepMs(step);
  } else {
    expected = asciiVoxDelayMsRound(ms, enc.cwDelayMinMs, enc.cwDelayMaxMs, enc.cwDelayStepMs);
    snprintf(cmd, sizeof(cmd), "SD%04u;", (unsigned)expected);
  }
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint16_t readBack = 0;
  return asciiQueryCwDelayMs(sp, readBack, 800) && readBack == expected;
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
  if (!sp.caps.getDialLock || !sp.commands->lockGet[0] || !sp.commands->lockReplyPrefix[0]) return false;
  String line;
  if (!transactAsciiCommand(sp.commands->lockGet, line, sp.commands->lockReplyPrefix, timeoutMs)) return false;
  const size_t at = strlen(sp.commands->lockReplyPrefix);
  return line.length() > at && asciiLockDigitDecode(line[at], sp.commands->lockOffDigits, onOut);
}

bool asciiSetLock(const RadioProfile& sp, bool on) {
  if (!sp.caps.setDialLock) return false;
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

// PROC, mic gain and VOX on the Yaesu ASCII radios and the TS-480. Both use MG, VX, VG and VD;
// the Yaesus answer PR0 and PL with one level, the TS-480 PR, and PL with an input and an
// output level (its PC command reference; Hamlib kenwood.c agrees).
static bool asciiHasTxSettings(const RadioProfile& sp) {
  return sp.protocol == PROTO_YAESU_FTDX_ASCII || sp.protocol == PROTO_KENWOOD_ASCII;
}

static bool isKenwood(const RadioProfile& sp) {
  return sp.protocol == PROTO_KENWOOD_ASCII;
}

// The FTDX10 manual says 1 = off and 2 = on. Hamlib found that the FTDX101, FT-891 and
// FT-991, whose manuals say the same, answer 0 = off and 1 = on, and flrig reads the FT-891 so;
// the FTDX10 is untested, so this follows them. 2 can only mean on. The TS-480 answers PR0 or
// PR1.
bool asciiQueryProc(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!asciiHasTxSettings(sp)) return false;
  const char* prefix = isKenwood(sp) ? "PR" : "PR0";
  String line;
  if (!transactAsciiCommand(isKenwood(sp) ? "PR;" : "PR0;", line, prefix, timeoutMs)) return false;
  const size_t at = strlen(prefix);
  if (line.length() <= at) return false;
  const char state = line[at];
  if (state == '0') { onOut = false; return true; }
  if (state == '1' || (!isKenwood(sp) && state == '2')) { onOut = true; return true; }
  return false;
}

// The TS-480's PL: the input level, which Hamlib reads as the processor level, and the output
// level.
static bool kenwoodQueryProcLevels(uint8_t& inputOut, uint8_t& outputOut, uint32_t timeoutMs) {
  String line;
  if (!transactAsciiCommand("PL;", line, "PL", timeoutMs)) return false;
  return kenwoodProcLevelsDecode(line.c_str() + 2, inputOut, outputOut);
}

bool asciiQueryProcLevel(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs) {
  if (!asciiHasTxSettings(sp)) return false;
  if (isKenwood(sp)) {
    uint8_t output = 0;
    return kenwoodQueryProcLevels(levelOut, output, timeoutMs);
  }
  String line;
  if (!transactAsciiCommand("PL;", line, "PL", timeoutMs)) return false;
  uint64_t value = 0;
  if (!parseAsciiUnsignedResponse(line, "PL", value) || value > 100) return false;
  levelOut = (uint8_t)value;
  return true;
}

// Reads "<code><level>;", 0..max.
static bool asciiQueryLevel(const char* code, uint8_t max, uint8_t& levelOut, uint32_t timeoutMs) {
  char cmd[6];
  snprintf(cmd, sizeof(cmd), "%s;", code);
  String line;
  if (!transactAsciiCommand(cmd, line, code, timeoutMs)) return false;
  uint64_t value = 0;
  if (!parseAsciiUnsignedResponse(line, code, value) || value > max) return false;
  levelOut = (uint8_t)value;
  return true;
}

bool asciiQueryMicGain(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs) {
  return asciiHasTxSettings(sp) && asciiQueryLevel("MG", 100, levelOut, timeoutMs);
}

bool asciiQueryVox(const RadioProfile& sp, bool& onOut, uint32_t timeoutMs) {
  uint8_t value = 0;
  if (!asciiHasTxSettings(sp) || !asciiQueryLevel("VX", 1, value, timeoutMs)) return false;
  onOut = value == 1;
  return true;
}

bool asciiQueryVoxGain(const RadioProfile& sp, uint8_t& levelOut, uint32_t timeoutMs) {
  return asciiHasTxSettings(sp) && asciiQueryLevel("VG", sp.settings.voxGainMax, levelOut, timeoutMs);
}

// VD answers a step (cat_setting_codec.h) or ms, as the profile's settings say.
bool asciiQueryVoxDelayMs(const RadioProfile& sp, uint16_t& msOut, uint32_t timeoutMs) {
  const VoxDelayEncoding encoding = sp.settings.voxDelay;
  if (!asciiHasTxSettings(sp) || encoding == VoxDelayEncoding::None) return false;
  String line;
  if (!transactAsciiCommand("VD;", line, "VD", timeoutMs)) return false;
  uint64_t value = 0;
  if (!parseAsciiUnsignedResponse(line, "VD", value)) return false;
  if (encoding == VoxDelayEncoding::YaesuSteps) {
    if (value >= kYaesuAsciiVoxDelaySteps) return false;
    msOut = yaesuAsciiVoxDelayStepMs((uint8_t)value);
    return true;
  }
  if (value > sp.settings.voxDelayMaxMs) return false;
  msOut = (uint16_t)value;
  return true;
}

// Sends "<code><level, 3 digits>;" and reads it back with query.
template <typename Query>
static bool asciiSetLevel(const RadioProfile& sp, const char* code, uint8_t level, uint8_t max, Query query) {
  if (!asciiHasTxSettings(sp) || level > max) return false;
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "%s%03u;", code, (unsigned)level);
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint8_t readBack = 0;
  return query(sp, readBack, 800) && readBack == level;
}

// Yaesu: PR01 / PR00 as Hamlib and flrig send on the FT-891 (Hamlib also on the FTDX101 and
// FT-991); the FTDX10 manual's 2 = on is tried when 1 does not read back as on. TS-480: PR1 / PR0.
bool asciiSetProc(const RadioProfile& sp, bool on) {
  if (!asciiHasTxSettings(sp) || !sp.caps.setProc) return false;
  bool readBack = !on;
  if (isKenwood(sp)) {
    return asciiSendSimpleCommand(on ? "PR1;" : "PR0;") && asciiQueryProc(sp, readBack, 800) && readBack == on;
  }
  if (asciiSendSimpleCommand(on ? "PR01;" : "PR00;") && asciiQueryProc(sp, readBack, 800) && readBack == on) {
    return true;
  }
  return on && asciiSendSimpleCommand("PR02;") && asciiQueryProc(sp, readBack, 800) && readBack;
}

// The TS-480 sets the input level and keeps the output level it has (Hamlib sets both to the
// one value).
bool asciiSetProcLevel(const RadioProfile& sp, uint8_t level) {
  if (!sp.caps.setProc) return false;
  if (!isKenwood(sp)) return asciiSetLevel(sp, "PL", level, 100, asciiQueryProcLevel);
  if (level > 100) return false;
  uint8_t input = 0;
  uint8_t output = 0;
  if (!kenwoodQueryProcLevels(input, output, 800)) return false;
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "PL%03u%03u;", (unsigned)level, (unsigned)output);
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint8_t outputBack = 0;
  return kenwoodQueryProcLevels(input, outputBack, 800) && input == level && outputBack == output;
}

bool asciiSetMicGain(const RadioProfile& sp, uint8_t level) {
  if (!sp.caps.setMicGain) return false;
  return asciiSetLevel(sp, "MG", level, 100, asciiQueryMicGain);
}

bool asciiSetVox(const RadioProfile& sp, bool on) {
  if (!asciiHasTxSettings(sp) || !sp.caps.setVox) return false;
  bool readBack = !on;
  return asciiSendSimpleCommand(on ? "VX1;" : "VX0;") && asciiQueryVox(sp, readBack, 800) && readBack == on;
}

bool asciiSetVoxGain(const RadioProfile& sp, uint8_t level) {
  if (!sp.caps.setVox) return false;
  return asciiSetLevel(sp, "VG", level, sp.settings.voxGainMax, asciiQueryVoxGain);
}

bool asciiSetVoxDelayMs(const RadioProfile& sp, uint16_t ms) {
  const SettingEncodings& enc = sp.settings;
  if (!asciiHasTxSettings(sp) || !sp.caps.setVox || enc.voxDelay == VoxDelayEncoding::None) return false;
  char cmd[10];
  uint16_t expected = 0;
  if (enc.voxDelay == VoxDelayEncoding::YaesuSteps) {
    const uint8_t best = yaesuAsciiVoxDelayNearestStep(ms);
    snprintf(cmd, sizeof(cmd), "VD%02u;", (unsigned)best);
    expected = yaesuAsciiVoxDelayStepMs(best);
  } else {
    expected = asciiVoxDelayMsRound(ms, enc.voxDelayMinMs, enc.voxDelayMaxMs, enc.voxDelayStepMs);
    snprintf(cmd, sizeof(cmd), "VD%04u;", (unsigned)expected);
  }
  if (!asciiSendSimpleCommand(cmd)) return false;
  uint16_t readBack = 0;
  return asciiQueryVoxDelayMs(sp, readBack, 800) && readBack == expected;
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
