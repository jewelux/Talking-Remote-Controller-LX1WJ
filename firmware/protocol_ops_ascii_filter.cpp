// The receive filter settings over ASCII CAT: the Yaesu newcat SH, IS, NA, BP0, CO0 and the EX cut
// menus, the TS-480 SL and SH. The encodings are in rx_filter_codec.h; each set is read back.

#include "protocol_ops_ascii.h"

#include "cat_setting_codec.h"
#include "protocol_ascii.h"
#include "protocol_ops_ascii_internal.h"
#include "rx_filter_codec.h"

namespace {

bool isYaesuFilter(const RadioProfile& sp) {
  return sp.protocol == PROTO_YAESU_FTDX_ASCII && rxFilterEncodingIsYaesu(sp.settings.rxFilter);
}

bool isKenwoodFilter(const RadioProfile& sp) {
  return sp.protocol == PROTO_KENWOOD_ASCII && sp.settings.rxFilter == RxFilterEncoding::KenwoodTs480;
}

// cmd, then the reply that starts with prefix: what follows the prefix.
bool transactAfterPrefix(const char* cmd, const char* prefix, String& line, uint32_t timeoutMs) {
  if (!cmd || !prefix) return false;
  if (!transactAsciiCommand(cmd, line, prefix, timeoutMs)) return false;
  line.remove(0, strlen(prefix));
  return true;
}

// The code of a level read as a code (width, cuts), on the radio's scale.
bool queryCode(const RadioProfile& sp, RxFilterLevel level, FilterModeGroup group, uint8_t& codeOut,
               uint32_t timeoutMs) {
  String rest;
  if (isKenwoodFilter(sp)) {
    if (level != RxFilterLevel::LowCut && level != RxFilterLevel::HighCut) return false;
    const bool low = level == RxFilterLevel::LowCut;
    return transactAfterPrefix(low ? "SL;" : "SH;", low ? "SL" : "SH", rest, timeoutMs) &&
           kenwoodCutDecode(rest.c_str(), codeOut);
  }
  if (level == RxFilterLevel::Width) {
    return transactAfterPrefix("SH0;", "SH0", rest, timeoutMs) && yaesuWidthDecode(rest.c_str(), codeOut);
  }
  char menu[12];
  if (!yaesuCutMenu(sp.settings.rxFilter, level, group, menu, sizeof(menu))) return false;
  char cmd[14];
  snprintf(cmd, sizeof(cmd), "%s;", menu);
  uint16_t code = 0;
  if (!transactAfterPrefix(cmd, menu, rest, timeoutMs) ||
      !asciiDigitsDecode(rest.c_str(), yaesuCutDigits(level), code)) {
    return false;
  }
  codeOut = (uint8_t)code;
  return true;
}

bool encodeCode(const RadioProfile& sp, RxFilterLevel level, FilterModeGroup group, uint8_t code, char* out,
                size_t size) {
  if (isKenwoodFilter(sp)) return kenwoodCutEncode(level, code, out, size);
  if (level == RxFilterLevel::Width) return yaesuWidthEncode(code, out, size);
  char menu[12];
  if (!yaesuCutMenu(sp.settings.rxFilter, level, group, menu, sizeof(menu))) return false;
  const int written = yaesuCutDigits(level) == 1 ? snprintf(out, size, "%s%u;", menu, (unsigned)code)
                                                 : snprintf(out, size, "%s%02u;", menu, (unsigned)code);
  return written > 0 && (size_t)written < size;
}

// The levels the radio takes as a number (IF shift, notch, contour and APF frequencies).
bool queryNumber(const RadioProfile& sp, RxFilterLevel level, int16_t& valueOut, uint32_t timeoutMs) {
  String rest;
  if (level == RxFilterLevel::IfShift) {
    return transactAfterPrefix("IS0;", "IS0", rest, timeoutMs) &&
           yaesuIfShiftDecode(sp.settings.rxFilter, rest.c_str(), valueOut);
  }
  return transactAfterPrefix(yaesuFreqReadCommand(level), yaesuFreqReplyPrefix(level), rest, timeoutMs) &&
         yaesuFreqDecode(level, rest.c_str(), valueOut);
}

bool encodeNumber(const RadioProfile& sp, RxFilterLevel level, int16_t value, char* out, size_t size) {
  if (level == RxFilterLevel::IfShift) return yaesuIfShiftEncode(sp.settings.rxFilter, value, out, size);
  return yaesuFreqEncode(level, value, out, size);
}

bool filterReady(const RadioProfile& sp, RxFilterLevel level, bool set) {
  return rxFilterLevelCap(sp.caps, level, set) && (isYaesuFilter(sp) || isKenwoodFilter(sp));
}

}  // namespace

bool asciiQueryFilterModeGroup(const RadioProfile& sp, FilterModeGroup& out, uint32_t timeoutMs) {
  if (!isYaesuFilter(sp) && !isKenwoodFilter(sp)) return false;
  if (!sp.caps.getMode || !sp.commands->modeGet[0]) return false;
  String rest;
  if (!transactAfterPrefix(sp.commands->modeGet, sp.commands->modeReplyPrefix, rest, timeoutMs) || !rest.length()) {
    return false;
  }
  const char code = rest[0];
  out = isKenwoodFilter(sp) ? kenwoodFilterModeGroup(code) : yaesuFilterModeGroup(code);
  return true;
}

RxFilterRead asciiQueryRxFilterLevel(const RadioProfile& sp, RxFilterLevel level, FilterModeGroup group,
                                     int16_t& valueOut, uint32_t timeoutMs) {
  if (!filterReady(sp, level, false)) return RxFilterRead::NoReply;
  if (!rxFilterLevelByMode(level)) {
    return isYaesuFilter(sp) && queryNumber(sp, level, valueOut, timeoutMs) ? RxFilterRead::Ok : RxFilterRead::NoReply;
  }
  RxFilterScale scale;
  uint8_t code = 0;
  if (!rxFilterScaleFor(sp.settings.rxFilter, level, group, scale) || !queryCode(sp, level, group, code, timeoutMs)) {
    return RxFilterRead::NoReply;
  }
  if (!scale.has(code)) return RxFilterRead::Unknown;
  valueOut = (int16_t)scale.valueOf(code);
  return RxFilterRead::Ok;
}

bool asciiSetRxFilterLevel(const RadioProfile& sp, RxFilterLevel level, FilterModeGroup group, int16_t value) {
  if (!filterReady(sp, level, true)) return false;
  char cmd[16];
  if (!rxFilterLevelByMode(level)) {
    if (!isYaesuFilter(sp) || !encodeNumber(sp, level, value, cmd, sizeof(cmd))) return false;
  } else {
    RxFilterScale scale;
    if (value < 0 || !rxFilterScaleFor(sp.settings.rxFilter, level, group, scale)) return false;
    const uint8_t code = rxFilterScaleCodeOf(scale, (uint16_t)value);
    if (scale.valueOf(code) != (uint16_t)value || !encodeCode(sp, level, group, code, cmd, sizeof(cmd))) return false;
  }
  if (!asciiSendSimpleCommand(cmd)) return false;
  int16_t readBack = 0;
  return asciiQueryRxFilterLevel(sp, level, group, readBack, kCatReplyTimeoutMs) == RxFilterRead::Ok &&
         readBack == value;
}

bool asciiQueryRxFilterSwitch(const RadioProfile& sp, RxFilterSwitch sw, bool& onOut, uint32_t timeoutMs) {
  if (!isYaesuFilter(sp) || !rxFilterSwitchCap(sp.caps, sw, false)) return false;
  String rest;
  return transactAfterPrefix(yaesuSwitchReadCommand(sw), yaesuSwitchReplyPrefix(sw), rest, timeoutMs) &&
         yaesuSwitchDecode(sw, rest.c_str(), onOut);
}

bool asciiSetRxFilterSwitch(const RadioProfile& sp, RxFilterSwitch sw, bool on) {
  if (!isYaesuFilter(sp) || !rxFilterSwitchCap(sp.caps, sw, true)) return false;
  char cmd[12];
  if (!yaesuSwitchEncode(sw, on, cmd, sizeof(cmd))) return false;
  if (!asciiSendSimpleCommand(cmd)) return false;
  bool readBack = false;
  return asciiQueryRxFilterSwitch(sp, sw, readBack, kCatReplyTimeoutMs) && readBack == on;
}
