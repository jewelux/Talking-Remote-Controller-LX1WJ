#include "qmx_menu.h"

#include <stdio.h>
#include <string.h>

namespace {

constexpr char kPaVoltPath[] = "Protection|Max. PA voltage";

// A reply, and the replies a read skips (menu redraw bytes), have this long to come.
constexpr uint32_t kReplyTimeoutMs = 800;
// After a Set: the menu redraw the QMX sends, and the gap before it takes the next command.
constexpr uint32_t kSetSettleMs = 200;
// Before a Get is asked again.
constexpr uint32_t kReadRetryMs = 200;
// After MU;, while the QMX reloads its configuration.
constexpr uint32_t kReloadSettleMs = 250;
// The volts a Get can answer: 0 while the menu is still loading, and anything above 20 V, is
// not a value.
constexpr uint16_t kPlausibleMaxTenths = 200;
// Replies skipped before a read gives up: a menu redraw is a few dozen.
constexpr uint8_t kMaxSkippedReplies = 64;

// Whole volts with at most one decimal, from text up to end; the same form the QMX answers in.
bool parseTenths(const char* text, const char* end, uint16_t& tenthsOut) {
  uint32_t whole = 0;
  const char* p = text;
  while (p < end && *p >= '0' && *p <= '9') {
    whole = whole * 10 + (uint32_t)(*p - '0');
    if (whole > 999) return false;
    ++p;
  }
  if (p == text) return false;
  uint32_t tenth = 0;
  if (p < end && *p == '.') {
    ++p;
    if (p == end || *p < '0' || *p > '9') return false;
    tenth = (uint32_t)(*p - '0');
    ++p;
  }
  if (p != end) return false;
  tenthsOut = (uint16_t)(whole * 10 + tenth);
  return true;
}

// One Get: Ok, Refused, NoReply, or Unexpected when only other replies came.
QmxPaVoltResult readOnce(QmxCatLink& link) {
  QmxPaVoltResult result;
  if (!link.send(kQmxPaVoltGet)) return result;
  char reply[sizeof(result.reply)];
  for (uint8_t skipped = 0; skipped <= kMaxSkippedReplies; ++skipped) {
    if (!link.readReply(reply, sizeof(reply), kReplyTimeoutMs)) return result;
    uint16_t tenths = 0;
    const QmxMenuReply kind = qmxParseVoltsReply(reply, tenths);
    if (kind == QmxMenuReply::Refused) {
      result.status = QmxPaVoltStatus::Refused;
      return result;
    }
    if (kind == QmxMenuReply::Volts) {
      result.status = QmxPaVoltStatus::Ok;
      result.tenths = tenths;
      return result;
    }
    result.status = QmxPaVoltStatus::Unexpected;
    snprintf(result.reply, sizeof(result.reply), "%s", reply);
  }
  return result;
}

}  // namespace

const char kQmxPaVoltGet[] = "MMProtection|Max. PA voltage;";

bool qmxPaVoltValid(uint16_t tenths) {
  return tenths >= kQmxPaVoltMinTenths && tenths <= kQmxPaVoltMaxTenths;
}

bool qmxFormatPaVoltSet(uint16_t tenths, char* out, size_t outLen) {
  if (!out || !qmxPaVoltValid(tenths)) return false;
  char cmd[48];
  const int n = snprintf(cmd, sizeof(cmd), "MM%s=%u.%u;", kPaVoltPath, (unsigned)(tenths / 10),
                         (unsigned)(tenths % 10));
  if (n <= 0 || (size_t)n >= outLen) return false;
  memcpy(out, cmd, (size_t)n + 1);
  return true;
}

bool qmxParseVolts(const char* text, uint16_t& tenthsOut) {
  if (!text) return false;
  uint16_t tenths = 0;
  if (!parseTenths(text, text + strlen(text), tenths) || !qmxPaVoltValid(tenths)) return false;
  tenthsOut = tenths;
  return true;
}

// Menu redraw text without a ';' of its own runs into the reply after it, so the reply is the
// end of the line: from its last "MM", or a closing "?;".
QmxMenuReply qmxParseVoltsReply(const char* line, uint16_t& tenthsOut) {
  if (!line) return QmxMenuReply::Other;
  const size_t len = strlen(line);
  if (len < 2 || line[len - 1] != ';') return QmxMenuReply::Other;
  if (line[len - 2] == '?') return QmxMenuReply::Refused;
  const char* reply = nullptr;
  for (const char* p = strstr(line, "MM"); p; p = strstr(p + 1, "MM")) reply = p;
  uint16_t tenths = 0;
  if (!reply || !parseTenths(reply + 2, line + len - 1, tenths)) return QmxMenuReply::Other;
  if (tenths == 0 || tenths > kPlausibleMaxTenths) return QmxMenuReply::Other;
  tenthsOut = tenths;
  return QmxMenuReply::Volts;
}

QmxPaVoltResult qmxReadPaVolt(QmxCatLink& link) {
  QmxPaVoltResult result;
  for (uint8_t attempt = 0; attempt < kQmxPaVoltReadAttempts; ++attempt) {
    if (attempt) link.pause(kReadRetryMs);
    result = readOnce(link);
    if (result.status == QmxPaVoltStatus::Ok || result.status == QmxPaVoltStatus::NoReply) break;
  }
  return result;
}

QmxPaVoltResult qmxSetPaVolt(QmxCatLink& link, uint16_t tenths) {
  QmxPaVoltResult result;
  char cmd[48];
  if (!qmxFormatPaVoltSet(tenths, cmd, sizeof(cmd))) {
    result.status = QmxPaVoltStatus::Invalid;
    return result;
  }
  result = qmxReadPaVolt(link);
  if (result.status != QmxPaVoltStatus::Ok) return result;
  const uint16_t before = result.tenths;
  // Written again only while the QMX still shows the value before: a refused or lost write.
  for (uint8_t attempt = 0; attempt < kQmxPaVoltSetAttempts && result.tenths == before &&
                            result.tenths != tenths;
       ++attempt) {
    link.send(cmd);
    link.pause(kSetSettleMs);
    result = qmxReadPaVolt(link);
    if (result.status != QmxPaVoltStatus::Ok) return result;
  }
  if (result.tenths != tenths) {
    result.status = QmxPaVoltStatus::Mismatch;
    return result;
  }
  // Also when it held the value already: an earlier write may be stored but not yet in effect.
  link.send("MU;");
  link.pause(kReloadSettleMs);
  return result;
}
