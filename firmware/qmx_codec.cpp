#include "qmx_codec.h"

#include <stdio.h>

namespace {

constexpr int32_t kS0Dbm = -127;
constexpr int32_t kDbPerSUnit = 6;
constexpr int32_t kS9OverS0Db = 9 * kDbPerSUnit;
constexpr int32_t kMaxDbOverS9 = 60;

}  // namespace

void qmxDecodeSMeter(int32_t db, uint8_t& sUnitsOut, uint8_t& dbOverS9Out) {
  const int32_t overS0 = db < 0 ? db - kS0Dbm : db;
  if (overS0 < kS9OverS0Db) {
    sUnitsOut = (uint8_t)(overS0 > 0 ? overS0 / kDbPerSUnit : 0);
    dbOverS9Out = 0;
    return;
  }
  int32_t overS9 = ((overS0 - kS9OverS0Db) / 10) * 10;
  if (overS9 > kMaxDbOverS9) overS9 = kMaxDbOverS9;
  sUnitsOut = 9;
  dbOverS9Out = (uint8_t)overS9;
}

QmxTxMeter qmxParseTxMeter(const char* body, int32_t& valueOut) {
  if (!body) return QmxTxMeter::Invalid;
  while (*body == ' ') ++body;
  if (!*body || *body == ';') return QmxTxMeter::Receive;
  int32_t value = 0;
  const char* p = body;
  while (*p >= '0' && *p <= '9') {
    if (value > 99999999) return QmxTxMeter::Invalid;
    value = value * 10 + (*p - '0');
    ++p;
  }
  while (*p == ' ') ++p;
  if (p == body || (*p && *p != ';')) return QmxTxMeter::Invalid;
  valueOut = value;
  return QmxTxMeter::Reading;
}

void qmxFormatFixed(uint32_t value, uint8_t decimals, char* out, size_t outLen) {
  if (!out || !outLen) return;
  uint32_t scale = 1;
  for (uint8_t i = 0; i < decimals; ++i) scale *= 10;
  snprintf(out, outLen, "%lu.%0*lu", (unsigned long)(value / scale), (int)decimals,
           (unsigned long)(value % scale));
}
