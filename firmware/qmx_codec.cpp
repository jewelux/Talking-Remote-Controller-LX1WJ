#include "qmx_codec.h"

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
