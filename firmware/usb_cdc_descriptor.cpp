#include "usb_cdc_descriptor.h"

namespace {

constexpr uint8_t kDescInterface = 0x04;
constexpr uint8_t kDescEndpoint = 0x05;
constexpr uint8_t kClassComm = 0x02;
constexpr uint8_t kClassCdcData = 0x0A;
constexpr uint8_t kSubclassAcm = 0x02;
constexpr uint8_t kEpDirIn = 0x80;
constexpr uint8_t kEpTypeMask = 0x03;
constexpr uint8_t kEpTypeBulk = 0x02;

}  // namespace

bool findCdcSerialPort(const uint8_t* config, size_t length, uint8_t index, CdcPortDesc& out) {
  if (!config || length < 2) return false;
  uint8_t lastComm = kCdcNoInterface;
  // The data interface being read, and its bulk endpoints so far.
  bool inData = false;
  CdcPortDesc candidate;
  uint8_t found = 0;

  // A data interface ends at the next interface descriptor or at the end of the configuration.
  auto finishData = [&]() -> bool {
    if (!inData) return false;
    inData = false;
    if (!candidate.epIn || !candidate.epOut) return false;
    if (found++ != index) return false;
    out = candidate;
    return true;
  };

  size_t pos = 0;
  while (pos + 2 <= length) {
    const uint8_t len = config[pos];
    const uint8_t type = config[pos + 1];
    if (len < 2 || pos + len > length) break;  // a broken descriptor ends the walk
    const uint8_t* d = config + pos;
    if (type == kDescInterface && len >= 9) {
      if (finishData()) return true;
      const uint8_t number = d[2];
      const uint8_t alt = d[3];
      const uint8_t cls = d[5];
      const uint8_t subclass = d[6];
      if (alt == 0) {
        if (cls == kClassComm && subclass == kSubclassAcm) {
          lastComm = number;
        } else if (cls == kClassCdcData) {
          inData = true;
          candidate = CdcPortDesc{};
          candidate.dataIntf = number;
          candidate.commIntf = lastComm;
        }
      }
    } else if (type == kDescEndpoint && len >= 7 && inData) {
      const uint8_t address = d[2];
      const uint8_t attributes = d[3];
      if ((attributes & kEpTypeMask) == kEpTypeBulk) {
        if (address & kEpDirIn) {
          candidate.epIn = address;
        } else {
          candidate.epOut = address;
        }
      }
    }
    pos += len;
  }
  return finishData();
}
