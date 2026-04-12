#include "protocol_yaesu_cat.h"

#include "transport_serial.h"

static void yaesuCatTraceFrame(const char* label, const uint8_t data[5]) {
  if (!g_yaesuCatTrace || !Serial) return;
  Serial.print("[YCAT] ");
  Serial.print(label);
  Serial.print(": ");
  yaesuCatPrintFrame(data);
  Serial.println();
}

static void yaesuCatTraceByte(const char* label, uint8_t data) {
  if (!g_yaesuCatTrace || !Serial) return;
  Serial.print("[YCAT] ");
  Serial.print(label);
  Serial.print(": 0x");
  if (data < 0x10) Serial.print('0');
  Serial.println(data, HEX);
}

void yaesuCatFlushInput() {
  serialTransportFlushInput();
}

void yaesuCatSend5(const uint8_t data[5]) {
  yaesuCatTraceFrame("TX", data);
  serialTransportWrite(data, 5);
  serialTransportFlushOutput();
}

bool yaesuCatRead1(uint8_t& out, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (serialTransportAvailable()) {
      out = (uint8_t)serialTransportRead();
      yaesuCatTraceByte("RX1", out);
      return true;
    }
    delay(1);
  }
  return false;
}

bool yaesuCatRead5(uint8_t out[5], uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t n = 0;
  while (millis() - start < timeoutMs) {
    while (serialTransportAvailable()) {
      out[n++] = (uint8_t)serialTransportRead();
      if (n >= 5) {
        yaesuCatTraceFrame("RX5", out);
        return true;
      }
    }
    delay(1);
  }
  return false;
}

bool yaesuCatTransact1(const uint8_t cmd[5], uint8_t& rsp, uint32_t timeoutMs) {
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  return yaesuCatRead1(rsp, timeoutMs);
}

bool yaesuCatTransact5(const uint8_t cmd[5], uint8_t rsp[5], uint32_t timeoutMs) {
  yaesuCatFlushInput();
  yaesuCatSend5(cmd);
  return yaesuCatRead5(rsp, timeoutMs);
}

void yaesuCatPrintFrame(const uint8_t data[5]) {
  for (int i = 0; i < 5; ++i) {
    if (i) Serial.print(' ');
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
  }
}

uint64_t yaesuCatDecodeFreqHz(const uint8_t data[4]) {
  uint64_t digits = 0;
  for (int i = 0; i < 4; ++i) {
    digits = digits * 100ULL + (uint64_t)(((data[i] >> 4) & 0x0F) * 10 + (data[i] & 0x0F));
  }
  return digits * 10ULL;
}

void yaesuCatEncodeFreqHz(uint64_t hz, uint8_t out[4]) {
  uint64_t units10 = hz / 10ULL;
  char buf[9];
  snprintf(buf, sizeof(buf), "%08llu", (unsigned long long)units10);
  for (int i = 0; i < 4; ++i) {
    uint8_t hi = (uint8_t)(buf[i * 2] - '0');
    uint8_t lo = (uint8_t)(buf[i * 2 + 1] - '0');
    out[i] = (uint8_t)((hi << 4) | lo);
  }
}

void yaesuCatEncodeRepeaterOffsetHz(uint64_t hz, uint8_t out[4]) {
  // FT-817 practical testing shows repeater offset uses the same 10 Hz BCD
  // scaling as the standard Yaesu frequency write path.
  uint64_t units10 = hz / 10ULL;
  char buf[9];
  snprintf(buf, sizeof(buf), "%08llu", (unsigned long long)units10);
  for (int i = 0; i < 4; ++i) {
    uint8_t hi = (uint8_t)(buf[i * 2] - '0');
    uint8_t lo = (uint8_t)(buf[i * 2 + 1] - '0');
    out[i] = (uint8_t)((hi << 4) | lo);
  }
}

bool parseHexByteString(const String& s, uint8_t& valueOut) {
  String t = s;
  t.trim();
  if (!t.length()) return false;
  char* endPtr = nullptr;
  long v = strtol(t.c_str(), &endPtr, 16);
  if (endPtr == t.c_str() || *endPtr != '\0' || v < 0 || v > 255) return false;
  valueOut = (uint8_t)v;
  return true;
}

String byteToUpperHex(uint8_t v) {
  char buf[3];
  snprintf(buf, sizeof(buf), "%02X", v);
  return String(buf);
}
