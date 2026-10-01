#include "protocol_yaesu_cat.h"

#include "transport_serial.h"

// FT-8x7 CAT has no framing: a reply that arrives after its reader timed out
// would be read as the start of the next reply. After a timeout, the next
// transaction waits for the line to go quiet instead of flushing instantly.
static constexpr uint32_t YAESU_CAT_LATE_REPLY_WINDOW_MS = 300;
static constexpr uint32_t YAESU_CAT_LINE_QUIET_MS = 30;
// Minimum gap between two commands so the radio's CAT parser keeps up.
static constexpr uint32_t YAESU_CAT_MIN_COMMAND_GAP_MS = 20;

static bool s_lineDirty = false;
static uint32_t s_lineDirtySinceMs = 0;
// Earliest time the next command may be sent.
static uint32_t s_nextTxAllowedMs = 0;

static void yaesuCatTraceFrame(const char* label, const uint8_t data[5]) {
  if (!g_yaesuCatTrace || !Serial) return;
  Serial.print("[YCAT] ");
  Serial.print(millis());
  Serial.print(" ms ");
  Serial.print(label);
  Serial.print(": ");
  yaesuCatPrintFrame(data);
  Serial.println();
}

static void yaesuCatTraceByte(const char* label, uint8_t data) {
  if (!g_yaesuCatTrace || !Serial) return;
  Serial.print("[YCAT] ");
  Serial.print(millis());
  Serial.print(" ms ");
  Serial.print(label);
  Serial.print(": 0x");
  if (data < 0x10) Serial.print('0');
  Serial.println(data, HEX);
}

void yaesuCatSniff(uint32_t windowMs) {
  const uint32_t start = millis();
  uint16_t count = 0;
  while (millis() - start < windowMs) {
    while (serialTransportAvailable()) {
      const uint8_t b = (uint8_t)serialTransportRead();
      yaesuCatTraceByte("SNIFF", b);
      ++count;
    }
    delay(1);
  }
  if (g_yaesuCatTrace && Serial) {
    Serial.print("[YCAT] ");
    Serial.print(millis());
    Serial.print(" ms SNIFF DONE bytes=");
    Serial.println(count);
  }
}

void yaesuCatMarkLineDirty() {
  s_lineDirty = true;
  s_lineDirtySinceMs = millis();
}

// Discards input until the line has been quiet for YAESU_CAT_LINE_QUIET_MS, bounded by
// the end of the late-reply window.
static void yaesuCatDrainLateReply() {
  uint32_t lastActivityMs = millis();
  while (millis() - s_lineDirtySinceMs < YAESU_CAT_LATE_REPLY_WINDOW_MS) {
    if (serialTransportAvailable()) {
      yaesuCatTraceByte("DRAIN", (uint8_t)serialTransportRead());
      lastActivityMs = millis();
    } else if (millis() - lastActivityMs >= YAESU_CAT_LINE_QUIET_MS) {
      break;
    } else {
      delay(1);
    }
  }
}

void yaesuCatFlushInput() {
  if (s_lineDirty) {
    yaesuCatDrainLateReply();
    s_lineDirty = false;
  }
  serialTransportFlushInput();
}

void yaesuCatNoteLineOpened() {
  s_nextTxAllowedMs = millis() + YAESU_CAT_MIN_COMMAND_GAP_MS;
}

void yaesuCatSend5(const uint8_t data[5]) {
  const int32_t waitMs = (int32_t)(s_nextTxAllowedMs - millis());
  if (waitMs > 0) delay((uint32_t)waitMs);
  yaesuCatTraceFrame("TX", data);
  serialTransportWrite(data, 5);
  serialTransportFlushOutput();
  s_nextTxAllowedMs = millis() + YAESU_CAT_MIN_COMMAND_GAP_MS;
}

bool yaesuCatRead1(uint8_t& out, uint32_t timeoutMs) {
  g_radioReplyTimedOut = false;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (serialTransportAvailable()) {
      out = (uint8_t)serialTransportRead();
      yaesuCatTraceByte("RX1", out);
      return true;
    }
    delay(1);
  }
  yaesuCatMarkLineDirty();
  g_radioReplyTimedOut = true;
  return false;
}

bool yaesuCatRead5(uint8_t out[5], uint32_t timeoutMs) {
  g_radioReplyTimedOut = false;
  uint32_t start = millis();
  size_t n = 0;
  while (millis() - start < timeoutMs) {
    while (serialTransportAvailable()) {
      out[n] = (uint8_t)serialTransportRead();
      yaesuCatTraceByte("RX", out[n]);
      ++n;
      if (n >= 5) {
        yaesuCatTraceFrame("RX5", out);
        return true;
      }
    }
    delay(1);
  }
  yaesuCatMarkLineDirty();
  g_radioReplyTimedOut = true;
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
