#include "ui_console_ft847.h"

#include "protocol_ft847.h"
#include "protocol_yaesu_cat.h"
#include "radio_catalog.h"

static constexpr uint32_t FT847_CONSOLE_TIMEOUT_MS = 800;

static bool isFt847Profile() { return currentProtocolType() == PROTO_YAESU_FT847; }

// Ten hex digits, spaces allowed: "0000000003" or "00 00 00 00 03".
static bool parseFrame(const String& text, uint8_t frame[5]) {
  String hex;
  for (size_t i = 0; i < text.length(); ++i) {
    const char c = text[i];
    if (c == ' ') continue;
    if (!isxdigit((unsigned char)c)) return false;
    hex += c;
  }
  if (hex.length() != 10) return false;
  for (int i = 0; i < 5; ++i) {
    if (!parseHexByteString(hex.substring(i * 2, i * 2 + 2), frame[i])) return false;
  }
  return true;
}

static void printHexByte(uint8_t v) {
  Serial.print("0x");
  Serial.print(byteToUpperHex(v));
}

static void printRxStatus(uint8_t rx) {
  uint8_t s = 0;
  uint8_t over = 0;
  ft847DecodeSMeter(rx, s, over);
  Serial.print("F847 RX STATUS ");
  printHexByte(rx);
  Serial.print(": S-meter dots=");
  Serial.print(ft847SMeterDots(rx));
  Serial.print(" -> S");
  Serial.print(s);
  if (over) {
    Serial.print("+");
    Serial.print(over);
    Serial.print("dB");
  }
  Serial.print(", squelch ");
  Serial.print(ft847SquelchOpen(rx) ? "open" : "closed");
  Serial.print(", CTCSS/DCS ");
  Serial.print((rx & 0x40) ? "unmatched" : "matched");
  Serial.print(", discriminator ");
  Serial.println((rx & 0x20) ? "off-center" : "centered");
}

static void printTxStatus(uint8_t tx) {
  Serial.print("F847 TX STATUS ");
  printHexByte(tx);
  Serial.print(": ");
  Serial.print(ft847TxStatusTransmitting(tx) ? "TX" : "RX");
  Serial.print(", PO/ALC meter=");
  Serial.println(ft847TxMeter(tx));
}

static const char* modeName(uint8_t base) {
  switch (base) {
    case FT847_MODE_LSB: return "LSB";
    case FT847_MODE_USB: return "USB";
    case FT847_MODE_CW: return "CW";
    case FT847_MODE_CWR: return "CW-R";
    case FT847_MODE_AM: return "AM";
    case FT847_MODE_FM: return "FM";
    default: return "?";
  }
}

void printFt847ConsoleHelp() {
  Serial.println("  Yaesu FT-847:");
  Serial.println("    RXTX?  SM?  (also FREQ, MODE and the keypad)");
  Serial.println("    F847?                      (CAT state and byte gap)");
  Serial.println("    F847CAT ON | OFF           (OFF: nothing is sent until ON)");
  Serial.println("    F847GAP <0..200> | F847GAP?  (ms between the 5 bytes, default 50)");
  Serial.println("    F847RX?                    (receiver status 0xE7, decoded)");
  Serial.println("    F847TX?                    (transmit status 0xF7, decoded)");
  Serial.println("    F847MODE?                  (mode byte, narrow filter included)");
  Serial.println("    F847TRACE ON | OFF         (print every frame sent and received)");
  Serial.println("    F847RAW <10 hex digits>    (send a frame, no reply expected)");
  Serial.println("    F847RAW1? <10 hex digits>  (send a frame, read 1 byte)");
  Serial.println("    F847RAW5? <10 hex digits>  (send a frame, read 5 bytes)");
  Serial.println("    CAUTION: F847RAW sends anything, e.g. 0000000008 = PTT ON (transmit).");
}

bool handleConsoleFt847Commands(const String& upper) {
  if (!upper.startsWith("F847")) return false;
  if (!isFt847Profile()) {
    Serial.println("F847 -> FT-847 profile required");
    return true;
  }

  if (upper == "F847?") {
    Serial.print("F847 CAT ");
    Serial.print(ft847CatHeldOff() ? "held OFF (F847CAT ON to resume)" : "on (sent automatically)");
    Serial.print(", byte gap ");
    Serial.print(ft847ByteGapMs());
    Serial.print(" ms, trace ");
    Serial.println(g_yaesuCatTrace ? "on" : "off");
    return true;
  }
  if (upper == "F847CAT ON") {
    ft847CatOn();
    Serial.println("F847 CAT ON sent");
    return true;
  }
  if (upper == "F847CAT OFF") {
    ft847CatOff();
    Serial.println("F847 CAT OFF sent; nothing more is sent until F847CAT ON");
    return true;
  }
  if (upper == "F847GAP?") {
    Serial.print("F847 byte gap ");
    Serial.print(ft847ByteGapMs());
    Serial.println(" ms");
    return true;
  }
  if (upper.startsWith("F847GAP ")) {
    const long ms = upper.substring(8).toInt();
    if (ms < 0 || ms > 200) {
      Serial.println("F847GAP -> 0..200 ms");
      return true;
    }
    ft847SetByteGapMs((uint32_t)ms);
    Serial.print("F847 byte gap ");
    Serial.print(ft847ByteGapMs());
    Serial.println(" ms");
    return true;
  }
  if (upper == "F847TRACE ON" || upper == "F847TRACE OFF") {
    g_yaesuCatTrace = upper.endsWith("ON");
    Serial.println(g_yaesuCatTrace ? "F847 trace on" : "F847 trace off");
    return true;
  }
  if (upper == "F847RX?") {
    uint8_t rx = 0;
    if (!ft847QueryRxStatus(rx, FT847_CONSOLE_TIMEOUT_MS)) {
      Serial.println("F847 RX STATUS -> no reply");
      return true;
    }
    printRxStatus(rx);
    return true;
  }
  if (upper == "F847TX?") {
    uint8_t tx = 0;
    if (!ft847QueryTxStatus(tx, FT847_CONSOLE_TIMEOUT_MS)) {
      Serial.println("F847 TX STATUS -> no reply");
      return true;
    }
    printTxStatus(tx);
    return true;
  }
  if (upper == "F847MODE?") {
    uint8_t modeByte = 0;
    if (!ft847QueryModeByte(modeByte, FT847_CONSOLE_TIMEOUT_MS)) {
      Serial.println("F847 MODE -> no reply or misaligned frame");
      return true;
    }
    Serial.print("F847 MODE ");
    printHexByte(modeByte);
    Serial.print(": ");
    Serial.print(modeName(ft847ModeBase(modeByte)));
    Serial.println(ft847ModeNarrow(modeByte) ? " (narrow)" : "");
    return true;
  }
  if (upper.startsWith("F847RAW ") || upper.startsWith("F847RAW1? ") || upper.startsWith("F847RAW5? ")) {
    const int space = upper.indexOf(' ');
    uint8_t frame[5];
    if (!parseFrame(upper.substring(space + 1), frame)) {
      Serial.println("F847RAW -> use 10 hex digits, e.g. 0000000003");
      return true;
    }
    if (upper.startsWith("F847RAW ")) {
      ft847SendRaw(frame);
      Serial.print("F847 SENT: ");
      yaesuCatPrintFrame(frame);
      Serial.println();
      return true;
    }
    if (upper.startsWith("F847RAW1? ")) {
      uint8_t rsp = 0;
      if (!ft847QueryRaw1(frame, rsp, FT847_CONSOLE_TIMEOUT_MS)) {
        Serial.println("F847RAW1? -> no reply");
        return true;
      }
      Serial.print("F847 RX: ");
      printHexByte(rsp);
      Serial.println();
      return true;
    }
    uint8_t rsp[5] = {0};
    if (!ft847QueryRaw5(frame, rsp, FT847_CONSOLE_TIMEOUT_MS)) {
      Serial.println("F847RAW5? -> no reply (or fewer than 5 bytes)");
      return true;
    }
    Serial.print("F847 RX: ");
    yaesuCatPrintFrame(rsp);
    Serial.println();
    return true;
  }
  Serial.println("F847 -> unknown command, type HELP");
  return true;
}
