#pragma once

// QRP Labs QMX menu manager (MM) over CAT: the Max. PA voltage of the Protection menu, which sets
// the output power. Without Arduino so the host tests can run it against a simulated QMX
// (tests/qmx).
//
// From the CAT manual (firmware 1_04_003), "Menu manager commands MM and ML": a Get is the menu
// path and ';', a Set the path, '=' and the value. A Set is stored in the QMX's EEPROM. Whether it
// takes effect at once depends on the QMX's "MM Effect" setting (System config, CAT config); on
// "On demand" it waits for the menu to be entered and left, or for MU;. An error is "?;".
//
// What the manual leaves out, measured on QMX radios by the qmx-panadapter project
// (github.com/SteffenLav/qmx-panadapter, main/cat/cat.c, 2026):
// - the Get reply is the bare value with one decimal: "MM11.5;". The Set takes the same form.
// - an MM Set makes the QMX redraw its menu, which sends ANSI cursor bytes to the CAT port;
//   a command sent into that redraw is refused with "?;", so the next one waits.
// - with "MM Effect" on "On demand" (its default) the radio kept full power after a Set until
//   MU; was sent. MU; reloads the whole configuration; it is sent once the value reads back.
// - an MM Get can answer 0 while the QMX is still loading its menu; it accepts only 0.1 to 20 V.

#include <stddef.h>
#include <stdint.h>

// 1.0 V: below it the PA voltage lowers the output no further (operating manual, Protection
// menu). 15.0 V: above the 12 V top of the QMX supply range, so that a radio run from a higher
// supply with this limit can be given its value back. The factory setting is 11.5 V.
constexpr uint16_t kQmxPaVoltMinTenths = 10;
constexpr uint16_t kQmxPaVoltMaxTenths = 150;

bool qmxPaVoltValid(uint16_t tenths);

// The Get command: "MMProtection|Max. PA voltage;".
extern const char kQmxPaVoltGet[];

// "MMProtection|Max. PA voltage=7.5;". False, out untouched, when tenths is out of range or out
// is too short.
bool qmxFormatPaVoltSet(uint16_t tenths, char* out, size_t outLen);

// Volts as typed, "7", "7.5" or "11.5": whole volts with at most one decimal. False on anything
// else, or outside the range.
bool qmxParseVolts(const char* text, uint16_t& tenthsOut);

// One CAT reply, read up to its ';'. Menu redraw text before it is skipped.
enum class QmxMenuReply : uint8_t {
  Volts,    // "MM7.5;" or "MM12;", 0.1 to 20 V
  Refused,  // "?;"
  Other,    // anything else: menu redraw bytes, another reply, "MM0;", MM text that is not volts
};

QmxMenuReply qmxParseVoltsReply(const char* line, uint16_t& tenthsOut);

// The CAT port the sequences below talk through. The firmware wraps transport_serial; the host
// tests put a simulated QMX behind it.
class QmxCatLink {
 public:
  virtual ~QmxCatLink() = default;
  // Drops what is waiting to be read, then sends cmd.
  virtual bool send(const char* cmd) = 0;
  // The next reply up to and including ';', without control bytes; a longer one keeps its end.
  // False when none came in timeoutMs.
  virtual bool readReply(char* out, size_t outLen, uint32_t timeoutMs) = 0;
  virtual void pause(uint32_t ms) = 0;
};

enum class QmxPaVoltStatus : uint8_t {
  Ok,
  Invalid,     // the value is out of range
  NoReply,     // the QMX did not answer
  Refused,     // it answered "?;"
  Unexpected,  // it answered, but not with volts; reply holds the last answer
  Mismatch,    // Set only: the QMX holds another value than the one asked for or the one before
};

struct QmxPaVoltResult {
  QmxPaVoltStatus status = QmxPaVoltStatus::NoReply;
  uint16_t tenths = 0;  // Ok and Mismatch: the value the QMX holds
  char reply[48] = "";  // Unexpected: the last reply, for the console
};

// Reads the Max. PA voltage. Skips replies that are not volts, such as menu redraw bytes; asks
// again, up to kQmxPaVoltReadAttempts times, after "?;" or an answer that is not volts.
constexpr uint8_t kQmxPaVoltReadAttempts = 3;
QmxPaVoltResult qmxReadPaVolt(QmxCatLink& link);

// Sets the Max. PA voltage: reads it, writes it, and reads it back. When the read-back still
// shows the value before, the write was refused or lost and is sent again, up to
// kQmxPaVoltSetAttempts times. A failed read is not answered with another write. Once the QMX
// holds the value, MU; makes it take effect.
constexpr uint8_t kQmxPaVoltSetAttempts = 3;
QmxPaVoltResult qmxSetPaVolt(QmxCatLink& link, uint16_t tenths);
