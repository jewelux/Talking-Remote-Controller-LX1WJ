// Tests for the QMX Max. PA voltage over the menu manager (firmware/qmx_menu.*), against a
// simulated QMX that behaves as the CAT manual and the qmx-panadapter measurements describe: a
// Set redraws the menu (ANSI bytes on the CAT port), a command sent into the redraw is refused,
// and with "MM Effect" on "On demand" a Set takes effect only after MU;.
#include <cstdlib>
#include <deque>
#include <string>
#include <vector>

#include "qmx_menu.h"
#include "test_runner.h"

namespace {

constexpr char kSetPrefix[] = "MMProtection|Max. PA voltage=";

class FakeQmx : public QmxCatLink {
 public:
  // The radio's settings.
  uint16_t storedTenths = 115;   // in the EEPROM, what a Get reads
  uint16_t runningTenths = 115;  // what the PA runs at
  bool mmEffectImmediate = false;
  uint16_t highestTenths = 150;  // a Set above it is kept at it
  bool hasPaVolt = true;         // false: firmware before the Protection menu

  // How it behaves on the CAT port.
  bool connected = true;
  bool redrawAfterSet = true;
  // The redraw is still coming after the next command is sent, and runs into its reply.
  bool lateRedraw = false;
  uint32_t busyAfterSetMs = 150;  // a command in this time after a Set is refused
  uint8_t refuseSets = 0;         // the next Sets answer "?;" whatever the time
  uint8_t zeroGets = 0;           // the next Gets answer "MM0;", the menu still loading
  const char* getReply = nullptr; // every Get answers this instead

  std::vector<std::string> sent;
  int eepromWrites = 0;

  bool send(const char* cmd) override {
    sent.push_back(cmd);
    input_.clear();
    if (!connected) return true;
    std::string pendingRedraw;
    if (lateRedraw && redrawPending_) pendingRedraw = redraw();
    redrawPending_ = false;
    const std::string c = cmd;
    if (nowMs_ < busyUntilMs_) {
      queue(pendingRedraw + "?;");
      return true;
    }
    if (!hasPaVolt) {
      queue("?;");
    } else if (c == kQmxPaVoltGet && getReply) {
      queue(pendingRedraw + getReply);
    } else if (c == kQmxPaVoltGet && zeroGets) {
      --zeroGets;
      queue(pendingRedraw + "MM0;");
    } else if (c == kQmxPaVoltGet) {
      char reply[16];
      snprintf(reply, sizeof(reply), "MM%u.%u;", storedTenths / 10u, storedTenths % 10u);
      queue(pendingRedraw + reply);
    } else if (c.rfind(kSetPrefix, 0) == 0 && c.back() == ';') {
      set(c.substr(sizeof(kSetPrefix) - 1, c.size() - sizeof(kSetPrefix)));
    } else if (c == "MU;") {
      runningTenths = storedTenths;
    } else {
      queue("?;");
    }
    return true;
  }

  bool readReply(char* out, size_t outLen, uint32_t) override {
    if (input_.empty()) return false;
    std::string reply = input_.front();
    input_.pop_front();
    if (reply.size() >= outLen) reply = reply.substr(reply.size() - (outLen - 1));
    snprintf(out, outLen, "%s", reply.c_str());
    return true;
  }

  void pause(uint32_t ms) override { nowMs_ += ms; }

  int count(const char* cmd) const {
    int n = 0;
    for (const std::string& s : sent) n += s == cmd;
    return n;
  }

 private:
  // The redraw as readReply would split it: control bytes gone, cut at each ';', the end
  // without one.
  static std::string redraw() { return "[2J[1;1HProtection[3;1HMax. PA voltage   [3;20H"; }

  void queue(const std::string& text) {
    size_t start = 0;
    for (size_t i = 0; i < text.size(); ++i) {
      if (text[i] != ';') continue;
      input_.push_back(text.substr(start, i - start + 1));
      start = i + 1;
    }
    if (start < text.size()) input_.push_back(text.substr(start));
  }

  void set(const std::string& value) {
    if (refuseSets) {
      --refuseSets;
      queue("?;");
      return;
    }
    const size_t dot = value.find('.');
    unsigned tenths = (unsigned)atoi(value.c_str()) * 10;
    if (dot != std::string::npos) tenths += (unsigned)(value[dot + 1] - '0');
    storedTenths = (uint16_t)(tenths > highestTenths ? highestTenths : tenths);
    ++eepromWrites;
    if (mmEffectImmediate) runningTenths = storedTenths;
    busyUntilMs_ = nowMs_ + busyAfterSetMs;
    if (!redrawAfterSet) return;
    if (lateRedraw) {
      redrawPending_ = true;
    } else {
      queue(redraw());
    }
  }

  std::deque<std::string> input_;
  uint32_t nowMs_ = 0;
  uint32_t busyUntilMs_ = 0;
  bool redrawPending_ = false;
};

}  // namespace

// --- Values and replies -----------------------------------------------------------------

TEST(pa_volt_range_is_one_to_fifteen_volts) {
  CHECK(!qmxPaVoltValid(9));
  CHECK(qmxPaVoltValid(10));
  CHECK(qmxPaVoltValid(115));
  CHECK(qmxPaVoltValid(150));
  CHECK(!qmxPaVoltValid(151));
}

TEST(pa_volt_set_command_has_one_decimal) {
  char cmd[48];
  CHECK(qmxFormatPaVoltSet(75, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "MMProtection|Max. PA voltage=7.5;");
  CHECK(qmxFormatPaVoltSet(120, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "MMProtection|Max. PA voltage=12.0;");
  CHECK_EQ(kQmxPaVoltGet, "MMProtection|Max. PA voltage;");
}

TEST(pa_volt_set_command_leaves_out_untouched_when_refused) {
  char cmd[48] = "untouched";
  CHECK(!qmxFormatPaVoltSet(9, cmd, sizeof(cmd)));
  CHECK(!qmxFormatPaVoltSet(151, cmd, sizeof(cmd)));
  CHECK_EQ(cmd, "untouched");
  char shortBuf[10] = "untouched";
  CHECK(!qmxFormatPaVoltSet(75, shortBuf, sizeof(shortBuf)));
  CHECK_EQ(shortBuf, "untouched");
}

TEST(typed_volts_take_at_most_one_decimal) {
  uint16_t tenths = 0;
  CHECK(qmxParseVolts("7", tenths));
  CHECK_EQ(tenths, 70);
  CHECK(qmxParseVolts("7.5", tenths));
  CHECK_EQ(tenths, 75);
  CHECK(qmxParseVolts("13.0", tenths));
  CHECK_EQ(tenths, 130);
  CHECK(qmxParseVolts("01.0", tenths));
  CHECK_EQ(tenths, 10);
  tenths = 99;
  CHECK(!qmxParseVolts("7.25", tenths));
  CHECK(!qmxParseVolts("7.", tenths));
  CHECK(!qmxParseVolts(".5", tenths));
  CHECK(!qmxParseVolts("7,5", tenths));
  CHECK(!qmxParseVolts("", tenths));
  CHECK(!qmxParseVolts("0.9", tenths));
  CHECK(!qmxParseVolts("15.1", tenths));
  CHECK(!qmxParseVolts("115.5", tenths));
  CHECK(!qmxParseVolts("99999", tenths));
  CHECK(!qmxParseVolts(nullptr, tenths));
  CHECK_EQ(tenths, 99);
}

TEST(volts_reply_is_read_with_or_without_decimal) {
  uint16_t tenths = 0;
  CHECK(qmxParseVoltsReply("MM11.5;", tenths) == QmxMenuReply::Volts);
  CHECK_EQ(tenths, 115);
  CHECK(qmxParseVoltsReply("MM6;", tenths) == QmxMenuReply::Volts);
  CHECK_EQ(tenths, 60);
  CHECK(qmxParseVoltsReply("MM20.0;", tenths) == QmxMenuReply::Volts);
  CHECK_EQ(tenths, 200);
}

TEST(volts_reply_after_redraw_text_is_read) {
  uint16_t tenths = 0;
  CHECK(qmxParseVoltsReply("[3;20HMM Effect  On demandMM7.5;", tenths) == QmxMenuReply::Volts);
  CHECK_EQ(tenths, 75);
  CHECK(qmxParseVoltsReply("Max. PA voltage   [3;20H?;", tenths) == QmxMenuReply::Refused);
}

// qmx-panadapter: a Get can answer 0 while the menu is still loading; it takes 0.1 to 20 V.
TEST(zero_and_implausible_volts_are_not_a_value) {
  uint16_t tenths = 42;
  CHECK(qmxParseVoltsReply("MM0;", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("MM0.0;", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("MM20.1;", tenths) == QmxMenuReply::Other);
  CHECK_EQ(tenths, 42);
}

TEST(refusal_and_other_replies_are_told_apart) {
  uint16_t tenths = 42;
  CHECK(qmxParseVoltsReply("?;", tenths) == QmxMenuReply::Refused);
  CHECK(qmxParseVoltsReply("MMIAMBIC A;", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("MM;", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("MM7.55;", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("MM7.5", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("FA00007074000;", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("[2J[1", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply("", tenths) == QmxMenuReply::Other);
  CHECK(qmxParseVoltsReply(nullptr, tenths) == QmxMenuReply::Other);
  CHECK_EQ(tenths, 42);
}

// --- Read -------------------------------------------------------------------------------

TEST(read_gives_the_stored_voltage) {
  FakeQmx qmx;
  qmx.storedTenths = 105;
  const QmxPaVoltResult r = qmxReadPaVolt(qmx);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(r.tenths, 105);
  CHECK_EQ(qmx.sent.size(), (size_t)1);
  CHECK_EQ(qmx.sent[0].c_str(), "MMProtection|Max. PA voltage;");
  CHECK_EQ(qmx.eepromWrites, 0);
}

TEST(read_without_a_radio_is_no_reply_after_one_get) {
  FakeQmx qmx;
  qmx.connected = false;
  const QmxPaVoltResult r = qmxReadPaVolt(qmx);
  CHECK(r.status == QmxPaVoltStatus::NoReply);
  CHECK_EQ(qmx.sent.size(), (size_t)1);
}

// Before firmware 1_03_000 there is no Protection menu: the Get is an error.
TEST(read_on_firmware_without_the_item_is_refused) {
  FakeQmx qmx;
  qmx.hasPaVolt = false;
  const QmxPaVoltResult r = qmxReadPaVolt(qmx);
  CHECK(r.status == QmxPaVoltStatus::Refused);
  CHECK_EQ(qmx.count(kQmxPaVoltGet), (int)kQmxPaVoltReadAttempts);
}

TEST(read_asks_again_while_the_menu_loads) {
  FakeQmx qmx;
  qmx.zeroGets = 2;
  const QmxPaVoltResult r = qmxReadPaVolt(qmx);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(r.tenths, 115);
  CHECK_EQ(qmx.count(kQmxPaVoltGet), 3);
}

TEST(read_of_an_answer_that_is_not_volts_keeps_it) {
  FakeQmx qmx;
  qmx.getReply = "MM;";
  const QmxPaVoltResult r = qmxReadPaVolt(qmx);
  CHECK(r.status == QmxPaVoltStatus::Unexpected);
  CHECK_EQ(r.reply, "MM;");
  CHECK_EQ(qmx.count(kQmxPaVoltGet), (int)kQmxPaVoltReadAttempts);
}

// --- Set --------------------------------------------------------------------------------

TEST(set_reads_writes_reads_back_and_applies_with_mu) {
  FakeQmx qmx;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 75);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(r.tenths, 75);
  CHECK_EQ(qmx.storedTenths, 75);
  CHECK_EQ(qmx.runningTenths, 75);
  CHECK_EQ(qmx.eepromWrites, 1);
  CHECK_EQ(qmx.sent.size(), (size_t)4);
  CHECK_EQ(qmx.sent[0].c_str(), "MMProtection|Max. PA voltage;");
  CHECK_EQ(qmx.sent[1].c_str(), "MMProtection|Max. PA voltage=7.5;");
  CHECK_EQ(qmx.sent[2].c_str(), "MMProtection|Max. PA voltage;");
  CHECK_EQ(qmx.sent[3].c_str(), "MU;");
}

TEST(set_to_the_value_held_writes_nothing_but_applies_it) {
  FakeQmx qmx;
  qmx.runningTenths = 90;  // stored by an earlier write, not yet in effect
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 115);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(qmx.eepromWrites, 0);
  CHECK_EQ(qmx.count("MU;"), 1);
  CHECK_EQ(qmx.runningTenths, 115);
}

TEST(set_waits_out_the_menu_redraw) {
  FakeQmx qmx;
  qmx.busyAfterSetMs = 190;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 60);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(qmx.eepromWrites, 1);
  CHECK_EQ(qmx.runningTenths, 60);
}

TEST(set_reads_back_through_a_longer_redraw_without_writing_again) {
  FakeQmx qmx;
  qmx.busyAfterSetMs = 500;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 60);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(qmx.count("MMProtection|Max. PA voltage=6.0;"), 1);
  CHECK_EQ(qmx.runningTenths, 60);
}

TEST(set_reads_back_through_a_late_redraw) {
  FakeQmx qmx;
  qmx.lateRedraw = true;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 82);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(r.tenths, 82);
  CHECK_EQ(qmx.eepromWrites, 1);
  CHECK_EQ(qmx.runningTenths, 82);
}

TEST(set_with_immediate_mm_effect_still_ends_with_mu) {
  FakeQmx qmx;
  qmx.mmEffectImmediate = true;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 90);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(qmx.runningTenths, 90);
  CHECK_EQ(qmx.count("MU;"), 1);
}

TEST(refused_set_is_tried_again) {
  FakeQmx qmx;
  qmx.refuseSets = 2;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 75);
  CHECK(r.status == QmxPaVoltStatus::Ok);
  CHECK_EQ(r.tenths, 75);
  CHECK_EQ(qmx.count("MMProtection|Max. PA voltage=7.5;"), 3);
  CHECK_EQ(qmx.eepromWrites, 1);
  CHECK_EQ(qmx.count("MU;"), 1);
  CHECK_EQ(qmx.runningTenths, 75);
}

TEST(set_refused_every_time_gives_up_without_mu) {
  FakeQmx qmx;
  qmx.refuseSets = kQmxPaVoltSetAttempts;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 75);
  CHECK(r.status == QmxPaVoltStatus::Mismatch);
  CHECK_EQ(r.tenths, 115);
  CHECK_EQ(qmx.count("MMProtection|Max. PA voltage=7.5;"), (int)kQmxPaVoltSetAttempts);
  CHECK_EQ(qmx.count("MU;"), 0);
  CHECK_EQ(qmx.runningTenths, 115);
}

// The QMX stored something else: writing the same again would not change that.
TEST(set_the_radio_stores_otherwise_is_one_write_and_a_mismatch) {
  FakeQmx qmx;
  qmx.highestTenths = 110;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 120);
  CHECK(r.status == QmxPaVoltStatus::Mismatch);
  CHECK_EQ(r.tenths, 110);
  CHECK_EQ(qmx.eepromWrites, 1);
  CHECK_EQ(qmx.count("MU;"), 0);
}

// The value may be stored: no second write, and no MU; for a value that was not read back.
TEST(set_whose_read_back_keeps_failing_is_not_written_again) {
  FakeQmx qmx;
  qmx.busyAfterSetMs = 1000;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 75);
  CHECK(r.status == QmxPaVoltStatus::Refused);
  CHECK_EQ(qmx.eepromWrites, 1);
  CHECK_EQ(qmx.count("MMProtection|Max. PA voltage=7.5;"), 1);
  CHECK_EQ(qmx.count("MU;"), 0);
}

TEST(set_out_of_range_sends_nothing) {
  FakeQmx qmx;
  CHECK(qmxSetPaVolt(qmx, 9).status == QmxPaVoltStatus::Invalid);
  CHECK(qmxSetPaVolt(qmx, 151).status == QmxPaVoltStatus::Invalid);
  CHECK(qmx.sent.empty());
}

TEST(set_without_a_radio_writes_nothing) {
  FakeQmx qmx;
  qmx.connected = false;
  const QmxPaVoltResult r = qmxSetPaVolt(qmx, 75);
  CHECK(r.status == QmxPaVoltStatus::NoReply);
  CHECK_EQ(qmx.sent.size(), (size_t)1);
  CHECK_EQ(qmx.count("MU;"), 0);
}
