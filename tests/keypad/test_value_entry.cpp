// Tests for the value entry model (firmware/keypad_value_entry.*).
#include "keypad_value_entry.h"
#include "test_runner.h"

#include <string>

namespace {

// RF power: tenths of a watt, whole watts typed, 5..100 W, 1 W up to 20 W and 5 W above.
int32_t stepPower(int32_t tenths, int8_t dir) {
  if (dir > 0) return tenths < 200 ? tenths + 10 : (tenths / 50 + 1) * 50;
  return tenths <= 200 ? tenths - 10 : (tenths - 1) / 50 * 50;
}

ValueSpec powerSpec() {
  ValueSpec spec;
  spec.name = "RFPOWER";
  spec.min = 50;
  spec.max = 1000;
  spec.scale = 1;
  spec.maxDigits = 3;
  spec.stepFn = stepPower;
  return spec;
}

ValueSpec toneSpec() {
  ValueSpec spec;
  spec.name = "CTCSS";
  spec.min = 670;
  spec.max = 2541;
  spec.scale = 1;
  spec.pointDigits = 1;
  spec.bareInUnits = true;
  spec.maxDigits = 4;
  return spec;
}

ValueSpec offsetSpec() {
  ValueSpec spec;
  spec.name = "RIT";
  spec.min = -9990;
  spec.max = 9990;
  spec.sign = true;
  spec.maxDigits = 4;
  spec.step = 10;
  return spec;
}

EntrySeed seedOf(int32_t value, bool staged = false) {
  EntrySeed seed;
  seed.known = true;
  seed.value = value;
  seed.staged = staged;
  return seed;
}

// Types keys; true when every one was taken.
bool typeAll(ValueEntry& e, const char* keys) {
  bool all = true;
  for (const char* k = keys; *k; ++k) all = e.type(*k) && all;
  return all;
}

int32_t resultOf(const ValueEntry& e) {
  int32_t v = -12345;
  return e.result(v) == ValueEntry::Result::Ok ? v : -12345;
}

}  // namespace

TEST(value_entry_point_and_bare_digits) {
  ValueEntry e;
  e.begin(toneSpec(), EntrySeed());
  CHECK(typeAll(e, "88*5"));
  CHECK_EQ(std::string(e.digits()), std::string("88.5"));
  CHECK_EQ(resultOf(e), 885);
  e.begin(toneSpec(), EntrySeed());
  CHECK(typeAll(e, "885"));
  CHECK_EQ(resultOf(e), 885);
  // Whole hertz with a point: "100*" is 100.0 Hz.
  e.begin(toneSpec(), EntrySeed());
  CHECK(typeAll(e, "100*"));
  CHECK_EQ(resultOf(e), 1000);
}

TEST(value_entry_rejects_extra_digits_and_points) {
  ValueEntry e;
  e.begin(toneSpec(), EntrySeed());
  CHECK(!e.type('*'));  // not first
  CHECK(typeAll(e, "88*5"));
  CHECK(!e.type('5'));  // one digit after the point
  CHECK(!e.type('*'));  // one point
  e.begin(toneSpec(), EntrySeed());
  CHECK(typeAll(e, "2541"));
  CHECK(!e.type('1'));  // four digits
  CHECK(!e.type('A'));
}

TEST(value_entry_scale_without_a_point) {
  ValueEntry e;
  e.begin(powerSpec(), EntrySeed());
  CHECK(typeAll(e, "45"));
  CHECK(!e.type('*'));
  CHECK_EQ(resultOf(e), 450);
}

TEST(value_entry_minus) {
  ValueEntry e;
  e.begin(offsetSpec(), EntrySeed());
  int32_t v = 0;
  CHECK(e.type('*'));
  CHECK(e.result(v) == ValueEntry::Result::None);  // a lone minus is no value
  CHECK(!e.type('*'));
  CHECK(typeAll(e, "120"));
  CHECK_EQ(std::string(e.digits()), std::string("-120"));
  CHECK_EQ(resultOf(e), -120);
}

TEST(value_entry_steps_from_the_seed) {
  ValueEntry e;
  e.begin(powerSpec(), seedOf(450));
  int32_t v = 0;
  CHECK(e.result(v) == ValueEntry::Result::None);
  CHECK(e.current(v));
  CHECK_EQ(v, 450);
  CHECK(e.step(1) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 500);
  CHECK(e.step(-1) == ValueEntry::Step::Moved);
  CHECK(e.step(-1) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 400);
}

TEST(value_entry_steps_from_the_typed_value) {
  ValueEntry e;
  e.begin(powerSpec(), seedOf(450));
  CHECK(typeAll(e, "19"));
  CHECK(e.step(1) == ValueEntry::Step::Moved);
  CHECK_EQ(std::string(e.digits()), std::string(""));
  CHECK_EQ(resultOf(e), 200);
  CHECK(e.step(1) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 250);
  // Typing after a step starts a new value.
  CHECK(typeAll(e, "7"));
  CHECK_EQ(resultOf(e), 70);
}

TEST(value_entry_stops_at_the_ends) {
  ValueEntry e;
  e.begin(powerSpec(), seedOf(960));
  CHECK(e.step(1) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 1000);
  CHECK(e.step(1) == ValueEntry::Step::AtLimit);
  CHECK_EQ(resultOf(e), 1000);
  e.begin(powerSpec(), seedOf(60));
  CHECK(e.step(-1) == ValueEntry::Step::Moved);
  CHECK(e.step(-1) == ValueEntry::Step::AtLimit);
  CHECK_EQ(resultOf(e), 50);
}

// A value typed past an end steps to that end first, whichever key.
TEST(value_entry_typed_out_of_range) {
  ValueEntry e;
  e.begin(powerSpec(), EntrySeed());
  CHECK(typeAll(e, "150"));
  int32_t v = 0;
  CHECK(e.result(v) == ValueEntry::Result::AboveMax);
  CHECK(e.current(v));
  CHECK_EQ(v, 1000);
  CHECK(e.step(-1) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 1000);
  e.begin(powerSpec(), EntrySeed());
  CHECK(typeAll(e, "2"));
  CHECK(e.result(v) == ValueEntry::Result::BelowMin);
  CHECK(e.step(1) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 50);
}

TEST(value_entry_snaps_a_typed_value_to_the_end) {
  ValueEntry e;
  e.begin(powerSpec(), EntrySeed());
  CHECK(typeAll(e, "45"));
  CHECK(!e.snapToRange());
  CHECK(typeAll(e, "0"));
  CHECK(e.snapToRange());
  CHECK_EQ(std::string(e.digits()), std::string(""));
  CHECK_EQ(resultOf(e), 1000);
}

TEST(value_entry_without_a_value) {
  ValueEntry e;
  e.begin(powerSpec(), EntrySeed());
  int32_t v = 0;
  CHECK(e.step(1) == ValueEntry::Step::NoValue);
  CHECK(!e.current(v));
  CHECK(e.result(v) == ValueEntry::Result::None);
}

TEST(value_entry_staged_seed_and_clamped_seed) {
  ValueEntry e;
  e.begin(powerSpec(), seedOf(450, true));
  CHECK_EQ(resultOf(e), 450);
  // A seed outside the range (a band change since) is kept within it.
  e.begin(powerSpec(), seedOf(2000, true));
  CHECK_EQ(resultOf(e), 1000);
}

TEST(value_entry_big_step_stops_at_the_end) {
  ValueEntry e;
  e.begin(powerSpec(), seedOf(180));
  CHECK(e.step(1, 5) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 350);  // 19, 20, 25, 30, 35 watts
  e.begin(powerSpec(), seedOf(900));
  CHECK(e.step(1, 5) == ValueEntry::Step::Moved);
  CHECK_EQ(resultOf(e), 1000);
  CHECK(e.step(1, 5) == ValueEntry::Step::AtLimit);
}
