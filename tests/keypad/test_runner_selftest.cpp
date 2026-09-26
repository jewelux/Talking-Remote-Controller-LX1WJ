// Checks the runner's own comparison helper, so a broken CHECK_EQ cannot
// make the real tests pass silently.
#include "test_runner.h"

#include <cstdint>
#include <string>

TEST(runner_compares_c_strings_by_content) {
  char buf[4] = {'1', '2', '\0', '\0'};
  CHECK(test::equal(buf, "12"));
  CHECK(!test::equal(buf, "123"));
  CHECK(!test::equal(buf, ""));
}

TEST(runner_handles_null_c_strings) {
  const char *none = nullptr;
  CHECK(test::equal(none, none));
  CHECK(!test::equal(none, ""));
  CHECK(!test::equal("", none));
}

TEST(runner_compares_std_string_with_literal) {
  CHECK(test::equal(std::string("cancel"), "cancel"));
  CHECK(!test::equal(std::string("cancel"), "clear"));
}

TEST(runner_compares_integers_and_enums) {
  enum class Mode : uint8_t { A, B };
  CHECK_EQ(Mode::B, Mode::B);
  CHECK(!test::equal(Mode::A, Mode::B));
  CHECK_EQ(uint8_t{220}, 220);
  CHECK_EQ('*', '*');
}
