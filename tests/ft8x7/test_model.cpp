// Tests for the FT-8x7 model from a profile variant (firmware/ft8x7_model.h).
#include "ft8x7_model.h"
#include "test_runner.h"

TEST(model_from_known_variants) {
  CHECK_EQ(ft8x7ModelForVariant("ft817"), Ft8x7Model::Ft817);
  CHECK_EQ(ft8x7ModelForVariant("ft818"), Ft8x7Model::Ft818);
  CHECK_EQ(ft8x7ModelForVariant("ft857_897"), Ft8x7Model::Ft857);
}

TEST(model_is_none_for_unknown_variants) {
  CHECK_EQ(ft8x7ModelForVariant(nullptr), Ft8x7Model::None);
  CHECK_EQ(ft8x7ModelForVariant(""), Ft8x7Model::None);
  CHECK_EQ(ft8x7ModelForVariant("FT817"), Ft8x7Model::None);
  CHECK_EQ(ft8x7ModelForVariant("ft857"), Ft8x7Model::None);
  CHECK_EQ(ft8x7ModelForVariant("ic7760"), Ft8x7Model::None);
}

TEST(ft818_is_in_the_ft817_family) {
  CHECK(ft8x7IsFt817Family(Ft8x7Model::Ft817));
  CHECK(ft8x7IsFt817Family(Ft8x7Model::Ft818));
  CHECK(!ft8x7IsFt817Family(Ft8x7Model::Ft857));
  CHECK(!ft8x7IsFt817Family(Ft8x7Model::None));
}
