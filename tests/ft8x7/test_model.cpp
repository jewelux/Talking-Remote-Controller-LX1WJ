// Tests for the FT-8x7 model from a profile's RadioModel (firmware/ft8x7_model.h).
#include "ft8x7_model.h"
#include "test_runner.h"

TEST(model_from_ft8x7_radio_models) {
  CHECK_EQ(ft8x7ModelFor(RadioModel::Ft817), Ft8x7Model::Ft817);
  CHECK_EQ(ft8x7ModelFor(RadioModel::Ft818), Ft8x7Model::Ft818);
  CHECK_EQ(ft8x7ModelFor(RadioModel::Ft857), Ft8x7Model::Ft857);
}

TEST(model_is_none_for_other_radio_models) {
  CHECK_EQ(ft8x7ModelFor(RadioModel::Generic), Ft8x7Model::None);
  CHECK_EQ(ft8x7ModelFor(RadioModel::Ic7760), Ft8x7Model::None);
  CHECK_EQ(ft8x7ModelFor(RadioModel::Ftdx10), Ft8x7Model::None);
}

TEST(ft818_is_in_the_ft817_family) {
  CHECK(ft8x7IsFt817Family(Ft8x7Model::Ft817));
  CHECK(ft8x7IsFt817Family(Ft8x7Model::Ft818));
  CHECK(!ft8x7IsFt817Family(Ft8x7Model::Ft857));
  CHECK(!ft8x7IsFt817Family(Ft8x7Model::None));
}
