// The keypad settings table and the spoken names of the keypad labels.
#include "keypad_labels.h"
#include "keypad_settings.h"
#include "test_runner.h"

TEST(settings_rows_follow_the_enum) {
  for (uint8_t i = 0; i < (uint8_t)Setting::Count; ++i) {
    CHECK_EQ(settingInfo((Setting)i).id, (Setting)i);
  }
  CHECK_EQ(settingInfo(Setting::Count).id, Setting::None);
}

// Help mode and the first nudge say a setting's name, so each has one.
TEST(settings_have_spoken_names) {
  for (uint8_t i = (uint8_t)Setting::Home + 1; i < (uint8_t)Setting::Count; ++i) {
    const SettingInfo &info = settingInfo((Setting)i);
    if (!spokenLabelWords(info.label)) fprintf(stderr, "  no spoken name for %s\n", info.label);
    CHECK(spokenLabelWords(info.label) != nullptr);
  }
  CHECK_EQ(settingInfo(Setting::None).label, "");
  CHECK_EQ(settingInfo(Setting::Home).label, "");
}

TEST(settings_with_a_centre) {
  CHECK(settingInfo(Setting::PbtInner).hasCentre);
  CHECK(settingInfo(Setting::RitOffset).hasCentre);
  CHECK(settingInfo(Setting::Profile).hasCentre);
  CHECK(!settingInfo(Setting::NrLevel).hasCentre);
  CHECK(!settingInfo(Setting::RfPower).hasCentre);
}

TEST(spoken_label_words_ignore_the_question_mark) {
  CHECK_EQ(spokenLabelWords("NRLEVEL"), "noisereduction level");
  CHECK_EQ(spokenLabelWords("NRLEVEL?"), "noisereduction level");
  CHECK_EQ(spokenLabelWords("NR?"), "noisereduction");
  CHECK_EQ(spokenLabelWords("VFOA FREQ"), "vfo a frequency");
  CHECK(spokenLabelWords("CWAUTO?") == nullptr);
  CHECK(spokenLabelWords("NRLEVELX") == nullptr);
  CHECK(spokenLabelWords("") == nullptr);
}
