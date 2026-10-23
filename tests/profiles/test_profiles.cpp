// Tests for the built-in radio profile table (firmware/radio_profile_table.cpp).
#include "radio_profile_table.h"
#include "test_runner.h"

namespace {

const RadioProfile& slot(uint8_t number) {
  static const RadioProfile kMissing{};
  const RadioProfile* p = profileForSlot(number);
  CHECK(p != nullptr);
  return p ? *p : kMissing;
}

bool isAscii(ProtocolType protocol) {
  return protocol == PROTO_KENWOOD_ASCII || protocol == PROTO_ELECRAFT_ASCII ||
         protocol == PROTO_YAESU_FTDX_ASCII;
}

}  // namespace

// ---- The table as a whole ----

TEST(slots_are_unique_and_ascending) {
  for (size_t i = 1; i < profileCount(); ++i) {
    CHECK(profileAt(i).slot > profileAt(i - 1).slot);
  }
  CHECK(profileAt(0).slot > 0);
}

TEST(every_radio_offers_its_default_baud) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    CHECK(p.link.bauds.count > 0);
    CHECK(p.link.bauds.contains(p.link.baud));
  }
}

TEST(every_radio_has_a_name_and_voice_digits) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    CHECK(p.name[0] != '\0');
    CHECK(p.voiceDigits[0] != '\0');
    for (const char* c = p.voiceDigits; *c; ++c) CHECK(*c >= '0' && *c <= '9');
  }
}

TEST(civ_radios_have_an_address) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (p.protocol == PROTO_CIV) CHECK(p.link.civAddr != 0);
  }
}

TEST(ascii_radios_have_freq_and_mode_commands) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (!isAscii(p.protocol)) continue;
    CHECK(p.commands->freqGet[0] != '\0');
    CHECK(p.commands->freqSetFormat[0] != '\0');
    CHECK(p.commands->modeGet[0] != '\0');
    CHECK(p.commands->modeSetFormat[0] != '\0');
  }
}

TEST(non_civ_radios_have_mode_codes) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (p.protocol == PROTO_CIV) continue;
    CHECK(p.modes->lsb[0] != '\0');
    CHECK(p.modes->usb[0] != '\0');
  }
}

TEST(ft8x7_radios_have_an_ft8x7_model_and_its_bauds) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (p.protocol != PROTO_YAESU_FT8X7) continue;
    CHECK(p.model == RadioModel::Ft817 || p.model == RadioModel::Ft818 || p.model == RadioModel::Ft857);
    CHECK_EQ(p.link.bauds.count, 3);
    CHECK(p.link.bauds.contains(4800) && p.link.bauds.contains(9600) && p.link.bauds.contains(38400));
    CHECK(!p.link.bauds.contains(19200));
  }
}

// ---- Lookup ----

// Each Icom gets the CI-V commands its own sources confirm (protocol_ops_civ.cpp checks them).
TEST(icom_caps_per_model) {
  const RadioProfile& ic7300 = slot(1);
  CHECK(ic7300.caps.getDialLock && ic7300.caps.getFilterShape && ic7300.caps.getNotchWidth);
  CHECK(ic7300.caps.getTransceive && ic7300.caps.getBandStack && ic7300.caps.getTuner);

  const RadioProfile& ic705 = slot(17);
  CHECK_EQ(ic705.rfPowerMaxWatts, 10);
  CHECK(ic705.caps.getRfPower && ic705.caps.getNrLevel && ic705.caps.getDialLock);
  CHECK(ic705.caps.getVfo && ic705.caps.getSplit && ic705.caps.getRit && ic705.caps.getRxTx);
  CHECK(ic705.caps.getTuner && ic705.caps.getTransceive && ic705.caps.getBandStack);
  CHECK(ic705.caps.getFilterShape && ic705.caps.getNotchWidth);

  const RadioProfile& ic7760 = slot(18);
  CHECK(ic7760.caps.getTuner && ic7760.caps.getDialLock && ic7760.caps.getBandStack);
  CHECK(ic7760.caps.getFilterShape && ic7760.caps.getNotchWidth);
  CHECK(!ic7760.caps.getTransceive && !ic7760.caps.setTransceive);

  const uint8_t ic706[] = {2, 4};
  for (uint8_t s : ic706) {
    CHECK(slot(s).caps.getSmeter);
    CHECK(!slot(s).caps.getPower && !slot(s).caps.getSwr && !slot(s).caps.getDialLock);
  }
}

TEST(profile_for_slot_finds_used_slots_only) {
  CHECK_EQ(slot(1).name, "Icom IC-7300");
  CHECK_EQ(slot(11).name, "Yaesu FTDX-10");
  CHECK_EQ(slot(18).name, "Icom IC-7760");
  CHECK(profileForSlot(0) == nullptr);
  CHECK_EQ(slot(16).name, "Yaesu FT-847");
  CHECK(profileForSlot(19) == nullptr);
  CHECK(profileForSlot(255) == nullptr);
}

TEST(default_slot_is_used) {
  CHECK(profileForSlot(kDefaultProfileSlot) != nullptr);
}

// Slots 1-18 are all in use since the FT-847 took 16; 19-24 are free.
TEST(adjacent_slot_skips_free_slots) {
  CHECK_EQ(adjacentProfileSlot(15, 1), 16);
  CHECK_EQ(adjacentProfileSlot(16, 1), 17);
  CHECK_EQ(adjacentProfileSlot(17, -1), 16);
  CHECK_EQ(adjacentProfileSlot(19, -1), 18);
  CHECK_EQ(adjacentProfileSlot(19, 1), 1);
}

TEST(adjacent_slot_wraps_around) {
  CHECK_EQ(adjacentProfileSlot(18, 1), 1);
  CHECK_EQ(adjacentProfileSlot(1, -1), 18);
}

TEST(adjacent_slot_without_direction_stays) {
  CHECK_EQ(adjacentProfileSlot(7, 0), 7);
}

// ---- Radios whose profile carries something easy to lose ----

TEST(ic7300_and_ic7760_link) {
  CHECK(slot(1).link.port == RadioPort::CivJack);
  CHECK_EQ(slot(1).link.civAddr, 0x94);
  CHECK(slot(3).link.port == RadioPort::Rs232);
  CHECK_EQ(slot(18).link.civAddr, 0xB2);
  CHECK_EQ(slot(18).link.baud, 19200u);
  CHECK_EQ(slot(18).rfPowerMaxWatts, 200);
  CHECK(slot(18).model == RadioModel::Ic7760);
}

// The model picks the CI-V menu addresses, e.g. the VOX delay.
TEST(icom_models) {
  CHECK(slot(1).model == RadioModel::Ic7300);
  CHECK(slot(3).model == RadioModel::Ic7300);
  CHECK(slot(17).model == RadioModel::Ic705);
  CHECK(slot(2).model == RadioModel::Generic);  // IC-706
  CHECK(slot(5).model == RadioModel::Generic);  // Xiegu G106
}

// PROC?, MICGAIN? and VOX? say "not available" where the cap is off, instead of waiting for a
// radio that has no such command.
TEST(transmit_setting_caps) {
  const uint8_t on[] = {1, 3, 17, 18, 8, 9, 10, 14, 11, 12, 13, 15, 7};
  for (uint8_t s : on) {
    CHECK(slot(s).caps.getMicGain);
    CHECK(slot(s).caps.getVox);
  }
  const uint8_t proc[] = {1, 3, 17, 18, 9, 10, 11, 15, 7};
  for (uint8_t s : proc) CHECK(slot(s).caps.getProc);
  const uint8_t off[] = {2, 4, 5, 6, 16};
  for (uint8_t s : off) {
    CHECK(!slot(s).caps.getProc);
    CHECK(!slot(s).caps.getMicGain);
    CHECK(!slot(s).caps.getVox);
  }
  CHECK(!slot(8).caps.getProc);   // FT-817
  CHECK(!slot(14).caps.getProc);  // FT-818
  CHECK(!slot(12).caps.getProc);  // FTDX101D: PR only in Hamlib
  CHECK(!slot(13).caps.getProc);  // FTDX101MP
}

// VD per radio: the FTDX10's step table, ms on the FT-891 and TS-480, none on the FTDX101.
TEST(ascii_vox_delay_and_gain_encodings) {
  CHECK(slot(11).settings.voxDelay == VoxDelayEncoding::YaesuSteps);
  CHECK(slot(12).settings.voxDelay == VoxDelayEncoding::None);
  CHECK(slot(13).settings.voxDelay == VoxDelayEncoding::None);
  const SettingEncodings& ft891 = slot(15).settings;
  CHECK(ft891.voxDelay == VoxDelayEncoding::Ms4);
  CHECK_EQ(ft891.voxDelayMinMs, 30);
  CHECK_EQ(ft891.voxDelayMaxMs, 3000);
  CHECK_EQ(ft891.voxDelayStepMs, 10);
  CHECK_EQ(ft891.voxGainMax, 100);
  const SettingEncodings& ts480 = slot(7).settings;
  CHECK(ts480.voxDelay == VoxDelayEncoding::Ms4);
  CHECK_EQ(ts480.voxDelayMinMs, 0);
  CHECK_EQ(ts480.voxDelayMaxMs, 3000);
  CHECK_EQ(ts480.voxDelayStepMs, 150);
  CHECK_EQ(ts480.voxGainMax, 9);
}

// A VD the ops can encode: a range, and a step for the ms encoding.
TEST(every_vox_delay_encoding_is_complete) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const SettingEncodings& s = profileAt(i).settings;
    if (s.voxDelay == VoxDelayEncoding::None) continue;
    CHECK(isAscii(profileAt(i).protocol));
    CHECK(profileAt(i).caps.getVox);
    CHECK(s.voxDelayMaxMs > s.voxDelayMinMs);
    if (s.voxDelay == VoxDelayEncoding::Ms4) CHECK(s.voxDelayStepMs > 0);
  }
}

TEST(g106_is_ttl_civ_with_freq_and_mode_only) {
  const RadioProfile& g106 = slot(5);
  CHECK(g106.protocol == PROTO_CIV);
  CHECK(g106.link.port == RadioPort::CatTtl);
  CHECK(g106.caps.setMode);
  CHECK(!g106.caps.getSmeter);
  CHECK(!g106.caps.getSwr);
}

TEST(ft847_is_its_own_protocol_on_rs232) {
  const RadioProfile& ft847 = slot(16);
  CHECK(ft847.protocol == PROTO_YAESU_FT847);
  CHECK(ft847.model == RadioModel::Generic);
  CHECK(ft847.link.port == RadioPort::Rs232);
  CHECK_EQ(ft847.link.baud, 4800u);
  CHECK(ft847.link.bauds.contains(9600) && ft847.link.bauds.contains(57600));
  CHECK(!ft847.link.bauds.contains(38400));
  CHECK(ft847.caps.setFreq && ft847.caps.setMode && ft847.caps.getSmeter && ft847.caps.getRxTx);
  CHECK(ft847.caps.getPower);
  CHECK(!ft847.caps.getSwr);
  // Nothing the first FT-847 step cannot do yet.
  CHECK(!ft847.caps.getDialLock && !ft847.caps.setDialLock);
  CHECK(!ft847.caps.getSplit && !ft847.caps.setSplit);
  CHECK(!ft847.caps.getVfo && !ft847.caps.setVfo);
  CHECK_EQ(ft847.modes->usb, "01");
  CHECK_EQ(ft847.modes->am, "04");
  // No FM: a frequency/mode read keys this FT-847 in FM (protocol_ft847.h).
  CHECK_EQ(ft847.modes->fm, "");
  CHECK_EQ(ft847.modes->rtty, "");
  CHECK_EQ(ft847.modes->digi, "");
}

TEST(kx2_has_nb_and_no_rtty_code) {
  CHECK(slot(6).caps.setNb);
  CHECK_EQ(slot(6).modes->rtty, "");
  CHECK_EQ(slot(6).modes->rttyR, "9");
}

TEST(ts480_model_and_nr) {
  CHECK(slot(7).model == RadioModel::Ts480);
  CHECK(slot(7).caps.setNr);
  CHECK_EQ(slot(7).commands->nrGet, "NR;");
}

TEST(ft857_and_ft897_cannot_pick_the_vfo) {
  CHECK(!slot(9).caps.setVfo);
  CHECK(!slot(10).caps.setVfo);
  CHECK(slot(8).caps.setVfo);
  CHECK(slot(14).caps.setVfo);
}

TEST(ftdx10_is_its_own_model_and_reads_mode_of_main) {
  CHECK(slot(11).model == RadioModel::Ftdx10);
  CHECK_EQ(slot(11).commands->modeGet, "MD0;");
  CHECK(slot(11).caps.getVfoMode);
  CHECK(slot(12).model == RadioModel::Generic);
  CHECK(!slot(12).caps.getVfoMode);
}

TEST(ft891_keeps_the_ftdx_lock_tuner_and_split) {
  const RadioProfile& ft891 = slot(15);
  CHECK(ft891.caps.setDialLock);
  CHECK(ft891.caps.startTune);
  CHECK(ft891.caps.setSplit);
  CHECK(!ft891.caps.getNr);
  CHECK_EQ(ft891.commands->lockGet, "LK;");
  CHECK_EQ(ft891.link.baud, 4800u);
}

// Hamlib newcat sends MD0 on every newcat radio, and the FTDX101 locks with LK7 / LK4.
TEST(newcat_mode_and_lock_commands) {
  const uint8_t newcat[] = {11, 12, 13, 15};
  for (uint8_t s : newcat) {
    CHECK_EQ(slot(s).commands->modeGet, "MD0;");
    CHECK_EQ(slot(s).commands->modeSetFormat, "MD0%s;");
    CHECK_EQ(slot(s).commands->modeReplyPrefix, "MD0");
  }
  CHECK_EQ(slot(11).commands->lockOnCmd, "LK1;");
  CHECK_EQ(slot(11).commands->lockOffDigits, "0");
  const uint8_t ftdx101[] = {12, 13};
  for (uint8_t s : ftdx101) {
    CHECK_EQ(slot(s).commands->lockOnCmd, "LK7;");
    CHECK_EQ(slot(s).commands->lockOffCmd, "LK4;");
    CHECK_EQ(slot(s).commands->lockOffDigits, "04");
  }
}

// TS-480: RM; answers all three meters, SWR first; 9 is FSK-R and there is no DIGI.
TEST(ts480_swr_and_mode_codes) {
  const RadioProfile& ts480 = slot(7);
  CHECK_EQ(ts480.commands->swrGet, "RM;");
  CHECK_EQ(ts480.commands->swrReplyPrefix, "RM1");
  CHECK_EQ(ts480.modes->rttyR, "9");
  CHECK_EQ(ts480.modes->digi, "");
}

TEST(model_names_are_set) {
  CHECK_EQ(radioModelName(RadioModel::Generic), "generic");
  CHECK_EQ(radioModelName(RadioModel::Ic7300), "IC-7300");
  CHECK_EQ(radioModelName(RadioModel::Ic705), "IC-705");
  CHECK_EQ(radioModelName(RadioModel::Ft857), "FT-857/897");
  CHECK_EQ(radioModelName(RadioModel::Ftdx10), "FTDX10");
}
