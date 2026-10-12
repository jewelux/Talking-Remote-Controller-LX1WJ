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

// The NanoVNA's vendor name says it all ("nano v n a").
TEST(every_radio_has_a_name_and_voice_digits) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    CHECK(p.name[0] != '\0');
    if (p.vendor != VoiceVendor::NanoVna) CHECK(p.voiceDigits[0] != '\0');
    for (const char* c = p.voiceDigits; *c; ++c) CHECK((*c >= '0' && *c <= '9') || (*c >= 'a' && *c <= 'z'));
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
    if (p.protocol == PROTO_CIV || p.protocol == PROTO_NANOVNA) continue;
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

// Slots 1-18 are all in use since the FT-847 took 16, the QMX has 21 and the NanoVNA 30; 19-20
// and 22-29 are free.
TEST(adjacent_slot_skips_free_slots) {
  CHECK_EQ(adjacentProfileSlot(15, 1), 16);
  CHECK_EQ(adjacentProfileSlot(16, 1), 17);
  CHECK_EQ(adjacentProfileSlot(17, -1), 16);
  CHECK_EQ(adjacentProfileSlot(18, 1), 21);
  CHECK_EQ(adjacentProfileSlot(21, -1), 18);
  CHECK_EQ(adjacentProfileSlot(19, -1), 18);
  CHECK_EQ(adjacentProfileSlot(19, 1), 21);
  CHECK_EQ(adjacentProfileSlot(21, 1), 30);
  CHECK_EQ(adjacentProfileSlot(30, -1), 21);
}

TEST(adjacent_slot_wraps_around) {
  CHECK_EQ(adjacentProfileSlot(30, 1), 1);
  CHECK_EQ(adjacentProfileSlot(1, -1), 30);
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
  const uint8_t on[] = {1, 3, 17, 18, 8, 9, 10, 14, 11};
  for (uint8_t s : on) {
    CHECK(slot(s).caps.getMicGain);
    CHECK(slot(s).caps.getVox);
  }
  const uint8_t proc[] = {1, 3, 17, 18, 9, 10, 11};
  for (uint8_t s : proc) CHECK(slot(s).caps.getProc);
  const uint8_t off[] = {2, 4, 5, 6, 7, 12, 13, 15, 16};
  for (uint8_t s : off) {
    CHECK(!slot(s).caps.getProc);
    CHECK(!slot(s).caps.getMicGain);
    CHECK(!slot(s).caps.getVox);
  }
  CHECK(!slot(8).caps.getProc);   // FT-817
  CHECK(!slot(14).caps.getProc);  // FT-818
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

TEST(qmx_is_kenwood_cat_over_usb_with_freq_mode_and_meters) {
  const RadioProfile& qmx = slot(21);
  CHECK(qmx.protocol == PROTO_KENWOOD_ASCII);
  CHECK(qmx.model == RadioModel::Qmx);
  CHECK(qmx.link.port == RadioPort::UsbHost);
  CHECK(qmx.caps.getFreq && qmx.caps.setFreq && qmx.caps.getMode && qmx.caps.setMode);
  // SM is in dB, which RadioModel::Qmx decodes.
  CHECK(qmx.caps.getSmeter);
  CHECK_EQ(qmx.commands->smeterGet, "SM;");
  CHECK_EQ(qmx.commands->smeterReplyPrefix, "SM");
  // PC in tenths of a watt and SW in hundredths, which RadioModel::Qmx reads too.
  CHECK(qmx.caps.getPower && qmx.caps.getSwr);
  CHECK_EQ(qmx.commands->powerGet, "PC;");
  CHECK_EQ(qmx.commands->powerReplyPrefix, "PC");
  CHECK_EQ(qmx.commands->swrGet, "SW;");
  CHECK_EQ(qmx.commands->swrReplyPrefix, "SW");
  // It has no NR or NB.
  CHECK(!qmx.caps.getNr && !qmx.caps.getNb);
  CHECK_EQ(qmx.modes->digi, "9");
  CHECK_EQ(qmx.modes->digiAlso, "6");
  // MD8 puts the QMX into SWR tune mode, so no mode may send it.
  const ModeCodes& m = *qmx.modes;
  const char* const codes[] = {m.lsb, m.usb, m.am, m.cw, m.rtty, m.fm, m.cwr, m.rttyR, m.digi, m.pkt, m.wfm, m.digiAlso};
  for (const char* code : codes) CHECK(code[0] != '8');
}

// Not a radio: no cap, so nothing polls its shell; it is reached through nanovna_client.*.
TEST(nanovna_is_a_shell_over_usb_with_no_radio_caps) {
  const RadioProfile& vna = slot(30);
  CHECK_EQ(vna.name, "NanoVNA");
  CHECK(vna.protocol == PROTO_NANOVNA);
  CHECK(vna.vendor == VoiceVendor::NanoVna);
  CHECK(vna.link.port == RadioPort::UsbHost);
  const bool* caps = reinterpret_cast<const bool*>(&vna.caps);
  for (size_t i = 0; i < sizeof(RadioCapabilities) / sizeof(bool); ++i) CHECK(!caps[i]);
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

TEST(model_names_are_set) {
  CHECK_EQ(radioModelName(RadioModel::Generic), "generic");
  CHECK_EQ(radioModelName(RadioModel::Ic7300), "IC-7300");
  CHECK_EQ(radioModelName(RadioModel::Ic705), "IC-705");
  CHECK_EQ(radioModelName(RadioModel::Ft857), "FT-857/897");
  CHECK_EQ(radioModelName(RadioModel::Ftdx10), "FTDX10");
  CHECK_EQ(radioModelName(RadioModel::Qmx), "QMX");
}
