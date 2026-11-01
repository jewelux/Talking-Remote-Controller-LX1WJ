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
    // One clip name: lowercase letters, digits and '_' (check_voice_tokens.py checks the clip).
    for (const char* c = p.voiceSuffix; *c; ++c) {
      CHECK((*c >= '0' && *c <= '9') || (*c >= 'a' && *c <= 'z') || *c == '_');
    }
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
    if (!isAsciiProtocol(p.protocol)) continue;
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
  CHECK_EQ(slot(22).name, "Xiegu X6100");
  CHECK_EQ(slot(23).name, "Icom IC-705 USB");
  CHECK(profileForSlot(24) == nullptr);
  CHECK(profileForSlot(255) == nullptr);
}

TEST(default_slot_is_used) {
  CHECK(profileForSlot(kDefaultProfileSlot) != nullptr);
}

// Slots 1-18 are all in use since the FT-847 took 16; the QMX has 21, the X6100 22, the IC-705 on
// USB 23 and the NanoVNA 70; 19-20 and 24-69 are free.
TEST(adjacent_slot_skips_free_slots) {
  CHECK_EQ(adjacentProfileSlot(15, 1), 16);
  CHECK_EQ(adjacentProfileSlot(16, 1), 17);
  CHECK_EQ(adjacentProfileSlot(17, -1), 16);
  CHECK_EQ(adjacentProfileSlot(18, 1), 21);
  CHECK_EQ(adjacentProfileSlot(21, -1), 18);
  CHECK_EQ(adjacentProfileSlot(19, -1), 18);
  CHECK_EQ(adjacentProfileSlot(19, 1), 21);
  CHECK_EQ(adjacentProfileSlot(21, 1), 22);
  CHECK_EQ(adjacentProfileSlot(22, 1), 23);
  CHECK_EQ(adjacentProfileSlot(23, 1), 70);
  CHECK_EQ(adjacentProfileSlot(70, -1), 23);
  CHECK_EQ(adjacentProfileSlot(23, -1), 22);
  CHECK_EQ(adjacentProfileSlot(30, -1), 23);
  CHECK_EQ(adjacentProfileSlot(30, 1), 70);
}

TEST(adjacent_slot_wraps_around) {
  CHECK_EQ(adjacentProfileSlot(70, 1), 1);
  CHECK_EQ(adjacentProfileSlot(1, -1), 70);
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
  CHECK(slot(22).model == RadioModel::Generic);  // Xiegu X6100
  CHECK(slot(23).model == RadioModel::Ic705);    // IC-705 on USB: the IC-705's menu addresses
}

// ---- The radios on the USB host ----

// The X6100's CAT is its CH342's port B (the second CDC port), a UART at 19200; DTR and RTS stay
// off because RTS keys CW. Address A4 as in Hamlib.
TEST(x6100_usb_link) {
  const RadioProfile& x = slot(22);
  CHECK(x.vendor == VoiceVendor::Xiegu);
  CHECK(x.protocol == PROTO_CIV);
  CHECK(x.link.port == RadioPort::UsbHost);
  CHECK_EQ(x.link.usbSerialIndex, 1);
  CHECK(!x.link.usbDtr && !x.link.usbRts);
  CHECK_EQ(x.link.baud, 19200u);
  CHECK_EQ(x.link.bauds.count, 1);
  CHECK_EQ(x.link.civAddr, 0xA4);
  CHECK_EQ(x.rfPowerMaxWatts, 10);
  CHECK_EQ(x.settings.preampLevels, 1);
}

// Only what the X6100's CI-V table, Hamlib and HamTRC's encoding agree on.
TEST(x6100_caps) {
  const RadioCapabilities& c = slot(22).caps;
  CHECK(c.getFreq && c.setFreq && c.getMode && c.setMode && c.getSmeter && c.getSwr);
  CHECK(c.getRfPower && c.setRfPower && c.getRxTx);
  CHECK(c.getNr && c.setNr && c.getNrLevel && c.setNrLevel && c.getNb && c.setNb && c.getNbLevel);
  CHECK(c.getDialLock && c.getTuner && c.startTune && c.getVfo && c.getVfoMode && c.getRit);
  CHECK(c.getAgc && c.getPreamp && c.getProc && c.getMicGain && c.getVox);
  // 11 toggles, 0F 01 refused, no 1C 03, 16 41 is a manual notch, unknown 14s answer 0000.
  CHECK(!c.getAtt && !c.setAtt && !c.getSplit && !c.setSplit && !c.getTxFreq);
  CHECK(!c.getNotch && !c.getAutoNotch && !c.getNotchWidth);
  CHECK(!c.getPbtInner && !c.getPbtOuter && !c.getFilterWidth && !c.getFilterShape);
  CHECK(!c.getMonitor && !c.getMonitorLevel && !c.getBreakIn && !c.getCwPitch && !c.getCwSpeed);
  CHECK(!c.getPower && !c.getAlc && !c.getTransceive && !c.getBandStack);
  CHECK(!c.getTone && !c.getCtcss && !c.getRptShift);
}

// The X6100 has LSB, USB, AM, CW, FM and CWR; every other CI-V radio takes every Icom mode.
TEST(civ_mode_lists) {
  const uint16_t x6100 = slot(22).civModes;
  const uint8_t offered[] = {0x00, 0x01, 0x02, 0x03, 0x05, 0x07};
  for (uint8_t m : offered) CHECK(civModeOffered(x6100, m));
  const uint8_t missing[] = {0x04, 0x06, 0x08, 0x11, 0x12};
  for (uint8_t m : missing) CHECK(!civModeOffered(x6100, m));
  const uint8_t icom[] = {1, 2, 3, 4, 5, 17, 18, 23};
  for (uint8_t s : icom) {
    CHECK_EQ(slot(s).civModes, 0);
    CHECK(civModeOffered(slot(s).civModes, 0x04));
  }
}

// The IC-705's own USB port: everything slot 17 has, CI-V on its first CDC port, DTR and RTS off
// (its USB SEND and USB Keying menus can key on either).
TEST(ic705_usb_link) {
  const RadioProfile& usb = slot(23);
  const RadioProfile& jack = slot(17);
  CHECK(usb.protocol == PROTO_CIV);
  CHECK(usb.link.port == RadioPort::UsbHost);
  CHECK_EQ(usb.link.usbSerialIndex, 0);
  CHECK(!usb.link.usbDtr && !usb.link.usbRts);
  CHECK_EQ(usb.link.civAddr, jack.link.civAddr);
  CHECK(usb.link.bauds.contains(usb.link.baud) && usb.link.bauds.contains(19200));
  CHECK_EQ(usb.rfPowerMaxWatts, jack.rfPowerMaxWatts);
  CHECK(usb.caps.getRptShift && usb.caps.getTransceive && usb.caps.getAtt && usb.caps.getSplit);
  CHECK_EQ(usb.settings.preampLevels, jack.settings.preampLevels);
  CHECK_EQ(usb.settings.attDb[0], jack.settings.attDb[0]);
}

// The QMX and the NanoVNA keep DTR on, as tested; no serial radio has a USB port index.
TEST(usb_link_defaults) {
  CHECK(slot(21).link.usbDtr && !slot(21).link.usbRts && slot(21).link.usbSerialIndex == 0);
  CHECK(slot(70).link.usbDtr && !slot(70).link.usbRts && slot(70).link.usbSerialIndex == 0);
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (p.link.port != RadioPort::UsbHost) CHECK_EQ(p.link.usbSerialIndex, 0);
  }
}

// PROC?, MICGAIN? and VOX? say "not available" where the cap is off, instead of waiting for a
// radio that has no such command.
TEST(transmit_setting_caps) {
  const uint8_t on[] = {1, 3, 17, 18, 22, 23, 8, 9, 10, 14, 11, 12, 13, 15, 7};
  for (uint8_t s : on) {
    CHECK(slot(s).caps.getMicGain);
    CHECK(slot(s).caps.getVox);
  }
  const uint8_t proc[] = {1, 3, 17, 18, 22, 23, 9, 10, 11, 15, 7};
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

// VD per radio: the step table on the FTDX10 and FTDX101, ms on the FT-891 and TS-480.
TEST(ascii_vox_delay_and_gain_encodings) {
  CHECK(slot(11).settings.voxDelay == VoxDelayEncoding::YaesuSteps);
  CHECK(slot(12).settings.voxDelay == VoxDelayEncoding::YaesuSteps);
  CHECK(slot(13).settings.voxDelay == VoxDelayEncoding::YaesuSteps);
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
    CHECK(isAsciiProtocol(profileAt(i).protocol));
    CHECK(profileAt(i).caps.getVox);
    CHECK(s.voxDelayMaxMs > s.voxDelayMinMs);
    if (s.voxDelay == VoxDelayEncoding::Ms4) CHECK(s.voxDelayStepMs > 0);
  }
}

// AGC: Icom 16 12, the ASCII radios by their GT encoding, the FT-8x7 from its EEPROM (set only
// with EXPERIMENTAL on).
TEST(agc_caps_and_encodings) {
  const uint8_t settable[] = {1, 3, 17, 18, 22, 23, 6, 7, 11, 12, 13, 15};
  for (uint8_t s : settable) CHECK(slot(s).caps.getAgc && slot(s).caps.setAgc);
  const uint8_t icom[] = {1, 3, 17, 18, 23};
  for (uint8_t s : icom) CHECK(slot(s).settings.agc == AgcEncoding::None);
  CHECK(slot(22).caps.getAgc && slot(22).caps.setAgc);
  CHECK(slot(22).settings.agc == AgcEncoding::CivWithOff);
  const uint8_t yaesu[] = {11, 12, 13, 15};
  for (uint8_t s : yaesu) CHECK(slot(s).settings.agc == AgcEncoding::YaesuGt0);
  CHECK(slot(7).settings.agc == AgcEncoding::KenwoodGt);
  CHECK(slot(6).settings.agc == AgcEncoding::ElecraftGt);
  const uint8_t ft8x7[] = {8, 9, 10, 14};
  for (uint8_t s : ft8x7) CHECK(slot(s).caps.getAgc && !slot(s).caps.setAgc);
  const uint8_t none[] = {2, 4, 5, 16};
  for (uint8_t s : none) CHECK(!slot(s).caps.getAgc && !slot(s).caps.setAgc);
}

// An ASCII radio reads AGC only through its GT encoding, and only ASCII radios have one; a CI-V
// radio has None (Icom) or CivWithOff (X6100).
TEST(every_ascii_agc_cap_has_an_encoding) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (isAsciiProtocol(p.protocol) && p.caps.getAgc) {
      CHECK(p.settings.agc != AgcEncoding::None && p.settings.agc != AgcEncoding::CivWithOff);
    }
    if (p.protocol == PROTO_CIV) {
      CHECK(p.settings.agc == AgcEncoding::None || p.settings.agc == AgcEncoding::CivWithOff);
    } else if (p.settings.agc != AgcEncoding::None) {
      CHECK(isAsciiProtocol(p.protocol));
    }
  }
}

// Preamp and attenuator: the Icoms by 16 02 and 11, the ASCII radios by PA and RA, the FT-857/897
// ATT from its EEPROM, read only.
TEST(preamp_and_att_caps_and_encodings) {
  const uint8_t settable[] = {1, 3, 17, 18, 23, 6, 7, 11, 12, 13, 15};
  for (uint8_t s : settable) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getPreamp && c.setPreamp && c.getAtt && c.setAtt);
    CHECK(slot(s).settings.preampLevels >= 1 && slot(s).settings.preampLevels <= 2);
    CHECK(slot(s).settings.attLevels >= 1 && slot(s).settings.attLevels <= 3);
  }
  const uint8_t icom7300[] = {1, 3, 17, 23};
  for (uint8_t s : icom7300) {
    CHECK_EQ(slot(s).settings.preampLevels, 2);
    CHECK_EQ(slot(s).settings.attLevels, 1);
    CHECK_EQ(slot(s).settings.attDb[0], 20);
  }
  CHECK_EQ(slot(18).settings.attLevels, 3);
  CHECK_EQ(slot(18).settings.attDb[2], 18);
  // The X6100: one preamp, and no attenuator (its 11 toggles).
  CHECK(slot(22).caps.getPreamp && slot(22).caps.setPreamp && !slot(22).caps.getAtt);
  CHECK_EQ(slot(22).settings.attLevels, 0);
  const uint8_t ftdx[] = {11, 12, 13};
  for (uint8_t s : ftdx) {
    CHECK(slot(s).settings.frontEnd == FrontEndEncoding::Yaesu);
    CHECK_EQ(slot(s).settings.preampLevels, 2);
    CHECK_EQ(slot(s).settings.attDb[0], 6);
  }
  CHECK(slot(15).settings.frontEnd == FrontEndEncoding::Yaesu);
  CHECK_EQ(slot(15).settings.attDb[0], 0);
  CHECK(slot(7).settings.frontEnd == FrontEndEncoding::Kenwood);
  CHECK(slot(6).settings.frontEnd == FrontEndEncoding::Kenwood);
  CHECK_EQ(slot(6).settings.attDb[0], 10);
  const uint8_t ft857[] = {9, 10};
  for (uint8_t s : ft857) CHECK(slot(s).caps.getAtt && !slot(s).caps.setAtt && !slot(s).caps.getPreamp);
  const uint8_t none[] = {2, 4, 5, 8, 14, 16};
  for (uint8_t s : none) CHECK(!slot(s).caps.getPreamp && !slot(s).caps.getAtt);
}

// The steps a profile gives are ascending, and an ASCII radio's caps come with its PA/RA encoding.
TEST(every_preamp_and_att_cap_has_an_encoding) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    const SettingEncodings& s = p.settings;
    CHECK(s.attLevels <= 3);
    for (uint8_t k = 1; k < s.attLevels; ++k) CHECK(s.attDb[k] > s.attDb[k - 1]);
    if (p.caps.getPreamp) CHECK(s.preampLevels > 0);
    if (p.caps.getAtt && p.protocol != PROTO_YAESU_FT8X7) CHECK(s.attLevels > 0);
    if (isAsciiProtocol(p.protocol) && (p.caps.getPreamp || p.caps.getAtt)) CHECK(s.frontEnd != FrontEndEncoding::None);
    if (s.frontEnd != FrontEndEncoding::None) CHECK(isAsciiProtocol(p.protocol));
  }
}

// RF power: Icom 14 0A, the ASCII radios' PC in watts; not on the FTDX101MP, whose manual and
// Hamlib disagree on the scale.
TEST(rf_power_caps_and_ranges) {
  const uint8_t settable[] = {1, 3, 17, 18, 22, 23, 6, 7, 11, 12, 15};
  for (uint8_t s : settable) CHECK(slot(s).caps.getRfPower && slot(s).caps.setRfPower);
  const uint8_t pc[] = {6, 7, 11, 12, 15};
  for (uint8_t s : pc) CHECK(slot(s).settings.rfPower == RfPowerEncoding::PcWatts);
  CHECK(!slot(13).caps.getRfPower && !slot(13).caps.setRfPower);
  CHECK_EQ(slot(6).rfPowerMaxWatts, 15);
  CHECK_EQ(slot(6).settings.rfPowerMinWatts, 0);
  const uint8_t five[] = {7, 11, 12, 15};
  for (uint8_t s : five) {
    CHECK_EQ(slot(s).settings.rfPowerMinWatts, 5);
    CHECK_EQ(slot(s).rfPowerMaxWatts, 100);
  }
  CHECK_EQ(slot(17).rfPowerMaxWatts, 10);
  CHECK_EQ(slot(22).rfPowerMaxWatts, 10);
  CHECK_EQ(slot(23).rfPowerMaxWatts, 10);
}

// An ASCII radio reads the RF power only through its PC encoding, and only ASCII radios have one.
TEST(every_ascii_rf_power_cap_has_an_encoding) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    if (isAsciiProtocol(p.protocol) && p.caps.getRfPower) CHECK(p.settings.rfPower != RfPowerEncoding::None);
    if (p.settings.rfPower != RfPowerEncoding::None) CHECK(isAsciiProtocol(p.protocol));
    CHECK(p.settings.rfPowerMinWatts < p.rfPowerMaxWatts);
  }
}

// Break-in: Icom 16 47, Yaesu BI; keyer: Yaesu KR; the FT-8x7 from its EEPROM (set only with
// EXPERIMENTAL on).
TEST(break_in_and_keyer_caps) {
  const uint8_t icom[] = {1, 3, 17, 18, 23};
  for (uint8_t s : icom) {
    CHECK(slot(s).caps.getBreakIn && slot(s).caps.setBreakIn);
    CHECK(!slot(s).caps.getKeyer);
  }
  const uint8_t yaesu[] = {11, 12, 13, 15};
  for (uint8_t s : yaesu) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getBreakIn && c.setBreakIn && c.getKeyer && c.setKeyer);
  }
  const uint8_t ft8x7[] = {8, 9, 10, 14};
  for (uint8_t s : ft8x7) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getBreakIn && !c.setBreakIn && c.getKeyer && !c.setKeyer);
  }
  const uint8_t none[] = {2, 4, 5, 6, 7, 16, 22};
  for (uint8_t s : none) CHECK(!slot(s).caps.getBreakIn && !slot(s).caps.getKeyer);
}

// CW speed, pitch and delay where the sources agree.
TEST(cw_level_caps_and_ranges) {
  const uint8_t icom[] = {1, 3, 17, 18, 23};
  for (uint8_t s : icom) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getCwPitch && c.setCwPitch && !c.getCwSpeed && !c.getCwDelay);
  }
  const uint8_t all[] = {11, 15};
  for (uint8_t s : all) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getCwSpeed && c.setCwSpeed && c.getCwPitch && c.setCwPitch && c.getCwDelay && c.setCwDelay);
    CHECK(slot(s).settings.cwPitchKp);
    CHECK_EQ(slot(s).settings.cwSpeedMinWpm, 4);
    CHECK_EQ(slot(s).settings.cwSpeedMaxWpm, 60);
  }
  CHECK(slot(11).settings.cwDelay == VoxDelayEncoding::YaesuSteps);
  CHECK(slot(15).settings.cwDelay == VoxDelayEncoding::Ms4);
  const uint8_t ftdx101[] = {12, 13};
  for (uint8_t s : ftdx101) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getCwSpeed && c.getCwPitch && !c.getCwDelay && !c.setCwDelay);
  }
  CHECK(slot(7).caps.getCwSpeed && slot(7).caps.getCwDelay && !slot(7).caps.getCwPitch);
  CHECK_EQ(slot(7).settings.cwDelayMaxMs, 1000);
  CHECK_EQ(slot(7).settings.cwDelayStepMs, 50);
  CHECK(slot(6).caps.getCwSpeed && !slot(6).caps.getCwPitch && !slot(6).caps.getCwDelay);
  CHECK_EQ(slot(6).settings.cwSpeedMaxWpm, 50);
  const uint8_t ft8x7[] = {8, 9, 10, 14};
  for (uint8_t s : ft8x7) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getCwSpeed && c.getCwPitch && c.getCwDelay && !c.setCwSpeed && !c.setCwPitch && !c.setCwDelay);
  }
}

// An ASCII radio's CW caps come with their encodings, and CI-V has the pitch only.
TEST(every_cw_level_cap_has_an_encoding) {
  for (size_t i = 0; i < profileCount(); ++i) {
    const RadioProfile& p = profileAt(i);
    const SettingEncodings& s = p.settings;
    if (p.protocol == PROTO_CIV) CHECK(!p.caps.getCwSpeed && !p.caps.getCwDelay);
    if (!isAsciiProtocol(p.protocol)) continue;
    if (p.caps.getCwSpeed) CHECK(s.cwSpeedMaxWpm > s.cwSpeedMinWpm);
    if (p.caps.getCwPitch) CHECK(s.cwPitchKp && p.protocol == PROTO_YAESU_FTDX_ASCII);
    if (p.caps.getCwDelay) {
      CHECK(s.cwDelay != VoxDelayEncoding::None);
      CHECK(s.cwDelayMaxMs > s.cwDelayMinMs);
      if (s.cwDelay == VoxDelayEncoding::Ms4) CHECK(s.cwDelayStepMs > 0);
    }
  }
}

// ALC: Icom 15 13 and Yaesu RM4; the FT-8x7 reads its own TX meter, not through this cap.
TEST(alc_caps) {
  const uint8_t alc[] = {1, 3, 17, 18, 23, 11, 12, 13, 15};
  for (uint8_t s : alc) CHECK(slot(s).caps.getAlc);
  const uint8_t none[] = {2, 4, 5, 6, 7, 8, 9, 10, 14, 16, 22};
  for (uint8_t s : none) CHECK(!slot(s).caps.getAlc);
}

// Auto notch: Icom 16 41, Yaesu BC0.
TEST(auto_notch_caps) {
  const uint8_t yes[] = {1, 3, 17, 18, 23, 11, 12, 13, 15};
  for (uint8_t s : yes) CHECK(slot(s).caps.getAutoNotch && slot(s).caps.setAutoNotch);
  const uint8_t no[] = {2, 4, 5, 6, 7, 8, 9, 10, 14, 16, 22};
  for (uint8_t s : no) CHECK(!slot(s).caps.getAutoNotch);
}

// NR and NB levels: CI-V 14 06 and 14 12 (IC-7300 family), Yaesu RL0 and NL0.
TEST(nr_nb_level_caps) {
  const uint8_t yes[] = {1, 3, 17, 18, 22, 23, 11, 12, 13, 15};
  for (uint8_t s : yes) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getNrLevel && c.setNrLevel && c.getNbLevel && c.setNbLevel);
  }
  const uint8_t no[] = {2, 4, 5, 6, 7, 8, 14, 16};
  for (uint8_t s : no) CHECK(!slot(s).caps.getNrLevel && !slot(s).caps.getNbLevel);
}

// Tone squelch and its CTCSS tone: the Icoms (16 42/43, 1B 00) and the Yaesu ASCII radios (CT0,
// CN00); the repeater shift on the IC-705 (0F) and these Yaesus (OS0). The FT-8x7 has its own.
TEST(tone_and_repeater_caps) {
  const uint8_t tone[] = {1, 3, 17, 18, 23, 11, 12, 13, 15};
  for (uint8_t s : tone) {
    const RadioCapabilities& c = slot(s).caps;
    CHECK(c.getTone && c.setTone && c.getCtcss && c.setCtcss);
  }
  const uint8_t rpt[] = {17, 23, 11, 12, 13, 15};
  for (uint8_t s : rpt) CHECK(slot(s).caps.getRptShift && slot(s).caps.setRptShift);
  const uint8_t noRpt[] = {1, 3, 18};
  for (uint8_t s : noRpt) CHECK(!slot(s).caps.getRptShift);
  const uint8_t none[] = {2, 4, 5, 6, 7, 8, 9, 10, 14, 16, 22};
  for (uint8_t s : none) CHECK(!slot(s).caps.getTone && !slot(s).caps.getCtcss && !slot(s).caps.getRptShift);
}

// Tone decode only: none yet. The FT-897 takes it over CAT but does not squelch on it.
TEST(no_profile_has_tone_decode_only) {
  for (uint8_t s = 1; s <= 18; ++s) CHECK(!slot(s).caps.setToneDecodeOnly);
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
  const RadioProfile& vna = slot(70);
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
  CHECK_EQ(radioModelName(RadioModel::Qmx), "QMX");
}
