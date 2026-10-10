#pragma once

#include "protocol_ops_yaesu.h"
#include "radio_types.h"

// Radio features the keypad and the console share. Each operation checks the
// profile's capability, talks to the radio, keeps `live` up to date and says
// what happened; the UI that called it prints and speaks the result
// (ui_features.h), so a key and its console command behave the same.

enum class FeatureStatus : uint8_t {
  Ok,
  Unsupported,  // the profile has no such feature
  Timeout,      // the radio did not answer (g_radioReplyTimedOut)
  NoReply,      // a read got no usable answer
  Failed,       // a write was not accepted
};

// Noise reduction. level is 1 or 2 on the TS-480, whose NR has two levels, and
// 0 elsewhere (or when off).
struct NrState {
  bool on = false;               // Antenna: true = rear
  uint8_t level = 0;
};

// On the TS-480 it gives the level too.
FeatureStatus nrQuery(NrState& out);
FeatureStatus nrSet(bool on);
// How many NR levels the radio has beyond on/off: 2 on the TS-480, else 0.
uint8_t nrLevelCount();
// Sets the NR level, 0 being off. Unsupported on a radio without levels and
// for a level above nrLevelCount().
FeatureStatus nrSetLevel(uint8_t level, NrState& out);
// On, off; on the TS-480 off -> 1 -> 2 -> off. Starts from the tracked state
// when it is known, else reads it.
FeatureStatus nrToggle(NrState& out);

// Noise blanker.
FeatureStatus nbQuery(bool& on);
FeatureStatus nbSet(bool on);
FeatureStatus nbToggle(bool& on);

// Notch filter. width is NOTCH_WIDTH_UNKNOWN when off or not known.
struct NotchState {
  bool on = false;
  NotchWidth width = NOTCH_WIDTH_UNKNOWN;
};

FeatureStatus notchQuery(NotchState& out);
FeatureStatus notchSet(bool on);
// Notch on at width (CI-V only).
FeatureStatus notchSetWidth(NotchWidth width);
// On CI-V off -> NAR -> MID -> WIDE -> off; elsewhere on, off.
FeatureStatus notchToggle(NotchState& out);

// Speech processor: FT-857/897 PROC and its menu 74 level, Icom COMP, FTDX10 PR and PL.
// Read only for now.
FeatureStatus procQuery(bool& on);
// 0..100.
FeatureStatus procLevelQuery(uint8_t& level);

// Mic gain, 0..100: Icom, FTDX10, and on the FT-817/818 and FT-857/897 the menu of the current
// mode (SSB, AM, FM, DIG, on the FT-817 also PKT). Unsupported in CW and WFM, and in the
// FT-857/897's PKT. Read only for now.
FeatureStatus micGainQuery(uint8_t& level);

// VOX: Icom, FTDX10 VX and VG, FT-857/897 and its menu 88 gain, FT-817/818 and its menu 51
// gain. Read only for now.
FeatureStatus voxQuery(bool& on);
// 0..100.
FeatureStatus voxGainQuery(uint8_t& level);
// In ms: FT-857/897 menu 87, FT-817/818 menu 50, IC-7300, FTDX10 VD.
FeatureStatus voxDelayQuery(uint16_t& ms);

// FT-857/897 settings read from the radio's EEPROM, since CAT has no command for them. They
// cannot be set. IPO, ATT and NAR are those of the current band. The FT-817/818 has RfPower,
// Menu, Row, Agc, BreakIn, Keyer, IfShift and Antenna.
// Menu and Row are the menu item and soft key row saved when the radio's menu was last exited.
// NrLevel (menu 49), NbLevel (63), LowCut (46, DSP HPF), HighCut (47, DSP LPF) and MicEq (48)
// are FT-857/897 DSP menu settings.
enum class Ft8x7Setting : uint8_t {
  Agc, Ipo, Att, Nar, Dbf, BreakIn, Keyer, RfPower, Menu, Row, IfShift,
  NrLevel, NbLevel, LowCut, HighCut, MicEq, Antenna,
};

struct Ft8x7SettingState {
  Ft8x7Setting setting = Ft8x7Setting::Agc;
  bool on = false;
  YaesuAgc agc = YaesuAgc::Off;  // Agc
  uint16_t wattsTenths = 0;      // RfPower: FT-857/897 for the current band group
  uint8_t number = 0;            // Menu, Row: from 1
  uint16_t value = 0;            // NrLevel 1..16, NbLevel 0..100, LowCut and HighCut in Hz
  YaesuFt857MicEq micEq = YaesuFt857MicEq::Off;  // MicEq
};

// The console command: "AGC?", "IPO?", ...
const char* ft8x7SettingLabel(Ft8x7Setting setting);
// Unsupported on other radios, and for IPO/ATT/NAR where the band has none (or is not known).
FeatureStatus ft8x7SettingQuery(Ft8x7Setting setting, Ft8x7SettingState& out);
