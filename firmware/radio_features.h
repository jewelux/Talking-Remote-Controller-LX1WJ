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
  Invalid,      // the value is out of range; nothing was sent
};

// On the FT-8x7, NR, NB and notch are bits in the EEPROM (FT-857/897 DNR, NB, DNF; FT-817/818
// NB only). No FT-8x7 profile has caps.setNr, setNb or setNotch, so they are set only with
// EXPERIMENTAL ON, and not while transmitting.

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
// Read only for now. These and the mic gain and VOX reads below are Unsupported where the
// profile's cap (getProc, getMicGain, getVox) is off.
FeatureStatus procQuery(bool& on);
// 0..100.
FeatureStatus procLevelQuery(uint8_t& level);

// Mic gain, 0..100: IC-7300, IC-705, IC-7760, FTDX10, and on the FT-817/818 and FT-857/897 the menu of the current
// mode (SSB, AM, FM, DIG, on the FT-817 also PKT). Unsupported in CW and WFM, and in the
// FT-857/897's PKT. Read only for now.
FeatureStatus micGainQuery(uint8_t& level);

// VOX: IC-7300, IC-705, IC-7760, FTDX10 VX and VG, FT-857/897 and its menu 88 gain, FT-817/818 and its menu 51
// gain. Read only for now.
FeatureStatus voxQuery(bool& on);
// 0..100.
FeatureStatus voxGainQuery(uint8_t& level);
// In ms: FT-857/897 menu 87, FT-817/818 menu 50, IC-7300, IC-705, IC-7760 (civVoxDelayAddr),
// FTDX10 VD.
FeatureStatus voxDelayQuery(uint16_t& ms);

// FT-857/897 settings read from the radio's EEPROM, since CAT has no command for them. Only
// Menu, Row, Dbf, NrLevel, NbLevel, LowCut and HighCut can be set, and only experimentally. IPO, ATT and NAR are those of the current band. The FT-817/818 has RfPower,
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
// Experimental (caps.setRadioMenu): sets Menu or Row, counted from 1, in the EEPROM; out holds
// the value read back. Invalid outside 1..the last item or row; Failed while transmitting, when
// the stored bytes are not as expected (nothing written) or when the read-back differs.
FeatureStatus ft8x7MenuRowSet(Ft8x7Setting setting, uint8_t number, Ft8x7SettingState& out);
// The values ft8x7LevelSet takes: NrLevel 1..16, NbLevel 0..100, LowCut 100..1000 Hz, HighCut
// 1000..6000 Hz. False for other settings.
bool ft8x7LevelRange(Ft8x7Setting setting, uint16_t& minOut, uint16_t& maxOut);
// Experimental (caps.setNrLevel, caps.setNbLevel, and caps.setRadioMenu for the cuts): sets the
// FT-857/897 NrLevel (menu 49), NbLevel (menu 63), LowCut (menu 46, DSP HPF) or HighCut (menu
// 47, DSP LPF) in the EEPROM, to the radio's step nearest value; out.value holds the value read
// back. Invalid outside ft8x7LevelRange; Failed while transmitting, when the stored byte is out
// of range (nothing written) or when the read-back differs.
FeatureStatus ft8x7LevelSet(Ft8x7Setting setting, uint16_t value, Ft8x7SettingState& out);
// The same, delta of the radio's steps from the value read (NR and NB: 1 each, LowCut 60 Hz,
// HighCut about 160 Hz), kept within the range.
FeatureStatus ft8x7LevelStep(Ft8x7Setting setting, int delta, Ft8x7SettingState& out);
// Experimental (caps.setRadioMenu): the FT-857/897 DSP bandpass filter (DBF) on or off in the
// EEPROM, both its bits as the radio's own key does. Failed while transmitting.
FeatureStatus ft8x7DbfSet(bool on, Ft8x7SettingState& out);
FeatureStatus ft8x7DbfToggle(Ft8x7SettingState& out);

// FT-8x7 tone squelch. The FT-857/897 reads it from the current band in the EEPROM, live;
// the FT-817/818 cannot, so there it is the mode HamTRC last set. Unsupported on other radios,
// outside the amateur bands, on the FT-817/818 before HamTRC has set a mode, and for a mode the
// radio has no CAT command for (on the FT-817/818 DCS encode only).
FeatureStatus toneModeQuery(YaesuToneMode& out);
FeatureStatus toneOff();
// CTCSS or DCS, keeping encode only (tx only) and decode only (rx only); from off or a mixed
// mode, both. Where the radio cannot keep it (FT-817/818 DCS), both.
FeatureStatus toneTypeSet(YaesuToneType type, YaesuToneMode& out);
// Both -> TX only (encode only) -> RX only (decode only) -> both, keeping the type; a mixed mode
// counts as both. The FT-817/818 has no RX only, so there TX only goes back to both. With the
// tone off nothing is sent and out is off.
FeatureStatus toneDirStep(YaesuToneMode& out);
// FT-857/897 split tone (menu 79) of the current band, which lets the radio's own keys send one
// tone type and wait for the other. CAT cannot switch it. Unsupported on other radios (the
// FT-817/818 has none) and outside the amateur bands.
FeatureStatus splitToneQuery(bool& on);
