#pragma once

#include "ft8x7_eeprom_map.h"
#include "protocol_ops_yaesu.h"
#include "radio_setting_types.h"
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
  Unexpected,   // the radio holds a value the map does not know; nothing was written
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
// On, off; on the TS-480 off -> 1 -> 2 -> off. Starts from the radio's state, read each
// time, since its own keys may have changed it; so do the toggles below.
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

// The Yaesu ASCII NR and NB levels (caps.getNrLevel, getNbLevel) in the radio's steps: RL0 1..15,
// NL0 0..10. CI-V keeps its own level path in percent (queryNrLevel), the FT-857/897 its menus.
bool yaesuRxLevelRange(RxLevel level, uint16_t& minOut, uint16_t& maxOut);
FeatureStatus yaesuRxLevelQuery(RxLevel level, uint16_t& valueOut);
// Invalid outside the range.
FeatureStatus yaesuRxLevelSet(RxLevel level, uint16_t value, uint16_t& valueOut);
// delta steps from the level read, kept within the range.
FeatureStatus yaesuRxLevelStep(RxLevel level, int delta, uint16_t& valueOut);

// Auto notch (caps.getAutoNotch): Icom 16 41, Yaesu BC0. Not the manual notch above.
FeatureStatus autoNotchQuery(bool& on);
FeatureStatus autoNotchSet(bool on);
// Off -> on -> off, from the state read.
FeatureStatus autoNotchToggle(bool& on);

// AGC (caps.getAgc): Icom 16 12, the ASCII radios' GT (settings.agc), the FT-8x7 from its
// EEPROM.
FeatureStatus agcQuery(AgcMode& out);
// Whether the radio can be set to mode: caps.setAgc over CAT (Icom FAST, MID, SLOW; Yaesu ASCII
// all five; TS-480 OFF, FAST, SLOW; KX2 FAST, SLOW), and experimentally (caps.setRadioMenu) on the
// FT-8x7, which has no MID.
bool agcCanSet(AgcMode mode);
// Sets mode, read back into out. Unsupported where agcCanSet is false; FT-8x7 errors as
// procLevelSet.
FeatureStatus agcSet(AgcMode mode, AgcMode& out);

// The RX attenuator. db is 0 where the radio does not say how many dB it takes off.
struct AttState {
  bool on = false;
  uint8_t db = 0;
};

// The attenuator (caps.getAtt): Icom 11, the ASCII radios' RA (settings.frontEnd), the FT-857/897
// from the current band in its EEPROM, Unsupported where the band has none.
FeatureStatus attQuery(AttState& out);
// The steps it can be set to (caps.setAtt), 0 where it cannot; step n is attStepDb(n) dB (0 where
// not known).
uint8_t attSteps();
uint8_t attStepDb(uint8_t step);
// Sets step (0 off, 1..attSteps()), read back into out. Invalid above attSteps().
FeatureStatus attSet(uint8_t step, AttState& out);
// Off -> step 1 -> ... -> the last step -> off, from the attenuator read.
FeatureStatus attStep(AttState& out);

// The preamp (caps.getPreamp): Icom 16 02, the ASCII radios' PA. Level 0 is off (Yaesu IPO).
// preampLevels() is 1 where it is only on or off, 2 with P.AMP1/AMP1 and P.AMP2/AMP2, 0 where
// there is none.
uint8_t preampLevels();
FeatureStatus preampQuery(uint8_t& levelOut);
// Sets level (0..preampLevels()), read back into levelOut. Unsupported without caps.setPreamp,
// Invalid above preampLevels().
FeatureStatus preampSet(uint8_t level, uint8_t& levelOut);
// Off -> 1 -> ... -> off, from the level read.
FeatureStatus preampStep(uint8_t& levelOut);

// RF power over CAT (caps.getRfPower), in tenths of a watt: Icom 14 0A scaled to the profile's
// rfPowerMaxWatts, the ASCII radios' PC (settings.rfPower). The FT-8x7 keeps its own
// (ft8x7SettingQuery, ft8x7RfPowerSet); here it is Unsupported.
FeatureStatus rfPowerQuery(uint16_t& tenthsOut);
// The watts it can be set to; false where it cannot (caps.setRfPower).
bool rfPowerRange(uint16_t& minOut, uint16_t& maxOut);
// Sets watts, read back into tenthsOut. Invalid outside rfPowerRange.
FeatureStatus rfPowerSet(uint16_t watts, uint16_t& tenthsOut);

// CW break-in (caps.getBreakIn) and the keyer (caps.getKeyer): Icom 16 47 (break-in only), Yaesu
// BI and KR, the FT-8x7 from its EEPROM.
FeatureStatus breakInQuery(BreakInMode& out);
// Whether the radio can be set to mode: Icom OFF, SEMI, FULL and ON (as SEMI); Yaesu OFF and ON;
// on the FT-8x7 OFF and ON, experimentally (caps.setRadioMenu).
bool breakInCanSet(BreakInMode mode);
// Sets mode, read back into out; Unsupported where breakInCanSet is false.
FeatureStatus breakInSet(BreakInMode mode, BreakInMode& out);
// Off -> on (Icom SEMI) -> off, from the break-in read.
FeatureStatus breakInToggle(BreakInMode& out);
FeatureStatus keyerQuery(bool& on);
FeatureStatus keyerSet(bool on, bool& out);
FeatureStatus keyerToggle(bool& out);

// The ALC meter in percent of full scale (caps.getAlc): Icom 15 13, 0..120; Yaesu RM4, 0..255.
// 0 when not transmitting. The FT-8x7 reads its own (ALC?, 0..15).
FeatureStatus alcQuery(uint8_t& percentOut);

// The CW keyer speed (WPM), pitch (Hz) and break-in delay (ms, 0 = full break-in on the TS-480 and
// FT-857/897): caps.getCwSpeed, getCwPitch, getCwDelay. Icom 14 09 (pitch only), the ASCII
// radios' KS, KP and SD, the FT-8x7 from its EEPROM menus.
FeatureStatus cwLevelQuery(CwLevel level, uint16_t& valueOut);
// The values it can be set to; false where it cannot (on the FT-8x7 only experimentally).
bool cwLevelRange(CwLevel level, uint16_t& minOut, uint16_t& maxOut);
// Sets value, to the radio's step nearest it, read back into valueOut. Invalid outside the range.
FeatureStatus cwLevelSet(CwLevel level, uint16_t value, uint16_t& valueOut);
// delta steps from the value read, kept within the range: 1 WPM, 10 Hz (FT-8x7: its menu step),
// one delay step of the radio (at least 100 ms on the FT-891 and TS-480).
FeatureStatus cwLevelStep(CwLevel level, int delta, uint16_t& valueOut);

// Speech processor: FT-857/897 PROC and its menu 74 level, Icom COMP, FTDX10 and FT-891 PR0 and
// PL, TS-480 PR and its PL input level.
// These and the mic gain and VOX reads below are Unsupported where the profile's cap (getProc,
// getMicGain, getVox) is off.
FeatureStatus procQuery(bool& on);
// 0..100.
FeatureStatus procLevelQuery(uint8_t& level);
// Writes: caps.setProc over CAT (IC-7300, IC-705, IC-7760, FTDX10, FT-891, TS-480), and
// experimentally (caps.setRadioMenu) on the FT-857/897, in the EEPROM. Toggles start from the
// radio's state.
// levelOut holds the level set (on the FT-857/897 read back). Invalid above 100; Failed when the
// radio does not take it, and on the FT-857/897 while transmitting; Unexpected when the stored
// byte is out of range (nothing written).
FeatureStatus procSet(bool on);
FeatureStatus procToggle(bool& on);
FeatureStatus procLevelSet(uint8_t level, uint8_t& levelOut);
// The level read, plus delta, kept within 0..100.
FeatureStatus procLevelStep(int delta, uint8_t& levelOut);

// Mic gain, 0..100: IC-7300, IC-705, IC-7760, FTDX10, FTDX101, FT-891, TS-480, and on the
// FT-817/818 and FT-857/897 the menu of the current mode (SSB, AM, FM, DIG, on the FT-817 also
// PKT). Unsupported in CW and WFM, and in the FT-857/897's PKT.
FeatureStatus micGainQuery(uint8_t& level);
// caps.setMicGain over CAT; experimentally (caps.setRadioMenu) on the FT-817/818 and FT-857/897
// the mic gain of the current mode, as micGainQuery picks it, in the EEPROM. Errors as
// procLevelSet.
FeatureStatus micGainSet(uint8_t level, uint8_t& levelOut);
FeatureStatus micGainStep(int delta, uint8_t& levelOut);

// VOX: IC-7300, IC-705, IC-7760, the Yaesu ASCII radios' and the TS-480's VX and VG, FT-857/897
// and its menu 88 gain, FT-817/818 and its menu 51 gain.
FeatureStatus voxQuery(bool& on);
// 0..100; on the TS-480 0..9.
FeatureStatus voxGainQuery(uint8_t& level);
// In ms: FT-857/897 menu 87, FT-817/818 menu 50, IC-7300, IC-705, IC-7760 (civVoxDelayAddr),
// FTDX10, FT-891 and TS-480 VD (settings.voxDelay; not the FTDX101).
FeatureStatus voxDelayQuery(uint16_t& ms);
// Writes: caps.setVox over CAT, and experimentally (caps.setRadioMenu) on the FT-8x7, in the
// EEPROM. Errors as procLevelSet.
FeatureStatus voxSet(bool on);
FeatureStatus voxToggle(bool& on);
// The VOX gains the radio takes: 1..100 on the FT-8x7, 0..9 on the TS-480, else 0..100. False
// where it cannot be set.
bool voxGainRange(uint8_t& minOut, uint8_t& maxOut);
FeatureStatus voxGainSet(uint8_t level, uint8_t& levelOut);
FeatureStatus voxGainStep(int delta, uint8_t& levelOut);
// The VOX delays in ms the radio takes: CI-V 0..2000, FTDX10 and FT-891 30..3000, TS-480 0..3000,
// FT-857/897 100..3000, FT-817/818 100..2500. False where it cannot be set.
bool voxDelayRange(uint16_t& minOut, uint16_t& maxOut);
// Sets the radio's step nearest ms; msOut holds the delay read back. Invalid outside the range.
FeatureStatus voxDelaySet(uint16_t ms, uint16_t& msOut);
// delta of the radio's steps (100 ms; FTDX10 its VD steps, TS-480 150 ms) from the delay read, kept
// within the range.
FeatureStatus voxDelayStep(int delta, uint16_t& msOut);

// FT-857/897 settings read from the radio's EEPROM, since CAT has no command for them. All but
// IPO, ATT, NAR and IfShift can be set, and only experimentally. IPO, ATT and NAR are those of the
// current band, kept in its band block, which the radio saves late. The FT-817/818 has RfPower,
// Menu, Row, Agc, BreakIn, Keyer, IfShift, Antenna and the CW settings below.
// Menu and Row are the menu item and soft key row saved when the radio's menu was last exited.
// NrLevel (menu 49), NbLevel (63), BpfWidth (45, DSP BPF), LowCut (46, DSP HPF), HighCut (47,
// DSP LPF) and MicEq (48) are FT-857/897 DSP menu settings.
// The CW menu settings (Ft8x7CwLevel for the levels' units): CwSpeed, CwPitch, CwDelay,
// CwWeight, Sidetone and KeyReverse on both families; Qsk, CwAutoMode, Paddle (on = MICKEY)
// and CwBfo on the FT-857/897 only. The FT-817/818 addresses are from the KA7OEI map, not yet
// measured.
enum class Ft8x7Setting : uint8_t {
  Agc, Ipo, Att, Nar, Dbf, BreakIn, Keyer, RfPower, Menu, Row, IfShift,
  NrLevel, NbLevel, BpfWidth, LowCut, HighCut, MicEq, Antenna,
  CwSpeed, CwPitch, CwDelay, CwWeight, Sidetone, Qsk, KeyReverse, CwAutoMode, Paddle, CwBfo,
};

struct Ft8x7SettingState {
  Ft8x7Setting setting = Ft8x7Setting::Agc;
  bool on = false;
  YaesuAgc agc = YaesuAgc::Off;  // Agc
  // RfPower: FT-857/897 for the current band group; FT-817/818 the level's watts on an
  // external supply.
  uint16_t wattsTenths = 0;
  bool hasPowerLevel = false;    // RfPower: FT-817/818, which has powerLevel
  Ft817PowerLevel powerLevel = Ft817PowerLevel::High;
  uint8_t number = 0;            // Menu, Row: from 1
  // NrLevel 1..16, NbLevel 0..100, BpfWidth, LowCut and HighCut in Hz; the CW levels in the
  // units of Ft8x7CwLevel (CwDelay 0 = FULL, CwWeight in tenths).
  uint16_t value = 0;
  YaesuFt857MicEq micEq = YaesuFt857MicEq::Off;  // MicEq
  Ft857CwBfo cwBfo = Ft857CwBfo::Auto;           // CwBfo
};

// The console command: "AGC?", "IPO?", ...
const char* ft8x7SettingLabel(Ft8x7Setting setting);
// Unsupported on other radios, and for IPO/ATT/NAR where the band has none (or is not known).
// Unexpected when a CW setting's stored value does not fit the map.
FeatureStatus ft8x7SettingQuery(Ft8x7Setting setting, Ft8x7SettingState& out);
// Experimental (caps.setRadioMenu): sets Menu or Row, counted from 1, in the EEPROM; out holds
// the value read back. Invalid outside 1..the last item or row; Failed while transmitting or
// when the read-back differs; Unexpected when the stored bytes are not as expected (nothing
// written).
FeatureStatus ft8x7MenuRowSet(Ft8x7Setting setting, uint8_t number, Ft8x7SettingState& out);
// The values ft8x7LevelSet takes: NrLevel 1..16, NbLevel 0..100, BpfWidth 60..240 Hz, LowCut
// 100..1000 Hz, HighCut 1000..6000 Hz, the CW levels as ft8x7CwLevelRange gives them for the
// radio. False for other settings, and for a CW level the radio does not keep.
bool ft8x7LevelRange(Ft8x7Setting setting, uint16_t& minOut, uint16_t& maxOut);
// Experimental (caps.setNrLevel, caps.setNbLevel, and caps.setRadioMenu for the width and the
// cuts): sets the FT-857/897 NrLevel (menu 49), NbLevel (menu 63), BpfWidth (menu 45, DSP BPF
// 60, 120 or 240 Hz), LowCut (menu 46, DSP HPF) or HighCut (menu 47, DSP LPF) in the EEPROM,
// to the radio's step nearest value; out.value holds the value read back. Invalid outside
// ft8x7LevelRange; Failed while transmitting or when the read-back differs; Unexpected when the
// stored byte is out of range (nothing written).
// Also experimental (caps.setRadioMenu): the CW levels, on the FT-817/818 too, CwDelay 0 for
// FULL where the radio has it.
FeatureStatus ft8x7LevelSet(Ft8x7Setting setting, uint16_t value, Ft8x7SettingState& out);
// The same, delta of the radio's steps from the value read (NR and NB: 1 each, BpfWidth one of
// its three widths, LowCut 60 Hz, HighCut about 160 Hz, the CW levels one menu step), kept
// within the range.
FeatureStatus ft8x7LevelStep(Ft8x7Setting setting, int delta, Ft8x7SettingState& out);
// Experimental (caps.setRadioMenu): Dbf (FT-857/897, both its bits as the radio's own key
// does), BreakIn, Keyer, KeyReverse, CwAutoMode or Paddle on or off in the EEPROM. Unsupported
// for other settings. Failed while transmitting.
FeatureStatus ft8x7FlagSettingSet(Ft8x7Setting setting, bool on, Ft8x7SettingState& out);
FeatureStatus ft8x7FlagSettingToggle(Ft8x7Setting setting, Ft8x7SettingState& out);
// Experimental (caps.setRadioMenu): the FT-817/818 antenna jack (menu 07) of the band group of
// the current frequency, out.on = rear as read back. Unsupported on the FT-857/897. Failed while
// transmitting.
FeatureStatus ft817AntennaSet(bool rear, Ft8x7SettingState& out);
FeatureStatus ft817AntennaToggle(Ft8x7SettingState& out);
// Experimental (caps.setRfPower): the FT-857/897 TX power in the EEPROM, in watts; out holds
// the value read back. Menu 75 of the band group of the current frequency, HF and 6 m 5..100,
// VHF 5..50, UHF 2..20; Invalid for any other value. Failed while transmitting or when the
// read-back differs; Unexpected when the stored byte is out of range (nothing written).
// Unsupported on the FT-817/818.
FeatureStatus ft8x7RfPowerSet(uint16_t watts, Ft8x7SettingState& out);
// The same for the FT-817/818 power level; Unsupported on the FT-857/897.
FeatureStatus ft817PowerLevelSet(Ft817PowerLevel level, Ft8x7SettingState& out);
// Experimental (caps.setRadioMenu): AGC in the EEPROM, on the FT-817/818 and FT-857/897; out
// holds the AGC read back. Failed while transmitting or when the read-back differs.
FeatureStatus ft8x7AgcSet(YaesuAgc agc, Ft8x7SettingState& out);
// Experimental (caps.setRadioMenu): the FT-857/897 TX equalizer (menu 48) in the EEPROM; out
// holds the value read back. Failed while transmitting or when the read-back differs.
FeatureStatus ft8x7MicEqSet(YaesuFt857MicEq eq, Ft8x7SettingState& out);
// Experimental (caps.setRadioMenu): the FT-857/897 CW BFO side (menu 23) in the EEPROM; out
// holds the value read back. Failed while transmitting or when the read-back differs.
FeatureStatus ft8x7CwBfoSet(Ft857CwBfo bfo, Ft8x7SettingState& out);

// Tone squelch. Over CAT (caps.getTone): Icom 16 42 and 16 43, Yaesu CT0, CTCSS only, both
// (encode and decode) or TX (encode only); the FT-891 also reads DCS. On the FT-8x7:
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
// The CTCSS tone over CAT (caps.getCtcss), in tenths of a Hz: Icom 1B 00 (a set writes the tone
// squelch tone 1B 01 too), Yaesu CN00. Invalid for a tone outside the 50 standard ones.
FeatureStatus ctcssToneQuery(uint16_t& tenthsOut);
FeatureStatus ctcssToneSet(uint16_t tenths, uint16_t& tenthsOut);
// The repeater shift over CAT (caps.getRptShift): the IC-705 0F, Yaesu OS0 (FM only).
FeatureStatus rptShiftQuery(RptShift& out);
FeatureStatus rptShiftSet(RptShift shift);
