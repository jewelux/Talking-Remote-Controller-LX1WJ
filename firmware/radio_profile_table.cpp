// The built-in radio profiles. Each family first defines what its radios share
// (baud rates, capabilities, commands, mode codes), then kProfiles at the end
// lists every radio in slot order.
//
// To add a radio: copy the entry of the closest radio in kProfiles, give it a
// free slot, and change what differs. The checks at the end of this file stop
// the build on a duplicate slot or a default baud the radio does not offer.

#include "radio_profile_table.h"

namespace {

// --- Icom CI-V (and the Xiegu G106, which speaks CI-V) ----------------------

// The rates the CI-V setup offers. The Icom CI-V jack itself goes up to 19200.
constexpr uint32_t kCivBauds[] = {4800, 9600, 19200, 38400, 57600, 115200};

constexpr RadioCapabilities kCivFreqModeCaps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
};

// The IC-706: frequency, mode and the S-meter. Hamlib gives no IC-706 variant the PO or SWR
// meter (15 11, 15 12), and the original and MKII have no levels at all.
constexpr RadioCapabilities kIc706Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true,
};

// The IC-7300: everything HamTRC can do over CI-V, checked against its CI-V command table.
constexpr RadioCapabilities kIc7300Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getPower = true, .getRfPower = true, .setRfPower = true,
    .getSwr = true, .getAlc = true, .getRxTx = true, .getTxFreq = true,
    .getNr = true, .setNr = true, .getNrLevel = true, .setNrLevel = true,
    .getNb = true, .setNb = true, .getNbLevel = true, .setNbLevel = true,
    .getNotch = true, .setNotch = true, .getNotchWidth = true, .setNotchWidth = true,
    .getAutoNotch = true, .setAutoNotch = true, .getTone = true, .setTone = true, .getCtcss = true, .setCtcss = true,
    .getPbtInner = true, .setPbtInner = true, .getPbtOuter = true, .setPbtOuter = true,
    .getFilterShape = true, .setFilterShape = true, .getFilterWidth = true, .setFilterWidth = true,
    .getDialLock = true, .setDialLock = true,
    .getMonitor = true, .setMonitor = true, .getMonitorLevel = true, .setMonitorLevel = true,
    .getTransceive = true, .setTransceive = true,
    .getTuner = true, .setTuner = true, .startTune = true,
    .getVfo = true, .setVfo = true, .getVfoMode = true, .setVfoMode = true,
    .getSplit = true, .setSplit = true, .getRit = true, .setRit = true,
    .getBandStack = true, .getAgc = true, .setAgc = true,
    .getAtt = true, .setAtt = true, .getPreamp = true, .setPreamp = true,
    .getBreakIn = true, .setBreakIn = true, .getCwPitch = true, .setCwPitch = true,
    .getProc = true, .setProc = true, .getMicGain = true, .setMicGain = true,
    .getVox = true, .setVox = true, .getRxWidth = true, .setRxWidth = true,
};

// 16 02 00..02 and 11 00/20 (the IC-7300 and IC-705 CI-V guides; Hamlib: P.AMP1, P.AMP2 and
// 20 dB). On 144 and 430 MHz the IC-705 has only one preamp, and its attenuator works on HF and
// 50 MHz only: a set the radio refuses reads back as an error. 1A 03 the filter width (the CI-V
// guides; Hamlib gives all three radios x1ax03_supported).
constexpr SettingEncodings kIc7300Settings = {.preampLevels = 2, .attLevels = 1, .attDb = {20},
                                              .rxFilter = RxFilterEncoding::Icom};

// The IC-705: every IC-7300 command is in the IC-705 CI-V reference guide too, and Hamlib has
// it. 1C 01 drives an external tuner (AH-705). It also has the repeater shift (0F 10..12).
constexpr RadioCapabilities kIc705Caps = [] {
  RadioCapabilities caps = kIc7300Caps;
  caps.getRptShift = true;
  caps.setRptShift = true;
  return caps;
}();

// The IC-7760: as the IC-7300 (its CI-V reference guide), but no transceive: Hamlib has no
// address for it. Main and sub go through 29.
constexpr RadioCapabilities kIc7760Caps = [] {
  RadioCapabilities caps = kIc7300Caps;
  caps.getTransceive = false;
  caps.setTransceive = false;
  return caps;
}();

// 16 02 00..02, and 11 in dB: Icom's guide has 3..45 in 3 dB steps, Hamlib 6, 12 and 18, so those
// three are set; a read of any other step is reported as it is.
constexpr SettingEncodings kIc7760Settings = {.preampLevels = 2, .attLevels = 3, .attDb = {6, 12, 18},
                                              .rxFilter = RxFilterEncoding::Icom};

// SWR (15 12, 0000..0255): 0 = 1.0, 48 = 1.5, 80 = 2.0, 120 = 3.0 (the IC-7300, IC-705 and
// IC-7760 CI-V guides); 240 = 6.0 is Hamlib's (IC7300_SWR_CAL, IC705_SWR_CAL, IC7760_SWR_CAL).
constexpr MeterCalPoint kIcomSwr[] = {{0, 100}, {48, 150}, {80, 200}, {120, 300}, {240, 600}};

// --- Elecraft ---------------------------------------------------------------

constexpr uint32_t kKx2Bauds[] = {4800, 9600, 19200, 38400};

constexpr RadioCapabilities kKx2Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getPower = true, .getRfPower = true, .setRfPower = true, .getSwr = true,
    .getNb = true, .setNb = true,
    .getAgc = true, .setAgc = true,
    .getAtt = true, .setAtt = true, .getPreamp = true, .setPreamp = true,
    .getCwSpeed = true, .setCwSpeed = true,
};

// GT002 fast and GT004 slow (the Elecraft programmer's reference and Hamlib). No off: Hamlib
// sends GT000, the reference has off only in the K2's extended format. PA0/PA1 and RA00/RA01, the
// attenuator 10 dB (both). PCnnn in watts, 0..15 (12 on 80-20 m: a set the radio refuses reads
// back as an error). KS 8..50 WPM (both); no pitch: the reference reads it with CW, Hamlib
// with PT.
constexpr SettingEncodings kKx2Settings = {
    .agc = AgcEncoding::ElecraftGt, .frontEnd = FrontEndEncoding::Kenwood, .preampLevels = 1,
    .attLevels = 1, .attDb = {10}, .rfPower = RfPowerEncoding::PcWatts, .cwSpeedMinWpm = 8,
    .cwSpeedMaxWpm = 50,
};

constexpr AsciiCommandSet kKx2Commands = {
    .freqGet = "FA;", .freqSetFormat = "FA%011llu;",
    .modeGet = "MD;", .modeSetFormat = "MD%s;",
    .ifGet = "IF;", .idGet = "ID;", .omGet = "OM;",
    .smeterGet = "SM;", .powerGet = "PO;", .swrGet = "SW;",
    .nbGet = "NB;", .nbOnCmd = "NB1;", .nbOffCmd = "NB0;",
    .powerStateGet = "PS;", .powerStateOnCmd = "PS1;", .powerStateOffCmd = "PS0;",
    .freqReplyPrefix = "FA", .modeReplyPrefix = "MD", .ifReplyPrefix = "IF",
    .idReplyPrefix = "ID", .omReplyPrefix = "OM", .smeterReplyPrefix = "SM",
    .powerReplyPrefix = "PO", .swrReplyPrefix = "SW", .nbReplyPrefix = "NB",
    .powerStateReplyPrefix = "PS",
};

constexpr ModeCodes kKx2Modes = {
    .lsb = "1", .usb = "2", .am = "5", .cw = "3", .fm = "4", .cwr = "7", .rttyR = "9", .digi = "6",
};

// SM 0..15: S9 = 6, S9+20 = 9, S9+40 = 12, S9+60 = 15 (the programmer's reference, basic format;
// connecting sends K30, so not the extended 0..21). The reference gives no points below S9; those
// and the steps between are Hamlib's K3_SM_CAL (elecraft.h).
constexpr MeterCalPoint kKx2SMeter[] = {
    {0, -54}, {1, -42}, {2, -36}, {3, -24}, {4, -12}, {5, -6}, {6, 0}, {7, 10},
    {8, 15}, {9, 20}, {10, 30}, {11, 35}, {12, 40}, {13, 50}, {14, 55}, {15, 60},
};

// SWnnn: the SWR in tenths, 010..999 (the programmer's reference; Hamlib divides by 10). Kept
// from the last transmit; KX2 firmware 2.92 and later.
constexpr MeterCalPoint kKx2Swr[] = {{10, 100}, {999, 9990}};

// --- Kenwood ----------------------------------------------------------------

constexpr uint32_t kTs480Bauds[] = {4800, 9600, 19200, 38400, 57600};

// PR, PL, MG, VX, VG and VD as in the TS-480 PC command reference and Hamlib kenwood.c.
constexpr RadioCapabilities kTs480Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getRfPower = true, .setRfPower = true, .getSwr = true,
    .getNr = true, .setNr = true, .getNb = true, .setNb = true,
    .getAgc = true, .setAgc = true,
    .getAtt = true, .setAtt = true, .getPreamp = true, .setPreamp = true,
    .getCwSpeed = true, .setCwSpeed = true, .getCwDelay = true, .setCwDelay = true,
    .getProc = true, .setProc = true, .getMicGain = true, .setMicGain = true,
    .getVox = true, .setVox = true, .getRxCut = true, .setRxCut = true,
};

// VD 0..3000 ms in 150 ms steps, VG 0..9, GT 000 off, 001 fast, 002 slow, PA0/PA1 and
// RA00/RA01 (TS-480 PC command reference; Hamlib agrees, and has 12 dB for both, which the
// reference does not give). PC in watts, 5..100 on the TS-480SAT (25 in AM; the HX goes to 200,
// which is not offered); Hamlib and flrig agree. KS 10..60 WPM, SD 0..1000 ms in 50 ms steps, 0
// full break-in (the reference; Hamlib agrees). No pitch: the sources disagree on its scale. SL and
// SH the slope tune cuts in SSB, FM and AM (the reference and Hamlib; Hamlib has RTTY too, the
// reference not), as they are with menu 45 (the data filter) off. No IS: the reference gives no
// unit, Hamlib has none.
constexpr SettingEncodings kTs480Settings = {
    .voxDelay = VoxDelayEncoding::Ms4, .voxDelayMinMs = 0, .voxDelayMaxMs = 3000, .voxDelayStepMs = 150,
    .voxGainMax = 9, .agc = AgcEncoding::KenwoodGt, .frontEnd = FrontEndEncoding::Kenwood,
    .preampLevels = 1, .attLevels = 1, .rfPower = RfPowerEncoding::PcWatts, .rfPowerMinWatts = 5,
    .cwSpeedMinWpm = 10, .cwSpeedMaxWpm = 60, .cwDelay = VoxDelayEncoding::Ms4, .cwDelayMinMs = 0,
    .cwDelayMaxMs = 1000, .cwDelayStepMs = 50, .rxFilter = RxFilterEncoding::KenwoodTs480,
};

constexpr AsciiCommandSet kTs480Commands = {
    .freqGet = "FA;", .freqSetFormat = "FA%011llu;",
    .modeGet = "MD;", .modeSetFormat = "MD%s;",
    .smeterGet = "SM0;", .swrGet = "RM;",
    .nrGet = "NR;", .nrOnCmd = "NR1;", .nrOffCmd = "NR0;",
    .nbGet = "NB;", .nbOnCmd = "NB1;", .nbOffCmd = "NB0;",
    .freqReplyPrefix = "FA", .modeReplyPrefix = "MD", .ifReplyPrefix = "IF",
    .idReplyPrefix = "ID", .omReplyPrefix = "OM", .smeterReplyPrefix = "SM0",
    .swrReplyPrefix = "RM1", .nrReplyPrefix = "NR",
    .nbReplyPrefix = "NB",
};

// 9 is FSK-R and 8 is no mode (Hamlib kenwood_mode_table, flrig TS480SAT). There is no DIGI.
constexpr ModeCodes kTs480Modes = {
    .lsb = "1", .usb = "2", .am = "5", .cw = "3", .rtty = "6", .fm = "4", .cwr = "7", .rttyR = "9",
};

// SM0 0..20, as the radio's own voice guide speaks it (instruction manual, VOICE2): 0 = S0,
// 1-3 = S1, 4..11 = S2..S9, 12 = +10, 13-14 = +20, 15 = +30, 16-17 = +40, 18 = +50, 19-20 = +60 dB.
// Hamlib's TS480_STR_CAL agrees at each of its points.
constexpr MeterCalPoint kTs480SMeter[] = {
    {0, -54}, {1, -48}, {3, -48}, {11, 0}, {12, 10}, {13, 20}, {14, 20}, {15, 30},
    {16, 40}, {17, 40}, {18, 50}, {19, 60},
};

// RM1 0..20 (RM; answers RM1, RM2 and RM3 at once): Hamlib's TS480_SWR_CAL, "approximately on
// LCD display bars". Kenwood's command reference was not at hand.
constexpr MeterCalPoint kTs480Swr[] = {{0, 100}, {4, 150}, {8, 200}, {12, 300}, {20, 1000}};

// --- Yaesu FT-817, FT-818, FT-857, FT-897 (5-byte CAT) -----------------------

constexpr uint32_t kFt8x7Bauds[] = {4800, 9600, 38400};

constexpr RadioCapabilities kFt817Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getPower = true, .getRfPower = true, .getSwr = true, .getRxTx = true,
    .getNb = true,
    .getVfo = true, .setVfo = true, .getVfoMode = true, .setVfoMode = true,
    .getSplit = true, .setSplit = true, .getAgc = true, .getBreakIn = true, .getKeyer = true,
    .getCwSpeed = true, .getCwPitch = true, .getCwDelay = true, .getMicGain = true, .getVox = true,
};

// The FT-857/897 has no CAT command to pick VFO A or B.
constexpr RadioCapabilities kFt857Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getPower = true, .getRfPower = true, .getSwr = true, .getRxTx = true,
    .getNr = true, .getNb = true, .getNotch = true,
    .getVfo = true,
    .getSplit = true, .setSplit = true, .getAgc = true, .getAtt = true,
    .getBreakIn = true, .getKeyer = true, .getCwSpeed = true, .getCwPitch = true, .getCwDelay = true,
    .getProc = true, .getMicGain = true, .getVox = true,
};

// SWR per 0xBD meter bar, 0..15, measured on an FT-817 by WA4YA/DL4YA (Hamlib's FT817_SWR_CAL);
// 10 and up read 10. Hamlib has no FT-857 or FT-897 table, so they share it.
constexpr MeterCalPoint kFt8x7Swr[] = {
    {0, 100}, {1, 140}, {2, 180}, {3, 213}, {4, 225}, {5, 370}, {6, 600}, {7, 700},
    {8, 800}, {9, 900}, {10, 1000},
};

constexpr ModeCodes kFt8x7Modes = {
    .lsb = "00", .usb = "01", .am = "04", .cw = "02", .fm = "08", .cwr = "03", .digi = "0A",
    .pkt = "0C", .wfm = "06",
};

// --- Yaesu FT-847 (5-byte CAT, its own opcodes) ------------------------------

// Menu 37 offers 4800, 9600 and 57600.
constexpr uint32_t kFt847Bauds[] = {4800, 9600, 57600};

// Frequency, mode, S-meter, RX/TX and the PO meter while transmitting (protocol_ft847.*).
constexpr RadioCapabilities kFt847Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getPower = true, .getRxTx = true,
};

// The mode bytes; the narrow variants (0x82 CW-N ...) read as their mode. No RTTY or DIGI, and
// no FM: a frequency/mode read keys the FT-847 in FM (see ft847FmGuardActive in protocol_ft847.h).
constexpr ModeCodes kFt847Modes = {
    .lsb = "00", .usb = "01", .am = "04", .cw = "02", .cwr = "03",
};

// --- Yaesu FTDX10, FTDX101, FT-891 (ASCII CAT) -------------------------------

constexpr uint32_t kYaesuAsciiBauds[] = {4800, 9600, 19200, 38400};

constexpr RadioCapabilities kFtdx10Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getPower = true, .getRfPower = true, .setRfPower = true, .getSwr = true,
    .getAlc = true, .getRxTx = true, .getTxFreq = true,
    .getNr = true, .setNr = true, .getNrLevel = true, .setNrLevel = true,
    .getNb = true, .setNb = true, .getNbLevel = true, .setNbLevel = true,
    .getNotch = true, .setNotch = true, .getAutoNotch = true, .setAutoNotch = true,
    .getTone = true, .setTone = true, .getCtcss = true, .setCtcss = true, .getRptShift = true, .setRptShift = true,
    .getDialLock = true, .setDialLock = true,
    .getTuner = true, .setTuner = true, .startTune = true,
    .getVfo = true, .setVfo = true, .getVfoMode = true, .setVfoMode = true,
    .getSplit = true, .setSplit = true, .getAgc = true, .setAgc = true,
    .getAtt = true, .setAtt = true, .getPreamp = true, .setPreamp = true,
    .getBreakIn = true, .setBreakIn = true, .getKeyer = true, .setKeyer = true,
    .getCwSpeed = true, .setCwSpeed = true, .getCwPitch = true, .setCwPitch = true,
    .getCwDelay = true, .setCwDelay = true,
    .getProc = true, .setProc = true, .getMicGain = true, .setMicGain = true,
    .getVox = true, .setVox = true,
    .getIfShift = true, .setIfShift = true, .getRxWidth = true, .setRxWidth = true,
    .getNarrow = true, .setNarrow = true, .getNotchFreq = true, .setNotchFreq = true,
    .getContour = true, .setContour = true, .getApf = true, .setApf = true,
    .getRxCut = true, .setRxCut = true,
};

constexpr RadioCapabilities kFtdx101Caps = [] {
  RadioCapabilities caps = kFtdx10Caps;
  caps.getRxTx = false;
  caps.getTxFreq = false;
  caps.getVfoMode = false;
  caps.setVfoMode = false;
  // MG, VX, VG, and VD as on the FTDX10 (the CAT manual and Hamlib; flrig sends ms). No PR or
  // PL: the manual says PR01 is off, Hamlib that it is on.
  caps.getProc = false;
  caps.setProc = false;
  // KS and KP as on the FTDX10; no SD: the manual gives it in ms, Hamlib as the FTDX10's steps.
  caps.getCwDelay = false;
  caps.setCwDelay = false;
  return caps;
}();

// The FTDX101MP: as the FTDX101D, but no RF power: its CAT manual gives PC 005..100, Hamlib
// scales PC to its 200 W.
constexpr RadioCapabilities kFtdx101MpCaps = [] {
  RadioCapabilities caps = kFtdx101Caps;
  caps.getRfPower = false;
  caps.setRfPower = false;
  return caps;
}();

// VD, GT0, PA0 and RA0 as the FTDX10 and FTDX101 CAT manuals and Hamlib have them: VD a step,
// 30..3000 ms; PA0 0 IPO, 1 AMP1, 2 AMP2; RA0 0 off, 1..3 6, 12 and 18 dB. The FTDX101 sends
// them for the main band. PC in watts, 5..100 (the manuals, Hamlib and flrig). KS 4..60 WPM,
// KP 300..1050 Hz, SD a step of the VD table, 30..3000 ms (the FTDX10 manual and Hamlib). The
// receive filter (rx_filter_codec.h): SH, IS, NA0, BP0, CO0 and the EX cut menus of the FTDX10
// manual; the FTDX101 the same for its main band, with its own SSB widths and menu numbers.
constexpr SettingEncodings kFtdx10Settings = {
    .voxDelay = VoxDelayEncoding::YaesuSteps, .voxDelayMinMs = 30, .voxDelayMaxMs = 3000,
    .agc = AgcEncoding::YaesuGt0, .frontEnd = FrontEndEncoding::Yaesu, .preampLevels = 2,
    .attLevels = 3, .attDb = {6, 12, 18}, .rfPower = RfPowerEncoding::PcWatts, .rfPowerMinWatts = 5,
    .cwSpeedMinWpm = 4, .cwSpeedMaxWpm = 60, .cwPitchKp = true, .cwDelay = VoxDelayEncoding::YaesuSteps,
    .cwDelayMinMs = 30, .cwDelayMaxMs = 3000, .rxFilter = RxFilterEncoding::YaesuFtdx10,
};

constexpr SettingEncodings kFtdx101Settings = [] {
  SettingEncodings settings = kFtdx10Settings;
  settings.rxFilter = RxFilterEncoding::YaesuFtdx101;
  return settings;
}();

constexpr RadioCapabilities kFt891Caps = {
    .getFreq = true, .setFreq = true, .getMode = true, .setMode = true,
    .getSmeter = true, .getPower = true, .getRfPower = true, .setRfPower = true, .getSwr = true,
    .getAlc = true, .getNrLevel = true, .setNrLevel = true, .getNbLevel = true, .setNbLevel = true,
    .getNotch = true, .setNotch = true, .getAutoNotch = true, .setAutoNotch = true, .getTone = true, .setTone = true, .getCtcss = true, .setCtcss = true,
    .getRptShift = true, .setRptShift = true, .getDialLock = true, .setDialLock = true,
    .getTuner = true, .setTuner = true, .startTune = true,
    .getVfo = true, .setVfo = true,
    .getSplit = true, .setSplit = true, .getAgc = true, .setAgc = true,
    .getAtt = true, .setAtt = true, .getPreamp = true, .setPreamp = true,
    .getBreakIn = true, .setBreakIn = true, .getKeyer = true, .setKeyer = true,
    .getCwSpeed = true, .setCwSpeed = true, .getCwPitch = true, .setCwPitch = true,
    .getCwDelay = true, .setCwDelay = true,
    .getProc = true, .setProc = true, .getMicGain = true, .setMicGain = true,
    .getVox = true, .setVox = true,
    .getIfShift = true, .setIfShift = true, .getNarrow = true, .setNarrow = true,
    .getNotchFreq = true, .setNotchFreq = true, .getContour = true, .setContour = true,
    .getApf = true, .setApf = true, .getRxCut = true, .setRxCut = true,
};

// PR0, PL, MG, VX, VG, VD, GT0, PA0 and RA0 as the FT-891 CAT manual and Hamlib newcat have
// them (flrig agrees but for GT, which it lacks); VD in ms, 30..3000 in 10 ms steps; PA0 0 IPO,
// 1 AMP; RA0 0 off, 1 on (Hamlib's 12 dB is not in the manual); PC in watts, 5..100; KS 4..60
// WPM, KP 300..1050 Hz, SD 30..3000 ms (Hamlib sends ms too). The receive filter as the FTDX10's
// but IS (an on digit) and the EX menu numbers; no SH: its widths depend on NA, and its two manual
// editions differ.
constexpr SettingEncodings kFt891Settings = {
    .voxDelay = VoxDelayEncoding::Ms4, .voxDelayMinMs = 30, .voxDelayMaxMs = 3000, .voxDelayStepMs = 10,
    .agc = AgcEncoding::YaesuGt0, .frontEnd = FrontEndEncoding::Yaesu, .preampLevels = 1, .attLevels = 1,
    .rfPower = RfPowerEncoding::PcWatts, .rfPowerMinWatts = 5, .cwSpeedMinWpm = 4, .cwSpeedMaxWpm = 60,
    .cwPitchKp = true, .cwDelay = VoxDelayEncoding::Ms4, .cwDelayMinMs = 30, .cwDelayMaxMs = 3000,
    .cwDelayStepMs = 10, .rxFilter = RxFilterEncoding::YaesuFt891,
};

constexpr AsciiCommandSet kFtdx10Commands = {
    .freqGet = "FA;", .freqSetFormat = "FA%09llu;",
    .modeGet = "MD0;", .modeSetFormat = "MD0%s;",
    .vfoAGet = "FA;", .vfoASetFormat = "FA%09llu;",
    .vfoBGet = "FB;", .vfoBSetFormat = "FB%09llu;",
    .ifGet = "IF;", .idGet = "ID;",
    .smeterGet = "SM0;", .powerGet = "RM5;", .swrGet = "RM6;",
    .nrGet = "NR0;", .nrOnCmd = "NR01;", .nrOffCmd = "NR00;",
    .nbGet = "NB0;", .nbOnCmd = "NB01;", .nbOffCmd = "NB00;",
    .powerStateGet = "PS;", .powerStateOnCmd = "PS1;", .powerStateOffCmd = "PS0;",
    .tunerGet = "AC;", .tunerOnCmd = "AC001;", .tunerOffCmd = "AC000;", .tuneStartCmd = "AC002;",
    .splitGet = "ST;", .splitOnCmd = "ST1;", .splitOffCmd = "ST0;",
    .vfoGet = "VS;", .vfoACmd = "VS0;", .vfoBCmd = "VS1;", .vfoSwapCmd = "SV;",
    // BP P1 P2 P3P3P3, three digits (the CAT manuals, Hamlib "BP00%03d", flrig).
    .notchGet = "BP00;", .notchOnCmd = "BP00001;", .notchOffCmd = "BP00000;",
    .lockGet = "LK;", .lockOnCmd = "LK1;", .lockOffCmd = "LK0;",
    .freqReplyPrefix = "FA", .modeReplyPrefix = "MD0", .ifReplyPrefix = "IF",
    .idReplyPrefix = "ID", .omReplyPrefix = "OM", .smeterReplyPrefix = "SM0",
    .powerReplyPrefix = "RM5", .swrReplyPrefix = "RM6", .nrReplyPrefix = "NR0",
    .nbReplyPrefix = "NB0",
    .powerStateReplyPrefix = "PS", .tunerReplyPrefix = "AC", .splitReplyPrefix = "ST",
    .vfoReplyPrefix = "VS", .notchReplyPrefix = "BP00", .lockReplyPrefix = "LK",
};

constexpr AsciiCommandSet kFtdx101Commands = [] {
  AsciiCommandSet cmds = kFtdx10Commands;
  cmds.lockOnCmd = "LK7;";
  cmds.lockOffCmd = "LK4;";
  cmds.lockOffDigits = "04";
  cmds.ifGet = "";
  cmds.idGet = "";
  return cmds;
}();

constexpr AsciiCommandSet kFt891Commands = {
    .freqGet = "FA;", .freqSetFormat = "FA%09llu;",
    .modeGet = "MD0;", .modeSetFormat = "MD0%s;",
    .vfoAGet = "FA;", .vfoASetFormat = "FA%09llu;",
    .vfoBGet = "FB;", .vfoBSetFormat = "FB%09llu;",
    .smeterGet = "SM0;", .powerGet = "RM5;", .swrGet = "RM6;",
    .tunerGet = "AC;", .tunerOnCmd = "AC001;", .tunerOffCmd = "AC000;", .tuneStartCmd = "AC002;",
    .splitGet = "ST;", .splitOnCmd = "ST1;", .splitOffCmd = "ST0;",
    .vfoGet = "VS;", .vfoACmd = "VS0;", .vfoBCmd = "VS1;", .vfoSwapCmd = "SV;",
    .notchGet = "BP00;", .notchOnCmd = "BP00001;", .notchOffCmd = "BP00000;",
    .lockGet = "LK;", .lockOnCmd = "LK1;", .lockOffCmd = "LK0;",
    .freqReplyPrefix = "FA", .modeReplyPrefix = "MD0", .ifReplyPrefix = "IF",
    .idReplyPrefix = "ID", .omReplyPrefix = "OM", .smeterReplyPrefix = "SM0",
    .powerReplyPrefix = "RM5", .swrReplyPrefix = "RM6", .tunerReplyPrefix = "AC",
    .splitReplyPrefix = "ST", .vfoReplyPrefix = "VS", .notchReplyPrefix = "BP00",
    .lockReplyPrefix = "LK",
};

constexpr ModeCodes kYaesuAsciiModes = {
    .lsb = "1", .usb = "2", .am = "5", .cw = "3", .rtty = "6", .fm = "4", .cwr = "7", .rttyR = "9",
    .digi = "8",
};

// S-meter (SM0, 000..255). The CAT manuals give only the raw range; these are Hamlib's str_cal
// tables. Not yet checked against a radio's own meter.
// FTDX10: Hamlib's newcat.c yaesu_default_str_cal (the FTDX10 has no table of its own).
constexpr MeterCalPoint kFtdx10SMeter[] = {
    {0, -54},  {26, -42}, {51, -30}, {81, -18}, {105, -9}, {130, 0},
    {157, 12}, {186, 25}, {203, 35}, {237, 50}, {255, 60},
};

// FTDX101D and FTDX101MP: Hamlib's ftdx101.h FTDX101D_STR_CAL.
constexpr MeterCalPoint kFtdx101SMeter[] = {
    {0, -60},  {17, -54},  {25, -48},  {34, -42},  {51, -36}, {68, -30},
    {85, -24}, {102, -18}, {119, -12}, {136, -6}, {160, 0},  {255, 60},
};

// FT-891: Hamlib's ft891.h FT891_STR_CAL ("TBC" in Hamlib).
constexpr MeterCalPoint kFt891SMeter[] = {
    {0, -54},  {12, -48}, {27, -42}, {40, -36},  {55, -30},  {65, -24},
    {80, -18}, {95, -12}, {112, -6}, {130, 0},   {150, 10},  {172, 20},
    {190, 30}, {220, 40}, {240, 50}, {255, 60},
};

// SWR (RM6, 000..255). As for the S-meter, the CAT manuals give only the raw range.
// FTDX10, FTDX101D and FTDX101MP: Hamlib's FTDX10_SWR_CAL and FTDX101D_SWR_CAL, which agree
// (the 101's from G3VPX's tests).
constexpr MeterCalPoint kFtdxSwr[] = {
    {0, 100}, {26, 120}, {52, 150}, {89, 200}, {126, 300}, {173, 400}, {236, 500}, {255, 2500},
};

// FT-891: Hamlib has no table of its own and uses newcat's yaesu_default_swr_cal, measured on
// an FT-991.
constexpr MeterCalPoint kFt891Swr[] = {{12, 100}, {39, 135}, {65, 150}, {89, 200}, {242, 500}};

// --- The profiles, in slot order ---------------------------------------------
//
// Free slots, and what they are kept for:
//   19-20   Kenwood and Elecraft (TS-590, K3/K3S)
//   21-22   Xiegu and other compact radios (G90, X6200)
//   23-24   experimental and protocol test profiles

constexpr RadioProfile kProfiles[] = {
    {.slot = 1, .name = "Icom IC-7300", .vendor = VoiceVendor::Icom, .voiceDigits = "7300",
     .model = RadioModel::Ic7300, .protocol = PROTO_CIV,
     .link = {.port = RadioPort::CivJack, .baud = 9600, .bauds = kCivBauds, .civAddr = 0x94},
     .caps = kIc7300Caps, .settings = kIc7300Settings, .swrScale = kIcomSwr},
    {.slot = 2, .name = "Icom 706", .vendor = VoiceVendor::Icom, .voiceDigits = "706",
     .protocol = PROTO_CIV,
     .link = {.port = RadioPort::CivJack, .baud = 9600, .bauds = kCivBauds, .civAddr = 0x58},
     .caps = kIc706Caps},
    {.slot = 3, .name = "Icom 7300 rs232", .vendor = VoiceVendor::Icom, .voiceDigits = "7300232",
     .model = RadioModel::Ic7300, .protocol = PROTO_CIV,
     .link = {.port = RadioPort::Rs232, .baud = 9600, .bauds = kCivBauds, .civAddr = 0x94},
     .caps = kIc7300Caps, .settings = kIc7300Settings, .swrScale = kIcomSwr},
    {.slot = 4, .name = "Icom 706 rs232", .vendor = VoiceVendor::Icom, .voiceDigits = "706232",
     .protocol = PROTO_CIV,
     .link = {.port = RadioPort::Rs232, .baud = 9600, .bauds = kCivBauds, .civAddr = 0x58},
     .caps = kIc706Caps},
    {.slot = 5, .name = "Xiegu 106", .vendor = VoiceVendor::Xiegu, .voiceDigits = "106",
     .protocol = PROTO_CIV,
     .link = {.port = RadioPort::CatTtl, .baud = 19200, .bauds = kCivBauds, .civAddr = 0x76},
     .caps = kCivFreqModeCaps},
    {.slot = 6, .name = "Elecraft KX2", .vendor = VoiceVendor::Elecraft, .voiceDigits = "2",
     .protocol = PROTO_ELECRAFT_ASCII,
     .link = {.port = RadioPort::Rs232, .baud = 38400, .bauds = kKx2Bauds},
     .caps = kKx2Caps, .commands = &kKx2Commands, .modes = &kKx2Modes, .settings = kKx2Settings,
     .rfPowerMaxWatts = 15, .smeterScale = kKx2SMeter, .swrScale = kKx2Swr},
    {.slot = 7, .name = "Kenwood TS-480", .vendor = VoiceVendor::Kenwood, .voiceDigits = "480",
     .model = RadioModel::Ts480, .protocol = PROTO_KENWOOD_ASCII,
     .link = {.port = RadioPort::Rs232, .baud = 9600, .bauds = kTs480Bauds},
     .caps = kTs480Caps, .commands = &kTs480Commands, .modes = &kTs480Modes, .settings = kTs480Settings,
     .smeterScale = kTs480SMeter, .swrScale = kTs480Swr},
    {.slot = 8, .name = "Yaesu FT-817", .vendor = VoiceVendor::Yaesu, .voiceDigits = "817",
     .model = RadioModel::Ft817, .protocol = PROTO_YAESU_FT8X7,
     .link = {.port = RadioPort::Rs232, .baud = 4800, .bauds = kFt8x7Bauds},
     .caps = kFt817Caps, .modes = &kFt8x7Modes, .swrScale = kFt8x7Swr},
    {.slot = 9, .name = "Yaesu FT-857", .vendor = VoiceVendor::Yaesu, .voiceDigits = "857",
     .model = RadioModel::Ft857, .protocol = PROTO_YAESU_FT8X7,
     .link = {.port = RadioPort::Rs232, .baud = 4800, .bauds = kFt8x7Bauds},
     .caps = kFt857Caps, .modes = &kFt8x7Modes, .swrScale = kFt8x7Swr},
    {.slot = 10, .name = "Yaesu FT-897", .vendor = VoiceVendor::Yaesu, .voiceDigits = "897",
     .model = RadioModel::Ft857, .protocol = PROTO_YAESU_FT8X7,
     .link = {.port = RadioPort::Rs232, .baud = 4800, .bauds = kFt8x7Bauds},
     .caps = kFt857Caps, .modes = &kFt8x7Modes, .swrScale = kFt8x7Swr},
    {.slot = 11, .name = "Yaesu FTDX-10", .vendor = VoiceVendor::Yaesu, .voiceDigits = "10",
     .model = RadioModel::Ftdx10, .protocol = PROTO_YAESU_FTDX_ASCII,
     .link = {.port = RadioPort::Rs232, .baud = 38400, .bauds = kYaesuAsciiBauds},
     .caps = kFtdx10Caps, .commands = &kFtdx10Commands, .modes = &kYaesuAsciiModes,
     .settings = kFtdx10Settings,
     .smeterScale = kFtdx10SMeter, .swrScale = kFtdxSwr},
    {.slot = 12, .name = "Yaesu FTDX-101D", .vendor = VoiceVendor::Yaesu, .voiceDigits = "101",
     .protocol = PROTO_YAESU_FTDX_ASCII,
     .link = {.port = RadioPort::Rs232, .baud = 38400, .bauds = kYaesuAsciiBauds},
     .caps = kFtdx101Caps, .commands = &kFtdx101Commands, .modes = &kYaesuAsciiModes,
     .settings = kFtdx101Settings,
     .smeterScale = kFtdx101SMeter, .swrScale = kFtdxSwr},
    {.slot = 13, .name = "Yaesu FTDX-101MP", .vendor = VoiceVendor::Yaesu, .voiceDigits = "101",
     .protocol = PROTO_YAESU_FTDX_ASCII,
     .link = {.port = RadioPort::Rs232, .baud = 38400, .bauds = kYaesuAsciiBauds},
     .caps = kFtdx101MpCaps, .commands = &kFtdx101Commands, .modes = &kYaesuAsciiModes,
     .settings = kFtdx101Settings,
     .smeterScale = kFtdx101SMeter, .swrScale = kFtdxSwr},
    {.slot = 14, .name = "Yaesu FT-818", .vendor = VoiceVendor::Yaesu, .voiceDigits = "818",
     .model = RadioModel::Ft818, .protocol = PROTO_YAESU_FT8X7,
     .link = {.port = RadioPort::Rs232, .baud = 4800, .bauds = kFt8x7Bauds},
     .caps = kFt817Caps, .modes = &kFt8x7Modes, .swrScale = kFt8x7Swr},
    {.slot = 15, .name = "Yaesu FT-891", .vendor = VoiceVendor::Yaesu, .voiceDigits = "891",
     .protocol = PROTO_YAESU_FTDX_ASCII,
     .link = {.port = RadioPort::Rs232, .baud = 4800, .bauds = kYaesuAsciiBauds},
     .caps = kFt891Caps, .commands = &kFt891Commands, .modes = &kYaesuAsciiModes,
     .settings = kFt891Settings,
     .smeterScale = kFt891SMeter, .swrScale = kFt891Swr},
    {.slot = 16, .name = "Yaesu FT-847", .vendor = VoiceVendor::Yaesu, .voiceDigits = "847",
     .protocol = PROTO_YAESU_FT847,
     .link = {.port = RadioPort::Rs232, .baud = 4800, .bauds = kFt847Bauds},
     .caps = kFt847Caps, .modes = &kFt847Modes},
    {.slot = 17, .name = "Icom IC-705", .vendor = VoiceVendor::Icom, .voiceDigits = "705",
     .model = RadioModel::Ic705, .protocol = PROTO_CIV,
     .link = {.port = RadioPort::CivJack, .baud = 9600, .bauds = kCivBauds, .civAddr = 0xA4},
     .caps = kIc705Caps, .settings = kIc7300Settings, .rfPowerMaxWatts = 10,
     .swrScale = kIcomSwr},
    {.slot = 18, .name = "Icom IC-7760", .vendor = VoiceVendor::Icom, .voiceDigits = "7760",
     .model = RadioModel::Ic7760, .protocol = PROTO_CIV,
     .link = {.port = RadioPort::CivJack, .baud = 19200, .bauds = kCivBauds, .civAddr = 0xB2},
     .caps = kIc7760Caps, .settings = kIc7760Settings, .rfPowerMaxWatts = 200,
     .swrScale = kIcomSwr},
};

constexpr size_t kProfileCount = sizeof(kProfiles) / sizeof(kProfiles[0]);

// --- Build-time checks ----------------------------------------------------------

constexpr bool slotsAscendAndAreUnique() {
  for (size_t i = 0; i < kProfileCount; ++i) {
    if (kProfiles[i].slot == 0) return false;
    if (i > 0 && kProfiles[i].slot <= kProfiles[i - 1].slot) return false;
  }
  return true;
}

constexpr bool defaultBaudsAreOffered() {
  for (const RadioProfile& p : kProfiles) {
    if (!p.link.bauds.contains(p.link.baud)) return false;
  }
  return true;
}

constexpr bool civRadiosHaveAnAddress() {
  for (const RadioProfile& p : kProfiles) {
    if (p.protocol == PROTO_CIV && p.link.civAddr == 0) return false;
  }
  return true;
}

constexpr bool asciiRadiosHaveFreqAndModeCommands() {
  for (const RadioProfile& p : kProfiles) {
    if (!isAsciiProtocol(p.protocol)) continue;
    const AsciiCommandSet& c = *p.commands;
    if (!c.freqGet[0] || !c.freqSetFormat[0] || !c.modeGet[0] || !c.modeSetFormat[0]) return false;
  }
  return true;
}

// ASCII and FT-8x7 radios send the mode as the radio's own code.
constexpr bool codedRadiosHaveModeCodes() {
  for (const RadioProfile& p : kProfiles) {
    if (p.protocol == PROTO_CIV) continue;
    if (!p.modes->lsb[0] || !p.modes->usb[0]) return false;
  }
  return true;
}

// Interpolation on a meter scale needs ascending raw values.
constexpr bool meterScalesAscend() {
  for (const RadioProfile& p : kProfiles) {
    if (!p.smeterScale.ascends() || !p.swrScale.ascends()) return false;
  }
  return true;
}

// Each radio's SWR answer is on its own scale; there is no generic one.
constexpr bool swrRadiosHaveAScale() {
  for (const RadioProfile& p : kProfiles) {
    if (p.caps.getSwr && p.swrScale.empty()) return false;
  }
  return true;
}

static_assert(kProfileCount > 0, "no profiles");
static_assert(slotsAscendAndAreUnique(), "kProfiles: slots must be unique, above 0 and in ascending order");
static_assert(defaultBaudsAreOffered(), "kProfiles: a default baud is missing from that radio's bauds");
static_assert(civRadiosHaveAnAddress(), "kProfiles: a CI-V radio has no CI-V address");
static_assert(asciiRadiosHaveFreqAndModeCommands(), "kProfiles: an ASCII radio lacks freq or mode commands");
static_assert(codedRadiosHaveModeCodes(), "kProfiles: an ASCII or FT-8x7 radio lacks mode codes");
static_assert(meterScalesAscend(), "kProfiles: a meter scale is not in ascending order");
static_assert(swrRadiosHaveAScale(), "kProfiles: a radio with getSwr has no swrScale");

}  // namespace

const RadioProfile* profileForSlot(uint8_t slot) {
  for (const RadioProfile& p : kProfiles) {
    if (p.slot == slot) return &p;
  }
  return nullptr;
}

size_t profileCount() { return kProfileCount; }

const RadioProfile& profileAt(size_t index) {
  return kProfiles[index < kProfileCount ? index : 0];
}

uint8_t adjacentProfileSlot(uint8_t slot, int direction) {
  if (direction > 0) {
    for (const RadioProfile& p : kProfiles) {
      if (p.slot > slot) return p.slot;
    }
    return kProfiles[0].slot;
  }
  if (direction < 0) {
    for (size_t i = kProfileCount; i-- > 0;) {
      if (kProfiles[i].slot < slot) return kProfiles[i].slot;
    }
    return kProfiles[kProfileCount - 1].slot;
  }
  return slot;
}

const char* radioModelName(RadioModel model) {
  switch (model) {
    case RadioModel::Generic: return "generic";
    case RadioModel::Ic7300: return "IC-7300";
    case RadioModel::Ic705: return "IC-705";
    case RadioModel::Ic7760: return "IC-7760";
    case RadioModel::Ts480: return "TS-480";
    case RadioModel::Ft817: return "FT-817";
    case RadioModel::Ft818: return "FT-818";
    case RadioModel::Ft857: return "FT-857/897";
    case RadioModel::Ftdx10: return "FTDX10";
  }
  return "generic";
}
