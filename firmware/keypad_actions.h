#pragma once

#include <stdint.h>

#include "keypad_settings.h"

// The keypad actions the keymap (keypad_keymap.cpp) can run. Only plain types
// here, so the keymap stays Arduino-free and the host tests (tests/keypad) can
// link it against a recording stub. The firmware implements these in the
// ui_keypad_bankN.cpp files.
//
// The keymap picks the action for the radio's layout; an action does not check
// the layout again. It may still work around what the protocol cannot do, e.g.
// the FT-8x7 CAT has no TX frequency reply, and Bank 8 needs
// CI-V. Actions named after a radio (ft857, Ft8x7, ...) are that radio's
// variant of a key.
//
// An action's serial trace names what it does, not the key: printKeypadAction
// prefixes the key being run (keypadActiveKey()), so one action can
// sit on any key. The comments give that trace.

// ---- Any bank ----
// "CMD <key> -> <cmd>", then the console command cmd. Most FTDX10 keys are one.
void sendKeypadCommand(const char* cmd);

// ---- Any bank: one Setting through its SettingOps ----
// "not available" where the radio lacks it.
void settingSay(Setting setting);    // its name and value
void settingEntry(Setting setting);  // its entry, from the radio's value: "<name> please"
// A short list (filter, preamp): the next value, from the last back to the first, set at once.
void settingNext(Setting setting);

// ---- A, B and C: the bank's focus (keypadActiveFocus()) ----
// "not available" with no focus, or where the radio lacks it.
void focusSay();    // A short: its name and value
void focusEntry();  // A long: its entry or pick
void focusCentre(); // A double: back to its centre (RIT 0, normal speed, the profile reset)
// B (dir 1) or C (-1): one step, or a big one. A live setting is set and said at once; RF
// power and the mode open their entry or pick one step away, staged for Enter.
void focusStep(int8_t dir);
void focusBigStep(int8_t dir);

// ---- Bank 1 ----
void queryBank1Frequency();
void beginBank1FrequencySet();
void roundActiveFrequency(uint32_t stepHz);
void queryBank1RxTx();
void queryBank1TxFrequency();
void queryBank1Ft857TxFrequency();  // FT-857/897: the frequency when split is off, else "not available"
void queryBank1Lock();
void toggleBank1Lock();
void queryBank1Power();    // "PO?", sent or staged
void queryBank1RfPower();  // "RFPOWER?", sent or staged
// The watts entry; on the FT-817/818 a level, picked by 1 (L1) .. 4 (High). Experimental on
// the FT-8x7, "experimental" unless EXPERIMENTAL is on.
void beginBank1RfPowerSet();
// QMX: the Max. PA voltage, which sets its output power. "PAVOLT?", sent or staged; the entry
// takes volts with one decimal.
void queryBank1PaVolt();
void beginBank1PaVoltSet();
void queryBank1Smeter();   // "SM?", sent or staged
void queryBank1Swr();      // "SWR?", sent or staged
void queryBank1Alc();      // "ALC?", sent or staged
void queryBank1Mode();     // "MODE?", sent or staged
void beginBank1ModeSelect();  // "MODE", mode select, "mode please"

// ---- Bank 2 ----
// The NR, NB and notch switches; the toggles go off -> on (TS-480 NR 1 -> NR 2, Icom notch
// NAR -> MID -> WIDE) -> off. Their levels are Settings (settingSay).
void queryBank2Nr();
void toggleBank2Nr();
void queryBank2Nb();
void toggleBank2Nb();
void queryBank2Notch();
void toggleBank2Notch();
void queryBank2Ft8x7AutoNotch();
void toggleBank2Ft8x7AutoNotch();
// Icom filter shape ("FILSHAPE?" is the console command): soft <-> sharp.
void toggleBank2FilterShape();
// AGC (agcQuery), and its select: 0 off, 1 auto, 2 fast, 3 slow, 4 mid, of the speeds the radio
// can be set to, then Enter sets it. On the FT-8x7 experimental, written to the EEPROM.
void queryBank2Agc();
void beginBank2AgcSelect();
// The auto notch (autoNotchQuery) and its toggle.
void queryBank2AutoNotch();
void toggleBank2AutoNotch();
// The attenuator (attQuery) and the preamp (preampQuery); the steps go to the next level, from
// the last one to off.
void queryBank2Att();
void stepBank2Att();
void queryBank2Preamp();
void stepBank2Preamp();
// FT-8x7 settings read from the radio's EEPROM.
void queryBank2Ft8x7Dbf();
void queryBank2Ft8x7Ipo();
void queryBank2Ft817Antenna();
// Experimental, written to the radio's EEPROM; "experimental" unless EXPERIMENTAL is on. The
// FT-857/897 DBF (both its bits, as the radio's key) and the FT-817/818 antenna jack of the
// current band.
void toggleBank2Ft8x7Dbf();
void toggleBank2Ft817Antenna();

// ---- Bank 3 ----
void queryBank3Split();
void toggleBank3Split();
void queryBank3TxFrequency();
void queryBank3VfoA();
void selectBank3VfoA();
void beginBank3VfoAFrequencySet();
void queryBank3VfoB();
void selectBank3VfoB();
void beginBank3VfoBFrequencySet();
// FT-817, FT-857/897: the tracked current and other VFO.
void queryBank3Ft8x7CurrentVfo();
void beginBank3Ft8x7CurrentVfoFrequencySet();
void beginBank3Ft8x7OtherVfoFrequencySet();
void queryBank3Ft8x7OtherVfo();
void toggleBank3Ft8x7Vfo();             // "A/B"
void copyBank3Ft817VfoToOther();         // "A=B"
void reportBank3Ft857VfoBUnsupported();  // "VFO B", "not available"
void queryBank3VfoAMode();
void beginBank3VfoAModeSet();
void queryBank3VfoBMode();
void beginBank3VfoBModeSet();
// FT-817: tell the VFO tracking that VFO A/B is active.
void syncBank3VfoA();
void syncBank3VfoB();
void queryBank3Ft8x7Rit();
void toggleBank3Ft8x7Rit();
void selectBank3Ft817ActiveVfoA();
void selectBank3Ft817ActiveVfoB();
void queryBank3RxTx();
void setBank3Ft857Ptt(bool on);
void queryBank3BandStack(uint8_t reg);
void recallBank3BandStack(uint8_t reg);

// ---- Bank 4 ----
void queryBank4Tuner();
void toggleBank4Tuner();
void triggerBank4Tune();

// ---- Bank 5 ----
void queryBank5Rit();
void toggleBank5Rit();
void setBank5RitOffset(int32_t hz);
void adjustBank5Rit(int32_t deltaHz);
void setBank5RitOff();  // "RIT OFF", RIT off or "RIT -> unsupported"

// ---- Bank 6 ----
// The repeater shift: on the FT-8x7 its own CAT command, elsewhere caps.setRptShift.
void setBank6RepeaterOff();
void setBank6RepeaterMinus();
void setBank6RepeaterPlus();
// FT-8x7: preset 1 or 2, the profile's rpt_offset_1 or rpt_offset_2 (default
// 600 kHz and 7.6 MHz).
void setBank6RepeaterOffsetPreset(uint8_t preset);
void beginBank6RepeaterOffsetEntry();
// The tone squelch type ("TONE?"), and off, CTCSS or DCS; CTCSS and DCS keep
// encode only. Over CAT CTCSS only.
void queryBank6Tone();
void setBank6ToneOff();
void setBank6ToneModeCtcss();
void setBank6ToneModeDcs();
// Encode only, decode only (where the radio has it) or both ("TONEDIR?"), and the step to the
// next of them.
void queryBank6ToneDir();
void stepBank6ToneDir();
// FT-857/897 split tone, menu 79 ("SPLITTONE?").
void queryBank6SplitTone();

// ---- Bank 8 ----
void queryBank8CivAddress();
void beginBank8CivAddressEntry();
void queryBank8Baud();  // "BAUD?", says "baud 4800"
// The rates the radio offers on keys 1, 2, ... (slowest first); Enter sets the one picked.
void beginBank8BaudSelect();
void toggleBank8Transceive();
// FT-8x7: the soft key (function) row and menu item saved when the radio's menu was last exited.
void queryBank8Ft8x7Row();
void queryBank8Ft8x7Menu();
// "USB?": the USB radio link, "usb on/off/error" with counts, or "usb not available".
void queryBank8Usb();
// Experimental: a row or menu item number, then Enter, written to the radio's EEPROM.
void beginBank8Ft8x7RowEntry();
void beginBank8Ft8x7MenuEntry();

// ---- Bank 9 ----
void queryBank9TuningSpeech();
void toggleBank9TuningSpeech();
void queryBank9Verbose();   // "VERBOSE?", says "verbose on/off"
void toggleBank9Verbose();
void queryBank9Experimental();   // "EXPERIMENTAL?", says "experimental on/off"
void toggleBank9Experimental();  // not saved, off after a restart
// -1/1: "VOLUME DOWN/UP"; -2/2: "VOLUME DOWN/UP FAST".
void queryBank9Volume();   // "VOLUME?", speaks the volume
void queryBank9Speed();    // "SPEED?", says "speed normal"
void stepBank9Speed();     // "SPEED NEXT": slow, normal, fast, slow again
void queryBank9Profile();  // "PROFILE?", speaks the profile
void beginBank9ProfileSelect();
// "PROFILE RESET": the saved baud and CI-V address back to the profile's defaults.
void resetBank9Profile();
void selectNextProfile();
void selectPrevProfile();

// ---- NanoVNA (KeypadLayout::NanoVna, Bank 1) ----
// "SWEEP": says "sweep", runs the scan, then "best match <swr> at <MHz>, worst match ...".
void measureNanoVna();
// "SWEEP?": the last result again, or "not available".
void repeatNanoVnaResult();
void queryNanoVnaStart();       // "START?": the sweep start in MHz
void beginNanoVnaStartEntry();  // "START": entry in MHz, "start please"
void queryNanoVnaStop();
void beginNanoVnaStopEntry();
void queryNanoVnaRange();  // "RANGE?": "band 80 meters", or "start ... stop ..."
// "BAND NEXT/PREV": the next or previous IARU Region 1 band preset, said as for RANGE?.
void stepNanoVnaBand(int direction);
void queryNanoVnaCalibration();        // "CAL?": the slot, or "auto"
void beginNanoVnaCalibrationSelect();  // "CAL": 0-6 a slot, A auto, D applies it
void testNanoVna();                    // "VERSION?": "version 1 point 0 ...", or "ok"
void queryNanoVnaBattery();            // "BATTERY?": "battery 4.26 volts"
void queryNanoVnaSerialNumber();       // "SN?": "s n 5 7 3 6 ..."

// ---- FTDX10 ----
// A key the FTDX10 layout hides: "<key> hidden on FTDX10", "not available".
void reportFtdx10HiddenKey();
