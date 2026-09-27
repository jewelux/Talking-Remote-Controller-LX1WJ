#pragma once

#include <stdint.h>

// The keypad actions the keymap (keypad_keymap.cpp) can run. Only plain types
// here, so the keymap stays Arduino-free and the host tests (tests/keypad) can
// link it against a recording stub. The firmware implements these in the
// ui_keypad_bankN.cpp files.
//
// Names that are not an older function's name stand for code that used to be
// inline in the dispatchers. Their serial output stays the same; the comment
// gives the command label and the command.

// ---- Bank 1 ----
void queryBank1Frequency();
void beginBank1FrequencySet();
void roundActiveFrequency(uint32_t stepHz);
void queryBank1RxTx();
void queryBank1TxFrequency();
void queryBank1Lock();
void toggleBank1Lock();
void queryBank1Power();    // sendOrStageBank1Command("BANK1 4 SHORT", "PO?")
void queryBank1RfPower();  // sendOrStageBank1Command("BANK1 6 SHORT", "RFPOWER?")
void beginBank1RfPowerSet();
void queryBank1Smeter();   // sendOrStageBank1Command("BANK1 7 SHORT", "SM?")
void queryBank1Swr();      // sendOrStageBank1Command("BANK1 8 SHORT", "SWR?")
void queryBank1Mode();     // sendOrStageBank1Command("BANK1 9 SHORT", "MODE?", true)
void beginBank1ModeSelect();  // "BANK1 9 LONG -> MODE", mode select, "mode please"
void ftdx10QueryTuner();    // "BANK1 5 SHORT -> TUNER?"       TUNER?
void ftdx10ToggleTuner();   // "BANK1 5 LONG -> TUNER TOGGLE"  TUNER TOGGLE
void ftdx10Tune();          // "BANK1 5 DOUBLE -> TUNE"        TUNE
void ftdx10QueryPreamp();   // "BANK1 6 SHORT -> PA?"          PA?
void ftdx10TogglePreamp();  // "BANK1 6 LONG -> PA TOGGLE"     PA TOGGLE

// ---- Bank 2 ----
void queryBank2Nr();
void toggleBank2Nr();
void queryBank2Nb();
void toggleBank2Nb();
void queryBank2Notch();
void toggleBank2Notch();
void queryBank2NrLevel();
void adjustBank2NrLevel(int deltaPercent);
void queryBank2NbLevel();
void adjustBank2NbLevel(int deltaPercent);
void queryBank2PbtInner();
void adjustBank2PbtInner(int delta);
void queryBank2PbtOuter();
void adjustBank2PbtOuter(int delta);
void sendBank2FilterShapeQuery();  // "BANK2 8 SHORT -> FILSHAPE?"  FILSHAPE?
void toggleBank2FilterShape();
void queryBank2FilterWidth();
void cycleBank2FilterWidth(int delta);
void ftdx10QueryAgc();         // "BANK2 4 SHORT -> GT?"      GT?
void ftdx10AgcFast();          // "BANK2 4 LONG -> GT FAST"   GT FAST
void ftdx10AgcSlow();          // "BANK2 4 DOUBLE -> GT SLOW" GT SLOW
void ftdx10QueryPowerState();  // "BANK2 5 SHORT -> PS?"      PS?
void ftdx10PowerOff();         // "BANK2 5 LONG -> PS OFF"    PS OFF
void ftdx10PowerOn();          // "BANK2 5 DOUBLE -> PS ON"   PS ON
void ftdx10QueryInfo();        // "BANK2 6 SHORT -> IF?"      IF?
void ftdx10QueryId();          // "BANK2 7 SHORT -> ID?"      ID?

// ---- Bank 3 ----
void queryBank3Split();
void toggleBank3Split();
void queryBank3TxFrequency();
void calibrateBank3Ft857Split();
void queryBank3VfoA();
void selectBank3VfoA();
void beginBank3VfoAFrequencySet();
void queryBank3VfoB();
void selectBank3VfoB();
void beginBank3VfoBFrequencySet();
void queryBank3VfoAMode(char key);     // key: the Bank 3 key pressed (3 on FT-817, 4 elsewhere)
void beginBank3VfoAModeSet(char key);
void queryBank3VfoBMode();
void beginBank3VfoBModeSet();
// FT-817, FT-857/897: tell the VFO tracking that VFO A/B is active.
void syncBank3VfoA();
void syncBank3VfoB();
void setBank3Ft857Clar(bool on);
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
void sendBank4MonitorQuery();     // "BANK4 1 SHORT -> MONITOR?"     MONITOR?
void toggleBank4Monitor();
void queryBank4MonitorLevel();
void adjustBank4MonitorLevel(int deltaPercent);
void sendBank4TransceiveQuery();  // "BANK4 3 SHORT -> TRANSCEIVE?"  TRANSCEIVE?
void toggleBank4Transceive();

// ---- Bank 5 ----
void queryBank5Rit();
void toggleBank5Rit();
void setBank5RitOffset(int32_t hz);
void adjustBank5Rit(int32_t deltaHz);
void setBank5RitOff();  // "BANK5 3 LONG -> RIT OFF", RIT off or "RIT -> unsupported"

// ---- Bank 6 ----
void setBank6RepeaterOff();
void setBank6RepeaterMinus();
void setBank6RepeaterPlus();
// preset 1 or 2: the profile's rpt_offset_1 or rpt_offset_2 (default 600 kHz
// and 7.6 MHz).
void setBank6RepeaterOffsetPreset(uint8_t preset);
void beginBank6RepeaterOffsetEntry();
void setBank6ToneOff();
void setBank6ToneModeCtcss();
void setBank6ToneModeDcs();
void queryBank6CtcssDefault();
void beginBank6CtcssEntry();
void queryBank6DcsDefault();
void beginBank6DcsEntry();

// ---- Bank 8 ----
void queryBank8CivAddress();
void beginBank8CivAddressEntry();
void cycleBank8Baud(int delta);

// ---- Bank 9 ----
// Light-Icom fallback: key '1'-'9' picks that built-in profile. Returns false
// when the fallback is not active.
bool selectBank9DirectProfile(char key);
void queryBank9TuningSpeech();
void toggleBank9TuningSpeech();
// -1/1: "BANK9 7/8 SHORT -> VOLUME DOWN/UP"; -2/2: "... LONG -> VOLUME DOWN/UP FAST".
void adjustBank9Volume(int delta);
void queryBank9Volume();   // "BANK9 9 SHORT -> VOLUME?", speaks the volume
void queryBank9Profile();  // "BANK9 A SHORT -> PROFILE?", speaks the profile
void beginBank9ProfileSelect();
void selectNextProfile();
void selectPrevProfile();

// ---- FTDX10 ----
// A key the FTDX10 layout hides: "<label> hidden on FTDX10", "not available".
void reportFtdx10HiddenKey(const char* label);
