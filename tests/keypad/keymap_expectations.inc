// Keypad characterization matrix: what each bank key does today, per radio
// family. Read from ui_keypad.cpp (keypadEvent, handleDeferredShortRelease,
// handleDoubleClick, shouldDelayShortRelease) and ui_keypad_actions.cpp
// (keypadHandleReleased) before the state machine refactor. The keymap tests
// check the new keypad_keymap against it.
//
// KEYMAP_ROW(families, bank, key, short, long, double, doubleHold, focus)
//
//   short   action run by a short press, in Normal mode.
//           "unassigned" = the "BANKn k -> unassigned" beep.
//   long    action run when the key is held (HOLD event). The release after it
//           is swallowed. "none" = nothing handles the hold, so the release
//           ran the short action as if it were a short press. The state
//           machine now beeps "BANKn k LONG -> unassigned" at the hold and
//           swallows the release instead.
//   double  "-" = the key does not wait for a double click; its short action
//           runs on release. Otherwise the short action is deferred by 220 ms
//           and this is the action for a second press within 220 ms.
//           "none" = the key waits, but has no double action: a quick second
//           press restarts the wait and the short action then runs once.
//   doubleHold  action run by a short press followed by a hold within 220 ms.
//           "none" = no action: the hold beeps.
//   focus   the label of the setting A, B and C move once the key has acted
//           (keypad_settings.cpp), or "-" for none.
//
// Rows marked "fixed" were changed on purpose after the refactor; the comment
// says what the old code did.
//
// Each (family, bank, key) is covered by at most one row. A key that no row
// covers is short "unassigned", long "none", double "-", double hold "none",
// focus "-". Every key that does
// something for any family has rows covering all families.
//
// Only the bank keys 0-9 and A-C are here. '*', '#', 'D', the entries, bank,
// profile and mode select belong to the input state machine and have their
// own tests.
//
// Families and the traits the old code reads for them:
//   IC7300  PROTO_CIV, caps.getRfPower (slots 1, 3, 18)
//   IC706   PROTO_CIV, no getRfPower (slots 2, 4, 5, 17)
//   TS480   PROTO_KENWOOD_ASCII
//   FTDX10  PROTO_YAESU_FTDX_ASCII, RadioModel::Ftdx10 (isFtdx10KeypadProfile)
//   FT891   PROTO_YAESU_FTDX_ASCII, another model
//   FT817   PROTO_YAESU_FT8X7, RadioModel::Ft817 (also Ft818)
//   FT857   PROTO_YAESU_FT8X7, RadioModel::Ft857 (FT-857, FT-897)
//   FT8X7_OTHER  PROTO_YAESU_FT8X7 with no known model (no built-in profile
//           today): the generic keys plus Bank 6
// Monitor, transceive, band stack and RIT support are PROTO_CIV only
// (radio_protocol.cpp), so they follow the family and need no trait of their own.
//
// Actions are named after the existing function where one exists, with its
// arguments. Inline code in the old dispatchers gets a name here:
//   queryBank1Power         sendBank1Query("PO?")
//   queryBank1Smeter        sendBank1Query("SM?")
//   queryBank1Swr           sendBank1Query("SWR?")
//   queryBank1Mode          sendBank1Query("MODE?")
//   queryBank1RfPower       sendBank1Query("RFPOWER?")
//   beginBank1ModeSelect    "MODE", mode select, "mode please"
//   send(CMD)               sendKeypadCommand(CMD): the console command
//                           CMD. Most FTDX10 keys; the old code had these
//                           inline or as ftdx10*() / send*Query().
//   setBank5RitOff          "RIT OFF", RIT off or "unsupported"
//   queryBank9Profile       "PROFILE?", speaks the profile
//   queryBank9Volume        "VOLUME?", speaks the volume
//   adjustBank9Volume(-1/1) "VOLUME DOWN/UP" + adjustVolumeLevel
//   adjustBank9Volume(-2/2) "VOLUME DOWN/UP FAST" + adjustBank9Volume
//   queryBank9Speed         "SPEED?", says the speech speed
//   stepBank9Speed          "SPEED NEXT", the next speech speed
//   focusSay, focusEntry, focusCentre, focusStep(dir), focusBigStep(dir)
//                           A, B and C on the bank's focus; the rows give the
//                           home focus (Bank 5 RITOFFSET, Bank 9 PROFILE)
//   reportFtdx10HiddenKey(K) also stands for reportFtdx10HiddenKeypadAction;
//                           K is the key the action is bound to (keypadActiveKey())
//
// The serial trace of an action starts with the key the keymap names, e.g.
// "CMD BANK3 2 LONG -> A=B"; the actions no longer spell out their key.
//
// After the refactor the actions stopped checking the radio: each radio branch
// inside an action became its own action picked here, named after the radio
// (e.g. queryBank3Ft857OtherVfo was the FT-857 branch of queryBank3VfoB).
//
// Dead branches found while building this table (no key sequence reaches them):
//   keypadHandleReleased: bank 1 '0' '1' '2', bank 2 CIV '4'-'7' '9',
//     bank 3 '7'-'9', bank 4 '0' '2' (all deferred or handled in keypadEvent)
//   handleDeferredShortRelease: bank 2 '1'-'3', FTDX10 '6'-'9', bank 3 '3'-'9',
//     bank 4 '1' '3', all of bank 9 (never deferred)
//   handleDoubleClick: bank 2 FTDX10 '6' '7' (never deferred on FTDX10)
//   Only reached from those: keypadBank2QueryNr/Nb/Notch, queryBank4Monitor,
//     queryBank4Transceive. queryBank2FilterShape has no caller at all.

// ---- Bank 1 ----------------------------------------------------------------
KEYMAP_ROW(FTDX10,          1, '0', "send(FREQ?)",           "beginBank1FrequencySet", "roundActiveFrequency(500)", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     1, '0', "queryBank1Frequency",   "beginBank1FrequencySet", "roundActiveFrequency(500)", "none", "-")
KEYMAP_ROW(FTDX10,          1, '1', "send(RXTX?)",           "none",                   "none", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     1, '1', "queryBank1RxTx",        "none",                   "none", "none", "-")
KEYMAP_ROW(FTDX10,          1, '2', "send(TXFREQ?)",         "none",                   "none", "none", "-")
KEYMAP_ROW(FT857,           1, '2', "queryBank1Ft857TxFrequency", "none",              "none", "none", "-")
KEYMAP_ROW(NOT(FTDX10 | FT857), 1, '2', "queryBank1TxFrequency", "none",               "none", "none", "-")
KEYMAP_ROW(FTDX10,          1, '3', "send(LOCK?)",           "send(LOCK TOGGLE)",      "-", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     1, '3', "queryBank1Lock",        "toggleBank1Lock",        "-", "none", "-")
KEYMAP_ROW(ALL,             1, '4', "queryBank1Power",       "none",                   "-", "none", "-")
// fixed: ALC was on 'A', and the FTDX10 tuner on '5' (it stays on Bank 4 '0').
KEYMAP_ROW(NOT(FT8X7),      1, '5', "queryBank1Alc",         "none",                   "-", "none", "-")
KEYMAP_ROW(FT8X7,           1, '5', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(IC7300 | TS480 | FTDX10 | FT891 | FT817 | FT857, 1, '6', "queryBank1RfPower", "beginBank1RfPowerSet", "-", "none", "RFPOWER")
KEYMAP_ROW(IC706,           1, '6', "unassigned",            "beginBank1RfPowerSet",   "-", "none", "RFPOWER")
KEYMAP_ROW(FT8X7_OTHER,     1, '6', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(ALL,             1, '7', "queryBank1Smeter",      "none",                   "-", "none", "-")
KEYMAP_ROW(ALL,             1, '8', "queryBank1Swr",         "none",                   "-", "none", "-")
KEYMAP_ROW(ALL,             1, '9', "queryBank1Mode",        "beginBank1ModeSelect",   "-", "none", "MODE")
KEYMAP_ROW(ALL,             1, 'A', "focusSay",              "focusEntry",             "-", "none", "-")
KEYMAP_ROW(ALL,             1, 'B', "focusStep(1)",          "focusBigStep(1)",        "-", "none", "-")
KEYMAP_ROW(ALL,             1, 'C', "focusStep(-1)",         "focusBigStep(-1)",       "-", "none", "-")

// ---- Bank 2 ----------------------------------------------------------------
KEYMAP_ROW(FT817,           2, '0', "queryBank2Ft817Antenna", "none",                  "-", "none", "-")
KEYMAP_ROW(NOT(FT817),      2, '0', "queryBank2AutoNotch",   "toggleBank2AutoNotch",   "-", "none", "-")
KEYMAP_ROW(ALL,             2, '1', "queryBank2Nr",          "toggleBank2Nr",          "-", "none", "-")
KEYMAP_ROW(ALL,             2, '2', "queryBank2Nb",          "toggleBank2Nb",          "-", "none", "-")
KEYMAP_ROW(ALL,             2, '3', "queryBank2Notch",       "toggleBank2Notch",       "-", "none", "-")
KEYMAP_ROW(CIV,             2, '4', "queryBank2NrLevel",     "adjustBank2NrLevel(10)", "adjustBank2NrLevel(-10)", "none", "-")
KEYMAP_ROW(FT817,           2, '4', "queryBank2Ft8x7NrLevel", "none",                  "-", "none", "-")
KEYMAP_ROW(FT857,           2, '4', "queryBank2Ft8x7NrLevel", "stepBank2Ft8x7NrLevel(1)", "stepBank2Ft8x7NrLevel(-1)", "none", "-")
KEYMAP_ROW(NOT(CIV | FT817 | FT857), 2, '4', "queryBank2YaesuRxLevel(0)", "stepBank2YaesuRxLevel(0,1)", "stepBank2YaesuRxLevel(0,-1)", "none", "-")
KEYMAP_ROW(CIV,             2, '5', "queryBank2NbLevel",     "adjustBank2NbLevel(10)", "adjustBank2NbLevel(-10)", "none", "-")
KEYMAP_ROW(FTDX10,          2, '5', "send(PS?)",             "send(PS OFF)",           "send(PS ON)", "none", "-")
KEYMAP_ROW(FT817,           2, '5', "queryBank2Ft8x7NbLevel", "none",                  "-", "none", "-")
KEYMAP_ROW(FT857,           2, '5', "queryBank2Ft8x7NbLevel", "stepBank2Ft8x7NbLevel(10)", "stepBank2Ft8x7NbLevel(-10)", "none", "-")
KEYMAP_ROW(NOT(CIV | FTDX10 | FT817 | FT857), 2, '5', "queryBank2YaesuRxLevel(1)", "stepBank2YaesuRxLevel(1,1)", "stepBank2YaesuRxLevel(1,-1)", "none", "-")
KEYMAP_ROW(CIV,             2, '6', "queryBank2PbtInner",    "adjustBank2PbtInner(10)", "adjustBank2PbtInner(-10)", "none", "-")
KEYMAP_ROW(FTDX10,          2, '6', "send(IF?)",             "none",                   "-", "none", "-")
KEYMAP_ROW(FT817,           2, '6', "queryBank2Ft8x7Dbf", "none",                  "-", "none", "-")
KEYMAP_ROW(FT857,           2, '6', "queryBank2Ft8x7Dbf", "toggleBank2Ft8x7Dbf", "-", "none", "-")
KEYMAP_ROW(NOT(CIV | FTDX10 | FT817 | FT857), 2, '6', "unassigned", "none",            "-", "none", "-")
KEYMAP_ROW(CIV,             2, '7', "queryBank2PbtOuter",    "adjustBank2PbtOuter(10)", "adjustBank2PbtOuter(-10)", "none", "-")
KEYMAP_ROW(FTDX10,          2, '7', "send(ID?)",             "none",                   "-", "none", "-")
KEYMAP_ROW(FT817,           2, '7', "queryBank2Ft8x7LowCut", "none",                  "-", "none", "-")
KEYMAP_ROW(FT857,           2, '7', "queryBank2Ft8x7LowCut", "stepBank2Ft8x7LowCut(1)", "stepBank2Ft8x7LowCut(-1)", "none", "-")
KEYMAP_ROW(NOT(CIV | FTDX10 | FT817 | FT857), 2, '7', "unassigned", "none",            "-", "none", "-")
KEYMAP_ROW(CIV,             2, '8', "send(FILSHAPE?)",       "toggleBank2FilterShape", "-", "none", "-")
KEYMAP_ROW(FTDX10,          2, '8', "reportFtdx10HiddenKey(BANK2 8 SHORT)", "none",          "-", "none", "-")  // fixed: was unassigned
KEYMAP_ROW(FT817,           2, '8', "queryBank2Ft8x7HighCut", "none",                  "-", "none", "-")
KEYMAP_ROW(FT857,           2, '8', "queryBank2Ft8x7HighCut", "stepBank2Ft8x7HighCut(1)", "stepBank2Ft8x7HighCut(-1)", "none", "-")
KEYMAP_ROW(NOT(CIV | FTDX10 | FT817 | FT857), 2, '8', "unassigned", "none",            "-", "none", "-")
KEYMAP_ROW(CIV,             2, '9', "queryBank2FilterWidth", "cycleBank2FilterWidth(1)", "cycleBank2FilterWidth(-1)", "none", "-")
KEYMAP_ROW(FTDX10,          2, '9', "reportFtdx10HiddenKey(BANK2 9 SHORT)", "none",          "-", "none", "-")  // fixed: was unassigned
KEYMAP_ROW(FT817 | FT857,   2, '9', "queryBank2Ft8x7MicEq",  "none",                   "-", "none", "-")
KEYMAP_ROW(NOT(CIV | FTDX10 | FT817 | FT857), 2, '9', "unassigned", "none",            "-", "none", "-")
KEYMAP_ROW(ALL,             2, 'A', "queryBank2Agc",         "beginBank2AgcSelect", "-", "none", "-")
KEYMAP_ROW(FT817 | FT857,   2, 'B', "queryBank2Ft8x7Ipo",    "none",                   "queryBank2Att", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857), 2, 'B', "queryBank2Preamp", "stepBank2Preamp",        "-", "none", "-")
KEYMAP_ROW(ALL,             2, 'C', "queryBank2Att",         "stepBank2Att",           "-", "none", "-")
KEYMAP_ROW(ALL,             7, '1', "queryBank7BreakIn",     "toggleBank7BreakIn",     "-", "none", "-")
KEYMAP_ROW(ALL,             7, '2', "queryBank7Keyer",       "toggleBank7Keyer",       "-", "none", "-")
KEYMAP_ROW(ALL,             7, '3', "queryBank7CwLevel(0)",  "stepBank7CwLevel(0,1)",  "stepBank7CwLevel(0,-1)", "none", "-")
KEYMAP_ROW(ALL,             7, '4', "queryBank7CwLevel(1)",  "stepBank7CwLevel(1,1)",  "stepBank7CwLevel(1,-1)", "none", "-")
KEYMAP_ROW(ALL,             7, '5', "queryBank7CwLevel(2)",  "stepBank7CwLevel(2,1)",  "stepBank7CwLevel(2,-1)", "none", "-")

// ---- Bank 3 ----------------------------------------------------------------
// FT-817 and FT-857/897 track the active VFO: '1' is the current VFO, '2' the
// other one.
KEYMAP_ROW(FTDX10,          3, '0', "send(SPLIT?)",          "send(SPLIT TOGGLE)",     "send(TXFREQ?)", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     3, '0', "queryBank3Split",       "toggleBank3Split",       "queryBank3TxFrequency", "none", "-")  // fixed: FT857 set split off / on / ON-then-OFF
// FT-817 '1' and '2' long: fixed, were the frequency entry like the double press.
KEYMAP_ROW(FT817 | FT857,   3, '1', "queryBank3Ft8x7CurrentVfo", "toggleBank3Ft8x7Vfo", "beginBank3Ft8x7CurrentVfoFrequencySet", "none", "-")
KEYMAP_ROW(FTDX10,          3, '1', "send(VFOA?)",           "send(VFO A)",            "beginBank3VfoAFrequencySet", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857 | FTDX10), 3, '1', "queryBank3VfoA", "selectBank3VfoA",   "beginBank3VfoAFrequencySet", "none", "-")
KEYMAP_ROW(FT817,           3, '2', "queryBank3Ft8x7OtherVfo", "copyBank3Ft817VfoToOther", "beginBank3Ft8x7OtherVfoFrequencySet", "none", "-")
KEYMAP_ROW(FT857,           3, '2', "queryBank3Ft8x7OtherVfo", "reportBank3Ft857VfoBUnsupported", "beginBank3Ft8x7OtherVfoFrequencySet", "none", "-")
KEYMAP_ROW(FTDX10,          3, '2', "send(VFOB?)",           "send(VFO B)",            "beginBank3VfoBFrequencySet", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857 | FTDX10), 3, '2', "queryBank3VfoB", "selectBank3VfoB",   "beginBank3VfoBFrequencySet", "none", "-")
KEYMAP_ROW(FT817,           3, '3', "queryBank3VfoAMode", "beginBank3VfoAModeSet", "-", "none", "-")
KEYMAP_ROW(NOT(FT817),      3, '3', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FT817,           3, '4', "syncBank3VfoA",         "syncBank3VfoB",          "-", "none", "-")
KEYMAP_ROW(FT857,           3, '4', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FTDX10,          3, '4', "send(VFOA MODE?)",      "beginBank3VfoAModeSet", "-", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857 | FTDX10), 3, '4', "queryBank3VfoAMode", "beginBank3VfoAModeSet", "-", "none", "-")
KEYMAP_ROW(FT817 | FT857,   3, '5', "queryBank3Ft8x7Rit",    "toggleBank3Ft8x7Rit",    "-", "none", "-")
KEYMAP_ROW(FTDX10,          3, '5', "send(VFOB MODE?)",      "beginBank3VfoBModeSet",  "-", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857 | FTDX10), 3, '5', "queryBank3VfoBMode", "beginBank3VfoBModeSet", "-", "none", "-")
KEYMAP_ROW(FT817,           3, '6', "selectBank3Ft817ActiveVfoA", "selectBank3Ft817ActiveVfoB", "-", "none", "-")
KEYMAP_ROW(FT857,           3, '6', "setBank3Ft857Ptt(false)", "setBank3Ft857Ptt(true)", "-", "none", "-")
KEYMAP_ROW(FTDX10,          3, '6', "send(RXTX?)",           "none",                   "-", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857 | FTDX10), 3, '6', "queryBank3RxTx", "none",              "-", "none", "-")
KEYMAP_ROW(FTDX10,          3, '7', "queryBank3BandStack(1)", "reportFtdx10HiddenKey(BANK3 7 LONG)", "-", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     3, '7', "queryBank3BandStack(1)", "recallBank3BandStack(1)", "-", "none", "-")
KEYMAP_ROW(FTDX10,          3, '8', "queryBank3BandStack(2)", "reportFtdx10HiddenKey(BANK3 8 LONG)", "-", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     3, '8', "queryBank3BandStack(2)", "recallBank3BandStack(2)", "-", "none", "-")
KEYMAP_ROW(FTDX10,          3, '9', "queryBank3BandStack(3)", "reportFtdx10HiddenKey(BANK3 9 LONG)", "-", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     3, '9', "queryBank3BandStack(3)", "recallBank3BandStack(3)", "-", "none", "-")
KEYMAP_ROW(ALL,             3, 'A', "focusSay",              "focusEntry",             "-", "none", "-")
KEYMAP_ROW(ALL,             3, 'B', "focusStep(1)",          "focusBigStep(1)",        "-", "none", "-")
KEYMAP_ROW(ALL,             3, 'C', "focusStep(-1)",         "focusBigStep(-1)",       "-", "none", "-")

// ---- Bank 4 ----------------------------------------------------------------
KEYMAP_ROW(FTDX10,          4, '0', "send(TUNER?)",          "send(TUNER TOGGLE)",     "send(TUNE)", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     4, '0', "queryBank4Tuner",       "toggleBank4Tuner",       "triggerBank4Tune", "none", "-")
KEYMAP_ROW(FTDX10,          4, '1', "reportFtdx10HiddenKey(BANK4 1 SHORT)", "reportFtdx10HiddenKey(BANK4 1 LONG)", "-", "none", "-")
KEYMAP_ROW(CIV,             4, '1', "send(MONITOR?)",        "toggleBank4Monitor",     "-", "none", "-")
KEYMAP_ROW(NOT(CIV | FTDX10), 4, '1', "unassigned",          "toggleBank4Monitor",     "-", "none", "-")
KEYMAP_ROW(FTDX10,          4, '2', "reportFtdx10HiddenKey(BANK4 2 SHORT)", "reportFtdx10HiddenKey(BANK4 2 LONG)", "reportFtdx10HiddenKey(BANK4 2 DOUBLE)", "none", "-")
KEYMAP_ROW(NOT(FTDX10),     4, '2', "queryBank4MonitorLevel", "adjustBank4MonitorLevel(10)", "adjustBank4MonitorLevel(-10)", "none", "-")
KEYMAP_ROW(FTDX10,          4, '3', "reportFtdx10HiddenKey(BANK4 3 SHORT)", "reportFtdx10HiddenKey(BANK4 3 LONG)", "-", "none", "-")
KEYMAP_ROW(CIV,             4, '3', "send(TRANSCEIVE?)",     "toggleBank4Transceive",  "-", "none", "-")
KEYMAP_ROW(NOT(CIV | FTDX10), 4, '3', "unassigned",          "toggleBank4Transceive",  "-", "none", "-")

// ---- Bank 5 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             5, '0', "queryBank5Rit",         "toggleBank5Rit",         "setBank5RitOffset(0)", "none", "-")
KEYMAP_ROW(ALL,             5, '1', "adjustBank5Rit(-10)",   "adjustBank5Rit(-100)",   "-", "none", "RITOFFSET")
KEYMAP_ROW(ALL,             5, '2', "adjustBank5Rit(10)",    "adjustBank5Rit(100)",    "-", "none", "RITOFFSET")
KEYMAP_ROW(ALL,             5, '3', "setBank5RitOffset(0)",  "setBank5RitOff",         "-", "none", "RITOFFSET")
KEYMAP_ROW(ALL,             5, '4', "adjustBank5Rit(-1)",    "adjustBank5Rit(-500)",   "-", "none", "RITOFFSET")
KEYMAP_ROW(ALL,             5, '5', "adjustBank5Rit(1)",     "adjustBank5Rit(500)",    "-", "none", "RITOFFSET")
KEYMAP_ROW(ALL,             5, 'A', "focusSay",              "focusEntry",             "focusCentre", "none", "-")
KEYMAP_ROW(ALL,             5, 'B', "focusStep(1)",          "focusBigStep(1)",        "-", "none", "-")
KEYMAP_ROW(ALL,             5, 'C', "focusStep(-1)",         "focusBigStep(-1)",       "-", "none", "-")

// ---- Bank 6 ----------------------------------------------------------------
// Not FT-8x7: fixed, was the actions' "BANK6 reserved" beep (with the double-click
// wait on '0'-'2'); now the keys are unassigned.
KEYMAP_ROW(FT8X7,           6, '0', "setBank6RepeaterOff",   "setBank6RepeaterMinus",  "setBank6RepeaterPlus", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, '0', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, '1', "setBank6RepeaterOffsetPreset(1)", "setBank6RepeaterOffsetPreset(2)", "beginBank6RepeaterOffsetEntry", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, '1', "unassigned",            "none",                   "-", "none", "-")
// '2' double hold (a short press, then a hold): setBank6ToneModeDcs, checked in test_keymap.cpp.
KEYMAP_ROW(FT8X7,           6, '2', "queryBank6Tone",        "setBank6ToneModeCtcss",  "setBank6ToneOff", "setBank6ToneModeDcs", "-")
KEYMAP_ROW(NOT(FT8X7),      6, '2', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, '3', "queryBank6CtcssDefault", "beginBank6CtcssEntry",  "-", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, '3', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, '4', "queryBank6DcsDefault",  "beginBank6DcsEntry",     "-", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, '4', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, '5', "queryBank6ToneDir",     "stepBank6ToneDir",       "-", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, '5', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, '6', "queryBank6SplitTone",   "none",                   "-", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, '6', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, 'A', "focusSay",              "focusEntry",             "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, 'B', "focusStep(1)",          "focusBigStep(1)",        "-", "none", "-")
KEYMAP_ROW(FT8X7,           6, 'C', "focusStep(-1)",         "focusBigStep(-1)",       "-", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, 'A', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, 'B', "unassigned",            "none",                   "-", "none", "-")
KEYMAP_ROW(NOT(FT8X7),      6, 'C', "unassigned",            "none",                   "-", "none", "-")

// ---- Bank 8 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             8, '1', "queryBank8CivAddress",  "beginBank8CivAddressEntry", "-", "none", "-")
KEYMAP_ROW(ALL,             8, '2', "queryBank8Baud",        "cycleBank8Baud(-1)",     "cycleBank8Baud(1)", "none", "-")
KEYMAP_ROW(FT817 | FT857,   8, '7', "queryBank8Ft8x7Row",    "beginBank8Ft8x7RowEntry", "-", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857), 8, '7', "unassigned",       "none",                   "-", "none", "-")
KEYMAP_ROW(FT817 | FT857,   8, '8', "queryBank8Ft8x7Menu",   "beginBank8Ft8x7MenuEntry", "-", "none", "-")
KEYMAP_ROW(NOT(FT817 | FT857), 8, '8', "unassigned",       "none",                   "-", "none", "-")
KEYMAP_ROW(ALL,             8, 'A', "focusSay",              "focusEntry",             "-", "none", "-")
KEYMAP_ROW(ALL,             8, 'B', "focusStep(1)",          "focusBigStep(1)",        "-", "none", "-")
KEYMAP_ROW(ALL,             8, 'C', "focusStep(-1)",         "focusBigStep(-1)",       "-", "none", "-")

// ---- Bank 9 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             9, '0', "queryBank9Experimental", "toggleBank9Experimental", "-", "none", "-")
KEYMAP_ROW(ALL,             9, '4', "queryBank9TuningSpeech", "toggleBank9TuningSpeech", "-", "none", "-")
KEYMAP_ROW(ALL,             9, '5', "queryBank9Verbose",     "toggleBank9Verbose",     "-", "none", "-")
KEYMAP_ROW(ALL,             9, '6', "queryBank9Speed",       "stepBank9Speed",         "-", "none", "SPEED")
KEYMAP_ROW(ALL,             9, '7', "adjustBank9Volume(-1)", "adjustBank9Volume(-2)",  "-", "none", "VOLUME")
KEYMAP_ROW(ALL,             9, '8', "adjustBank9Volume(1)",  "adjustBank9Volume(2)",   "-", "none", "VOLUME")
KEYMAP_ROW(ALL,             9, '9', "queryBank9Volume",      "none",                   "-", "none", "VOLUME")
// fixed: A, B and C were the profile's alone; the profile is now the home focus.
KEYMAP_ROW(ALL,             9, 'A', "focusSay",              "focusEntry",             "focusCentre", "none", "-")
KEYMAP_ROW(ALL,             9, 'B', "focusStep(1)",          "focusBigStep(1)",        "-", "none", "-")
KEYMAP_ROW(ALL,             9, 'C', "focusStep(-1)",         "focusBigStep(-1)",       "-", "none", "-")
