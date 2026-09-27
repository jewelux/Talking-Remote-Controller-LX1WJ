// Keypad characterization matrix: what each bank key does today, per radio
// family. Read from ui_keypad.cpp (keypadEvent, handleDeferredShortRelease,
// handleDoubleClick, shouldDelayShortRelease) and ui_keypad_actions.cpp
// (keypadHandleReleased) before the state machine refactor. The keymap tests
// check the new keypad_keymap against it.
//
// KEYMAP_ROW(families, bank, key, short, long, double)
//
//   short   action run by a short press, in Normal mode.
//           "unassigned" = the "BANKn k -> unassigned" beep.
//   long    action run when the key is held (HOLD event). The release after it
//           is swallowed. "none" = nothing handles the hold, so the release
//           runs the short action as if it were a short press.
//   double  "-" = the key does not wait for a double click; its short action
//           runs on release. Otherwise the short action is deferred by 220 ms
//           and this is the action for a second press within 220 ms.
//           "none" = the key waits, but has no double action: a quick second
//           press restarts the wait and the short action then runs once.
//
// Rows marked "fixed" were changed on purpose after the refactor; the comment
// says what the old code did.
//
// Each (family, bank, key) is covered by at most one row. A key that no row
// covers is short "unassigned", long "none", double "-". Every key that does
// something for any family has rows covering all families. Bank 7 is empty.
//
// Only the bank keys 0-9 and A-C are here. '*', '#', 'D', the entries, bank,
// profile and mode select belong to the input state machine and have their
// own tests.
//
// Families and the traits the old code reads for them:
//   IC7300  PROTO_CIV, caps.getRfPower (ic7300.ini, built-in slots 1, 2, 6)
//   IC706   PROTO_CIV, no getRfPower (ic706_*.ini, built-in slots 3, 4)
//   LIGHT   PROTO_CIV fallback when the SD profiles did not load
//           (lightIcomFallbackActive); built-in slot 1, IC-7300, getRfPower
//   TS480   PROTO_KENWOOD_ASCII
//   FTDX10  PROTO_YAESU_FTDX_ASCII, voice "yaesu" "10" (isFtdx10KeypadProfile)
//   FT891   PROTO_YAESU_FTDX_ASCII, other voice digits
//   FT817   PROTO_YAESU_FT8X7, variant "ft817" (also FT-818)
//   FT857   PROTO_YAESU_FT8X7, variant "ft857_897" (FT-857, FT-897)
// Monitor, transceive, band stack and RIT support are PROTO_CIV only
// (radio_protocol.cpp), so they follow the family and need no trait of their own.
//
// Actions are named after the existing function where one exists, with its
// arguments. Inline code in the old dispatchers gets a name here; the serial
// output must stay the same:
//   queryBank1Power         sendOrStageBank1Command("BANK1 4 SHORT", "PO?")
//   queryBank1Smeter        sendOrStageBank1Command("BANK1 7 SHORT", "SM?")
//   queryBank1Swr           sendOrStageBank1Command("BANK1 8 SHORT", "SWR?")
//   queryBank1Mode          sendOrStageBank1Command("BANK1 9 SHORT", "MODE?", true)
//   queryBank1RfPower       sendOrStageBank1Command("BANK1 6 SHORT", "RFPOWER?")
//   beginBank1ModeSelect    "BANK1 9 LONG -> MODE", mode select, "mode please"
//   ftdx10QueryTuner        "BANK1 5 SHORT -> TUNER?"    TUNER?
//   ftdx10ToggleTuner       "BANK1 5 LONG -> TUNER TOGGLE" TUNER TOGGLE
//   ftdx10Tune              "BANK1 5 DOUBLE -> TUNE"     TUNE
//   ftdx10QueryPreamp       "BANK1 6 SHORT -> PA?"       PA?
//   ftdx10TogglePreamp      "BANK1 6 LONG -> PA TOGGLE"  PA TOGGLE
//   ftdx10QueryAgc          "BANK2 4 SHORT -> GT?"       GT?
//   ftdx10AgcFast           "BANK2 4 LONG -> GT FAST"    GT FAST
//   ftdx10AgcSlow           "BANK2 4 DOUBLE -> GT SLOW"  GT SLOW
//   ftdx10QueryPowerState   "BANK2 5 SHORT -> PS?"       PS?
//   ftdx10PowerOff          "BANK2 5 LONG -> PS OFF"     PS OFF
//   ftdx10PowerOn           "BANK2 5 DOUBLE -> PS ON"    PS ON
//   ftdx10QueryInfo         "BANK2 6 SHORT -> IF?"       IF?
//   ftdx10QueryId           "BANK2 7 SHORT -> ID?"       ID?
//   sendBank2FilterShapeQuery "BANK2 8 SHORT -> FILSHAPE?" FILSHAPE?
//   sendBank4MonitorQuery   "BANK4 1 SHORT -> MONITOR?"  MONITOR?
//   sendBank4TransceiveQuery "BANK4 3 SHORT -> TRANSCEIVE?" TRANSCEIVE?
//   setBank5RitOff          "BANK5 3 LONG -> RIT OFF", RIT off or "unsupported"
//   queryBank9Profile       "BANK9 A SHORT -> PROFILE?", speaks the profile
//   queryBank9Volume        "BANK9 9 SHORT -> VOLUME?", speaks the volume
//   adjustBank9Volume(-1/1) "BANK9 7/8 SHORT -> VOLUME DOWN/UP" + adjustVolumeLevel
//   adjustBank9Volume(-2/2) "BANK9 7/8 LONG -> VOLUME DOWN/UP FAST" + adjustBank9Volume
//   reportFtdx10HiddenKey   also stands for reportFtdx10HiddenKeypadAction (same output)
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
KEYMAP_ROW(ALL,             1, '0', "queryBank1Frequency",   "beginBank1FrequencySet", "roundActiveFrequency(500)")
KEYMAP_ROW(ALL,             1, '1', "queryBank1RxTx",        "none",                   "none")
KEYMAP_ROW(ALL,             1, '2', "queryBank1TxFrequency", "none",                   "none")
KEYMAP_ROW(ALL,             1, '3', "queryBank1Lock",        "toggleBank1Lock",        "-")
KEYMAP_ROW(ALL,             1, '4', "queryBank1Power",       "none",                   "-")
KEYMAP_ROW(FTDX10,          1, '5', "ftdx10QueryTuner",      "ftdx10ToggleTuner",      "ftdx10Tune")
KEYMAP_ROW(NOT(FTDX10),     1, '5', "unassigned",            "none",                   "-")
KEYMAP_ROW(FTDX10,          1, '6', "ftdx10QueryPreamp",     "ftdx10TogglePreamp",     "-")
KEYMAP_ROW(IC7300 | LIGHT,  1, '6', "queryBank1RfPower",     "beginBank1RfPowerSet",   "-")
KEYMAP_ROW(IC706,           1, '6', "unassigned",            "beginBank1RfPowerSet",   "-")
KEYMAP_ROW(NOT(FTDX10 | CIV), 1, '6', "unassigned",          "none",                   "-")
KEYMAP_ROW(ALL,             1, '7', "queryBank1Smeter",      "none",                   "-")
KEYMAP_ROW(ALL,             1, '8', "queryBank1Swr",         "none",                   "-")
KEYMAP_ROW(ALL,             1, '9', "queryBank1Mode",        "beginBank1ModeSelect",   "-")

// ---- Bank 2 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             2, '1', "queryBank2Nr",          "toggleBank2Nr",          "-")
KEYMAP_ROW(ALL,             2, '2', "queryBank2Nb",          "toggleBank2Nb",          "-")
KEYMAP_ROW(ALL,             2, '3', "queryBank2Notch",       "toggleBank2Notch",       "-")
KEYMAP_ROW(CIV,             2, '4', "queryBank2NrLevel",     "adjustBank2NrLevel(10)", "adjustBank2NrLevel(-10)")
KEYMAP_ROW(FTDX10,          2, '4', "ftdx10QueryAgc",        "ftdx10AgcFast",          "ftdx10AgcSlow")
KEYMAP_ROW(NOT(CIV | FTDX10), 2, '4', "unassigned",          "none",                   "-")
KEYMAP_ROW(CIV,             2, '5', "queryBank2NbLevel",     "adjustBank2NbLevel(10)", "adjustBank2NbLevel(-10)")
KEYMAP_ROW(FTDX10,          2, '5', "ftdx10QueryPowerState", "ftdx10PowerOff",         "ftdx10PowerOn")
KEYMAP_ROW(NOT(CIV | FTDX10), 2, '5', "unassigned",          "none",                   "-")
KEYMAP_ROW(CIV,             2, '6', "queryBank2PbtInner",    "adjustBank2PbtInner(10)", "adjustBank2PbtInner(-10)")
KEYMAP_ROW(FTDX10,          2, '6', "ftdx10QueryInfo",       "none",                   "-")
KEYMAP_ROW(NOT(CIV | FTDX10), 2, '6', "unassigned",          "none",                   "-")
KEYMAP_ROW(CIV,             2, '7', "queryBank2PbtOuter",    "adjustBank2PbtOuter(10)", "adjustBank2PbtOuter(-10)")
KEYMAP_ROW(FTDX10,          2, '7', "ftdx10QueryId",         "none",                   "-")
KEYMAP_ROW(NOT(CIV | FTDX10), 2, '7', "unassigned",          "none",                   "-")
KEYMAP_ROW(CIV,             2, '8', "sendBank2FilterShapeQuery", "toggleBank2FilterShape", "-")
KEYMAP_ROW(FTDX10,          2, '8', "reportFtdx10HiddenKey(BANK2 8)", "none",          "-")  // fixed: was unassigned
KEYMAP_ROW(NOT(CIV | FTDX10), 2, '8', "unassigned",          "none",                   "-")
KEYMAP_ROW(CIV,             2, '9', "queryBank2FilterWidth", "cycleBank2FilterWidth(1)", "cycleBank2FilterWidth(-1)")
KEYMAP_ROW(FTDX10,          2, '9', "reportFtdx10HiddenKey(BANK2 9)", "none",          "-")  // fixed: was unassigned
KEYMAP_ROW(NOT(CIV | FTDX10), 2, '9', "unassigned",          "none",                   "-")

// ---- Bank 3 ----------------------------------------------------------------
KEYMAP_ROW(FT857,           3, '0', "queryBank3Split",       "toggleBank3Split",       "calibrateBank3Ft857Split")
KEYMAP_ROW(NOT(FT857),      3, '0', "queryBank3Split",       "toggleBank3Split",       "queryBank3TxFrequency")
KEYMAP_ROW(FT817,           3, '1', "queryBank3VfoA",        "beginBank3VfoAFrequencySet", "beginBank3VfoAFrequencySet")
KEYMAP_ROW(NOT(FT817),      3, '1', "queryBank3VfoA",        "selectBank3VfoA",        "beginBank3VfoAFrequencySet")
KEYMAP_ROW(FT817,           3, '2', "queryBank3VfoB",        "beginBank3VfoBFrequencySet", "beginBank3VfoBFrequencySet")
KEYMAP_ROW(NOT(FT817),      3, '2', "queryBank3VfoB",        "selectBank3VfoB",        "beginBank3VfoBFrequencySet")
KEYMAP_ROW(FT817,           3, '3', "queryBank3VfoAMode(3)", "beginBank3VfoAModeSet(3)", "-")
KEYMAP_ROW(NOT(FT817),      3, '3', "unassigned",            "none",                   "-")
KEYMAP_ROW(FT8X7,           3, '4', "syncBank3VfoA",         "syncBank3VfoB",          "-")
KEYMAP_ROW(NOT(FT8X7),      3, '4', "queryBank3VfoAMode(4)", "beginBank3VfoAModeSet(4)", "-")
KEYMAP_ROW(FT857,           3, '5', "setBank3Ft857Clar(true)", "setBank3Ft857Clar(false)", "-")
KEYMAP_ROW(NOT(FT857),      3, '5', "queryBank3VfoBMode",    "beginBank3VfoBModeSet",  "-")
KEYMAP_ROW(FT817,           3, '6', "selectBank3Ft817ActiveVfoA", "selectBank3Ft817ActiveVfoB", "-")
KEYMAP_ROW(FT857,           3, '6', "queryBank3RxTx",        "setBank3Ft857Ptt(true)", "-")
KEYMAP_ROW(NOT(FT8X7),      3, '6', "queryBank3RxTx",        "none",                   "-")
KEYMAP_ROW(FTDX10,          3, '7', "queryBank3BandStack(1)", "reportFtdx10HiddenKey(BANK3 BSTACK)", "-")
KEYMAP_ROW(NOT(FTDX10),     3, '7', "queryBank3BandStack(1)", "recallBank3BandStack(1)", "-")
KEYMAP_ROW(FTDX10,          3, '8', "queryBank3BandStack(2)", "reportFtdx10HiddenKey(BANK3 BSTACK)", "-")
KEYMAP_ROW(NOT(FTDX10),     3, '8', "queryBank3BandStack(2)", "recallBank3BandStack(2)", "-")
KEYMAP_ROW(FTDX10,          3, '9', "queryBank3BandStack(3)", "reportFtdx10HiddenKey(BANK3 BSTACK)", "-")
KEYMAP_ROW(NOT(FTDX10),     3, '9', "queryBank3BandStack(3)", "recallBank3BandStack(3)", "-")

// ---- Bank 4 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             4, '0', "queryBank4Tuner",       "toggleBank4Tuner",       "triggerBank4Tune")
KEYMAP_ROW(FTDX10,          4, '1', "reportFtdx10HiddenKey(BANK4)", "reportFtdx10HiddenKey(BANK4)", "-")
KEYMAP_ROW(CIV,             4, '1', "sendBank4MonitorQuery", "toggleBank4Monitor",     "-")
KEYMAP_ROW(NOT(CIV | FTDX10), 4, '1', "unassigned",          "toggleBank4Monitor",     "-")
KEYMAP_ROW(FTDX10,          4, '2', "reportFtdx10HiddenKey(BANK4)", "reportFtdx10HiddenKey(BANK4)", "reportFtdx10HiddenKey(BANK4)")
KEYMAP_ROW(NOT(FTDX10),     4, '2', "queryBank4MonitorLevel", "adjustBank4MonitorLevel(10)", "adjustBank4MonitorLevel(-10)")
KEYMAP_ROW(FTDX10,          4, '3', "reportFtdx10HiddenKey(BANK4)", "reportFtdx10HiddenKey(BANK4)", "-")
KEYMAP_ROW(CIV,             4, '3', "sendBank4TransceiveQuery", "toggleBank4Transceive", "-")
KEYMAP_ROW(NOT(CIV | FTDX10), 4, '3', "unassigned",          "toggleBank4Transceive",  "-")

// ---- Bank 5 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             5, '0', "queryBank5Rit",         "toggleBank5Rit",         "setBank5RitOffset(0)")
KEYMAP_ROW(ALL,             5, '1', "adjustBank5Rit(-10)",   "adjustBank5Rit(-100)",   "-")
KEYMAP_ROW(ALL,             5, '2', "adjustBank5Rit(10)",    "adjustBank5Rit(100)",    "-")
KEYMAP_ROW(ALL,             5, '3', "setBank5RitOffset(0)",  "setBank5RitOff",         "-")
KEYMAP_ROW(ALL,             5, '4', "adjustBank5Rit(-1)",    "adjustBank5Rit(-500)",   "-")
KEYMAP_ROW(ALL,             5, '5', "adjustBank5Rit(1)",     "adjustBank5Rit(500)",    "-")

// ---- Bank 6 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             6, '0', "setBank6RepeaterOff",   "setBank6RepeaterMinus",  "setBank6RepeaterPlus")
KEYMAP_ROW(ALL,             6, '1', "setBank6RepeaterOffsetPreset(1)", "setBank6RepeaterOffsetPreset(2)", "beginBank6RepeaterOffsetEntry")
KEYMAP_ROW(ALL,             6, '2', "setBank6ToneOff",       "setBank6ToneModeCtcss",  "setBank6ToneModeDcs")
KEYMAP_ROW(ALL,             6, '3', "queryBank6CtcssDefault", "beginBank6CtcssEntry",  "-")
KEYMAP_ROW(ALL,             6, '4', "queryBank6DcsDefault",  "beginBank6DcsEntry",     "-")

// ---- Bank 8 ----------------------------------------------------------------
KEYMAP_ROW(ALL,             8, '1', "queryBank8CivAddress",  "beginBank8CivAddressEntry", "-")
KEYMAP_ROW(ALL,             8, '2', "cycleBank8Baud(1)",     "cycleBank8Baud(-1)",     "none")

// ---- Bank 9 ----------------------------------------------------------------
// LIGHT: digits 1-9 pick a built-in profile directly, ahead of any other short.
KEYMAP_ROW(LIGHT,           9, '1', "selectBank9DirectProfile(1)", "none",            "-")
KEYMAP_ROW(LIGHT,           9, '2', "selectBank9DirectProfile(2)", "none",            "-")
KEYMAP_ROW(LIGHT,           9, '3', "selectBank9DirectProfile(3)", "none",            "-")
KEYMAP_ROW(LIGHT,           9, '4', "selectBank9DirectProfile(4)", "toggleBank9TuningSpeech", "-")
KEYMAP_ROW(LIGHT,           9, '5', "selectBank9DirectProfile(5)", "none",            "-")
KEYMAP_ROW(LIGHT,           9, '6', "selectBank9DirectProfile(6)", "none",            "-")
KEYMAP_ROW(LIGHT,           9, '7', "selectBank9DirectProfile(7)", "adjustBank9Volume(-2)", "-")
KEYMAP_ROW(LIGHT,           9, '8', "selectBank9DirectProfile(8)", "adjustBank9Volume(2)", "-")
KEYMAP_ROW(LIGHT,           9, '9', "selectBank9DirectProfile(9)", "none",            "-")
KEYMAP_ROW(NOT(LIGHT),      9, '1', "unassigned",            "none",                   "-")
KEYMAP_ROW(NOT(LIGHT),      9, '2', "unassigned",            "none",                   "-")
KEYMAP_ROW(NOT(LIGHT),      9, '3', "unassigned",            "none",                   "-")
KEYMAP_ROW(NOT(LIGHT),      9, '4', "queryBank9TuningSpeech", "toggleBank9TuningSpeech", "-")
KEYMAP_ROW(NOT(LIGHT),      9, '5', "unassigned",            "none",                   "-")
KEYMAP_ROW(NOT(LIGHT),      9, '6', "unassigned",            "none",                   "-")
KEYMAP_ROW(NOT(LIGHT),      9, '7', "adjustBank9Volume(-1)", "adjustBank9Volume(-2)",  "-")
KEYMAP_ROW(NOT(LIGHT),      9, '8', "adjustBank9Volume(1)",  "adjustBank9Volume(2)",   "-")
KEYMAP_ROW(NOT(LIGHT),      9, '9', "queryBank9Volume",      "none",                   "-")
KEYMAP_ROW(ALL,             9, 'A', "queryBank9Profile",     "beginBank9ProfileSelect", "-")
KEYMAP_ROW(ALL,             9, 'B', "selectNextProfile",     "none",                   "-")
KEYMAP_ROW(ALL,             9, 'C', "selectPrevProfile",     "none",                   "-")
