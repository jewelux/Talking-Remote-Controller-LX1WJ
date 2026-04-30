# Changelog

## V3.5.5 Voice and FTDX10 Test Refresh

The repository is now aligned to the local `V3.5.5` firmware state.

Main points:

- synced the current local firmware sources into `firmware/`
- refreshed generated `voice_data.h` and the matching `voice_assets` clips
- added the current FTDX10 planning and regression notes from the local project folder
- renamed the main sketch entry to `TalkingRemoteControllerLX1WJ_V3_5_5.ino`

## V3.5.4 FTDX10 Family Test Update

The repository is now aligned to the local `V3.5.4` firmware state.

Main points:

- synced the newer FTDX10 family firmware files and profiles into the repository
- documented the first practical FTDX10 / FTDX101D / FTDX101MP keypad block for assisted testing
- added VFO mode, IF, ID, AGC, power-state, and preamp details to the published FTDX10 family notes
- tightened the user-facing documentation so it stays short and practical
- separated user help from specialist notes more clearly

## V3.5.1 FT8x7 Reliability Update

This repository now tracks the local `V3.5.1` firmware build and its matching radio documentation.

Main points:

- fixed FT8x7 VFO A/B selection so FT-817 family VFO operations no longer fail silently
- corrected FT8x7 split-status decoding to use the proper TX-status bit instead of the former ALC bit mix-up
- disabled the unsafe pseudo memory-write path that could unintentionally switch CTCSS/DCS settings on FT8x7 radios
- changed FT8x7 keypad write flows so frequency and mode writes are verified more carefully and rejected while the radio is in TX
- added clearer FT8x7 lock-state caching, poll suppression, and serial trace handling for more reliable keypad interaction
- updated `ft817.ini`, `ft818.ini`, `ft857.ini`, and `ft897.ini` with the current FT8x7 capability and Bank 6 defaults
- added SD profile boot-status reporting in the main firmware startup path
- refreshed FT8x7 radio pages and the support matrix so the published per-radio descriptions match the current code

## V3.5 Transition

- moved from the older single-sketch public state to the modular `V3.5` firmware structure
- introduced the parallel short name `HamTRC-LX1WJ`
- added SD card based profile handling and modular protocol code
