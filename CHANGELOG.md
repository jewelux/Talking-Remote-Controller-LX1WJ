# Changelog

## V3.5 Transition

This repository has been updated from the earlier public single-sketch state to the newer modular `V3.5` project structure.
The project name **HamTRC-LX1WJ** is now also introduced in the documentation as a shorter parallel name for Talking Remote Controller LX1WJ.

Main changes in this transition:

- replaced the older `firmware/TalkingRemoteControllerLX1WJ_V1_0.ino` centered layout with the modular `V3_5` firmware structure
- added protocol, runtime, UI, profile-loading, and parser source files as separate modules
- introduced SD card based radio profile files in `firmware/SDCard`
- retained the flash-based spoken token approach through `firmware/voice_data.h`
- updated the top-level documentation so the project description matches the new architecture while keeping the existing licensing and safety/disclaimer approach

This transition marks the step from the earlier proof-of-concept repository state toward a more extensible multi-radio platform.

## Documentation Update - IC-7300 Command Map

- added `docs/radios/icom-ic-7300.md` as a dedicated radio-specific command page
- kept the general user guide separate from per-radio command descriptions
- updated the support matrix note for the IC-7300 profile to point to the dedicated page
- synced updated `V3.5` firmware source files into the repository
- added dedicated `V3.5` radio pages for Yaesu FT-817 and FT-857

## FT8x7 Family Update

The Yaesu FT8x7 family has been expanded and clarified in the modular `V3.5` codebase.

Main points:

- added clearer FT8x7 family handling with separate `ft817` and `ft857_897` profile variants
- expanded the documented Yaesu CAT implementation and practical hardware verification work for FT-817 and FT-857
- introduced a dedicated FT8x7 repeater and tone keypad bank (`Bank 6`) while keeping the IC-7300-led bank structure intact
- added dedicated FT8x7 documentation pages for keypad layout and CAT implementation status
- synced the current FT8x7 firmware and profile state into the repository

This update improves both practical radio support and the published documentation without turning the changelog into a command-by-command lab report.

## FTDX10 Family Bring-Up

A first documented Yaesu FTDX10/101 ASCII CAT bring-up has been added to the modular V3.5 codebase.

Main points:

- expanded the generic ASCII layer with documented lock and tuner support for the FTDX10 family
- enriched the FTDX10, FTDX101D, and FTDX101MP SD profiles with documented CAT command mappings for tuner, lock, NR, NB, notch, preamp, AGC, and power-state handling
- added a concise first-test document for remote hardware validation at docs/radios/yaesu-ftdx10-test-plan.md
- kept the scope intentionally focused on a safe first real-radio validation block before keypad specialization

## FTDX10 VFO and Split Expansion

The first FTDX10 bring-up block has been extended with documented VFO and split handling.

Main points:

- added documented ASCII VFO A, VFO B, VFO-A frequency, and VFO-B frequency support in the generic protocol layer
- added documented ASCII split query and split set handling for the FTDX10/101 family profiles
- extended the first public FTDX10 test plan accordingly
- added a simple blind-friendly FTDX10 test list for assisted keypad and serial validation
