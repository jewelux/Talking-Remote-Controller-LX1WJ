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
