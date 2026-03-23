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
