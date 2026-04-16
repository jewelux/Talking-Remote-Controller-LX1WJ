# Talking Remote Controller LX1WJ Builder Guide

This guide is the short technical entry point for builders, contributors, and test helpers.

## Current Firmware Line

The repository now documents and ships the modular firmware line **V3.5.4**.
The current focus is a clean first Yaesu **FTDX10 family** field-test block on top of the existing multi-radio platform.

## Builder Path

Use this repository if you need to:

- build the ESP32-S3 controller
- load the current firmware
- adjust SD card profiles
- extend CAT command handling
- document real-radio behavior

## Main Technical References

- [QUICKSTART.md](QUICKSTART.md)
- [docs/hardware_wiring.md](docs/hardware_wiring.md)
- [docs/radio-support-matrix.md](docs/radio-support-matrix.md)
- [docs/radios/yaesu-ftdx10-family.md](docs/radios/yaesu-ftdx10-family.md)
- [docs/radios/yaesu-ftdx10-test-plan.md](docs/radios/yaesu-ftdx10-test-plan.md)

## Firmware Structure

`firmware/TalkingRemoteControllerLX1WJ_V3_5_4.ino` is the main sketch entry.
The project is split into modular source files for:

- protocol handling
- keypad and speech UI
- runtime and live radio state
- SD card profile loading

## Current FTDX10 Family Scope

The present documented scope is the first practical CAT block:

- frequency read and set
- mode read and set
- VFO A and VFO B select
- VFO-A and VFO-B frequency read and set
- VFO-A and VFO-B mode read and set
- split read and set
- S-meter, power, and SWR
- NR, NB, notch
- lock
- tuner query, toggle, and tune
- preamp query and toggle
- AGC query plus fast and slow presets
- power-state query plus on and off path
- IF and ID query

Anything beyond that should be treated as later expansion unless documented otherwise.

## Documentation Rule

Keep user-facing operation notes separate from engineering detail.
If a function is still experimental, say so clearly and do not present it as normal keypad behavior.