# Talking Remote Controller LX1WJ - Builder Guide

This guide is the technical entry point for builders, contributors, and experimenters working on the Talking Remote Controller project.
If you are mainly interested in operating the finished controller, start with the [User Guide](user-guide.md) instead.

---

## Purpose

The Talking Remote Controller is an accessibility-oriented project for amateur radio transceivers.
Its goal is to provide a voice-first remote control platform that can be built, adapted, tested, and donated within the amateur radio community.

Version **3.5.1** is the current modular architecture.
It moves beyond the older single-sketch proof of concept toward a profile-driven platform with SD card based radio definitions.

---

## Builder Path Overview

Use this repository path if you need to:

- build the ESP32-S3 based controller hardware
- compile and load the firmware
- work on radio-specific profiles or protocol support
- improve technical documentation
- validate behavior on real radios

Primary technical references:

- [QUICKSTART.md](QUICKSTART.md)
- [docs/hardware_wiring.md](docs/hardware_wiring.md)
- [docs/radio-support-matrix.md](docs/radio-support-matrix.md)
- [docs/radios/yaesu-ft8x7-status.md](docs/radios/yaesu-ft8x7-status.md)
- [docs/radios/yaesu-ftdx10-test-plan.md](docs/radios/yaesu-ftdx10-test-plan.md)

---

## Hardware Requirements

- ESP32-S3 development board
- 4x4 matrix keypad
- Audio output using an I2S DAC or external amplifier
- MicroSD card support for profile files
- One or more radio interfaces, depending on the active profile:
  - ICOM CI-V
  - RS-232
  - TTL CAT
  - ASCII style serial protocols

The exact wiring and tested hardware notes continue to evolve.
See [docs/hardware_wiring.md](docs/hardware_wiring.md) for the current notes.

---

## Firmware Structure

The firmware is no longer a single self-contained sketch only.
The main file `firmware/TalkingRemoteControllerLX1WJ_V3_5_1.ino` now ties together a modular codebase for:

- protocol handling
- user interface logic
- profile loading
- runtime state management
- spoken feedback support

Speech output is based on pre-recorded voice tokens stored in flash memory via `voice_data.h`.
This keeps spoken interaction deterministic and independent of online text-to-speech systems.

---

## SD Card Profile System

Version 3.5.1 introduces profile-driven configuration from the SD card.

Each radio profile can define:

- protocol family
- supported commands
- query and control mappings
- spoken labels
- slot assignments

Current profile families include radios such as:

- ICOM IC-7300
- ICOM IC-706
- Xiegu G106
- Kenwood TS-480
- Yaesu FT-817 / FT-857 / FT-897 / FTDX series
- Elecraft KX2

Profile coverage is still evolving and not every profile exposes the same command set yet.

---

## Current Project Direction

The current modular branch focuses on:

- expanding reliable command coverage
- improving radio profile quality
- documenting real hardware behavior more clearly
- keeping end-user documentation separate from engineering detail
- preserving the blind-friendly voice and keypad operating model

---

## Project Collaboration

This project benefits from practical support by several radio amateurs in concept work, implementation, testing, and documentation.

- **Richard DO9RE**: concept support and test pilot
- **Stefan DK7STJ**: support for KX2 and Elecraft topics
- **Tom OK1ICQ** and **Jan OK1TE**: support for the Yaesu FT8x7 family
- **Damian SP9QLO**: support for Yaesu FTDX10 work
- ...

---

## Disclaimer

This project is provided **for experimental and educational use only**, **AS IS**, without any warranty.
You are responsible for safe wiring, safe RF practices, and compliance with local regulations.
