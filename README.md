# Talking Remote Controller LX1WJ for Ham Radio Transceivers

## Overview

The **Talking Remote Controller** is an accessibility-oriented remote control for amateur radio transceivers.
The project may also be referred to as **HamTRC-LX1WJ** as a shorter project name.
All interaction and radio status feedback is provided through spoken audio output, enabling operation without visual reference.
It is inspired by earlier projects like Hampod and Digimatel 2000.

Version **3.5** is the current modular architecture.
It extends the earlier single-sketch concept into a profile-driven firmware platform with SD card based radio definitions while preserving the voice-guided operating concept.

---

## Project Philosophy & Status
**This project is not for commercial sale.**
Its purpose is to **empower the amateur radio community** to **build and donate** accessible controllers for blind or visually impaired operators under the **open-source model**:

- **Current Stage**: proof-of-concept and field expansion across multiple radio families.
- **Goal**: provide a **modular, adaptable hardware/software platform** that anyone can build, modify, and share.
- **Future**: the design will continue to evolve with new profiles, voice tokens, and hardware refinements.

---

## Disclaimer / Safety Notice
This project is provided **for experimental and educational use only**, **AS IS**, without any warranty.

- The device is **not a certified instrument**.
- You are responsible for correct wiring, safe RF practices, and compliance with local regulations.

**Use at your own risk.**

---

## Related and Historical Projects
This project continues the tradition of accessible amateur radio interfaces with audio feedback, following in the footsteps of pioneering systems like **[Hampod](http://hampod.com)** and Digimatel 2000. These early projects enabled blind and visually impaired operators to control transceivers via keyboard input with spoken status updates. The *Talking Remote Controller* builds on these concepts with updated hardware (ESP32-S3), modular radio profiles, and a reusable voice architecture.

---

## Core Characteristics

- ESP32-S3 based controller
- Spoken feedback for radio status and user actions
- Deterministic keypad interaction using short and long presses
- Immediate speech interruption on new user input
- Numeric entry with spoken confirmation
- Non-volatile storage of user settings
- Modular radio protocol layer
- SD card based profile and slot configuration
- Flash-resident spoken token set via `voice_data.h`

---

## Supported Information Output

- Frequency
- Operating mode
- S-meter
- SWR
- Output power
- Active bank and radio profile
- Profile-dependent control and query functions

Additional spoken information will be added as the project evolves.

---

## Hardware Requirements

- ESP32-S3 development board
- 4x4 matrix keypad
- Audio output (I2S DAC or external amplifier)
- MicroSD card support for profile files
- One or more radio interfaces, depending on the active profile:
  - ICOM CI-V
  - RS-232
  - TTL CAT
  - ASCII style serial protocols

The exact wiring and tested hardware notes continue to evolve in the documentation.

---

## Repository Layout

```text
firmware/
  TalkingRemoteControllerLX1WJ_V3_5.ino
  *.cpp / *.h modular source files
  voice_data.h
  SDCard/*.ini radio profiles and slots

docs/
  supporting hardware and project documentation
```

The firmware is no longer a single self-contained sketch only. The main `.ino` now ties together a modular codebase for protocol handling, user interface logic, profile loading, and runtime state.
Internal working notes are intentionally kept out of the public repository so the published project tree stays focused and readable.

---

## Voice Architecture

Speech output is based on pre-recorded voice tokens stored in flash memory.

Design principles:

- Reusable single-word or short-phrase tokens
- Numeric values composed from digit tokens
- Protocol-independent command vocabulary
- Radio-specific name tokens
- Immediate interruption when new user input arrives

This keeps spoken interaction fast and predictable while avoiding dependence on online text-to-speech systems.

---

## SD Card Profile System

Version 3.5 introduces profile-driven configuration from the SD card.

Each radio profile can define:

- Protocol family
- Supported commands
- Query and control mappings
- Spoken labels
- Slot assignments

Current profile files in `firmware/SDCard` include definitions for radios such as:

- ICOM IC-7300
- ICOM IC-706
- Xiegu G106
- Kenwood TS-480
- Yaesu FT-817 / FT-857 / FT-897 / FTDX series
- Elecraft KX2

Profile coverage is still evolving and not every profile necessarily exposes the same command set yet.

---

## Keypad Interaction Model

- **Short press**: query or immediate action
- **Long press**: mode change, configuration, or numeric input
- **ENTER**: confirms numeric input or staged actions
- **Cancel**: aborts modal flows without requiring visual feedback

This interaction model remains consistent as the project grows.

Please visit [User Guide ->](user-guide.md)
for keypad behavior and spoken feedback examples.
The current per-radio support state is tracked in [docs/radio-support-matrix.md](docs/radio-support-matrix.md).
Radio-specific command references currently include
[ICOM IC-7300](docs/radios/icom-ic-7300.md),
[Yaesu FT-817](docs/radios/yaesu-ft-817.md), and
[Yaesu FT-857](docs/radios/yaesu-ft-857.md).



A first FTDX10 bring-up document is now also available:

- [Yaesu FTDX10 first test plan](docs/radios/yaesu-ftdx10-test-plan.md)

---

## Project Scope and Evolution

Versions prior to the current modular branch were integration and design iterations.
The newer architecture separates protocol logic, runtime state, UI behavior, and profile data so the project can expand without rewriting the whole sketch for each radio.

---

## What Changed From Earlier Versions

The earlier public repository state was centered around a much simpler single-sketch firmware approach.
That version already demonstrated the core accessibility concept, but it was still tightly coupled to a smaller set of radios and did not yet reflect the current modular direction of the project.

The current `V3.5` branch introduces several major changes:

- the firmware moved from one primary sketch to a modular multi-file structure
- radio handling is now separated into protocol, runtime, UI, and profile-related components
- SD card based profile files now describe supported radios and slot assignments
- the repository now represents a broader platform for further radio expansion rather than a narrow proof-of-concept build
- the original voice-token approach with `voice_data.h` remains in place, but is now used inside a more extensible architecture

This means the repository history now contains a visible transition from the earlier compact prototype toward a more maintainable and expandable controller platform.

Future development focuses on:

- Extended command coverage
- Additional radio profiles
- Expanded voice token sets
- Cleaner documentation for builders and operators
- Continued refinement of the Yaesu FT8x7 family using the same keypad structure principles already established for the IC-7300`r`n- First documented bring-up of the Yaesu FTDX10/101 ASCII CAT family

---

## License

This project uses the same license model as the **Talking SWR Meter** project by the same author.
See the `LICENSE` file for code and `LICENSE-docs` for documentation material.

