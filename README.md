# Talking Remote Controller LX1WJ for Ham Radio Transceivers

The **Talking Remote Controller** is an accessibility-oriented remote control for amateur radio transceivers.
The project may also be referred to as **HamTRC-LX1WJ** as a shorter project name.
All interaction and radio status feedback is provided through spoken audio output, enabling operation without visual reference.

Version **3.5** is the current modular architecture.
It extends the earlier single-sketch concept into a profile-driven firmware platform with SD card based radio definitions while preserving the voice-guided operating concept.

---

## Start Here

This repository serves two main audiences.
Please choose the path that best matches what you need:

- **I want to use the controller** -> see the [User Guide](user-guide.md)
- **I want to build, adapt, or document the project** -> see the [Builder Guide](builder-guide.md)

Useful companion documents:

- [QUICKSTART.md](QUICKSTART.md) for a short technical setup overview
- [docs/radio-support-matrix.md](docs/radio-support-matrix.md) for the current per-radio support state
- Radio-specific command references:
  [ICOM IC-7300](docs/radios/icom-ic-7300.md),
  [Yaesu FT-817](docs/radios/yaesu-ft-817.md),
  [Yaesu FT-857](docs/radios/yaesu-ft-857.md)
- [Yaesu FTDX10 blind test list](docs/radios/yaesu-ftdx10-blind-test-list.txt)

---

## What This Project Is

The Talking Remote Controller is a voice-first keypad controller for amateur radio operation.
It is intended to help blind or visually impaired operators access radio functions through spoken feedback and predictable keypad control.
The project continues the tradition of earlier accessible amateur radio interfaces such as **[Hampod](http://hampod.com)** and Digimatel 2000.

Core characteristics:

- ESP32-S3 based controller
- Spoken feedback for radio status and user actions
- Deterministic keypad interaction using short and long presses
- Immediate speech interruption on new user input
- Numeric entry with spoken confirmation
- Non-volatile storage of user settings
- Modular radio protocol layer
- SD card based profile and slot configuration
- Flash-resident spoken token set via `voice_data.h`

Typical spoken information currently includes:

- Frequency
- Operating mode
- S-meter
- SWR
- Output power
- Active bank and radio profile
- Profile-dependent control and query functions

Additional spoken information will continue to be added as the project evolves.

---

## Project Philosophy And Status

**This project is not for commercial sale.**
Its purpose is to support the amateur radio community in building, adapting, and donating accessible controllers under an open-source model.

- **Current stage**: proof-of-concept and field expansion across multiple radio families
- **Goal**: provide a modular, adaptable hardware and software platform that others can build, modify, test, and share
- **Direction**: continue expanding profiles, voice tokens, hardware notes, and blind-friendly documentation

---

## Documentation Paths

### End User Documentation

The end-user path should stay as plain and practical as possible.
Use these pages if the main question is how to operate the controller:

- [user-guide.md](user-guide.md)
- Radio-specific user behavior pages in [docs/radios](docs/radios)

### Builder And Technical Documentation

The builder path covers hardware, firmware structure, profiles, and implementation context:

- [builder-guide.md](builder-guide.md)
- [QUICKSTART.md](QUICKSTART.md)
- [docs/hardware_wiring.md](docs/hardware_wiring.md)
- [docs/circuit-diagram-under-construction.pdf](docs/circuit-diagram-under-construction.pdf)
- [docs/radio-support-matrix.md](docs/radio-support-matrix.md)

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

The firmware is no longer a single self-contained sketch only.
The main `.ino` now ties together a modular codebase for protocol handling, user interface logic, profile loading, and runtime state.

---

## Project Collaboration

This project benefits from practical support by several radio amateurs in concept work, implementation, testing, and documentation.

- **Richard DO9RE**: concept support and test pilot
- **Stefan DK7STJ**: support for KX2 and Elecraft topics
- **Tom OK1ICQ** and **Jan OK1TE**: support for the Yaesu FT8x7 family
- **Damian SP9QLO**: support for Yaesu FTDX10 work
- **Gena M0EBP**: support for blind-friendly documentation structure and wording

---

## Disclaimer And Safety Notice

This project is provided **for experimental and educational use only**, **AS IS**, without any warranty.

- The device is **not a certified instrument**
- You are responsible for correct wiring, safe RF practices, and compliance with local regulations

**Use at your own risk.**

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

Future development focuses on:

- extended command coverage
- additional radio profiles
- expanded voice token sets
- cleaner documentation for builders and operators
- continued refinement of the Yaesu FT8x7 family
- further bring-up and validation work for the Yaesu FTDX10/101 ASCII CAT family

---

## License

This project uses the same license model as the **Talking SWR Meter** project by the same author.
See the `LICENSE` file for code and `LICENSE-docs` for documentation material.
