# Talking Remote Controller LX1WJ - QuickStart (V3.5)

## Purpose

The ESP32 Talking Remote Controller provides spoken feedback for controlling and monitoring amateur radio transceivers.
All interaction is performed via a 4x4 keypad and audio output, enabling operation without visual reference.
The project may also be referred to as **HamTRC-LX1WJ** as a shorter project name.

Version 3.5 uses a modular firmware architecture plus SD card based radio profiles.

---

## What Changed From The Older Single-Sketch Version

- The project is now split into multiple source files instead of one large sketch.
- Radio definitions are loaded from profile files on the SD card.
- The spoken token data still remains compiled into flash through `voice_data.h`.
- The operating concept stays voice-first and keypad-driven.

---

## Basic Setup

1. Open `firmware/TalkingRemoteControllerLX1WJ_V3_5.ino` in Arduino IDE.
2. Compile for your ESP32-S3 target.
3. Keep `voice_data.h` in the firmware folder so the speech data is compiled into flash.
4. Copy the contents of `firmware/SDCard` to the SD card used by the controller.
5. Insert the SD card before normal operation so profile and slot data can be loaded.

---

## Basic Operating Concept

- Functions are organized in banks.
- Each key supports short press and long press behavior.
- Spoken output confirms actions, states, and values.
- Any new key press immediately interrupts the current spoken message.
- Exact behavior depends on the loaded radio profile.

---

## Keypad Model

The overall keypad philosophy remains the same:

- **Short press** -> immediate action or query
- **Long press** -> mode change, configuration, or numeric input
- **D / ENTER** -> confirm staged actions or frequency entry
- **# / CANCEL** -> leave modal flows safely
- *** / STAR** -> bank-related navigation functions

Because Version 3.5 is profile-driven, the exact mapping can differ by radio family and firmware state.

---

## SD Card Content

The `firmware/SDCard` folder currently contains:

- `slots.ini` for slot assignment
- profile `.ini` files for supported radios

Examples include ICOM, Yaesu, Kenwood, Xiegu, and Elecraft related profiles.

---

## Notes

This repository reflects the newer modular branch of the project.
Documentation will continue to be expanded while keeping the original accessibility focus, licensing model, and safety disclaimer intact.
For the current per-radio support state, see `docs/radio-support-matrix.md`.
