# Talking Remote Controller (V1.0)

## Overview

The **Talking Remote Controller** is an accessibility-oriented remote control for amateur radio transceivers.  
All interaction and radio status feedback is provided through spoken audio output, enabling operation without visual reference.

Version **1.0** defines a stable control and speech architecture.  
The project remains actively developed, extended, and documented.

---

## ⚠️ Disclaimer / Safety Notice
This project is provided **for experimental and educational use only**, **AS IS**, without any warranty.

- The device is **not a certified measuring instrument**.
- Measurements may be inaccurate and can lead to **equipment damage** or **unsafe RF operation** if relied upon.
- You are responsible for correct wiring, safe RF practices, and compliance with local regulations.
- Always verify results with trusted instruments (e.g. wattmeter/dummy load) before relying on them.

---

## Core Characteristics

- ESP32-S3 based controller
- Spoken feedback for radio status and user actions
- Deterministic keypad interaction using short and long presses
- Immediate speech interruption on new user input
- Numeric entry with spoken confirmation
- Non-volatile storage of user settings
- Modular radio profile architecture

---

## Supported Information Output

- Frequency
- Operating mode
- S-meter
- SWR
- Output power
- Active bank and radio profile

Additional spoken information will be added as the project evolves.

---

## Hardware Requirements

- ESP32-S3 development board
- 4×4 matrix keypad
- Audio output (I2S DAC or external amplifier)
- One or more radio interfaces:
  - ICOM CI-V
  - RS-232
  - TTL-CAT (profile dependent)

Wiring and tested hardware configurations are documented in the futur `hardware` directory.

---

## Firmware Layout

firmware/

         ├─ ESP32S3_TalkingRemote_V1.0.ino
         
         └─ voice_data.h

The firmware is self-contained and uses the Arduino ESP32 core.

---

## Voice Architecture

Speech output is based on pre-recorded voice tokens stored in flash memory.

Design principles:

- Reusable single-word or short-phrase tokens
- Numeric values composed from digit tokens
- Protocol-independent command vocabulary
- Radio-specific name tokens

Original WAV sources and documentation are located in the `voice` directory.

---

## Keypad Interaction Model

- **Short press**: query or immediate action
- **Long press**: mode change, configuration, or numeric input
- **ENTER**: confirms numeric input

This interaction model is consistent across all banks and radio profiles.

---

## Radio Profiles

Each transceiver is defined by a profile specifying:

- Supported commands
- Protocol mapping
- Interface type
- Spoken labels

Profiles are documented in the futur `profiles` directory.

### Currently implemented

- **ICOM IC-7300** (CI-V)
- **ICOM IC-706MKIIG** (CI-V)
- **ICOM IC-706MKIIG** (ct-17, RS232-Max3232 module)
- **Xiegu G106**

Additional radio profiles will be added as the project evolves.

---

## Project Scope and Evolution

Versions prior to **1.0** were integration and design iterations.  
Version **1.0** establishes a stable foundation for further expansion.

Future development focuses on:

- Extended command coverage
- Additional radio profiles
- Expanded voice token sets

---

## License

This project uses the same license model as the **Talking SWR Meter** project by the same author.  
See the `LICENSE` file for details.
