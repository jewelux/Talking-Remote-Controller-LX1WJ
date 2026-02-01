# ESP32 Talking Remote Controller – User Guide (V1.0)

## Purpose

The ESP32 Talking Remote Controller provides spoken feedback for controlling and monitoring amateur radio transceivers.
All interaction is performed via a 4×4 keypad and audio output, enabling operation without visual reference.

---

## Basic Operating Concept

- Functions are organized in **banks**
- Each key supports:
  - **Short press** → immediate action or query
  - **Long press** → mode change, configuration, or numeric input
- Spoken output confirms all actions
- Any new key press **immediately interrupts** the current spoken message

---

## Keypad Reference (V1.0)

### Bank 1 – Radio Status and Control

| Key | Short Press | Long Press |
|-----|------------|------------|
| 0   | Speak current frequency | Enter frequency input mode |
| 4   | Speak operating mode | — |
| 7   | Speak S-meter | — |
| 8   | Speak SWR | — |
| 9   | Speak output power | — |
| D   | — | Confirm numeric input |

---

### Bank 3 – System and Profile Functions

| Key | Short Press | Long Press |
|-----|------------|------------|
| A   | Select radio profile | Announce current radio profile |
| B   | Toggle tuning frequency announcements | Announce tuning announcement state |
| *   | Announce current bank | Switch to next bank |

*Exact functions may depend on the active radio profile.*

---

### Global Keys

| Key | Function |
|-----|----------|
| * (long) | Cycle through banks |
| D | ENTER / confirm numeric input |

---

## Frequency Entry

Frequency entry is performed in Bank 1:

1. Activate frequency input mode
2. Enter digits using keys `0–9`
3. Confirm with **ENTER**
4. The new frequency is sent to the radio and spoken once for confirmation

---

## System Behavior

### Automatic Frequency Announcements During Tuning

When enabled, frequency changes received from the radio (e.g. VFO tuning) are spoken automatically.
This function can be toggled and is stored in non-volatile memory.

---

### Speech Interruption

Any new key press immediately stops the current spoken message.
The new command is processed without delay.

---

### Power-Up Behavior

After power-up:
- The last active radio profile is restored
- The active profile is announced
- Stored user preferences are reloaded

---

## Supported Radios (V1.0)

- ICOM IC-7300 (CI-V)
- ICOM IC-706MKIIG (CI-V and RS-232)
- Xiegu G106

---

## Notes

Version 1.0 defines a stable operating model.
Additional commands, voice tokens, and radio profiles will be added in future versions.
