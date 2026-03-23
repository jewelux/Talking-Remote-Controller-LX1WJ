# Talking Remote Controller LX1WJ - User Guide (English)

**Firmware:** `firmware/TalkingRemoteControllerLX1WJ_V3_5.ino`
**Board:** ESP32-S3 DevKitC-1 (N16R8)

---

## Keypad Layout (4x4 Matrix)

Assumed labeling:

   1  2  3  A

   4  5  6  B

   7  8  9  C

   *  0  #  D   (star, zero, hash, delta)

---

## Global Keys

The exact action set is now profile-dependent, but the operating philosophy remains stable.

### * (STAR)
- Bank-related navigation and spoken bank feedback.

### D (ENTER)
- Applies staged actions.
- Confirms frequency entry and similar modal operations.

### # (CANCEL)
- Cancels active modal flows.
- Clears staged commands where supported.

---

## Banks (Layers)

The controller uses bank-based interaction so multiple functions can be reached from a compact keypad.
The exact assignment may vary by firmware revision and active radio profile.

---

## Typical Functions Available In The Current Project

Depending on the selected profile, the controller can speak or control:

- current frequency
- operating mode
- output power
- S-meter
- SWR
- active radio profile
- tuning announcements
- profile and slot related functions

---

## Frequency Entry

Frequency entry is still based on spoken digit confirmation and an explicit ENTER step.
This keeps operation predictable for non-visual use.

General pattern:

1. Enter frequency input mode.
2. Type digits.
3. Confirm with **D / ENTER**.
4. The new frequency is sent to the radio and spoken back when supported.

---

## Speech Behavior

- Spoken feedback confirms user actions.
- New key presses interrupt current speech immediately.
- Voice output uses pre-recorded tokens stored in flash.

---

## Profile-Driven Operation

Version 3.5 separates firmware logic from radio-specific profile data.
That means the user experience stays familiar while the supported radios can grow without rewriting the full project from scratch.

Profile data is loaded from the SD card.

---

## Notes

This guide will continue to grow with the project.
The repository now represents the newer modular branch rather than the earlier single-sketch-only release.
