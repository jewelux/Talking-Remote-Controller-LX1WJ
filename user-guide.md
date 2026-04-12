# Talking Remote Controller LX1WJ - User Guide (English)

**Firmware:** `firmware/TalkingRemoteControllerLX1WJ_V3_5_1.ino`  
**Board:** ESP32-S3 DevKitC-1 (N16R8)

---

## Who This Guide Is For

This guide is for operators and helpers who want to use the controller in practice.
It focuses on spoken behavior and keypad use, not on firmware structure or hardware development.

If you want to build or adapt the project, use the [Builder Guide](builder-guide.md).

---

## Keypad Layout (4x4 Matrix)

Assumed labeling:

   1  2  3  A

   4  5  6  B

   7  8  9  C

   *  0  #  D

---

## Global Operating Rules

- A new key press interrupts ongoing speech immediately.
- `D` is the confirmation key for staged actions.
- `#` cancels active entry or staging states and speaks `OK`.
- `*` short press speaks the current bank number.
- `*` long press starts bank selection.
- Exact command behavior depends on the active radio profile.

### Bank Selection

1. Hold `*`
2. Enter the target bank number
3. Press `D` to confirm

Example:
Hold `*`, press `2`, press `D` -> switch to Bank 2.

---

## Global Actions Typically Used Across Profiles

### `0` short

- Query the current frequency from the radio.
- The controller speaks `frequency` and then the value when available.

### `0` long

- Start manual frequency entry.
- Enter digits in **kHz**.
- Press `D` to send the new frequency.
- The controller then speaks the resulting frequency.

Example:
Hold `0`, enter `14070`, press `D` -> sets `14.070 MHz` and speaks the value.

### `7` short

- Query S-meter when the active profile supports it.
- The controller speaks the S-meter result when supported.

### `8` short

- Query SWR when the active profile supports it.
- The controller speaks the SWR result when supported.

### `9` short

- Query operating mode.
- The controller speaks the detected mode.

### `9` long

- Start mode selection.
- Press a digit from `1` to `9` to stage a mode.
- Press `D` to apply the staged mode.
- The controller speaks the selected mode.

Current mode digit mapping:

- `1` = LSB
- `2` = USB
- `3` = CW
- `4` = AM
- `5` = FM
- `6` = DIGI
- `7` = RTTY
- `8` = CWR
- `9` = RTTYR

Example:
Hold `9`, press `2`, press `D` -> set mode to `USB`.

---

## Profile-Dependent Functions

Many keys outside the core frequency and mode flow are radio-dependent.
Some functions are only available if the selected profile reports support for
them or if the protocol family implements them.

### `1` short

- Query noise reduction state when supported.
- The controller speaks `noise reduction on` or `noise reduction off` when supported.

### `1` long

- Toggle noise reduction when supported.

### `2` short

- Query noise blanker state when supported.
- The controller speaks `noise blanker on` or `noise blanker off` when supported.

### `2` long

- Toggle noise blanker when supported.

### `3` short

- Query notch filter state when supported.
- The controller speaks the current notch status.

### `3` long

- Toggle notch behavior when supported.

---

## Cancel and Confirmation Behavior

### `D`

- Confirms bank selection
- Confirms profile selection
- Confirms frequency entry
- Confirms staged mode change

### `#`

- Cancels active frequency entry
- Cancels staged mode changes
- Cancels profile selection
- Cancels bank selection
- Clears staged commands and speaks `OK`

---

## Notes

- Frequency entry currently uses kHz digits, not direct MHz text entry.
- Exact support depends on the selected radio profile and implemented command set.
- The support status per radio is tracked in `docs/radio-support-matrix.md`.
- Radio-specific command pages should be documented separately from this general
  guide.
- The ICOM IC-7300 command reference is documented in
  `docs/radios/icom-ic-7300.md`.
- The Yaesu FT-817 command reference is documented in
  `docs/radios/yaesu-ft-817.md`.
- The Yaesu FT-857 command reference is documented in
  `docs/radios/yaesu-ft-857.md`.
