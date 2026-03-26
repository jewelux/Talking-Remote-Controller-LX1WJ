# Talking Remote Controller LX1WJ - User Guide (English)

**Firmware:** `firmware/TalkingRemoteControllerLX1WJ_V3_5.ino`  
**Board:** ESP32-S3 DevKitC-1 (N16R8)

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

### Bank Selection

1. Hold `*`
2. Enter the target bank number
3. Press `D` to confirm

Example:
Hold `*`, press `2`, press `D` -> switch to Bank 2.

---

## Bank 1 - Live Radio Queries and Direct Actions

### `1` short

- Toggle automatic tuning-frequency announcements on or off.
- The controller speaks `frequency on` or `frequency off`.

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

### `4` short

- Query output power.
- The controller speaks the power value when supported by the active profile.

### `7` short

- Query S-meter.
- The controller speaks the S-meter result when supported.

### `8` short

- Query SWR.
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

### `A` short

- Decrease speech volume by one step.
- The controller speaks the new volume level.

### `A` long

- Increase speech volume by one step.
- The controller speaks the new volume level.

---

## Bank 2 - Feature Queries and Profile-Specific Actions

Bank 2 depends more strongly on the active radio profile.
Some functions are only available if the selected profile reports support for them.

### `1` short

- Query noise reduction state.
- The controller speaks `noise reduction on` or `noise reduction off` when supported.

### `1` long

- Toggle noise reduction.
- For radios such as the TS-480, the current implementation can cycle between multiple NR levels.

### `2` short

- Query noise blanker state.
- The controller speaks `noise blanker on` or `noise blanker off` when supported.

### `2` long

- Toggle noise blanker.

### `3` short

- Query notch filter state.
- The controller speaks the current notch status.

### `3` long

- Toggle notch behavior.
- On CI-V based radios, repeated long presses can cycle through notch states such as narrow, mid, wide, and off.

### Yaesu FT-8x7 specific actions

The following keys are active only for supported Yaesu FT-8x7 style profiles:

#### `4` short

- Query Yaesu status.

#### `4` long

- Toggle VFO.

#### `5` short

- Select VFO A.

#### `5` long

- Select VFO B.

#### `6` short

- Turn clarifier on.

#### `6` long

- Turn clarifier off.

#### `7` short

- Turn split on.

#### `7` long

- Turn split off.

#### `8` short

- Turn PTT on.

#### `8` long

- Turn PTT off.

#### `9` short

- Query the full Yaesu status block.

---

## Bank 3 - Profiles, Tuning Speech, and Staged Volume

### `A` short

- Speak the currently active radio profile.

### `A` long

- Start profile selection.
- Enter the profile number.
- Press `D` to confirm.
- The controller loads the profile and speaks its name.

Example:
Hold `A`, press `3`, press `D` -> load profile slot 3 and speak it.

### `B` short

- Toggle automatic tuning-frequency announcements on or off.
- The controller speaks the new state.

### `B` long

- Speak the current tuning-frequency announcement state without changing it.

### Staged volume keys

- `0` -> stage volume level 0
- `7` -> stage volume level 1
- `8` -> stage volume level 2
- `9` -> stage volume level 3

After selecting one of these levels, press `D` to apply it.
The controller speaks the selected volume level.

Example:
Press `8`, then `D` in Bank 3 -> set speech volume to level 2.

---

## Cancel and Confirmation Behavior

### `D`

- Confirms bank selection
- Confirms profile selection
- Confirms frequency entry
- Confirms staged mode change
- Confirms staged volume selection

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
