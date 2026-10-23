# ICOM IC-7300

## Overview

This page documents the current `V3.5.8` support state for the **ICOM IC-7300**
inside the **Talking Remote Controller LX1WJ** project.

The IC-7300 is currently one of the most complete radio profiles in the
firmware. It supports:

- spoken keypad control
- serial monitor control
- two built-in profiles, for the CI-V jack and for RS-232
- CI-V based status queries and control functions

The implementation is designed around the current modular architecture:

- protocol handling in the firmware core
- radio capability flags in the profile layer
- built-in radio profiles for radio-specific configuration
- bank-based keypad interaction with short, long, and double-click actions

---

## Current Support Level

The following function groups are implemented for the IC-7300:

- status queries
- receive and DSP functions
- VFO and split handling
- tuning and antenna functions
- RIT fine tuning
- monitor and transceive control
- band stacking register access

This profile has been expanded and checked in real-device testing during
development. The support state is no longer just "basic CAT"; it is a broad
spoken operating profile.

---

## Profiles

The IC-7300 has two built-in profiles:

- slot 1: the CI-V jack
- slot 3: RS-232

Both declare the extended capability set for the radio, including features
such as:

- `RXTX`
- `TXFREQ`
- `NRLEVEL`
- `NBLEVEL`
- `PBT1`
- `PBT2`
- `FILSHAPE`
- `FILWIDTH`
- `LOCK`
- `MONITOR`
- `MONLEVEL`
- `TRANSCEIVE`
- `TUNER`
- `VFO`
- `VFO MODE`
- `SPLIT`
- `RIT`
- `BSTACK`

New features and new radios come with a new firmware.

---

## Serial Monitor Command Families

### Status

- `FREQ?`
- `MODE?`
- `SM?`
- `SWR?`
- `PO?`
- `RXTX?`
- `TXFREQ?`
- `ROUND`, `ROUND <Hz>` (round the frequency to a step, default 500 Hz)

### Receive / DSP

- `NR?`, `NR ON`, `NR OFF`
- `NRLEVEL?`, `NRLEVEL <0..100>`, `NRLEVEL STEP <+-n>`
- `NB?`, `NB ON`, `NB OFF`
- `NBLEVEL?`, `NBLEVEL <0..100>`, `NBLEVEL STEP <+-n>`
- `NOTCH?`, `NOTCH ON`, `NOTCH OFF`
- `NOTCH NAR`, `NOTCH MID`, `NOTCH WIDE`
- `PBT1?`, `PBT1 <-128..127>`, `PBT1 CENTER`, `PBT1 STEP <+-n>`
- `PBT2?`, `PBT2 <-128..127>`, `PBT2 CENTER`, `PBT2 STEP <+-n>`
- `FILSHAPE?`, `FILSHAPE SHARP`, `FILSHAPE SOFT`, `FILSHAPE TOGGLE`
- `FILWIDTH?`, `FILWIDTH <1..3>`, `FILWIDTH NEXT`, `FILWIDTH PREV`

### Transmit

- `PROC?`, `PROC ON | OFF | TOGGLE` (speech compressor, COMP)
- `PROCLEVEL?`, `PROCLEVEL <0..100>`, `PROCLEVEL STEP <+-n>` (its level, %)
- `MICGAIN?`, `MICGAIN <0..100>`, `MICGAIN STEP <+-n>` (mic gain, %)
- `VOX?`, `VOX ON | OFF | TOGGLE`
- `VOXGAIN?`, `VOXGAIN <0..100>`, `VOXGAIN STEP <+-n>` (its gain, %)
- `VOXDELAY?`, `VOXDELAY <0..2000>`, `VOXDELAY STEP <+-n>` (its delay in ms, set to the nearest
  100 ms; a step is 100 ms)

Each set is read back. The sets are not yet tested on a radio.

The IC-705 and IC-7760 profiles answer these too. The VOX delay sits at a different menu
address on each model: IC-7300 `1A 05 0191`, IC-705 `0359`, as each CI-V guide and Hamlib give
them. The IC-7760 has no `VOXDELAY`: Hamlib reads `0182`, which its CI-V guide lists as the
Internet access line, and the guide puts the VOX delay at `0365`.

### VFO / Split / Bandstack

- `SPLIT?`, `SPLIT ON`, `SPLIT OFF`
- `VFO A`, `VFO B`
- `VFOA?`, `VFOB?`
- `VFOA <kHz>`, `VFOB <kHz>`
- `VFOA MODE?`, `VFOB MODE?`
- `VFOA MODE <n>`, `VFOB MODE <n>`
- `BSTACK? 1`, `BSTACK? 2`, `BSTACK? 3`
- `BSTACK 1`, `BSTACK 2`, `BSTACK 3`

### TX / Monitor / Antenna

- `TUNER?`, `TUNER ON`, `TUNER OFF`
- `TUNE`
- `MONITOR?`, `MONITOR ON`, `MONITOR OFF`, `MONITOR TOGGLE`
- `MONLEVEL?`, `MONLEVEL <0..100>`, `MONLEVEL STEP <+-n>`
- `TRANSCEIVE?`, `TRANSCEIVE ON`, `TRANSCEIVE OFF`, `TRANSCEIVE TOGGLE`
- `LOCK?`, `LOCK ON`, `LOCK OFF`

### Fine Tuning

- `RIT?`
- `RIT ON`, `RIT OFF`, `RIT TOGGLE`
- `RIT <Hz>`, `RIT STEP <+-Hz>`

### Connection

- `CIVADDR?`, `CIVADDR <hex>`
- `BAUD?`, `BAUD <rate>` (4800, 9600, 19200, 38400, 57600, 115200)
- `PROFILE RESET`, `PROFILE RESET ALL` (baud and CI-V address back to the defaults)

The IC-7300's CI-V jack itself goes up to 19200 baud; set the same rate in the
radio's CI-V menu.

---

## Keypad Layout

The current keypad concept uses radio-specific banks with:

- short press for query or immediate action
- long press for toggle or staged action
- double click where a third function of the same family is useful

### Bank 1 - Status

| Command family | Commands | Bank / keys |
|---|---|---|
| Status | `FREQ?`, `FREQ <MHz>`, `ROUND 500` | Bank 1: `0` short / `0` long / `0` double click |
| Status | `RXTX?` | Bank 1: `1` short |
| Status | `TXFREQ?` | Bank 1: `2` short |
| Status | `LOCK?`, `LOCK ON/OFF` | Bank 1: `3` short / `3` long |
| Status | `PO?` | Bank 1: `4` short |
| Status | `SM?` | Bank 1: `7` short |
| Status | `SWR?` | Bank 1: `8` short |
| Status | `MODE?`, `MODE <n>` | Bank 1: `9` short / `9` long, then digit, then `Enter` |

### Bank 2 - Receive / DSP

| Command family | Commands | Bank / keys |
|---|---|---|
| Receive / DSP | `NR?`, `NR ON/OFF` | Bank 2: `1` short / `1` long |
| Receive / DSP | `NB?`, `NB ON/OFF` | Bank 2: `2` short / `2` long |
| Receive / DSP | `NOTCH?`, `NOTCH ON/OFF`, `NOTCH NAR/MID/WIDE` | Bank 2: `3` short / `3` long |
| Receive / DSP | `NRLEVEL?`, `NRLEVEL STEP 10/-10` | Bank 2: `1` double click; `1` short then hold for the entry; then `B` / `C` |
| Receive / DSP | `NBLEVEL?`, `NBLEVEL STEP 10/-10` | Bank 2: `2` double click; `2` short then hold for the entry; then `B` / `C` |
| Receive / DSP | `PBT1?`, `PBT1 STEP 10/-10` | Bank 2: `4` short / `4` long for the entry; then `B` / `C`, `A` double click centre |
| Receive / DSP | `PBT2?`, `PBT2 STEP 10/-10` | Bank 2: `6` short / `6` long for the entry; then `B` / `C`, `A` double click centre |
| Receive / DSP | `FILWIDTH?`, `FILWIDTH NEXT/PREV` | Bank 2: `5` short / `5` long; then `B` / `C` |
| Receive / DSP | `FILSHAPE?`, `FILSHAPE TOGGLE` | Bank 2: `5` double click / `5` short then hold |
| Receive / DSP | `AGC?`, `AGC <speed>` | Bank 2: `7` short / `7` long |
| Receive / DSP | `PREAMP?`, `ATT?` | Bank 2: `8` and `9` short / long for the next level |

### Bank 3 - VFO / Split / Bandstack

| Command family | Commands | Bank / keys |
|---|---|---|
| VFO / Split / Bandstack | `SPLIT?`, `SPLIT ON/OFF` | Bank 3: `0` short / `0` long |
| VFO / Split / Bandstack | `VFO A`, `VFOA?`, `VFOA <freq>` | Bank 3: `1` short / `1` long / `1` double click |
| VFO / Split / Bandstack | `VFO B`, `VFOB?`, `VFOB <freq>` | Bank 3: `2` short / `2` long / `2` double click |
| VFO / Split / Bandstack | `VFOA MODE?`, `VFOA MODE <n>` | Bank 3: `4` short / `4` long |
| VFO / Split / Bandstack | `VFOB MODE?`, `VFOB MODE <n>` | Bank 3: `5` short / `5` long |
| VFO / Split / Bandstack | `BSTACK? 1`, `BSTACK 1` | Bank 3: `7` short / `7` long |
| VFO / Split / Bandstack | `BSTACK? 2`, `BSTACK 2` | Bank 3: `8` short / `8` long |
| VFO / Split / Bandstack | `BSTACK? 3`, `BSTACK 3` | Bank 3: `9` short / `9` long |

### Bank 4 - Phone Transmit

| Command family | Commands | Bank / keys |
|---|---|---|
| Tuner | `TUNER?`, `TUNER ON/OFF`, `TUNE` | Bank 4: `0` short / `0` long / `0` double click |
| Speech processor | `PROC?`, `PROC TOGGLE`, `PROCLEVEL?`, `PROCLEVEL <0..100>` | Bank 4: `1` short / `1` long / `1` double click / `1` short, then hold |
| VOX | `VOX?`, `VOX TOGGLE`, `VOXGAIN?`, `VOXGAIN <0..100>` | Bank 4: `2` short / `2` long / `2` double click / `2` short, then hold |
| Monitor | `MONITOR?`, `MONITOR TOGGLE`, `MONLEVEL?`, `MONLEVEL <0..100>` | Bank 4: `3` short / `3` long / `3` double click / `3` short, then hold |
| Mic gain | `MICGAIN?`, `MICGAIN <0..100>` | Bank 4: `7` short / `7` long |
| RF power | `RFPOWER?`, `RFPOWER <W>` | Bank 4: `8` short / `8` long |
| VOX delay | `VOXDELAY?`, `VOXDELAY <ms>` | Bank 4: `9` short / `9` long |

`A`, `B` and `C` act on the level last touched: `B`/`C` step it by 5 (VOX delay 100 ms).

### Bank 5 - Fine Tuning / RIT

| Command family | Commands | Bank / keys |
|---|---|---|
| Fine tuning / RIT | `RIT?`, `RIT TOGGLE`, `RIT 0` | Bank 5: `0` short / `0` long / `0` double click |
| Fine tuning / RIT | `RIT STEP -10`, `RIT STEP -100` | Bank 5: `1` short / `1` long |
| Fine tuning / RIT | `RIT STEP 10`, `RIT STEP 100` | Bank 5: `2` short / `2` long |
| Fine tuning / RIT | `RIT 0`, `RIT OFF` | Bank 5: `3` short / `3` long |
| Fine tuning / RIT | `RIT STEP -1`, `RIT STEP -500` | Bank 5: `4` short / `4` long |
| Fine tuning / RIT | `RIT STEP 1`, `RIT STEP 500` | Bank 5: `5` short / `5` long |

### Bank 8 - Transceive

| Command family | Commands | Bank / keys |
|---|---|---|
| Transceive | `TRANSCEIVE?`, `TRANSCEIVE TOGGLE` | Bank 8: `3` short / `3` long |

### Bank 9 - Profile / System

| Command family | Commands | Bank / keys |
|---|---|---|
| Profile / System | Speak current profile | Bank 9: `A` short |
| Profile / System | Start profile selection | Bank 9: `A` long, then `1..9`, then `Enter` |
| Profile / System | `PROFILE RESET` (baud and CI-V address to the defaults) | Bank 9: `A` double click |
| Profile / System | `PROFILE NEXT` | Bank 9: `B` short |
| Profile / System | `PROFILE PREV` | Bank 9: `C` short |
| Profile / System | `EXPERIMENTAL?`, `EXPERIMENTAL TOGGLE` | Bank 9: `0` short / `0` long |
| Profile / System | `TUNINGSPEECH?`, `TUNINGSPEECH TOGGLE` | Bank 9: `4` short / `4` long |
| Profile / System | `VERBOSE?`, `VERBOSE TOGGLE` | Bank 9: `5` short / `5` long |
| Profile / System | `SPEED?`, next speed (slow, normal, fast) | Bank 9: `6` short / `6` long |
| Profile / System | `VOLUME DOWN / DOWN FAST` | Bank 9: `7` short / `7` long |
| Profile / System | `VOLUME UP / UP FAST` | Bank 9: `8` short / `8` long |
| Profile / System | `VOLUME?` | Bank 9: `9` short |

---

## Verified During Development

The following families were checked during real-device work on the IC-7300:

- `NR`, `NB`, `NOTCH`
- `TUNER`, `TUNE`
- `RXTX`, `TXFREQ`
- `VFO A/B`
- `VFO A/B MODE`
- `SPLIT`
- `RIT`
- `PBT1`, `PBT2`
- `FILWIDTH`
- `MONLEVEL`
- `TRANSCEIVE`
- `BSTACK`

`LOCK`, `FILSHAPE`, the notch width and `MONITOR` were on this list, but they sent `1A 05`
menu items 0022, 0023, 0024 and `1C 02`, which the IC-7300 manual lists as the beep level
limit, the confirmation beep, the band edge beep and the transmit frequency monitor. They
now send `16 50`, `16 56`, `16 57` and `16 45`, as in the manual and in Hamlib, and need
a new test on the radio.

This does not mean every edge case is finished forever, but it does mean the
profile is already well beyond a minimal proof of concept.

---

## Notes

- `Enter` refers to the keypad confirmation key `D`.
- In frequency entry `*` is the decimal point: `14*074`, then `Enter`, tunes to 14.074 MHz.
- `double click` refers to a profile-specific double-press action where
  supported.
- The IC-7300 profile is one of the best references for how the `V3.5.8`
  architecture is intended to work across protocol, runtime, keypad, and
  profile layers.
