# ICOM IC-7300

## Overview

This page documents the current keypad mapping and spoken-control layout for the
ICOM IC-7300 profile in the Talking Remote Controller LX1WJ project.

It is a radio-specific reference page and is intentionally separated from the
general user guide so each supported radio can have its own command description.

---

## Bank 1 - Status

| Command Family | Commands | Bank / Keys |
|---|---|---|
| Status | `FREQ?` | Bank 1: `0` short |
| Status | `RXTX?` | Bank 1: `1` short |
| Status | `TXFREQ?` | Bank 1: `2` short |
| Status | `LOCK?`, `LOCK ON/OFF` | Bank 1: `3` short / `3` long |
| Status | `PO?` | Bank 1: `4` short |
| Status | `SM?` | Bank 1: `7` short |
| Status | `SWR?` | Bank 1: `8` short |
| Status | `MODE?`, `MODE <n>` | Bank 1: `9` short / `9` long, then digit, then `Enter` |

---

## Bank 2 - Receive / DSP

| Command Family | Commands | Bank / Keys |
|---|---|---|
| Receive / DSP | `NR?`, `NR ON/OFF` | Bank 2: `1` short / `1` long |
| Receive / DSP | `NB?`, `NB ON/OFF` | Bank 2: `2` short / `2` long |
| Receive / DSP | `NOTCH?`, `NOTCH ON/OFF`, `NOTCH NAR/MID/WIDE` | Bank 2: `3` short / `3` long |
| Receive / DSP | `NRLEVEL?`, `NRLEVEL +/-` | Bank 2: `4` short / `4` long / `4` double click |
| Receive / DSP | `NBLEVEL?`, `NBLEVEL +/-` | Bank 2: `5` short / `5` long / `5` double click |
| Receive / DSP | `PBT1?`, `PBT1 +/-` | Bank 2: `6` short / `6` long / `6` double click |
| Receive / DSP | `PBT2?`, `PBT2 +/-` | Bank 2: `7` short / `7` long / `7` double click |
| Receive / DSP | `FILSHAPE?`, `FILSHAPE SHARP/SOFT` | Bank 2: `8` short / `8` long |
| Receive / DSP | `FILWIDTH?`, `FILWIDTH` forward/back | Bank 2: `9` short / `9` long / `9` double click |

---

## Bank 3 - VFO / Split / Bandstack

| Command Family | Commands | Bank / Keys |
|---|---|---|
| VFO / Split / Bandstack | `SPLIT?`, `SPLIT ON/OFF` | Bank 3: `0` short / `0` long |
| VFO / Split / Bandstack | `VFO A`, `VFOA?`, `VFOA <freq>` | Bank 3: `1` short / `1` long / `1` double click |
| VFO / Split / Bandstack | `VFO B`, `VFOB?`, `VFOB <freq>` | Bank 3: `2` short / `2` long / `2` double click |
| VFO / Split / Bandstack | `VFOA MODE?`, `VFOA MODE <n>` | Bank 3: `4` short / `4` long |
| VFO / Split / Bandstack | `VFOB MODE?`, `VFOB MODE <n>` | Bank 3: `5` short / `5` long |
| VFO / Split / Bandstack | `BSTACK? 1`, `BSTACK 1` | Bank 3: `7` short / `7` long |
| VFO / Split / Bandstack | `BSTACK? 2`, `BSTACK 2` | Bank 3: `8` short / `8` long |
| VFO / Split / Bandstack | `BSTACK? 3`, `BSTACK 3` | Bank 3: `9` short / `9` long |

---

## Bank 4 - TX / Monitor / Antenna

| Command Family | Commands | Bank / Keys |
|---|---|---|
| TX / Monitor / Antenna | `TUNER?`, `TUNER ON/OFF`, `TUNE` | Bank 4: `0` short / `0` long / `0` double click |
| TX / Monitor / Antenna | `MONITOR?`, `MONITOR ON/OFF` | Bank 4: `1` short / `1` long |
| TX / Monitor / Antenna | `MONLEVEL?`, `MONLEVEL +/-` | Bank 4: `2` short / `2` long / `2` double click |
| TX / Monitor / Antenna | `TRANSCEIVE?`, `TRANSCEIVE ON/OFF` | Bank 4: `3` short / `3` long |

---

## Bank 5 - Fine Tuning / RIT

| Command Family | Commands | Bank / Keys |
|---|---|---|
| Fine Tuning / RIT | `RIT?`, `RIT ON/OFF`, `RIT 0` | Bank 5: `0` short / `0` long / `0` double click |
| Fine Tuning / RIT | `RIT -10`, `RIT -100` | Bank 5: `1` short / `1` long |
| Fine Tuning / RIT | `RIT +10`, `RIT +100` | Bank 5: `2` short / `2` long |
| Fine Tuning / RIT | `RIT 0`, `RIT OFF` | Bank 5: `3` short / `3` long |
| Fine Tuning / RIT | `RIT -1`, `RIT -500` | Bank 5: `4` short / `4` long |
| Fine Tuning / RIT | `RIT +1`, `RIT +500` | Bank 5: `5` short / `5` long |

---

## Bank 9 - Profile / System

| Command Family | Commands | Bank / Keys |
|---|---|---|
| Profile / System | Speak current profile | Bank 9: `A` short |
| Profile / System | Start profile selection | Bank 9: `A` long, then `1..9`, then `Enter` |
| Profile / System | `PROFILE NEXT` | Bank 9: `1` short |
| Profile / System | `PROFILE PREV` | Bank 9: `2` short |
| Profile / System | `TUNINGSPEECH?`, `TUNINGSPEECH ON/OFF` | Bank 9: `4` short / `4` long |
| Profile / System | `VOLUME?` | Bank 9: `5` short |
| Profile / System | `VOLUME 1/2/3` | Bank 9: `7` short, `8` short, `9` short, then `Enter` |

---

## Notes

- `Enter` refers to the keypad confirmation key `D`.
- `double click` refers to a profile-specific double-press action where supported.
- This page documents the intended IC-7300 layout for the expanded command set.
