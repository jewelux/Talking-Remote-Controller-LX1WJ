# FT8x7 Keypad Layout

This document describes the current keypad layout for the Yaesu FT-817, FT-857, and FT-897 family.

The intention is:

- keep the overall bank structure consistent across the FT8x7 family
- only expose functions that are documented and practically usable
- move repeater and tone functions into a dedicated Bank 6
- Bank 6 now favors direct manual entry for repeater offset, CTCSS, and DCS, with only a small set of SD-card defaults in `[bank6]`

Important practical note:

- Some FT8x7 CAT write functions are context-sensitive on the radio side. The firmware sends the documented CAT bytes, but the radio may only apply them cleanly when the current band and mode fit the function. In practical repeater use this usually means being on the appropriate VHF/UHF band and already in FM. For example, common repeater shift workflows are much more predictable when the radio is already on 2 m FM.
- For FT8x7 repeater and tone work, treat `2 m` or `70 cm` plus `FM` as the expected operating context. If the radio is on another band or not already in `FM`, `RPT`, `RPTSHIFT`, `CTCSS`, and `DCS` writes may appear inconsistent even though the firmware is sending the documented CAT sequence.
- Recent FT-817 testing also suggests that documented CAT commands can work correctly once the radio is in the right background state, but that state is not yet characterized well enough to call the whole path fully stable. More testing is still needed around those hidden preconditions.

## Bank 1 - Status

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `FREQ?` | `FREQ?` |
| `0` long | `FREQ <MHz>`, then digits, then `Enter` | `FREQ <MHz>`, then digits, then `Enter` |
| `0` double click | `ROUND 500` | `ROUND 500` |
| `1` short | not available (`RXTX?` readback is unreliable) | `RXTX?` |
| `2` short | `TXFREQ?`; falls back to `FREQ?` when the radio gives no TX frequency | `FREQ?` when split is known to be off, otherwise not available |
| `3` short | `LOCK?` (tracked state) | `LOCK?` (tracked state) |
| `3` long | `LOCK ON/OFF` | `LOCK ON/OFF` |
| `4` short | `PO?` | `PO?` |
| `6` short | — | `RFPOWER?`: menu 75 power of the current band ("power 10 watts") |
| `7` short | `SM?` | `SM?` |
| `8` short | `SWR?` | `SWR?` |
| `9` short | `MODE?` | `MODE?` |
| `9` long | `MODE <n>`, then digit, then `Enter` | `MODE <n>`, then digit, then `Enter` |

`LOCK?` does not read the radio: FT8x7 CAT has no lock readback, so the firmware speaks the lock state it last set. `3` long flips that tracked state.

## Bank 2 - Radio Settings (FT-857/897)

These keys read settings the FT-857/897 keeps in its EEPROM, like the HamPod does. They only read: CAT cannot change these settings, so a long press on `1`–`3` says "not available". IPO, ATT and NAR are those of the current band and VFO, and only on the amateur bands (IPO and ATT on HF and 6 m). On the FT-817 the keys are not available.

| Key | FT-857/897 | Says |
|---|---|---|
| `1` short | `NR?` (DSP noise reduction, DNR) | "noise reduction on" |
| `2` short | `NB?` | "noise blanker off" |
| `3` short | `NOTCH?` (DSP auto notch, DNF) | "notch filter on" |
| `4` short | `AGC?` | "a g c auto" (auto, fast, slow or off) |
| `5` short | `IPO?` | "i p o off" |
| `5` long | `ATT?` | "a t t off" |
| `6` short | `DBF?` (DSP bandpass filter) | "d b f off" |
| `7` short | `BK?` (break-in) | "b k on" |
| `7` long | `KYR?` (keyer) | "k y r off" |
| `8` short | `NAR?` (FM narrow) | "n a r off" |
| `9` short | `MENU?`: the menu item the radio's menu was last left on | "menu 7 6" |
| `9` long | `ROW?`: the soft key row, as saved when the menu was last exited | "row 1 1" |

The radio saves the menu item and the row only when its menu is exited, so `9` long says the row you were on when you last left the menu.

## Bank 3 - VFO / Split

FT-817 note:

- If the spoken `SPLIT?` state is out of sync, toggle `0` long a few times to bring the spoken state back into sync with the radio.

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `SPLIT?` | `SPLIT?` |
| `0` long | `SPLIT ON/OFF` | `SPLIT ON/OFF` |
| `0` double click | `TXFREQ?` | `TXFREQ?`: `FREQ?` when split is off, otherwise not available |
| `1` short | current `VFOA/VFOB?` | current `VFOA/VFOB?` |
| `1` long | `A/B` | `A/B` |
| `1` double click | current `VFOA/VFOB <MHz>`, then digits, then `Enter` | current `VFOA/VFOB <MHz>`, then digits, then `Enter` |
| `2` short | other `VFOA/VFOB?` | other `VFOA/VFOB?` |
| `2` long | `A=B` (copy the active VFO to the other) | not available |
| `2` double click | other `VFOA/VFOB <MHz>`, then digits, then `Enter` | other `VFOA/VFOB <MHz>`, then digits, then `Enter` |
| `3` short | `VFOA MODE?` | — |
| `3` long | `VFOA MODE <n>`, then digit, then `Enter` | — |
| `4` short | `SYNC VFOA` | `SYNC VFOA` |
| `4` long | `SYNC VFOB` | `SYNC VFOB` |
| `5` short | `VFOB MODE?` | `CLAR ON` |
| `5` long | `VFOB MODE <n>`, then digit, then `Enter` | `CLAR OFF` |
| `6` short | active `VFO A` | `PTT OFF` (RX) |
| `6` long | active `VFO B` | `PTT ON` (TX) |

FT-817 Bank 3 note:

- The FT-817 branch currently mixes a tracked `current/other VFO` workflow with explicit `SYNC VFOA/VFOB` and explicit active-`VFO A/B` selection.
- `1`/`2` double click lead into staged frequency entry for the tracked current/other VFO, `1` long toggles `A/B` and `2` long copies the active VFO to the other (`A=B`, also the console command `VFO A=B`), while `3`/`5` handle `VFOA MODE` and `VFOB MODE`.
- Because of that design, `4` sync is still important after any unknown front-panel A/B change.

## Bank 6 - Repeater / Tone

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `RPT OFF` | `RPT OFF` |
| `0` long | `RPT MINUS` | `RPT MINUS` |
| `0` double click | `RPT PLUS` | `RPT PLUS` |
| `1` short | `RPTSHIFT` preset 1 (`rpt_offset_1`, default `0.600`) | `RPTSHIFT` preset 1 (`rpt_offset_1`, default `0.600`) |
| `1` long | `RPTSHIFT` preset 2 (`rpt_offset_2`, default `7.600`) | `RPTSHIFT` preset 2 (`rpt_offset_2`, default `7.600`) |
| `1` double click | `RPTSHIFT <kHz>`, then digits, then `Enter` | `RPTSHIFT <kHz>`, then digits, then `Enter` |
| `2` short | `TONE OFF` | `TONE OFF` |
| `2` long | `TONE CTCSS` | `TONE CTCSS` |
| `2` double click | `TONE DCS` | `TONE DCS` |
| `3` short | `CTCSS?` (speak only) | `CTCSS?` (speak only) |
| `3` long | `CTCSS <tone>`, then digits, then `Enter` | `CTCSS <tone>`, then digits, then `Enter` |
| `4` short | `DCS?` (speak only) | `DCS?` (speak only) |
| `4` long | `DCS <code>`, then digits, then `Enter` | `DCS <code>`, then digits, then `Enter` |

For Bank 6 tone handling, `2` is an explicit mode selector:

- `2` short = tone off
- `2` long = CTCSS on
- `2` double click = DCS on

`3` short and `4` short do not read the radio: they speak the last CTCSS tone or DCS code set from the keypad, or the `[bank6]` SD-card default (`88.5`, `023` unless changed) when nothing has been set yet.

## Bank 9 - Profile / System

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `A` short | `PROFILE?` | `PROFILE?` |
| `A` long | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) |
| `B` short | `PROFILE NEXT` | `PROFILE NEXT` |
| `C` short | `PROFILE PREV` | `PROFILE PREV` |
| `4` short | `TUNINGSPEECH?` | `TUNINGSPEECH?` |
| `4` long | `TUNINGSPEECH TOGGLE` | `TUNINGSPEECH TOGGLE` |
| `7` short / long | `VOLUME DOWN` / `VOLUME DOWN FAST` | `VOLUME DOWN` / `VOLUME DOWN FAST` |
| `8` short / long | `VOLUME UP` / `VOLUME UP FAST` | `VOLUME UP` / `VOLUME UP FAST` |
| `9` short | `VOLUME?` | `VOLUME?` |

## Known Limits

| Function | FT-817 | FT-857/897 |
|---|---|---|
| `SPLIT` | usable | usable; the state is read from the radio |
| `Bank 6 repeater/tone writes` | expect best results only on `2 m` or `70 cm` and already in `FM`; other contexts can make valid CAT writes look unreliable | expect best results only on the intended `VHF/UHF` band and already in `FM`; other contexts can make valid CAT writes look unreliable |
| `CLAR OFF` | usable | usable in current testing |
| `VFO A/B tracking` | usable with sync support | usable with sync support |
| `manual front-panel A/B changes` | resync recommended | resync recommended |
| `LOCK?` | tracked state only, no CAT readback | tracked state only, no CAT readback |
| FT-817 hidden background conditions | documented CAT commands can work well, but some success still appears to depend on not-yet-characterized radio state; more testing is needed | not the main current concern |
| `BANK 2 NR/NB/NOTCH/FILTER` | not available | not available |
| `BSTACK` | not available | not available |
| `MEM READ/WRITE` | experimental | experimental |
| `VOL/SQL/PO/SWR` | not cleanly validated | not cleanly validated |

## Notes

- `Enter` refers to the keypad confirmation key `D`.
- In frequency entry `*` is the decimal point: `145*500`, then `Enter`, tunes to 145.500 MHz.
- `double click` refers to a quick second press of the same key; a key with no double-click action runs its short action at once.
- `*` short speaks the current bank; `*` long, then a digit, selects a bank.
- `—` marks a key with no function on that radio.
