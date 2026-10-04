# FT8x7 Keypad Layout

This document describes the current keypad layout for the Yaesu FT-817, FT-857, and FT-897 family.
It lists the keys these radios have of their own; [the keypad map](../keypad-map.md) has every
bank, Bank 4, 5 and 7 included, for all radios.

> The layout is experimental and may still change.

The intention is:

- keep the overall bank structure consistent across the FT8x7 family
- only expose functions that are documented and practically usable
- move repeater and tone functions into a dedicated Bank 6
- Bank 6 now favors direct manual entry for repeater offset, CTCSS, and DCS, with only a small set of built-in defaults

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
| `1` short | `RXTX?` | `RXTX?` |
| `2` short | `TXFREQ?`; falls back to `FREQ?` when the radio gives no TX frequency | `FREQ?` when split is known to be off, otherwise not available |
| `3` short | `LOCK?` (read from the radio) | `LOCK?` (read from the radio) |
| `3` long | `LOCK ON/OFF` | `LOCK ON/OFF` |
| `4` short | `PO?` | `PO?` |
| `6` short | `RFPOWER?`: the TX power setting | `RFPOWER?`: menu 75 power of the current band |
| `7` short | `SM?` | `SM?` |
| `8` short | `SWR?` | `SWR?` |
| `9` short | `MODE?` | `MODE?` |
| `9` long | `MODE <n>`, then digit, then `Enter` | `MODE <n>`, then digit, then `Enter` |

## Bank 2 - Radio Settings

All Bank 2 keys only read settings from the radio's EEPROM. CAT don't change them, so a long press on `1`–`3` says "not available".

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `ANT?`: antenna jack of the current band (menu 07), "antenna front" or "antenna rear" | not available |
| `1` short | not available (no DSP) | `NR?` (DSP noise reduction) |
| `2` short | `NB?` | `NB?` |
| `3` short | not available (no DSP) | `NOTCH?` (DSP auto notch) |
| `4` short | not available (no DSP) | `NRLEVEL?` (menu 49, 1–16) |
| `5` short | not available | `NBLEVEL?` (menu 63, 0–100) |
| `6` short | not available (no DSP) | `DBF?` (DSP bandpass filter) |
| `7` short | not available (no DSP) | `HPF?`: low cut (menu 46, DSP HPF cutoff), "h p f 300 hertz" |
| `8` short | not available (no DSP) | `LPF?`: high cut (menu 47, DSP LPF cutoff), "l p f 2800 hertz" |
| `9` short | not available (no DSP) | `MICEQ?`: TX equalizer (menu 48), "equalizer off", "l p f", "h p f" or "both" |
| `A` short | not available | `IPO?` (HF and 6 m) |
| `A` long | not available | `ATT?` (HF and 6 m) |
| `A` double click | `AGC?` | `AGC?` |

IPO and ATT are those of the current band and VFO. The menu settings are read as the radio last saved them. Break-in, keyer and FM narrow have no key; the console commands `BK?`, `KYR?` and `NAR?` read them.

## Bank 3 - VFO / Split

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
| `4` short | `SYNC VFOA` | — |
| `4` long | `SYNC VFOB` | — |
| `5` short | `RIT?` | `RIT?` |
| `5` long | `RIT TOGGLE` | `RIT TOGGLE` |
| `6` short | active `VFO A` | `PTT OFF` (RX) |
| `6` long | active `VFO B` | `PTT ON` (TX) |

FT-817 Bank 3 note:

- The FT-817 branch currently mixes a tracked `current/other VFO` workflow with explicit `SYNC VFOA/VFOB` and explicit active-`VFO A/B` selection.
- `1`/`2` double click lead into staged frequency entry for the tracked current/other VFO, `1` long toggles `A/B` and `2` long copies the active VFO to the other (`A=B`, also the console command `VFO A=B`), and `3` handles `VFOA MODE`, then returns to the VFO in use. VFO B's mode: `1` long, then Bank 1 `9`.
- The FT-817 cannot report its active VFO, so `4` sync is still important after any unknown front-panel A/B change.
- `5` works as on the FT-857/897: RIT is the clarifier, the short press of the radio's CLAR key; IF shift (the long press) is left alone. If RIT is on, asking switches it off and straight back on.

FT-857/897 Bank 3 note:

- `5` short switches RIT off and straight back on when RIT is on, since the radio cannot report RIT otherwise. If the clarifier knob was tuning IF shift at that moment, it tunes RIT afterwards (EEPROM `0x6A` bit 3). `5` long does not do this: it moves the knob to RIT only when it switches RIT on.

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

`3` short and `4` short do not read the radio: they speak the last CTCSS tone or DCS code set from the keypad, or the built-in default (`88.5`, `023`) when nothing has been set yet.

## Bank 8 - Radio Menu

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `7` short | `ROW?` (function row) | `ROW?` (soft key row) |
| `8` short | `MENU?` (last menu item) | `MENU?` (last menu item) |

`MENU?` and `ROW?` are saved only when the radio's menu is exited.

## Bank 9 - Profile / System

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `A` short | `PROFILE?` | `PROFILE?` |
| `A` long | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) |
| `B` short | `PROFILE NEXT` | `PROFILE NEXT` |
| `C` short | `PROFILE PREV` | `PROFILE PREV` |
| `4` short | `TUNINGSPEECH?` | `TUNINGSPEECH?` |
| `4` long | `TUNINGSPEECH TOGGLE` | `TUNINGSPEECH TOGGLE` |
| `5` short | `VERBOSE?` | `VERBOSE?` |
| `5` long | `VERBOSE TOGGLE` | `VERBOSE TOGGLE` |
| `7` short / long | `VOLUME DOWN` / `VOLUME DOWN FAST` | `VOLUME DOWN` / `VOLUME DOWN FAST` |
| `8` short / long | `VOLUME UP` / `VOLUME UP FAST` | `VOLUME UP` / `VOLUME UP FAST` |
| `9` short | `VOLUME?` | `VOLUME?` |

## Known Limits

| Function | FT-817 | FT-857/897 |
|---|---|---|
| `Bank 6 repeater/tone writes` | expect best results only on `2 m` or `70 cm` and already in `FM`; other contexts can make valid CAT writes look unreliable | expect best results only on the intended `VHF/UHF` band and already in `FM`; other contexts can make valid CAT writes look unreliable |
| `CLAR OFF` | replaced by `RIT?` / `RIT TOGGLE`; the CAT clarifier commands switch RIT | replaced by `RIT?` / `RIT TOGGLE`; the CAT clarifier commands switch RIT |
| `VFO A/B tracking` | usable with sync support; the radio has no readable VFO | read from the EEPROM |
| `manual front-panel A/B changes` | resync recommended | followed; no sync needed |
| FT-817 hidden background conditions | documented CAT commands can work well, but some success still appears to depend on not-yet-characterized radio state; more testing is needed | not the main current concern |
| `Bank 2 settings` | read only; only antenna, NB and AGC (no DSP, IPO/ATT not read) | read only |
| `MEM READ/WRITE` | experimental | experimental |
| `VOL/SQL` | not cleanly validated | not cleanly validated |

## Notes

- `Enter` refers to the keypad confirmation key `D`.
- In frequency entry `*` is the decimal point: `145*500`, then `Enter`, tunes to 145.500 MHz.
- `double click` refers to a quick second press of the same key; a key with no double-click action runs its short action at once.
- `*` short speaks the current bank; `*` long, then a digit, selects a bank.
- `—` marks a key with no function on that radio.
