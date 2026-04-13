# FT8x7 Keypad Layout

This document describes the current keypad layout for the Yaesu FT-817, FT-857, and FT-897 family in relation to the Icom-driven keypad structure already used elsewhere in the project.

The intention is:

- keep the overall bank structure consistent with the IC-7300
- only expose functions that are documented and practically usable
- move repeater and tone functions into a dedicated Bank 6
- Bank 6 now favors direct manual entry for repeater offset, CTCSS, and DCS, with only a small set of SD-card defaults in `[bank6]`

Important practical note:

- Some FT8x7 CAT write functions are context-sensitive on the radio side. The firmware sends the documented CAT bytes, but the radio may only apply them cleanly when the current band and mode fit the function. In practical repeater use this usually means being on the appropriate VHF/UHF band and already in FM. For example, common repeater shift workflows are much more predictable when the radio is already on 2 m FM.
- For FT8x7 repeater and tone work, treat `2 m` or `70 cm` plus `FM` as the expected operating context. If the radio is on another band or not already in `FM`, `RPT`, `RPTSHIFT`, `CTCSS`, and `DCS` writes may appear inconsistent even though the firmware is sending the documented CAT sequence.
- Recent FT-817 testing also suggests that documented CAT commands can work correctly once the radio is in the right background state, but that state is not yet characterized well enough to call the whole path fully stable. More testing is still needed around those hidden preconditions.

## Bank 1

| Key | Icom | FT-817 | FT-857/897 |
|---|---|---|---|
| `T0 short` | `FREQ?` | `FREQ?` | `FREQ?` |
| `T0 long` | `FREQ set` | `FREQ set` | `FREQ set` |
| `T1 short` | `RXTX?` | `RXTX?` | `RXTX?` |
| `T2 short` | `TXFREQ?` | `TXFREQ?` | `TXFREQ?` with FT8x7 fallback |
| `T3 short` | `LOCK?` | `LOCK?` | no clean readback |
| `T3 long` | `LOCK toggle` | `LOCK toggle` | not placed on keypad |
| `T4 short` | `PO?` | experimental | experimental |
| `T7 short` | `SM?` | `SM?` | `SM?` |
| `T8 short` | `SWR?` | experimental | experimental |
| `T9 short` | `MODE?` | `MODE?` | `MODE?` |
| `T9 long` | `MODE set` | `MODE set` | `MODE set` |

## Bank 3

| Key | Icom | FT-817 | FT-857/897 |
|---|---|---|---|
| `T0 short` | `SPLIT?` | `SPLIT?` | `SPLIT?` |
| `T0 long` | `SPLIT toggle` | `SPLIT toggle` | `SPLIT toggle` |
| `T0 double` | `TXFREQ?` | `TXFREQ?` | `TXFREQ?` |
| `T1 short` | `VFOA?` | current `VFOA/VFOB?` | current `VFOA/VFOB?` |
| `T1 long` | `VFO A` | current tracked `VFOA/VFOB FREQ set` | `A/B` |
| `T1 double` | `VFOA FREQ` | current tracked `VFOA/VFOB FREQ set` | current `VFOA/VFOB FREQ` |
| `T2 short` | `VFOB?` | other `VFOA/VFOB?` | other `VFOA/VFOB?` |
| `T2 long` | `VFO B` | other tracked `VFOA/VFOB FREQ set` | reserved |
| `T2 double` | `VFOB FREQ` | other tracked `VFOA/VFOB FREQ set` | other `VFOA/VFOB FREQ` |
| `T3 short` | free | `VFOA MODE?` | free |
| `T3 long` | free | `VFOA MODE set` | free |
| `T4 short` | `VFOA MODE?` | `SYNC VFOA` | `SYNC VFOA` |
| `T4 long` | `VFOA MODE set` | `SYNC VFOB` | `SYNC VFOB` |
| `T5 short` | `VFOB MODE?` | `VFOB MODE?` | `CLAR ON` |
| `T5 long` | `VFOB MODE set` | `VFOB MODE set` | `CLAR OFF` |
| `T6 short` | tx/rx related | active `VFO A` | `RX` |
| `T6 long` | tx/rx related | active `VFO B` | `TX` |
| `T7 short` | `BSTACK? 1` | `BSTACK? 1` | `BSTACK? 1` |
| `T7 long` | `BSTACK 1` | `BSTACK 1` | `BSTACK 1` |
| `T8 short` | `BSTACK? 2` | `BSTACK? 2` | `BSTACK? 2` |
| `T8 long` | `BSTACK 2` | `BSTACK 2` | `BSTACK 2` |
| `T9 short` | `BSTACK? 3` | `BSTACK? 3` | `BSTACK? 3` |
| `T9 long` | `BSTACK 3` | `BSTACK 3` | `BSTACK 3` |

FT-817 Bank 3 note:

- The FT-817 branch currently mixes a tracked `current/other VFO` workflow with explicit `SYNC VFOA/VFOB` and explicit active-`VFO A/B` selection.
- In the current `V3_5_3` software, `T1/T2` long and double both lead into staged frequency entry for the tracked current/other VFO, while `T3/T5` handle `VFOA MODE` and `VFOB MODE`.
- Because of that design, `T4` sync is still important after any unknown front-panel A/B change.

## Bank 6

| Key | Icom | FT-817 | FT-857/897 |
|---|---|---|---|
| `T0 short` | free | `RPT OFF` | `RPT OFF` |
| `T0 long` | free | `RPT MINUS` | `RPT MINUS` |
| `T0 double` | free | `RPT PLUS` | `RPT PLUS` |
| `T1 short` | free | `RPTSHIFT 0.600` | `RPTSHIFT 0.600` |
| `T1 long` | free | `RPTSHIFT 7.600` | `RPTSHIFT 7.600` |
| `T1 double` | free | `RPTSHIFT entry` | `RPTSHIFT entry` |
| `T2 short` | free | `TONE OFF` | `TONE OFF` |
| `T2 long` | free | `TONE CTCSS` | `TONE CTCSS` |
| `T2 double` | free | `TONE DCS` | `TONE DCS` |
| `T3 short` | free | `CTCSS 88.5` | `CTCSS 88.5` |
| `T3 long` | free | `CTCSS entry` | `CTCSS entry` |
| `T4 short` | free | `DCS 023` | `DCS 023` |
| `T4 long` | free | `DCS entry` | `DCS entry` |
| `T5 short` | free | free | free |
| `T6 short` | free | free | free |
| `T7 short` | free | free | free |
| `T8 short` | free | free | free |
| `T9 short` | free | free | free |

For Bank 6 tone handling, `T2` is an explicit mode selector:

- `T2 short` = tone off
- `T2 long` = CTCSS on
- `T2 double` = DCS on

## Bank 9

| Key | Icom | FT-817 | FT-857/897 |
|---|---|---|---|
| `A short` | `PROFILE?` | `PROFILE?` | `PROFILE?` |
| `A long` | `PROFILE SELECT` | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) |
| `B short` | `PROFILE NEXT` | `PROFILE NEXT` | `PROFILE NEXT` |
| `C short` | `PROFILE PREV` | `PROFILE PREV` | `PROFILE PREV` |
| `T4 short` | `TUNINGSPEECH?` | same | same |
| `T4 long` | `TUNINGSPEECH toggle` | same | same |
| `T7 short` | `speech volume 1` | same | same |
| `T8 short` | `speech volume 2` | same | same |
| `T9 short` | `speech volume 3` | same | same |

## Known Limits

| Function | FT-817 | FT-857/897 |
|---|---|---|
| `SPLIT` | usable | usable, but readback is still uncertain; if a defined state matters, toggle SPLIT a few times until the radio is known to be in the expected state |
| `Bank 6 repeater/tone writes` | expect best results only on `2 m` or `70 cm` and already in `FM`; other contexts can make valid CAT writes look unreliable | expect best results only on the intended `VHF/UHF` band and already in `FM`; other contexts can make valid CAT writes look unreliable |
| `CLAR OFF` | usable | usable in current testing |
| `VFO A/B tracking` | usable with sync support | usable with sync support |
| `manual front-panel A/B changes` | resync recommended | resync recommended |
| FT-817 hidden background conditions | documented CAT commands can work well, but some success still appears to depend on not-yet-characterized radio state; more testing is needed | not the main current concern |
| `BANK 2 NR/NB/NOTCH/FILTER` | not a current FT8x7 focus | mostly not available via documented FT8x7 CAT |
| `MEM READ/WRITE` | experimental | experimental |
| `VOL/SQL/PO/SWR` | not cleanly validated | not cleanly validated |
