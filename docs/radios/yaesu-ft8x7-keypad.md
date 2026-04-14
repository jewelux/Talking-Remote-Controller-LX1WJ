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

## Bank 1

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `T0 short` | `FREQ?` | `FREQ?` |
| `T0 long` | `FREQ set` | `FREQ set` |
| `T1 short` | `RXTX?` | `RXTX?` |
| `T2 short` | `TXFREQ?` | `TXFREQ?` with FT8x7 fallback |
| `T3 short` | `LOCK?` | no clean readback |
| `T3 long` | `LOCK toggle` | not placed on keypad |
| `T4 short` | experimental | experimental |
| `T7 short` | `SM?` | `SM?` |
| `T8 short` | experimental | experimental |
| `T9 short` | `MODE?` | `MODE?` |
| `T9 long` | `MODE set` | `MODE set` |

## Bank 3

FT-817 note:

- If the spoken `SPLIT?` state is out of sync, toggle `T0 long` a few times to bring the spoken state back into sync with the radio.

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `T0 short` | `SPLIT?` | `SPLIT?` |
| `T0 long` | `SPLIT toggle` | `SPLIT toggle` |
| `T0 double` | `TXFREQ?` | `TXFREQ?` |
| `T1 short` | current `VFOA/VFOB?` | current `VFOA/VFOB?` |
| `T1 long` | current tracked `VFOA/VFOB FREQ set` | `A/B` |
| `T1 double` | current tracked `VFOA/VFOB FREQ set` | current `VFOA/VFOB FREQ` |
| `T2 short` | other `VFOA/VFOB?` | other `VFOA/VFOB?` |
| `T2 long` | other tracked `VFOA/VFOB FREQ set` | reserved |
| `T2 double` | other tracked `VFOA/VFOB FREQ set` | other `VFOA/VFOB FREQ` |
| `T3 short` | `VFOA MODE?` | free |
| `T3 long` | `VFOA MODE set` | free |
| `T4 short` | `SYNC VFOA` | `SYNC VFOA` |
| `T4 long` | `SYNC VFOB` | `SYNC VFOB` |
| `T5 short` | `VFOB MODE?` | `CLAR ON` |
| `T5 long` | `VFOB MODE set` | `CLAR OFF` |
| `T6 short` | active `VFO A` | `RX` |
| `T6 long` | active `VFO B` | `TX` |
| `T7 short` | `BSTACK? 1` | `BSTACK? 1` |
| `T7 long` | `BSTACK 1` | `BSTACK 1` |
| `T8 short` | `BSTACK? 2` | `BSTACK? 2` |
| `T8 long` | `BSTACK 2` | `BSTACK 2` |
| `T9 short` | `BSTACK? 3` | `BSTACK? 3` |
| `T9 long` | `BSTACK 3` | `BSTACK 3` |

FT-817 Bank 3 note:

- The FT-817 branch currently mixes a tracked `current/other VFO` workflow with explicit `SYNC VFOA/VFOB` and explicit active-`VFO A/B` selection.
- In the current `V3_5_3` software, `T1/T2` long and double both lead into staged frequency entry for the tracked current/other VFO, while `T3/T5` handle `VFOA MODE` and `VFOB MODE`.
- Because of that design, `T4` sync is still important after any unknown front-panel A/B change.

## Bank 6

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `T0 short` | `RPT OFF` | `RPT OFF` |
| `T0 long` | `RPT MINUS` | `RPT MINUS` |
| `T0 double` | `RPT PLUS` | `RPT PLUS` |
| `T1 short` | `RPTSHIFT 0.600` | `RPTSHIFT 0.600` |
| `T1 long` | `RPTSHIFT 7.600` | `RPTSHIFT 7.600` |
| `T1 double` | `RPTSHIFT entry` | `RPTSHIFT entry` |
| `T2 short` | `TONE OFF` | `TONE OFF` |
| `T2 long` | `TONE CTCSS` | `TONE CTCSS` |
| `T2 double` | `TONE DCS` | `TONE DCS` |
| `T3 short` | `CTCSS 88.5` | `CTCSS 88.5` |
| `T3 long` | `CTCSS entry` | `CTCSS entry` |
| `T4 short` | `DCS 023` | `DCS 023` |
| `T4 long` | `DCS entry` | `DCS entry` |
| `T5 short` | free | free |
| `T6 short` | free | free |
| `T7 short` | free | free |
| `T8 short` | free | free |
| `T9 short` | free | free |

For Bank 6 tone handling, `T2` is an explicit mode selector:

- `T2 short` = tone off
- `T2 long` = CTCSS on
- `T2 double` = DCS on

## Bank 9

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `A short` | `PROFILE?` | `PROFILE?` |
| `A long` | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) | `PROFILE SELECT` (`1..24`, one or two digits, then `Enter`) |
| `B short` | `PROFILE NEXT` | `PROFILE NEXT` |
| `C short` | `PROFILE PREV` | `PROFILE PREV` |
| `T4 short` | same | same |
| `T4 long` | same | same |
| `T7 short` | same | same |
| `T8 short` | same | same |
| `T9 short` | same | same |

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
