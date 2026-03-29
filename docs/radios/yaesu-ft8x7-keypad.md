# FT8x7 Keypad Layout

This document describes the current keypad layout for the Yaesu FT-817, FT-857, and FT-897 family in relation to the Icom-driven keypad structure already used elsewhere in the project.

The intention is:

- keep the overall bank structure consistent with the IC-7300
- only expose functions that are documented and practically usable
- move repeater and tone functions into a dedicated Bank 6

## Bank 1

| Key | Icom | FT-817 | FT-857/897 |
|---|---|---|---|
| `T0 short` | `FREQ?` | `FREQ?` | `FREQ?` |
| `T0 long` | `FREQ set` | `FREQ set` | `FREQ set` |
| `T1 short` | `RXTX?` | `RXTX?` | `RXTX?` |
| `T2 short` | `TXFREQ?` | `TXFREQ?` | not reliable |
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
| `T0 short` | `SPLIT?` | `SPLIT?` | not reliable |
| `T0 long` | `SPLIT toggle` | `SPLIT toggle` | not reliable |
| `T0 double` | `TXFREQ?` | `TXFREQ?` | unavailable |
| `T1 short` | `VFOA?` | `VFOA?` | `CLAR console only` |
| `T1 long` | `VFO A` | `VFO A` | `CLAR console only` |
| `T1 double` | `VFOA FREQ` | `VFOA FREQ` | `CLAR OFFSET console` |
| `T2 short` | `VFOB?` | `VFOB?` | `VFOB unsupported` |
| `T2 long` | `VFO B` | `VFO B` | `VFO B unsupported` |
| `T2 double` | `VFOB FREQ` | `VFOB FREQ` | `VFOB FREQ unsupported` |
| `T4 short` | `VFOA MODE?` | `VFOA MODE?` | unsupported |
| `T4 long` | `VFOA MODE set` | `VFOA MODE set` | unsupported |
| `T5 short` | `VFOB MODE?` | `VFOB MODE?` | unsupported |
| `T5 long` | `VFOB MODE set` | `VFOB MODE set` | unsupported |
| `T6 short` | tx/rx related | `RXTX?` | special-case logic, not prioritized |
| `T7 short` | `BSTACK? 1` | `BSTACK? 1` | `BSTACK? 1` |
| `T7 long` | `BSTACK 1` | `BSTACK 1` | `BSTACK 1` |
| `T8 short` | `BSTACK? 2` | `BSTACK? 2` | `BSTACK? 2` |
| `T8 long` | `BSTACK 2` | `BSTACK 2` | `BSTACK 2` |
| `T9 short` | `BSTACK? 3` | `BSTACK? 3` | `BSTACK? 3` |
| `T9 long` | `BSTACK 3` | `BSTACK 3` | `BSTACK 3` |

## Bank 6

| Key | Icom | FT-817 | FT-857/897 |
|---|---|---|---|
| `T0 short` | free | `RPT OFF` | `RPT OFF` |
| `T0 long` | free | `RPT MINUS` | `RPT MINUS` |
| `T0 double` | free | `RPT PLUS` | `RPT PLUS` |
| `T1 short` | free | `RPTSHIFT 0.600` | `RPTSHIFT 0.600` |
| `T1 long` | free | `RPTSHIFT 1.600` | `RPTSHIFT 1.600` |
| `T1 double` | free | `RPTSHIFT 5.000` | `RPTSHIFT 5.000` |
| `T2 short` | free | `TONE OFF` | `TONE OFF` |
| `T2 long` | free | `TONE CTCSS` | `TONE CTCSS` |
| `T2 double` | free | `TONE DCS` | `TONE DCS` |
| `T3 short` | free | `CTCSS 88.5` | `CTCSS 88.5` |
| `T4 short` | free | `DCS 023` | `DCS 023` |
| `T5 short` | free | `CTCSS 67.0` | `CTCSS 67.0` |
| `T6 short` | free | `CTCSS 71.9` | `CTCSS 71.9` |
| `T7 short` | free | `CTCSS 77.0` | `CTCSS 77.0` |
| `T8 short` | free | `CTCSS 82.5` | `CTCSS 82.5` |
| `T9 short` | free | `CTCSS 100.0` | `CTCSS 100.0` |

## Bank 9

| Key | Icom | FT-817 | FT-857/897 |
|---|---|---|---|
| `T0 short` | `PROFILE?` | `PROFILE?` | `PROFILE?` |
| `T0 long` | `PROFILE SELECT` | `PROFILE SELECT` | `PROFILE SELECT` |
| `T1 short` | `PROFILE NEXT` | `PROFILE NEXT` | `PROFILE NEXT` |
| `T2 short` | `PROFILE PREV` | `PROFILE PREV` | `PROFILE PREV` |
| `T3 short` | `BANK?` | `BANK?` | `BANK?` |
| `T4 short` | `TUNINGSPEECH?` | same | same |
| `T4 long` | `TUNINGSPEECH toggle` | same | same |
| `T5 short` | `VOLUME?` | same | same |

## Known Limits

| Function | FT-817 | FT-857/897 |
|---|---|---|
| `SPLIT` | usable | not reliable |
| `CLAR OFF` | usable | still questionable |
| absolute `VFO A/B` | usable | not cleanly usable |
| `MEM READ/WRITE` | experimental | experimental |
| `VOL/SQL/PO/SWR` | not cleanly validated | not cleanly validated |
