# Radio Support Matrix

This table tracks the current documented support state.
It is meant as a short engineering overview, not as a promise list.

| Radio / Profile | Protocol | Frequency | Mode | Power | S-Meter | SWR | Profile Load | Hardware Status | Notes |
|---|---|---|---|---|---|---|---|---|---|
| ICOM IC-7300 | CI-V | implemented | implemented | implemented | implemented | implemented | implemented | tested on hardware | See `docs/radios/icom-ic-7300.md` for the extended command map. |
| ICOM IC-7300 RS-232 | RS-232 | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | Intended to mirror the CI-V path. |
| ICOM IC-706 CI-V | CI-V | implemented | implemented | partial | partial | partial | implemented | not recently verified | Legacy support carried into the modular branch. |
| ICOM IC-706 RS-232 | RS-232 | partial | partial | partial | partial | partial | implemented | not recently verified | Needs recheck on live hardware. |
| Xiegu G106 | CAT | implemented | partial | partial | partial | partial | implemented | not recently verified | Existing support should be rechecked command by command. |
| Kenwood TS-480 | ASCII / CAT | partial | partial | planned | planned | planned | implemented | not recently verified | Good candidate for future expansion. |
| Elecraft KX2 | ASCII / CAT | partial | partial | partial | planned | planned | implemented | not recently verified | Evolving profile. |
| Yaesu FT-817 | CAT | implemented | implemented | implemented | implemented | implemented | implemented | tested on hardware | FT8x7 family notes are documented separately. |
| Yaesu FT-818 | CAT | implemented | implemented | partial | implemented | partial | implemented | not recently verified | Mirrors the FT-817 family path. |
| Yaesu FT-857 | CAT | implemented | implemented | partial | implemented | partial | implemented | tested on hardware | Split and VFO handling are already documented and tested. |
| Yaesu FT-897 | CAT | partial | partial | partial | partial | partial | implemented | not recently verified | Shares the FT-857/897 family handling. |
| Yaesu FT-847 | CAT (own protocol) | implemented | implemented (no FM) | planned | implemented | n/a | implemented | tested on hardware | Frequency, mode, S-meter, RX/TX. FM is not supported: a frequency read keys the radio in FM. See `docs/radios/yaesu-ft-847.md`. |
| Yaesu FTDX10 | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Current `V3.5.8` block includes VFO, VFO mode, split, lock, tuner, preamp, AGC, IF, ID, and power-state paths. See `docs/radios/yaesu-ftdx10-family.md`. |
| Yaesu FTDX101D | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Uses the same first-block documentation as FTDX10. |
| Yaesu FTDX101MP | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Uses the same first-block documentation as FTDX10. |

## Profiles and baud rates

Every profile is built into the firmware. Bank 8 `2` says the baud rate, and its long and double
press (console `BAUD`) step down and up through the rates the radio offers; the choice is kept for
the profile, so set the same rate on the radio. On CI-V radios, Bank 8 `1` (console `CIVADDR`)
speaks or sets the CI-V address. Bank 9 `A` double press (console `PROFILE RESET`, or
`PROFILE RESET ALL` for every profile) puts both back to the defaults below.

| Slot | Radio | Port | Default baud | Selectable baud |
|---|---|---|---|---|
| 1 | Icom IC-7300 | CI-V jack | 9600 | 4800, 9600, 19200, 38400, 57600, 115200 |
| 2 | Icom IC-706 | CI-V jack | 9600 | 4800, 9600, 19200, 38400, 57600, 115200 |
| 3 | Icom IC-7300 | RS-232 | 9600 | 4800, 9600, 19200, 38400, 57600, 115200 |
| 4 | Icom IC-706 | RS-232 | 9600 | 4800, 9600, 19200, 38400, 57600, 115200 |
| 5 | Xiegu G106 | TTL CAT | 19200 | 4800, 9600, 19200, 38400, 57600, 115200 |
| 6 | Elecraft KX2 | RS-232 | 38400 | 4800, 9600, 19200, 38400 |
| 7 | Kenwood TS-480 | RS-232 | 9600 | 4800, 9600, 19200, 38400, 57600 |
| 8 | Yaesu FT-817 | RS-232 | 4800 | 4800, 9600, 38400 |
| 9 | Yaesu FT-857 | RS-232 | 4800 | 4800, 9600, 38400 |
| 10 | Yaesu FT-897 | RS-232 | 4800 | 4800, 9600, 38400 |
| 11 | Yaesu FTDX10 | RS-232 | 38400 | 4800, 9600, 19200, 38400 |
| 12 | Yaesu FTDX101D | RS-232 | 38400 | 4800, 9600, 19200, 38400 |
| 13 | Yaesu FTDX101MP | RS-232 | 38400 | 4800, 9600, 19200, 38400 |
| 14 | Yaesu FT-818 | RS-232 | 4800 | 4800, 9600, 38400 |
| 15 | Yaesu FT-891 | RS-232 | 4800 | 4800, 9600, 19200, 38400 |
| 16 | Yaesu FT-847 | RS-232 | 4800 | 4800, 9600, 57600 |
| 17 | Icom IC-705 | CI-V jack | 9600 | 4800, 9600, 19200, 38400, 57600, 115200 |
| 18 | Icom IC-7760 | CI-V jack | 19200 | 4800, 9600, 19200, 38400, 57600, 115200 |

The Icom CI-V jack itself goes up to 19200 baud on the IC-7300, IC-705 and IC-706; HamTRC
still offers the faster rates on every CI-V profile.

Frequency handling is common to all profiles: entry, announcement and display now work to 10 Hz
resolution via the shared `RadioFrequency` class (number = MHz, `*` = decimal point), and Bank 1
`0` double-press rounds the current frequency to the nearest 500 Hz.
