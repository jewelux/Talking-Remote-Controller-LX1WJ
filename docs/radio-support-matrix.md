# Radio Support Matrix

This table tracks the current documented support state.
It is meant as a short engineering overview, not as a promise list.

| Radio / Profile | Protocol | Frequency | Mode | Power | S-Meter | SWR | Profile Load | Hardware Status | Notes |
|---|---|---|---|---|---|---|---|---|---|
| ICOM IC-7300 | CI-V | implemented | implemented | implemented | implemented | implemented | implemented | tested on hardware | See `docs/radios/icom-ic-7300.md` for the extended command map. |
| ICOM IC-7300 RS-232 | RS-232 | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | Intended to mirror the CI-V path. |
| ICOM IC-705 | CI-V | implemented | implemented | implemented | implemented | implemented | implemented | not tested | Everything the IC-7300 has, checked against the IC-705 CI-V reference guide; the tuner command drives an AH-705. |
| ICOM IC-7760 | CI-V | implemented | implemented | implemented | implemented | implemented | implemented | not tested | As the IC-7300 (its CI-V reference guide) but no transceive and no VOX delay; main and sub through command 29. |
| ICOM IC-706 CI-V | CI-V | implemented | implemented | n/a | implemented | n/a | implemented | not recently verified | Frequency, mode and S-meter. No IC-706 variant has the PO or SWR meter command. |
| ICOM IC-706 RS-232 | RS-232 | implemented | implemented | n/a | implemented | n/a | implemented | not recently verified | As the CI-V slot. |
| Xiegu G106 | CI-V (TTL) | implemented | implemented | n/a | n/a | n/a | implemented | not recently verified | Frequency and mode only. |
| Kenwood TS-480 | ASCII / CAT | implemented | implemented | n/a | implemented | implemented | implemented | not tested | SWR from `RM;` (SWR, COMP and ALC in one reply). NR and NB; PROC, mic gain and VOX (see below). |
| Elecraft KX2 | ASCII / CAT | implemented | implemented | implemented | implemented | implemented | implemented | not tested | NB, preamp, attenuator, AGC fast and slow, power switch. |
| Yaesu FT-817 | CAT | implemented | implemented | implemented | implemented | implemented | implemented | tested on hardware | FT8x7 family notes are documented separately. |
| Yaesu FT-818 | CAT | implemented | implemented | partial | implemented | partial | implemented | not recently verified | Mirrors the FT-817 family path. |
| Yaesu FT-857 | CAT | implemented | implemented | partial | implemented | partial | implemented | tested on hardware | Split and VFO handling are already documented and tested. |
| Yaesu FT-897 | CAT | partial | partial | partial | partial | partial | implemented | not recently verified | Shares the FT-857/897 family handling. |
| Yaesu FT-847 | CAT (own protocol) | implemented | implemented (no FM) | meter 0..31 | implemented | n/a | implemented | tested on hardware | Frequency, mode, S-meter, RX/TX. FM is not supported: a frequency read keys the radio in FM. See `docs/radios/yaesu-ft-847.md`. |
| Yaesu FTDX10 | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Current `V3.5.8` block includes VFO, VFO mode, split, lock, tuner, preamp, attenuator, AGC, IF, ID, and power-state paths. See `docs/radios/yaesu-ftdx10-family.md`. |
| Yaesu FTDX101D | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Uses the same first-block documentation as FTDX10. |
| Yaesu FTDX101MP | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Uses the same first-block documentation as FTDX10. |
| Yaesu FT-891 | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | not tested | Frequency, VFO A/B, mode, meters, lock, tuner (external), split; PROC, mic gain and VOX (see below). |

## Radio settings over CAT

Console commands that read and set the radio's settings. Each set is read back. A setting is
offered only where Hamlib and a second source (the maker's CAT manual, flrig or OmniRig) agree on
its command; the others say "not available". The FT-8x7 has no CAT command for these and keeps
them in its EEPROM: see the FT-8x7 pages.

| Radio | `AGC` | `PREAMP` | `ATT` | `RFPOWER` | `PROC`, `PROCLEVEL` | `MICGAIN` | `VOX`, `VOXGAIN` | `VOXDELAY` | Sources |
|---|---|---|---|---|---|---|---|---|---|
| IC-7300, IC-705 | fast, mid, slow | off, 1, 2 | 20 dB | 0–100 W, IC-705 0–10 W | COMP, 0–100 | 0–100 | 0–100 | 0–2000 ms | CI-V guide, Hamlib |
| IC-7760 | fast, mid, slow | off, 1, 2 | 6, 12, 18 dB | 0–200 W | COMP, 0–100 | 0–100 | 0–100 | – | CI-V guide, Hamlib |
| FTDX10 | off, fast, mid, slow, auto | IPO, AMP1, AMP2 | 6, 12, 18 dB | 5–100 W | `PR0`, `PL` 0–100 | 0–100 | 0–100 | 30–3000 ms (VD step) | CAT manual, Hamlib |
| FTDX101D/MP | off, fast, mid, slow, auto | IPO, AMP1, AMP2 | 6, 12, 18 dB | D: 5–100 W, MP: – | – | 0–100 | 0–100 | 30–3000 ms (VD step) | CAT manual, Hamlib, flrig |
| FT-891 | off, fast, mid, slow, auto | IPO, AMP | on | 5–100 W | `PR0`, `PL` 0–100 | 0–100 | 0–100 | 30–3000 ms | CAT manual, Hamlib, flrig |
| TS-480 | off, fast, slow | on | on | 5–100 W | `PR`, `PL` input 0–100 | 0–100 | `VG` 0–9 | 0–3000 ms, 150 ms steps | PC command reference, Hamlib |
| KX2 | fast, slow | on | 10 dB | 0–15 W | – | – | – | – | programmer's reference, Hamlib |

A Yaesu set to auto reads back as auto, whichever speed the radio picked. The TS-480 has no AGC
setting in FM. The IC-7760 attenuator reads every 3 dB step Icom lists, but HamTRC sets only the
6, 12 and 18 dB that Hamlib has too. On the IC-705 the 144 and 430 MHz bands have one preamp level
and no attenuator. "on" means the sources do not agree on the dB. The FTDX101MP CAT manual gives
`PC` 5–100 where Hamlib scales it to 200 W, so its RF power says "not available". The TS-480HX
goes to 200 W, but HamTRC sets up to 100 W, as the TS-480SAT; in AM both stop at 25 or 50 W, and
the KX2 at 12 W on 80–20 m, so a higher set reads back as an error.

None of these sets is tested on a radio yet.

`NRLEVEL` and `NBLEVEL` are in percent on the Icoms (`14 06`, `14 12`), in the radio's own steps on
the FTDX10, FTDX101 and FT-891 (`RL0` 1–15, `NL0` 0–10, their CAT manuals and Hamlib), and the
menu values on the FT-857/897.

`ANOTCH?` and `ANOTCH OFF | ON | TOGGLE` (Bank 2 `0`) read and set the auto notch, not the
manual notch of `NOTCH`: Icom `16 41` and Yaesu `BC0`, 0 off and 1 on (the CI-V guides and CAT
manuals, Hamlib). The TS-480 reference has no notch command.

`TONE`, `TONEDIR` and `CTCSS` read and set the tone squelch over CAT, CTCSS only: both ways (Icom
TSQL, `16 43`; Yaesu `CT0` 1) or encode only (Icom TONE, `16 42`; Yaesu `CT0` 2), and the tone
(Icom `1B 00`, which a set also writes to the TSQL tone `1B 01`; Yaesu `CN00`, the 50-tone table of
the CAT manuals). The FT-891 reads DCS too, but HamTRC sets CTCSS only. `RPT` sets the repeater
shift on the IC-705 (`0F 10`–`12`) and the FTDX10, FTDX101 and FT-891 (`OS0`, FM only); the
IC-7300 and IC-7760 have no CI-V command for it. Sources: the CI-V guides and CAT manuals, Hamlib.
The offset itself is not set over CAT.

`ALC?` (Bank 1 `5`) reads the ALC meter while transmitting, in percent of full scale: on the
Icoms `15 13`, 0–120 (their CI-V guides and Hamlib), on the FTDX10, FTDX101 and FT-891 `RM4`,
0–255 (their CAT manuals; flrig scales it so, Hamlib reads the same command). The FT-8x7 has its
own `ALC?`, 0–15, console only.

### CW

| Radio | `BK` | `KYR` | `CWSPEED` | `CWPITCH` | `CWDELAY` | Sources |
|---|---|---|---|---|---|---|
| IC-7300, IC-705, IC-7760 | off, semi, full | – | – | 300–900 Hz | – | CI-V guide, Hamlib |
| FTDX10 | on/off | on/off | 4–60 WPM | 300–1050 Hz | 30–3000 ms (SD step) | CAT manual, Hamlib |
| FTDX101D/MP | on/off | on/off | 4–60 WPM | 300–1050 Hz | – | CAT manual, Hamlib, flrig |
| FT-891 | on/off | on/off | 4–60 WPM | 300–1050 Hz | 30–3000 ms | CAT manual, Hamlib |
| TS-480 | – | – | 10–60 WPM | – | 0–1000 ms, 50 ms steps, 0 full break-in | PC command reference, Hamlib |
| KX2 | – | – | 8–50 WPM | – | – | programmer's reference, Hamlib |

Left out where the sources disagree: the Icom keyer speed (Hamlib's table is not linear) and
delay, the FTDX101 delay (its manual sends ms, Hamlib steps), the TS-480 and KX2 pitch. The Icom
pitch is set to its nearest 5 Hz step. These are not tested on a radio yet either.

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
