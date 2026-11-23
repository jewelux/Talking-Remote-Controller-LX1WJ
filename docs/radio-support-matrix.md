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
| Xiegu G106 | CI-V (TTL) | implemented | implemented | n/a | n/a | n/a | implemented | tested on hardware | Frequency and mode only. Read and set tested on the Mini-HamTRC TTL CAT port (J3), November 2026. |
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

| Radio | `AGC` | `PREAMP` | `ATT` | `RFGAIN` | `RFPOWER` | `PROC`, `PROCLEVEL` | `MICGAIN` | `VOX`, `VOXGAIN` | `VOXDELAY` | Sources |
|---|---|---|---|---|---|---|---|---|---|---|
| IC-7300, IC-705 | fast, mid, slow | off, 1, 2 | 20 dB | `14 02` 0–255 | 0–100 W, IC-705 0–10 W | COMP, 0–100 | 0–100 | 0–100 | 0–2000 ms | CI-V guide, Hamlib |
| IC-7760 | fast, mid, slow | off, 1, 2 | 6, 12, 18 dB | `14 02` 0–255 | 0–200 W | COMP, 0–100 | 0–100 | 0–100 | – | CI-V guide, Hamlib |
| FTDX10 | off, fast, mid, slow, auto | IPO, AMP1, AMP2 | 6, 12, 18 dB | `RG0` 0–255 | 5–100 W | `PR0`, `PL` 0–100 | 0–100 | 0–100 | 30–3000 ms (VD step) | CAT manual, Hamlib |
| FTDX101D/MP | off, fast, mid, slow, auto | IPO, AMP1, AMP2 | 6, 12, 18 dB | `RG0` 0–255 | D: 5–100 W, MP: – | – | 0–100 | 0–100 | 30–3000 ms (VD step) | CAT manual, Hamlib, flrig |
| FT-891 | off, fast, mid, slow, auto | IPO, AMP | on | `RG0` 0–30 | 5–100 W | `PR0`, `PL` 0–100 | 0–100 | 0–100 | 30–3000 ms | CAT manual, Hamlib, flrig |
| TS-480 | off, fast, slow | on | on | `RG` 0–100 | 5–100 W | `PR`, `PL` input 0–100 | 0–100 | `VG` 0–9 | 0–3000 ms, 150 ms steps | PC command reference, Hamlib |
| KX2 | fast, slow | on | 10 dB | `RG` 190–250 | 0–15 W | – | – | – | – | programmer's reference, Hamlib |

A Yaesu set to auto reads back as auto, whichever speed the radio picked. The TS-480 has no AGC
setting in FM. The IC-7760 attenuator reads every 3 dB step Icom lists, but HamTRC sets only the
6, 12 and 18 dB that Hamlib has too. On the IC-705 the 144 and 430 MHz bands have one preamp level
and no attenuator. "on" means the sources do not agree on the dB. The FTDX101MP CAT manual gives
`PC` 5–100 where Hamlib scales it to 200 W, so its RF power says "not available". The TS-480HX
goes to 200 W, but HamTRC sets up to 100 W, as the TS-480SAT; in AM both stop at 25 or 50 W, and
the KX2 at 12 W on 80–20 m, so a higher set reads back as an error.

`RFGAIN` is in percent of the radio's range on every radio, so the same value means the same
part of the range: on the KX2 0 % is its lowest setting, -60 dB, and 100 % is 0 dB. The FT-891 has
only 31 steps and the KX2 61, so a set there reads back as the nearest step (`RFGAIN 51` on the
FT-891 gives 50), and `RFGAIN STEP` always moves at least one step. The FTDX101 is set for its main band
and the IC-7760 for the band selected. flrig sends the FTDX10 and FTDX101 only up to 250; HamTRC
uses the manuals' 255. The IC-706 (no `14` command), the G106, the FT-8x7 and the FT-847 have no
RF gain over CAT.

None of these sets is tested on a radio yet.

`NRLEVEL` and `NBLEVEL` are in percent on the Icoms (`14 06`, `14 12`), in the radio's own steps on
the FTDX10, FTDX101 and FT-891 (`RL0` 1–15, `NL0` 0–10, their CAT manuals and Hamlib), and the
menu values on the FT-857/897.

`ANOTCH?` and `ANOTCH OFF | ON | TOGGLE` (Bank 2 `0`) read and set the auto notch, not the
manual notch of `NOTCH`: Icom `16 41` and Yaesu `BC0`, 0 off and 1 on (the CI-V guides and CAT
manuals, Hamlib). The TS-480 reference has no notch command. `NOTCH OFF | ON` on the FTDX10,
FTDX101D/MP and FT-891 sends `BP00000;` and `BP00001;` (three digits, as the CAT manuals, Hamlib
and flrig have it) and is read back.

### Receive filter

Console commands; most are on Bank 2 and Bank 7 `6` too (see `docs/keypad-map.md`), but not
the IF shift, the slopes and the Icom width. Each takes `?`, a value, and `STEP <+-n>`; a value goes to the
radio's nearest step, and each set is read back. The width and the cuts depend on the mode: they
are read and set for the current mode, and say "not available" in a mode without them. `STEP`
moves a low cut up from `OFF` to its lowest value and a Yaesu high cut down from `OFF` to its
highest; back to `OFF` only by setting it. `WIDTH NORMAL` has no place among the widths, so
`WIDTH STEP` needs a width set first. The slopes take only 6 or 18.

| Radio | `WIDTH` | `NAR` | `IFSHIFT` | `NOTCHFREQ` | `CONTOUR`, `CONTOURFREQ` | `APF`, `APFFREQ` | `HPF`, `LPF`, slopes | Sources |
|---|---|---|---|---|---|---|---|---|
| IC-7300, IC-705, IC-7760 | `1A 03`: SSB, CW 50–3600 Hz, RTTY to 2700, AM 200–10000; not FM | – | – | – | – | – | – | CI-V guides, Hamlib |
| FTDX10 | `SH0`: SSB 300–4000 Hz, CW, RTTY, DATA 50–3000, `NORMAL`; not AM, FM | `NA0` | `IS0` ±1200 Hz, 20 Hz steps | `BP01` 10–3200 Hz | `CO00`, `CO01` 10–3200 Hz | `CO02`, `CO03` ±250 Hz | EX menu of the mode: 100–1000 and 700–4000 Hz, `OFF`, 6 or 18 dB/oct | CAT manual, Hamlib, flrig; cuts CAT manual |
| FTDX101D/MP | as FTDX10, SSB 300–3200 Hz | `NA0` | as FTDX10 | as FTDX10 | as FTDX10 | as FTDX10 | as FTDX10, its own menu numbers | CAT manuals, Hamlib, flrig; cuts CAT manuals |
| FT-891 | – | `NA0` | `IS0`, on digit | `BP01` | as FTDX10 | as FTDX10 | its EX menus, no FM | CAT manual, Hamlib, flrig; cuts CAT manual |
| TS-480 | – | – | – | – | – | – | `SL`, `SH` in SSB, FM and AM: low 0–1000 Hz, high 1000–5000 Hz | PC command reference, Hamlib |
| FT-857/897 | – | `NAR?` (read) | – | – | – | – | `HPF`, `LPF`, menus 46 and 47 (experimental) | EEPROM map |

The FTDX10 SSB widths 12–14 are 2250, 2400 and 2450 Hz in its CAT manual and flrig; Hamlib uses
the FTDX101's 2200, 2300 and 2400. HamTRC follows the manual. The low and high cut are menu
settings: their EX item numbers are only in the CAT manuals, not in Hamlib or flrig, but Hamlib's
repeater offset items sit where those manuals put them. The FTDX101 is set for its main band. The
TS-480 cuts are as with menu 45 (the data filter) off, which HamTRC cannot read.

Left out: the FT-891 width (which widths it takes depends on `NA`, and its two manual editions
differ); the KX2 width and shift (its quantization is not documented, Hamlib has neither); the
TS-480 IF shift (no unit in the reference, not in Hamlib); the Icom manual notch position `14 0D`
(no Hz scale from Icom); the IC-7760 APF (`16 32` sets on and width at once); the Icom RX
low and high cut (`1A 05`, byte layout not confirmed); the IC-706 and G106 (no such commands);
contour level and width and APF width (menu settings). None of this is tested on a radio yet.

`TONE`, `TONEDIR` and `CTCSS` read and set the tone squelch over CAT, CTCSS only: both ways (Icom
TSQL, `16 43`; Yaesu `CT0` 1) or encode only (Icom TONE, `16 42`; Yaesu `CT0` 2), and the tone
(Icom `1B 00`, which a set also writes to the TSQL tone `1B 01`; Yaesu `CN00`, the 50-tone table of
the CAT manuals). The FT-891 reads DCS too, but HamTRC sets CTCSS only. `RPT` sets the repeater
shift on the IC-705 (`0F 10`–`12`) and the FTDX10, FTDX101 and FT-891 (`OS0`, FM only); the
IC-7300 and IC-7760 have no CI-V command for it. Sources: the CI-V guides and CAT manuals, Hamlib.
The offset itself is not set over CAT.

`ALC?` (Bank 1 `5`) reads the ALC meter while transmitting, in percent of full scale: on the
Icoms `15 13`, 0–120 (their CI-V guides and Hamlib), on the FTDX10, FTDX101 and FT-891 `RM4`,
0–255 (their CAT manuals; flrig scales it so, Hamlib reads the same command). The FT-8x7 says its
own 0–15 meter, from the undocumented `BD`.

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

Every profile is built into the firmware. Bank 8 `2` says the baud rate, `2` long picks one, and
Bank 8 `B` and `C` (console `BAUD`) step up and down through the rates the radio offers; the choice is kept for
the profile, so set the same rate on the radio. On CI-V radios, Bank 8 `1` (console `CIVADDR`)
speaks or sets the CI-V address. Bank 9 `1` double press (console `PROFILE RESET`, or
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
