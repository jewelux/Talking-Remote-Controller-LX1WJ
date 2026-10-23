# Yaesu FTDX10 Family Status

This page is the short technical summary for the current **V3.5.8** FTDX10 family state.

Profiles (built in):

- slot 11: FTDX-10
- slot 12: FTDX-101D
- slot 13: FTDX-101MP
- slot 15: FT-891

## Current First Block

Implemented and documented for the current field-test phase:

- frequency read and set
- mode read and set
- VFO-A and VFO-B frequency read and set
- VFO-A and VFO-B select
- VFO-A and VFO-B mode read and set
- split read and set
- S-meter, power meter, SWR
- lock read and set
- tuner read, set, and tune start
- NR, NB, notch read and set
- preamp and attenuator read and set (`PREAMP? | PREAMP OFF | 1 | 2`, `ATT? | ATT OFF | 6 | 12 | 18`,
  on the FT-891 `PREAMP OFF | ON` and `ATT OFF | ON`, Bank 2 `B` and `C`): `PA0` 0 IPO, 1 AMP1,
  2 AMP2 and `RA0` 0 off, 1..3 for 6, 12 and 18 dB, as the CAT manuals and Hamlib have them, each
  set read back. The raw `PA` console commands are gone.
- break-in and keyer (`BK? | BK OFF | ON | TOGGLE`, `KYR? | KYR OFF | ON | TOGGLE`, Bank 7 `1` and
  `2`): `BI` and `KR`, 0 off and 1 on, as the CAT manuals and Hamlib have them, read back.
- RF power (`RFPOWER? | RFPOWER <5..100 W>`, Bank 1 `6`): `PC` in watts, as the CAT manuals,
  Hamlib and flrig have it, read back. Not on the FTDX101MP: its manual says 005-100 and Hamlib
  scales `PC` to 200 W.
- AGC read and set (`AGC?`, `AGC OFF | FAST | MID | SLOW | AUTO`, Bank 2 `A`): `GT0`, as the CAT
  manual and Hamlib have it; auto reads back as auto, whichever speed the radio picked
- speech processor and its level (`PR0`, `PL`), mic gain (`MG`), VOX with its gain and delay
  (`VX`, `VG`, `VD`) read and set, console only (`PROC`, `PROCLEVEL`, `MICGAIN`, `VOX`,
  `VOXGAIN`, `VOXDELAY`). Each set is read back. The sets are not yet tested on a radio: `PROC ON`
  sends `PR01`, and `PR02` (the manual's on) when that does not read back as on
- on the FT-891 the same, but `VD` is in ms (`VD0030`–`VD3000`, Hamlib and flrig agree). flrig
  notes that the FT-891 takes `PR` only in SSB; in other modes `PROC` says "error"
- on the FTDX101D/MP the same but `PR` and `PL`: its CAT manual says `PR01` is off and `PR02` on,
  Hamlib that `PR01` is on. `VD` is a step as on the FTDX10 (the CAT manual and Hamlib; flrig
  sends ms). Hamlib notes that the FTDX101 takes no `MG` in CW and RTTY
- power-state query, off, and on path
- IF query
- ID query
- RX/TX query path

## Keypad Scope

The keypad now exposes the practical first block needed for Damian's assisted test.
Functions without a clear speech-friendly workflow are intentionally still left out or hidden.

## Not In Normal Keypad Use Yet

Examples that should still be treated as later work:

- memory and QMB handling
- menu and service commands
- contour, width, IF-shift, and similar fine controls
- detailed monitor and TX-audio controls
- broad meter-source and scope handling

## Companion Notes

- Blind helper list: [yaesu-ftdx10-blind-test-list.txt](./yaesu-ftdx10-blind-test-list.txt)
- Technical serial checks: [yaesu-ftdx10-test-plan.md](./yaesu-ftdx10-test-plan.md)