# Yaesu FTDX10 Family Status

This page is the short technical summary for the current **V3.5.7** FTDX10 family state.

Profiles:

- `ftdx10.ini`
- `ftdx101d.ini`
- `ftdx101mp.ini`

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
- preamp read and set
- AGC query plus fast and slow presets
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