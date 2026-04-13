# Yaesu FT-817

## Status

The FT-817 is now best understood as part of the broader Yaesu FT8x7 family support in firmware `V3.5.1`.

For the current public state, please use these documents first:

- [Yaesu FT8x7 keypad layout](./yaesu-ft8x7-keypad.md)
- [Yaesu FT8x7 CAT status](./yaesu-ft8x7-status.md)

## Why This Page Is Short

This page is intentionally kept minimal so the repository does not present two conflicting descriptions of the same family.

The FT-817 remains important because it is the more fully verified member of the FT8x7 CAT family, and `V3.5.1` specifically fixes several FT8x7 control-path bugs that affected practical use.

## Summary

Current FT-817 strengths in the project include:

- documented CAT core functions verified on real hardware
- reliable handling of frequency and mode control with keypad-side write verification
- verified lock, split, VFO, clarifier, repeater, tone, DCS, and power test paths
- alignment with the shared FT8x7 keypad structure where practical
- the newer FT-817 Bank 3 layout that now gives `VFOA MODE` its own direct keypad position while keeping `SYNC` and explicit active-`VFO A/B` selection

Important `V3.5.1` notes:

- VFO A/B selection no longer fails silently in the FT8x7 path
- split status now uses the correct TX-status bit interpretation
- keypad writes are blocked while the radio reports TX to avoid accidental changes on air
- keypad `RXTX?` remains disabled on FT-817 because field results are still not stable enough
- repeater and tone functions should be treated as an `FM` plus `2 m/70 cm` workflow; outside that context, the radio may ignore otherwise documented CAT writes
- recent testing suggests some documented FT-817 CAT commands still depend on background radio state that is not yet fully characterized, so more field testing is still needed
