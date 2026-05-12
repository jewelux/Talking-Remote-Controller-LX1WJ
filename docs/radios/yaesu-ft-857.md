# Yaesu FT-857

## Status

The FT-857 is now best understood as part of the broader Yaesu FT8x7 family support in firmware `V3.5.7`.

For the current public state, please use these documents first:

- [Yaesu FT8x7 keypad layout](./yaesu-ft8x7-keypad.md)
- [Yaesu FT8x7 CAT status](./yaesu-ft8x7-status.md)

## Why This Page Is Short

This page is intentionally kept minimal so the repository does not present two conflicting descriptions of the same family.

The FT-857 remains important because it defines much of the `ft857_897` variant behavior, and `V3.5.7` keeps that shared logic around split handling and keypad write safety.

## Summary

Current FT-857 strengths in the project include:

- documented CAT core functions verified on real hardware
- reliable handling of frequency and mode control
- verified lock, VFO toggle, repeater, tone, DCS, metering, RX/TX query, and split control paths
- shared FT8x7 keypad structure with family-specific limitations clearly documented

Known limitations are intentionally tracked in the FT8x7 family status page rather than repeated here.

Important `V3.5.7` notes:

- the shipped profile now exposes split query and split set
- split status decoding was corrected to the proper FT8x7 TX-status bit
- Bank 6 repeater and tone defaults are now included in the shipped profile
- manual front-panel A/B changes can still require keypad resync before absolute VFO operations
- repeater and tone/DCS writes are still best treated as a practical `VHF/UHF + FM` workflow even when the documented CAT bytes are correct
