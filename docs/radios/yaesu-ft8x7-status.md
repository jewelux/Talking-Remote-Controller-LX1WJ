# FT8x7 CAT Status

This document summarizes the current CAT support status for the Yaesu FT-817, FT-857, and FT-897 family in this project.

The family is currently handled in two sub-variants:

- `ft817`
- `ft857_897`

The goal of this document is to separate:

- implemented and verified functions
- implemented but unreliable functions
- experimental or incomplete functions

## FT-817

### Implemented and verified

| Function | Status |
|---|---|
| Frequency read/write | implemented and verified |
| Mode read/write | implemented and verified |
| PTT on/off | implemented and verified |
| Lock on/off | implemented and verified |
| Split on/off | implemented and verified |
| VFO A/B handling | implemented and verified |
| Clarifier on/off | implemented and verified |
| Clarifier offset | implemented and verified |
| Repeater shift | implemented and verified |
| Repeater offset | implemented and verified |
| Tone/DCS mode | implemented and verified |
| CTCSS tone write | implemented and verified |
| DCS code write | implemented and verified |
| Power on/off | implemented and verified |

### Implemented but still incomplete

| Function | Status |
|---|---|
| RX/TX status bit interpretation | raw commands exist, bit meaning not fully documented from a primary source |
| Meter/status interpretation | partially usable, not fully finalized |

### Experimental or incomplete

| Function | Status |
|---|---|
| Memory read/write raw path | experimental |
| Volume / SQL / PO / SWR extras | not cleanly validated |

## FT-857

### Implemented and verified

| Function | Status |
|---|---|
| Frequency read/write | implemented and verified |
| Mode read/write | implemented and verified |
| Raw mode set/query (`YSETMODE`, `YMODEBYTE?`) | implemented and verified |
| PTT on/off | implemented and verified |
| Lock on/off | implemented and verified |
| VFO toggle | implemented and verified |
| Clarifier on | implemented and verified |
| Clarifier offset | implemented and verified |
| Repeater shift | implemented and verified |
| Repeater offset | implemented and verified |
| Tone/DCS mode | implemented and verified |
| CTCSS tone write | implemented and verified |
| DCS code write | implemented and verified |
| Extended CTCSS encode/decode modes | implemented and verified |
| Extended DCS encode/decode modes | implemented and verified |
| S-meter read | implemented and verified |
| ALC read | implemented and verified |
| RX status raw read | implemented and verified |
| TX status raw read | implemented and verified |
| RX/TX state query | implemented and verified |

### Implemented but unreliable

| Function | Status |
|---|---|
| Split on/off | documented, but not reliable in practical testing |
| Split status query | documented, but not reliable in practical testing |
| Clarifier off | not yet cleanly confirmed in practical testing |

### Experimental or incomplete

| Function | Status |
|---|---|
| Absolute VFO A/B selection | not cleanly usable for FT-857 |
| Memory read/write raw path | experimental |
| Volume / SQL / PO / SWR extras | not cleanly validated |

## FT-897

### Current handling

| Function group | Status |
|---|---|
| Variant model | grouped with `ft857_897` |
| Documented CAT assumptions | treated the same as FT-857 |
| Practical device verification | not yet available |

## Notes

- The keypad layout for this family is documented separately in [FT8X7_KEYPAD.md](./FT8X7_KEYPAD.md).
- Repeater and tone functions are now concentrated in Bank 6.
- For FT-857/897, only functions that behaved well in practical testing should be considered product-ready.
