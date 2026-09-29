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
| Bank 3 split keypad workflow | implemented and verified |
| Bank 3 current/other VFO frequency read workflow | implemented and verified |
| Bank 3 current/other VFO staged frequency entry | implemented and verified |
| Bank 3 `VFOA MODE` on `T3` | implemented and verified |
| Bank 3 sync `VFOA/VFOB` | implemented and verified |
| Bank 3 active `VFO A/B` selection on `T6` | implemented and verified |

### Implemented but still incomplete

| Function | Status |
|---|---|
| RX/TX status bit interpretation | keypad `RXTX?` is disabled for FT-817. The unstable results were likely the old code reading TX status bit 0 (a PO meter bit) instead of bit 7 (PTT); retest before enabling |
| Split status query | TX status bit 5 (1 = on, as on the FT-897) while transmitting, EEPROM `0x7A` bit 7 in receive; not yet verified on the radio |
| Meter/status interpretation | partially usable, not fully finalized |
| Hidden FT-817 background conditions around some documented CAT functions | practical tests suggest the documented commands can work correctly, but the exact conditions for stable behavior are still not fully mapped |
| Repeater and tone/DCS write paths outside normal VHF/UHF FM context | CAT bytes are implemented, but practical success is much more predictable when the radio is already on `2 m` or `70 cm` and already in `FM` |

### Experimental or incomplete

| Function | Status |
|---|---|
| Memory read/write raw path | experimental |
| PO / ALC / SWR meters | PO from TX status bits 3..0, ALC and SWR from the undocumented `BD`; implemented, not yet verified on the radio, off in the profile |
| Volume / SQL extras | not cleanly validated |

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
| RX status raw read | implemented and verified |
| TX status raw read | implemented and verified |
| RX/TX state query | verified on an FT-897: TX status bit 7 (0 = transmitting), `0xFF` in receive |
| `TXFREQ?` fallback on keypad | implemented and practically usable |
| Bank 3 current/other VFO read | implemented and practically usable |
| Bank 3 current/other VFO frequency set | implemented and practically usable |
| Bank 3 sync `VFOA/VFOB` | implemented and practically usable |
| Bank 3 `A/B` toggle | implemented and practically usable |
| Clarifier off | implemented and practically usable |

### Implemented but still worth more testing

| Function | Status |
|---|---|
| Split on/off | currently usable from keypad, but should still be cross-checked more broadly |
| Split status query | verified on an FT-897: EEPROM `0x8D` bit 7 in receive, TX status bit 5 while transmitting (1 = on; the manual says 0 = on, which is wrong) |
| Repeater and tone/DCS write paths outside normal FM repeater context | CAT bytes are implemented, but practical success can still depend on the radio already being in the appropriate VHF/UHF band and FM context |
| FT-857/897 VFO tracking after manual front-panel A/B changes | keypad workflow is usable, but sync is recommended |
| Settings read from the EEPROM (Bank 2, `MENU?`, `ROW?`, `AGC?`, `IPO?`, `ATT?`, `NAR?`, `DBF?`, `BK?`, `KYR?`, `NR?`, `NB?`, `NOTCH?`, `RFPOWER?`) | verified on an FT-897, read only. Addresses from the yo3ggx FT8x7EE map: `0x6A` bit 5 NB, bits 1..0 AGC speed (00 slow, 01 auto, 10 fast); `0x6B` bit 5 BK, bit 4 KYR; `0xA8` bit 5 AGC on, bits 3..2 DBF, bit 1 DNR, bit 0 DNF; `0x9B`/`0xAA`/`0xAB`/`0xAC` menu 75 power for HF/6 m/VHF/UHF (bits 6..0 = W); `0x88` menu item and `0x89` soft key row, both from 0 and written only when the radio's menu is exited (not in the yo3ggx map, measured). Per band and VFO, a 28-byte block: +1 bit 3 NAR, +2 bit 5 IPO, +2 bit 4 ATT, +12..+15 last frequency. VFO A blocks start at `0xBA` (160 m), VFO B blocks `0x1C0` higher; 5 MHz is `0xF2`, not `0x260` as in the map. `0x68` bit 0 names the VFO but does not follow CAT A/B toggles, so the block holding the current frequency decides |

### Experimental or incomplete

| Function | Status |
|---|---|
| Absolute VFO A/B without sync | not reliable after manual front-panel A/B changes |
| Memory read/write raw path | experimental |
| PO / ALC / SWR meters | verified on an FT-897 into a dummy load (PO 10, ALC 8, SWR 1.0): PO from TX status bits 3..0, ALC and SWR from the undocumented `BD`, sent only while transmitting since the radio does not answer it in receive. High-SWR flag (TX status bit 6) not yet seen set |
| Volume / SQL extras | not cleanly validated |

## FT-897

### Current handling

| Function group | Status |
|---|---|
| Variant model | grouped with `ft857_897` |
| Documented CAT assumptions | treated the same as FT-857 |
| Practical device verification | not yet available |

## Notes

- EEPROM reads (`BB`) must stay few and on demand. An FT-897 hung, together with HamTRC, when a test logger read 192 bytes every 5 s while the dial was being turned; a power cycle recovered it. `YEEPROM?` reads at most 32 bytes. The write (`BC`) and factory reset (`BE`) opcodes are never sent.

- The keypad layout for this family is documented separately in [yaesu-ft8x7-keypad.md](./yaesu-ft8x7-keypad.md).
- Repeater and tone functions are now concentrated in Bank 6.
- Profile selection in the current firmware supports up to 24 slots. On the keypad this means Bank 9 `A` long, then one or two digits, then `Enter`.
- For FT8x7 repeater work, `FM` on the intended `VHF/UHF` band should be treated as a practical precondition, not just a recommendation.
- For FT-857/897, only functions that behaved well in practical testing should be considered product-ready.
- The recent FT-857/897 Bank 3 work was intentionally kept separate from the FT-817 branch logic; FT-817 and FT-857/897 now have different keypad handling where that matches real device behavior better.
