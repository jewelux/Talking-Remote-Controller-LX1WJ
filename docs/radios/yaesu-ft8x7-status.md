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
| Bank 3 `A/B` toggle | implemented and practically usable |
| Clarifier off | implemented and practically usable |

### Implemented but still worth more testing

| Function | Status |
|---|---|
| Split on/off | currently usable from keypad, but should still be cross-checked more broadly |
| Split status query | verified on an FT-897: EEPROM `0x8D` bit 7 in receive, TX status bit 5 while transmitting (1 = on; the manual says 0 = on, which is wrong) |
| Repeater and tone/DCS write paths outside normal FM repeater context | CAT bytes are implemented, but practical success can still depend on the radio already being in the appropriate VHF/UHF band and FM context |
| Active VFO read from the EEPROM (`0x68`) | verified on an FT-897: follows both the front panel A/B key and the CAT toggle; Bank 3 reads it before each VFO action (`get_vfo=1`) |
| Settings read from the EEPROM (Bank 2, `MENU?`, `ROW?`, `AGC?`, `IPO?`, `ATT?`, `NAR?`, `DBF?`, `BK?`, `KYR?`, `NR?`, `NB?`, `NOTCH?`, `RFPOWER?`) | verified on an FT-897, read only; addresses in the [EEPROM map](#ft-857897-eeprom-map) |
| Further settings read from the EEPROM (`YSETTINGS?`: VOX, PROC, lock, fast tuning, DSP row, filter, menu levels, clarifier offset) | protocol only, no keys yet; verified on an FT-897 against the values set on the radio |

### FT-857/897 EEPROM map

Read with the undocumented `BB` (2 bytes per read). Measured on an FT-897 unless marked yo3ggx (the [FT8x7EE map](https://www.yo3ggx.ro/ft8x7ee/eeprom.html), also confirmed on the FT-897). Method: one setting changed on the radio at a time, menu values set to both ends of their range, and the radio's menu exited before reading, since menu values are saved on exit.

Shared by all bands:

| Address | Bits | Setting |
|---|---|---|
| `0x68` | 0 | VFO, 1 = B; follows the front panel A/B key and the CAT toggle (`81`) |
| `0x6A` | 7 | fast tuning, **inverted** (1 = off) |
| `0x6A` | 6 | lock, **inverted** (0 = locked); follows the front panel key and CAT lock |
| `0x6A` | 5 | NB (yo3ggx) |
| `0x6A` | 1..0 | AGC speed: 00 slow, 01 auto, 10 fast (yo3ggx) |
| `0x6B` | 7 | VOX |
| `0x6B` | 5 | BK (yo3ggx) |
| `0x6B` | 4 | KYR (yo3ggx) |
| `0x75` | 5..0 | CW speed (menu 30), value + 4 = WPM, 4..60 |
| `0x76` | 6..0 | VOX gain (menu 88), 1..100 |
| `0x77` | all | VOX delay (menu 87), × 100 ms, 100..3000 ms |
| `0x7A` | 6..0 | SSB mic gain (menu 81), 0..100 |
| `0x7B` | all | AM mic gain (menu 5), 0..100 |
| `0x7C` | all | FM mic gain (menu 51), 0..100 |
| `0x7D` | all | DIG gain (menu 37), 0..100 |
| `0x7E` | all | PKT 1200 level (menu 71), 0..100 |
| `0x7F` | all | PKT 9600 level (menu 72), 0..100 |
| `0x88` | all | menu item, from 0, saved when the menu is exited |
| `0x89` | all | soft key row, from 0, saved when the menu is exited |
| `0x8D` | 7 | split (Hamlib) |
| `0x90` | all | DIG VOX (menu 40), 0..100 |
| `0x93` | 7..4 | DSP NR level (menu 49), value + 1 = 1..16 |
| `0x93` | 3..2 | DSP BPF width (menu 45): 0 = 60, 1 = 120, 2 = 240 Hz (1 not seen) |
| `0x94` | 4..0 | DSP LPF cutoff (menu 47), 0..31 = 1000..6000 Hz (only the ends measured) |
| `0x95` | 3..0 | DSP HPF cutoff (menu 46), 100 + 60 × value Hz, 100..1000 Hz |
| `0x99` | all | NB level (menu 63), 0..100 |
| `0x9A` | all | PROC level (menu 74), 0..100 |
| `0x9B` | 6..0 | RF power (menu 75), HF, watts (yo3ggx) |
| `0xA7` | 7 | filter 2 selected on the CFIL row; clear = built-in. Filter 1 not measured (no filter fitted). Bit 1 was set when the built-in filter was chosen in CW, not in USB |
| `0xA8` | 7 | DSP soft key row shown, saved at once |
| `0xA8` | 5 | AGC on (yo3ggx) |
| `0xA8` | 3..2 | DBF (yo3ggx) |
| `0xA8` | 1 | DNR (yo3ggx) |
| `0xA8` | 0 | DNF (yo3ggx) |
| `0xA9` | 1 | PROC |
| `0xAA` / `0xAB` / `0xAC` | 6..0 | RF power (menu 75) for 6 m / VHF / UHF, watts (yo3ggx) |

Per band and VFO, a 28-byte block. VFO A blocks: 160 m `0xBA`, 80 m `0xD6`, 5 MHz `0xF2` (not `0x260` as in the yo3ggx map), 40 m `0x10E`, 30 m `0x12A`, 20 m `0x146`, 17 m `0x162`, 15 m `0x17E`, 12 m `0x19A`, 10 m `0x1B6`, 6 m `0x1D2`, FM broadcast `0x1EE`, air band `0x20A`, 2 m `0x226`, 70 cm `0x242`, general coverage `0x25E`. VFO B blocks are `0x1C0` higher.

| Offset | Bits | Setting |
|---|---|---|
| +1 | 3 | FM narrow (NAR) |
| +2 | 5 | IPO |
| +2 | 4 | ATT |
| +10..+11 | all | clarifier offset, signed, 10 Hz units, big-endian (`FF E1` = −310 Hz); kept when the clarifier is off |
| +12..+15 | all | frequency, 10 Hz units, big-endian |

The radio saves a band block on events such as key presses, not while the dial turns, so the block can lag the current frequency. The clarifier on/off state was not found in `0x00`..`0xBF` or the band blocks. Seen changing without a known cause: `0x6C`/`0x6D`, and `0xA9` bit 7 (set together with DIG VOX 100).

### Experimental or incomplete

| Function | Status |
|---|---|
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

- EEPROM reads (`BB`) must stay few and on demand. An FT-897 hung, together with HamTRC, when a test logger read 192 bytes every 5 s while the dial was being turned; a power cycle recovered it. `YEEPROM?` reads at most 32 bytes. The firmware never sends the factory reset (`BE`) opcode, and sends the write (`BC`, `<addr hi> <addr lo> <byte> <next byte> BC`, the format Hamlib uses; it takes effect at once) only for the console command `YEEPROM!`. The command takes an address and the bytes for it and the next address, e.g. `YEEPROM! 0068 1F 00`; odd addresses work (`BB` at `0069` returned the bytes at `0069` and `006A` on an FT-897), and is refused while transmitting. **Use it with caution:** a wrong address or value can wipe the radio's memories and calibration. While mapping, `BC` was used only to restore changed menu values. For mapping, keep one console connection open for the whole session: each new USB connection restarts HamTRC and can leave its keypad dead.

- The keypad layout for this family is documented separately in [yaesu-ft8x7-keypad.md](./yaesu-ft8x7-keypad.md).
- Repeater and tone functions are now concentrated in Bank 6.
- Profile selection in the current firmware supports up to 24 slots. On the keypad this means Bank 9 `A` long, then one or two digits, then `Enter`.
- For FT8x7 repeater work, `FM` on the intended `VHF/UHF` band should be treated as a practical precondition, not just a recommendation.
- For FT-857/897, only functions that behaved well in practical testing should be considered product-ready.
- The recent FT-857/897 Bank 3 work was intentionally kept separate from the FT-817 branch logic; FT-817 and FT-857/897 now have different keypad handling where that matches real device behavior better.
