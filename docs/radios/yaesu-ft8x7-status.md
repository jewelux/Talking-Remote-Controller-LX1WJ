# FT8x7 CAT Status

This document summarizes the current CAT support status for the Yaesu FT-817, FT-857, and FT-897 family in this project.

The family is currently handled as two models:

- FT-817 (and FT-818, the same apart from its power levels)
- FT-857/897

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
| RIT on/off | implemented and verified |
| RIT offset | implemented and verified |
| Repeater shift | implemented and verified |
| Repeater offset | implemented and verified |
| Tone/DCS mode | implemented and verified |
| CTCSS tone write | implemented and verified |
| DCS code write | implemented and verified |
| Tone mode (`TONE?`, `TONEDIR?`, Bank 6 `2` and `5`) | tested on an FT-817: the mode HamTRC last set, "not available" before one is set. Since the radio keeps it per band block, it holds only on the band it was set on (outside the amateur bands, on the same frequency); a switch to the other VFO on the same band is not seen. The radio does store the mode live: band block +4 bits 1..0 (0 off, 1 encode, 2 TSQ, 3 DCS), written at once by CAT and the front panel, in the active VFO's block (VFO A blocks from `0x7D`, VFO B from `0x203`, 26 bytes each). HamTRC does not read it because the active VFO cannot be read, and the block's frequency lags dial tuning, so it cannot be matched either. No DCS encode only: `TONE DCS` sets DCS with its squelch, `TONEDIR NEXT` on DCS says not available |
| Power on/off | implemented and verified |
| Bank 3 split keypad workflow | implemented and verified |
| Bank 3 current/other VFO frequency read workflow | implemented and verified |
| Bank 3 current/other VFO staged frequency entry | implemented and verified |
| Bank 3 `VFOA MODE` on `T3` | implemented and verified |
| Bank 3 sync `VFOA/VFOB` | implemented and verified |
| Bank 3 active `VFO A/B` selection on `T6` | implemented and verified |
| RX/TX state query (Bank 1 `1`, `RXTX?`) | verified: TX status bit 7 (0 = transmitting), `0xFF` in receive |
| Split status query | verified: EEPROM `0x7A` bit 7 in receive, following both CAT and the front panel; TX status bit 5 (1 = on) while transmitting |
| PO / ALC / SWR meters | verified at 5 W FM into a dummy load (PO 8, ALC 2, SWR 1.0): PO from TX status bits 3..0, ALC and SWR from the undocumented `BD` (bytes `82 08`), sent only while transmitting. With split on the radio transmits on the other VFO, so its mode decides whether there is power to read. MOD (`MOD?`, the low nibble of the second `BD` byte) is read the same way; not yet checked against the radio's MOD meter |
| RIT query and toggle (Bank 3 `5`, `RIT?`, `RIT TOGGLE`) | verified: like the FT-897, `05`/`85` switch RIT (a short press of CLAR), and answer `00` when they switched, `F0` when it was already in that state, also after a front panel change and with IF shift (the long press of CLAR) on, which they leave alone |
| Lock query (`LOCK?`, Bank 1 `3`) | verified: EEPROM `0x57` bit 6, inverted (0 = locked), following both the front panel key and the CAT lock |
| Noise blanker query (`NB?`, Bank 2 `2`) | verified: EEPROM `0x57` bit 5 (1 = on), following the front panel. There is no DSP, so `NR?` and `NOTCH?` say unsupported |
| AGC, BK, KYR and VOX (`AGC?`, `BK?`, `KYR?`, `VOX?`, Bank 2 `7` for AGC) and fast tuning and IF shift (`YSETTINGS?`) | verified, read only; addresses below |
| AGC write (`AGC AUTO/FAST/SLOW/OFF`, Bank 2 `7` long) | experimental, only with EXPERIMENTAL on; not yet tested on the radio. Writes `0x57` bits 1..0, keeps the other bits, and reads them back |
| VOX gain and delay (`VOXGAIN?`, `VOXDELAY?`) | verified, read only: follow menus 51 and 50 |
| Menu item and function row write (`MENU <n>`, `ROW <n>`, Bank 8 `8`/`7` long) | experimental, only with EXPERIMENTAL on; not yet tested on the radio. Writes `0x75`/`0x76` together (`BC`), keeps the bits that are not the item or row, and reads them back. Menu 1–52, or 1–57 with menu 52 EXTEND on; rows 1–12 (measured). The FT-818 is taken to match |
| Mic gain (`MICGAIN?`) | verified, read only: the one of the current mode, SSB, AM, FM and DIG matched their EEPROM bytes. In PKT the choice between the 1200 and 9600 baud level by the packet rate is not tested |
| Mic gain write (`MICGAIN <0..100>`, `MICGAIN STEP <+-n>`) | experimental, only with EXPERIMENTAL on; not yet tested on the radio. Writes bits 6..0 of the current mode's byte, as `MICGAIN?` picks it, keeps bit 7 (on `0x6B` menu 52 EXTEND), and reads it back |
| VOX, BK, KYR and VOX gain and delay write (`VOX`, `BK`, `KYR ON/OFF/TOGGLE`, `VOXGAIN <1..100>`, `VOXDELAY <100..2500>`, each also `STEP <+-n>` where it has a level) | experimental, only with EXPERIMENTAL on; not yet tested on the radio. Read-modify-write of `0x58` bits 7, 5 and 4, `0x63` bits 6..0 (bit 7 kept) and `0x64` bits 4..0 (bits 7..5 kept, the delay to the nearest 100 ms), each read back |
| Antenna jack write (`ANT FRONT/REAR/TOGGLE`) | experimental, only with EXPERIMENTAL on; not yet tested on the radio. Sets the `0x7A` bit of the band group of the current frequency, keeping the other groups and split (bit 7), and reads it back |
| IPO, ATT and NAR | not read: they sit in the per-band VFO blocks (IPO appeared at block +0 bit 5), which the radio saves only on a band change or power-off, so a read would lag the radio |
| VFO A/B commands (`VFOA?`, `VFOB?`, `VFOA MODE?`, `VFOB MODE?` and their sets) | verified: switch to the VFO, wait 120 ms (one retry for a query) and switch back to the VFO in use |

### Read from the EEPROM

Read with `BB`, on demand only. Addresses from the FT8x7Com FT817Setup project, not measured here except where noted.

| Address | Bits | Setting |
|---|---|---|
| `0x57` | 6 | lock, **inverted** (0 = locked), like the FT-857/897's `0x6A` bit 6 (measured; the [KA7OEI map](https://www.ka7oei.com/ft817_memmap.html) says 1 = on) |
| `0x57` | 7 | fast tuning, **inverted** (0 = on), like the FT-857/897 (measured; the map says 1 = on). Only the MH-31 microphone's FST key switches it |
| `0x57` | 5 | NB (measured, as in the KA7OEI map) |
| `0x57` | 4 | IF shift on (a long press of CLAR; the map's "PBT"), follows the front panel at once (measured). `YSETTINGS?` only |
| `0x57` | 1..0 | AGC: 00 auto, 01 fast, 10 slow, 11 off (measured, as in the map) |
| `0x58` | 7 | VOX (measured, as in the map) |
| `0x58` | 5 | BK (measured, as in the map) |
| `0x58` | 4 | KYR (measured, as in the map) |
| `0x58` | 2 | CW paddle reverse (menu 19), 1 = reverse (`KEYREV?`; KA7OEI map, not measured) |
| `0x5E` | 3..0 | CW pitch (menu 20), 300 + 50 × value Hz, 300..1000 (`CWPITCH?`; KA7OEI map, not measured) |
| `0x5F` | 4..0 | CW weight (menu 22), 1:(2.5 + 0.1 × value), 1:2.5..1:4.5 (`CWWEIGHT?`; KA7OEI map, not measured) |
| `0x60` | all | CW delay (menu 17), × 10 ms, 10..2500 ms (`CWDELAY?`; KA7OEI map, not measured) |
| `0x61` | 6..0 | CW side tone (menu 44), 0..100 (`SIDETONE?`; KA7OEI map, not measured) |
| `0x62` | 5..0 | CW speed (menu 21), value + 4 = WPM, 4..60 (`CWSPEED?`; KA7OEI map, not measured). Bits 7..6 are the charge time |
| `0x5D` | 2 | packet rate (menu 40), 1 = 9600 (KA7OEI map); picks the PKT mic gain |
| `0x63` | 6..0 | VOX gain (menu 51), 1..100 (measured, as in the KA7OEI map) |
| `0x64` | 4..0 | VOX delay (menu 50), × 100 ms, 100..2500 ms (measured: 300 ms is 3; the KA7OEI map's 0 = 100 ms is one step off) |
| `0x67` | 6..0 | SSB mic gain (menu 46), 0..100 (KA7OEI map) |
| `0x68` | 6..0 | AM mic gain (menu 5), 0..100 (KA7OEI map) |
| `0x69` | 6..0 | FM mic gain (menu 29), 0..100 (KA7OEI map) |
| `0x6A` | 6..0 | DIG mic gain (menu 25), 0..100 (KA7OEI map) |
| `0x6B` | 7 | EXTEND (menu 52), 1 = on: shows menu items 53–57 (measured) |
| `0x6B` | 6..0 | PKT mic gain at 1200 baud (menu 39), 0..100 (KA7OEI map) |
| `0x6C` | 6..0 | PKT mic gain at 9600 baud (menu 3), 0..100 (KA7OEI map) |
| `0x75` | 5..0 | menu item (`MENU?`, Bank 8 `8`), said as stored + 1 like the FT-857/897; last item 52, or 57 with EXTEND on (measured). Bits 7..6 unknown, kept on a write |
| `0x76` | 3..0 | function row (`ROW?`, Bank 8 `7`), said as stored + 1; 7 (row 8) is the NB/AGC row and 9 (row 10) VOX/BK/KYR, 11 (row 12) the last (measured). Saved at once when the row is stepped. Bits 7..4 unknown, kept on a write |
| `0x79` | 1..0 | TX power (`RFPOWER?`, Bank 1 `6`): High, L3, L2, L1; on an external supply FT-817 5, 2.5, 1, 0.5 W, FT-818 6, 5, 2.5, 1 W (not measured) |
| `0x7A` | 5..0 | antenna jack per band group (menu 07, `ANT?`, Bank 2 `0`), 1 = rear: bit 0 HF (below 33 MHz), 1 6 m (33–76 MHz), 2 FM broadcast (76–108), 3 air (108–137), 4 2 m (137–420), 5 UHF (from 420) (measured, as in the KA7OEI map) |
| `0x7A` | 7 | split (measured) |
| `0x7B` | 4, 3..0 | charging on, charge hours (not used) |
| band blocks | | 26 bytes per band from `0x7D` (160 m, 80 m, 40 m, ... on an FT-817 without 60 m); the frequency at +10..+13 in 10 Hz units, big-endian. Saved late, like on the FT-897 (measured) |

The active VFO cannot be read: `0x55` bit 0, which Hamlib's `get_vfo` and the KA7OEI map give, did not follow A/B from the front panel or CAT, not even across a power cycle. HamTRC tracks the VFO itself, and Bank 3 `4` syncs it after a front panel change.

### Implemented but still incomplete

| Function | Status |
|---|---|
| Hidden FT-817 background conditions around some documented CAT functions | practical tests suggest the documented commands can work correctly, but the exact conditions for stable behavior are still not fully mapped |
| Repeater and tone/DCS write paths outside normal VHF/UHF FM context | CAT bytes are implemented, but practical success is much more predictable when the radio is already on `2 m` or `70 cm` and already in `FM` |

### Experimental or incomplete

| Function | Status |
|---|---|
| Memory read/write raw path | experimental |
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
| RIT on/off (the CAT commands `05`/`85`) | verified on an FT-897: they switch RIT, the short press of the radio's CLAR key; IF shift (long press) has no CAT command |
| RIT query and toggle (Bank 3 `5`, `RIT?`, `RIT TOGGLE`) | verified on an FT-897, also after a front panel change. RIT on/off is not in the EEPROM, but the radio answers `05`/`85` with `00` when it switched and `F0` when RIT was already in that state. `RIT?` sends `85`: `F0` means off; `00` means it was on, and `05` switches it back at once. That brief switch hands the knob to RIT if IF shift had it |
| IF shift read (`IFSHIFT?`, printed, not spoken) | verified on an FT-897, EEPROM `0x6A` bit 4; read only. Earlier documented as a second RIT: the long press of CLAR is IF shift |
| RIT offset | implemented and verified |
| Repeater shift | implemented and verified |
| Repeater offset | implemented and verified |
| Tone/DCS mode | implemented and verified |
| CTCSS tone write | implemented and verified |
| DCS code write | implemented and verified |
| Extended CTCSS encode/decode modes | implemented and verified |
| Extended DCS encode/decode modes | implemented and verified |
| Tone mode query (`TONE?`, `TONEDIR?`, Bank 6 `2` and `5` short) | verified on an FT-897 for CAT changes: band block +4 of the band the active VFO is on (`0x6C`), see the [EEPROM map](#ft-857897-eeprom-map). Works outside the amateur bands too, in the general coverage block (set, stepped and read back on 26.8 MHz). The CAT tone command answers `00` whether or not the mode changed, so the reply tells nothing. Changes on the front panel are read at once too (checked through CTCSS and DCS, encode only and both, and off). On a memory channel or one being tuned (`0x68` bit 6) the tone is kept with the channel, not in the band block, so these and `SPLITTONE?` say "not available" (not yet tested on the radio) |
| Decode only and split tone (`TONEDIR NEXT`, `SPLITTONE?`, Bank 6 `5` long and `6` short) | verified on an FT-897: CAT decode only (CTCSS `3A`, DCS `0B`) works with split tone off. Split tone (menu 79) and the mixed modes set with the radio's A (encode) and B (decode) keys read back from band block +4; the keypad says "tone ctcss rx dcs tx", "tone split" and "split tone on" |
| S-meter read | implemented and verified |
| RX status raw read | implemented and verified |
| TX status raw read | implemented and verified |
| RX/TX state query | verified on an FT-897: TX status bit 7 (0 = transmitting), `0xFF` in receive |
| `TXFREQ?` fallback on keypad | implemented and practically usable |
| Bank 3 current/other VFO read | implemented and practically usable |
| Bank 3 current/other VFO frequency set | implemented and practically usable |
| Bank 3 `A/B` toggle | implemented and practically usable |

### Implemented but still worth more testing

| Function | Status |
|---|---|
| Split on/off | currently usable from keypad, but should still be cross-checked more broadly |
| Split status query | verified on an FT-897: EEPROM `0x8D` bit 7 in receive, TX status bit 5 while transmitting (1 = on; the manual says 0 = on, which is wrong) |
| Repeater and tone/DCS write paths outside normal FM repeater context | CAT bytes are implemented, but practical success can still depend on the radio already being in the appropriate VHF/UHF band and FM context |
| Active VFO read from the EEPROM (`0x68`) | verified on an FT-897: follows both the front panel A/B key and the CAT toggle; Bank 3 reads it before each VFO action (`get_vfo=1`) |
| Settings read from the EEPROM (Bank 2, Bank 8 `7`/`8`, `MENU?`, `ROW?`, `AGC?`, `IPO?`, `ATT?`, `NAR?`, `DBF?`, `BK?`, `KYR?`, `NR?`, `NB?`, `NOTCH?`, `NRLEVEL?`, `NBLEVEL?`, `BPF?`, `HPF?`, `LPF?`, `MICEQ?`, `PROC?`, `PROCLEVEL?`, `MICGAIN?`, `VOX?`, `VOXGAIN?`, `VOXDELAY?`, `RFPOWER?`) | verified on an FT-897, read only; addresses in the [EEPROM map](#ft-857897-eeprom-map). IPO, ATT, NAR and `RITOFFSET` come from the block of the band the active VFO is on (`0x6C`), also outside the amateur bands |
| Menu item and soft key row write (`MENU <n>`, `ROW <n>`, Bank 8 `8`/`7` long) | experimental, only with EXPERIMENTAL on; verified on an FT-897: the radio's menu opens on the item written and keeps the row, out-of-range numbers write nothing. Writes `0x88`/`0x89` together (`BC`), changing only the one byte, and reads them back. Menu 1–88, or 1–91 with menu 1 EXT MENU on; rows 1–17 (measured). The FT-857 is taken to match. The radio overwrites both when its menu is open and then exited |
| DSP settings write (`NR`, `NB`, `NOTCH`, `DBF ON/OFF/TOGGLE`, `NRLEVEL`, `NBLEVEL`, `BPF`, `HPF`, `LPF` with a value or `STEP`, Bank 2 `1`–`6` and their focus) | experimental, only with EXPERIMENTAL on; verified on an FT-897 by ear: the radio follows at once (`BPF` verified on an FT-897 over the console only: 60, 120 and 240 Hz each changed only bits 3..2 of `0x93`, steps go through the three widths and stop at the ends). Read-modify-write of the setting's bits (`BC`), refused while transmitting, read back. Ranges NR level 1–16, NB level 0–100, low cut 100–1000 Hz, high cut 1000–6000 Hz; a cut is set to the radio's nearest step. DBF sets or clears both bits, as the radio's key does in SSB and CW (measured). The FT-817 sets NB only |
| AGC write (`AGC AUTO/FAST/SLOW/OFF`, Bank 2 `7` long) | experimental, only with EXPERIMENTAL on; verified on an FT-897 by ear: the radio follows at once. Off clears `0xA8` bit 5 and keeps the speed; a speed writes `0x6A` bits 1..0, then sets `0xA8` bit 5. Each is read back |
| Mic gain, speech processor and TX equalizer write (`MICGAIN`, `PROCLEVEL` with a value or `STEP`, `PROC ON/OFF/TOGGLE`, `MICEQ OFF/LPF/HPF/BOTH`, Bank 4 `1` and `4`-`7`) | experimental, only with EXPERIMENTAL on; verified on an FT-897: the radio's menus 81 (SSB mic gain) and 74 continued from the values written, a menu 49 save kept the EQ written, and the PROC key switched PROC on from the off written. Read-modify-write as the DSP settings: the mic gain of the current mode (`0x7A`–`0x7D`, not in CW and PKT) and the PROC level (`0x9A`) 0–100 in bits 6..0, keeping bit 7 (set in `0x7A` on the radio tested), PROC `0xA9` bit 1, mic EQ `0x93` bits 1..0, keeping the NR level and BPF width. Only the SSB mic gain was tested |
| VOX, BK, KYR and VOX gain and delay write (`VOX`, `BK`, `KYR ON/OFF/TOGGLE`, `VOXGAIN <1..100>`, `VOXDELAY <100..3000>`, each also `STEP <+-n>` where it has a level; VOX on Bank 4 `2` and `9`) | experimental, only with EXPERIMENTAL on; not yet tested on the radio. Read-modify-write as the DSP settings: `0x6B` bits 7, 5 and 4, the VOX gain `0x76` bits 6..0 (bit 7 kept), the VOX delay `0x77` to the nearest 100 ms, each read back |
| CW menus (`CWSPEED?`, `CWPITCH?`, `CWDELAY?`, `CWWEIGHT?`, `SIDETONE?`, `QSK?`, `KEYREV?`, `CWAUTO?`, `PADDLE?`, `CWBFO?`, and with EXPERIMENTAL on the same names with a value, `STEP <+-n>`, `ON/OFF/TOGGLE`, `ELEKEY/MICKEY/TOGGLE` or `USB/LSB/AUTO`) | verified on an FT-897 over the console: each read matched the menu value set on the radio; each write changed only its bits (the delay both bytes, `CWDELAY 3000` = `2C 85`, `FULL` = `02`), steps stopped at the ends and stepped through FULL, out-of-range values were refused, and everything was restored. Addresses in the [EEPROM map](#ft-857897-eeprom-map). CW TRAINING (menu 31) not offered. The FT-817/818 has speed, pitch, delay, weight, side tone and key reverse, from the KA7OEI map, not yet tested |
| IPO, ATT, NAR and split tone write | not offered: they are kept in the band blocks, which the radio saves late from its own working state, so a write could be ignored or overwritten. IF shift stays read only too |
| RF power write (`RFPOWER <W>`, FT-817/818 `RFPOWER HIGH/L3/L2/L1`, Bank 1 `6` long) | experimental, only with EXPERIMENTAL on. FT-857/897: whole watts of menu 75 for the band group of the current frequency; verified on an FT-897 (its menu 75 continued from the value written). FT-817/818: the level at `0x79`; verified on an FT-817 by reading it back. TX output not measured |
| EEPROM write timing (`BC`) | the FT-817 answers a write with one byte after about 110 ms and ignores commands sent before it; HamTRC waits for it, and 120 ms on the FT-857/897. The read-back is tried twice, since the FT-817 missed the first one after a menu item write (measured) |
| Further settings read from the EEPROM (`YSETTINGS?`: VOX, PROC, lock, fast tuning, NB, BK, KYR, DSP row, IF shift, filter, SQL/RF knob, mic EQ, menu levels, the RIT offset) | protocol only, no keys except those in Bank 2; verified on an FT-897 against the values set on the radio |

### FT-857/897 EEPROM map

Read with the undocumented `BB` (2 bytes per read). Measured on an FT-897 unless marked yo3ggx (the [FT8x7EE map](https://www.yo3ggx.ro/ft8x7ee/eeprom.html), also confirmed on the FT-897). Method: one setting changed on the radio at a time, menu values set to both ends of their range, and the radio's menu exited before reading, since menu values are saved on exit.

Shared by all bands:

| Address | Bits | Setting |
|---|---|---|
| `0x68` | 0 | VFO, 1 = B; follows the front panel A/B key and the CAT toggle (`81`) |
| `0x6C` | 3..0 / 7..4 | the band VFO A / VFO B is on, numbered in block order below (13 = 2 m, 14 = 70 cm, 15 = general coverage, used outside the other bands); follows the band keys |
| `0x6A` | 7 | fast tuning, **inverted** (1 = off) |
| `0x6A` | 6 | lock, **inverted** (0 = locked); follows the front panel key and CAT lock |
| `0x6A` | 5 | NB (yo3ggx) |
| `0x6A` | 4 | IF shift on (long press of CLAR); saved at once, survives a power cycle. The IF shift offset does not survive one and is not stored anywhere in `0x000`–`0x5FF`, also not after a band change saved the band block (measured) |
| `0x6A` | 3 | the knob tunes IF shift (1) or RIT (0): set by switching IF shift, cleared by RIT going on, kept when RIT goes off. RIT on/off itself was not found |
| `0x6A` | 1..0 | AGC speed: 00 auto, 01 fast, 10 slow (measured; yo3ggx has 00 slow, 01 auto, 10 fast) |
| `0x6B` | 7 | VOX |
| `0x6B` | 5 | BK (yo3ggx) |
| `0x6B` | 4 | KYR (yo3ggx) |
| `0x6B` | 2 | CW KEY REV (menu 25), 1 = reverse |
| `0x71` | 2..0 | CW PITCH (menu 27), 400 + 100 × value Hz, 400..800 Hz. Bits 7..3 are other settings (bit 4 set on the radio tested) |
| `0x72` | 4..0 | CW WEIGHT (menu 32), 1:(2.5 + 0.1 × value), 1:2.5..1:4.5 |
| `0x73`, `0x74` | 7..0, 7 | CW DELAY (menu 24), 9 bits: `0x74` bit 7 is bit 8. 2 = FULL, 3..300 = 30..3000 ms in 10 ms steps (3000 ms = `0x12C`) |
| `0x74` | 6..0 | CW SIDE TONE (menu 29), 0..100 |
| `0x75` | 5..0 | CW speed (menu 30), value + 4 = WPM, 4..60 |
| `0x76` | 6..0 | VOX gain (menu 88), 1..100 |
| `0x77` | all | VOX delay (menu 87), × 100 ms, 100..3000 ms |
| `0x72` | 7 | SQL/RF GAIN (menu 80): 1 = SQL, 0 = RF gain |
| `0x7A` | 6..0 | SSB mic gain (menu 81), 0..100 |
| `0x7B` | 6..0 | AM mic gain (menu 5), 0..100 |
| `0x7B` | 7 | CW PADDLE (menu 26): 0 = ELEKEY, 1 = MICKEY |
| `0x7C` | all | FM mic gain (menu 51), 0..100 |
| `0x7D` | all | DIG gain (menu 37), 0..100 |
| `0x7E` | all | PKT 1200 level (menu 71), 0..100 |
| `0x7F` | all | PKT 9600 level (menu 72), 0..100 |
| `0x88` | all | menu item, from 0, saved when the menu is exited; last item 88, or 91 with EXT MENU on |
| `0x89` | all | soft key row, from 0, saved when the menu is exited; 17 rows |
| `0x8D` | 7 | split (Hamlib) |
| `0x90` | all | DIG VOX (menu 40), 0..100 |
| `0x91` | 7 | EXT MENU (menu 1), 1 = on: shows menu items 89–91 |
| `0x92` | 7 | CW AUTO MODE (menu 22), 1 = on |
| `0x92` | 6 | changed with CW TRAINING (menu 31); the menu's values not measured |
| `0x92` | 4..2 | CW QSK (menu 28), 10 + 5 × value ms, 10..30 ms |
| `0x92` | 1..0 | CW BFO (menu 23): 0 = USB, 1 = LSB, 2 = AUTO |
| `0x93` | 7..4 | DSP NR level (menu 49), value + 1 = 1..16 |
| `0x93` | 3..2 | DSP BPF width (menu 45): 0 = 60, 1 = 120, 2 = 240 Hz (1 not seen from the radio's menu, but written and read back by `BPF 120`; `BPF?`) |
| `0x93` | 1..0 | DSP MIC EQ (menu 48): 0 = off, 1 = LPF, 2 = HPF, 3 = both |
| `0x94` | 4..0 | DSP LPF cutoff (menu 47), 0..31 = 1000..6000 Hz (measured at the ends and 11 = 2770 Hz) |
| `0x95` | 3..0 | DSP HPF cutoff (menu 46), 100 + 60 × value Hz, 100..1000 Hz |
| `0x99` | all | NB level (menu 63), 0..100 |
| `0x9A` | all | PROC level (menu 74), 0..100 |
| `0x9B` | 7..0 | RF power (menu 75), HF: bits 6..0 watts, 5..100 (yo3ggx); bit 7 set from 20 W up (measured on HF and UHF) |
| `0xA7` | 7 | filter 2 selected on the CFIL row; clear = built-in. Filter 1 not measured (no filter fitted). Bit 1 was set when the built-in filter was chosen in CW, not in USB |
| `0xA8` | 7 | DSP soft key row shown, saved at once |
| `0xA8` | 5 | AGC on (yo3ggx) |
| `0xA8` | 3..2 | DBF: the radio's key sets or clears both bits, in LSB and CW (measured) |
| `0xA8` | 1 | DNR (yo3ggx) |
| `0xA8` | 0 | DNF (yo3ggx) |
| `0xA9` | 1 | PROC |
| `0xAA` / `0xAB` / `0xAC` | 7..0 | RF power (menu 75) for 6 m / VHF / UHF, as `0x9B`: 6 m 5..100, VHF 5..50, UHF 2..20 W (yo3ggx) |

Per band and VFO, a 28-byte block. VFO A blocks: 160 m `0xBA`, 80 m `0xD6`, 5 MHz `0xF2` (not `0x260` as in the yo3ggx map), 40 m `0x10E`, 30 m `0x12A`, 20 m `0x146`, 17 m `0x162`, 15 m `0x17E`, 12 m `0x19A`, 10 m `0x1B6`, 6 m `0x1D2`, FM broadcast `0x1EE`, air band `0x20A`, 2 m `0x226`, 70 cm `0x242`, general coverage `0x25E`. VFO B blocks are `0x1C0` higher.

| Offset | Bits | Setting |
|---|---|---|
| +1 | 3 | FM narrow (NAR) |
| +2 | 5 | IPO |
| +2 | 4 | ATT |
| +4 | 7 | unknown; not RIT (set in blocks with a zero RIT offset too). Saved with the band's working state |
| +4 | 3..0 | tone mode, saved as soon as it changes: bits 3..2 what is sent, bits 1..0 what the squelch waits for, each 01 = CTCSS, 10 = DCS (CTCSS 5, CTCSS encode 4, CTCSS decode 1, DCS A, DCS encode 8, DCS decode 2, off 0). Measured by setting each CAT tone mode on 2 m FM; front panel changes show the same. Mixed modes with split tone: DCS sent, CTCSS awaited 9; CTCSS sent, DCS awaited 6. `TONE?`, `TONEDIR?`, Bank 6 `2` and `5` |
| +4 | 4 | split tone (menu 79), per band, saved on leaving the menu (2 m and 10 m kept separate values). Switching it off keeps a mixed mode in bits 3..0. `SPLITTONE?`, Bank 6 `6` |
| +6, +7 | all | CTCSS tone, as its index in the radio's tone list (0 = 67.0 Hz, 8 = 88.5 Hz, 10 = 94.8 Hz); both bytes held the same value. Written at once by the CAT tone command, no band change needed. Measured in the general coverage block on 26.935 MHz FM; +7 may be the decode tone with split tone, not checked |
| +10..+11 | all | the RIT offset, signed, 10 Hz units, big-endian (`FF AA` = −860 Hz), measured on 20 m against the frequency the radio reports with RIT on. Kept when RIT is switched off. Current only after the radio saves the block: a band change saves it, turning the knob or a short press of CLAR does not. The IF shift offset is not here. `YSETTINGS?` shows it as `RITOFFSET` |
| +12..+15 | all | frequency, 10 Hz units, big-endian |

The radio saves a band block on events such as key presses, not while the dial turns, so the block can lag the current frequency. A CAT frequency jump into another band carries the current working state there: that band's block is saved with the mode bytes (+3, +6, +7), +4 and the RIT offset of the band that was left. The band keys work differently: the RIT offset is kept per band, and a band key brings back the offset saved for that band. Seen changing without a known cause: `0x6D`, and `0xA9` bit 7 (set together with DIG VOX 100).

### Experimental or incomplete

| Function | Status |
|---|---|
| Memory read/write raw path | experimental |
| PO / ALC / SWR meters | verified on an FT-897 into a dummy load (PO 10, ALC 8, SWR 1.0): PO from TX status bits 3..0, ALC and SWR from the undocumented `BD`, sent only while transmitting since the radio does not answer it in receive. High-SWR flag (TX status bit 6) verified: "swr high" is said when the radio flags it. MOD (`MOD?`, the low nibble of the second `BD` byte) is read the same way; not yet checked against the radio's MOD meter |
| Volume / SQL extras | not cleanly validated |

## FT-897

### Current handling

| Function group | Status |
|---|---|
| Model | grouped with the FT-857 |
| Documented CAT assumptions | treated the same as FT-857 |
| Practical device verification | not yet available |

## Notes

- EEPROM reads (`BB`) must stay few and on demand. An FT-897 hung, together with HamTRC, when a test logger read 192 bytes every 5 s while the dial was being turned; a power cycle recovered it. `YEEPROM?` reads at most 32 bytes. The firmware never sends the factory reset (`BE`) opcode, and sends the write (`BC`, `<addr hi> <addr lo> <byte> <next byte> BC`, the format Hamlib uses; it takes effect at once) only for the console command `YEEPROM!`. The command takes an address and the bytes for it and the next address, e.g. `YEEPROM! 0068 1F 00`; odd addresses work (`BB` at `0069` returned the bytes at `0069` and `006A` on an FT-897), and is refused while transmitting. **Use it with caution:** a wrong address or value can wipe the radio's memories and calibration. While mapping, `BC` was used only to restore changed menu values. For mapping, keep one console connection open for the whole session: each new USB connection restarts HamTRC and can leave its keypad dead.

- The keypad layout for this family is documented separately in [yaesu-ft8x7-keypad.md](./yaesu-ft8x7-keypad.md).
- Repeater and tone functions are now concentrated in Bank 6.
- Profile selection in the current firmware supports up to 24 slots. On the keypad this means Bank 9 `1` long, then one or two digits, then `Enter`.
- For FT8x7 repeater work, `FM` on the intended `VHF/UHF` band should be treated as a practical precondition, not just a recommendation.
- For FT-857/897, only functions that behaved well in practical testing should be considered product-ready.
- The recent FT-857/897 Bank 3 work was intentionally kept separate from the FT-817 branch logic; FT-817 and FT-857/897 now have different keypad handling where that matches real device behavior better.
