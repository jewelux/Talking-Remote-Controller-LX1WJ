# FT8x7 Keypad Layout

This document describes the current keypad layout for the Yaesu FT-817, FT-857, and FT-897 family.
It lists the keys these radios have of their own; [the keypad map](../keypad-map.md) has every
bank, Bank 5 and 7 included, for all radios.

> The layout is experimental and may still change.

The intention is:

- keep the overall bank structure consistent across the FT8x7 family
- only expose functions that are documented and practically usable
- move repeater and tone functions into a dedicated Bank 6
- Bank 6 now favors direct manual entry for repeater offset, CTCSS, and DCS, with only a small set of built-in defaults

Important practical note:

- Some FT8x7 CAT write functions are context-sensitive on the radio side. The firmware sends the documented CAT bytes, but the radio may only apply them cleanly when the current band and mode fit the function. In practical repeater use this usually means being on the appropriate VHF/UHF band and already in FM. For example, common repeater shift workflows are much more predictable when the radio is already on 2 m FM.
- For FT8x7 repeater and tone work, treat `2 m` or `70 cm` plus `FM` as the expected operating context. If the radio is on another band or not already in `FM`, `RPT`, `RPTSHIFT`, `CTCSS`, and `DCS` writes may appear inconsistent even though the firmware is sending the documented CAT sequence.
- Recent FT-817 testing also suggests that documented CAT commands can work correctly once the radio is in the right background state, but that state is not yet characterized well enough to call the whole path fully stable. More testing is still needed around those hidden preconditions.

## Bank 1 - Status

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `FREQ?` | `FREQ?` |
| `0` long | `FREQ <MHz>`, then digits, then `Enter` | `FREQ <MHz>`, then digits, then `Enter` |
| `0` double click | `ROUND 500` | `ROUND 500` |
| `1` short | `RXTX?` | `RXTX?` |
| `2` short | `TXFREQ?`; falls back to `FREQ?` when the radio gives no TX frequency | `FREQ?` when split is known to be off, otherwise not available |
| `3` short | `LOCK?` (read from the radio) | `LOCK?` (read from the radio) |
| `3` long | `LOCK ON/OFF` | `LOCK ON/OFF` |
| `4` short | `PO?` | `PO?` |
| `6` short | `RFPOWER?`: the TX power level | `RFPOWER?`: menu 75 power of the current band |
| `6` long | experimental: `RFPOWER <level>`, `1` L1, `2` L2, `3` L3 or `4` High, then `Enter` | experimental: `RFPOWER <W>`, the watts, then `Enter` |
| `7` short | `SM?` | `SM?` |
| `8` short | `SWR?` | `SWR?` |
| `9` short | `MODE?` | `MODE?` |
| `9` long | `MODE <n>`, then digit, then `Enter` | `MODE <n>`, then digit, then `Enter` |

## Bank 2 - Radio Settings

All Bank 2 keys read settings from the radio's EEPROM, since CAT cannot. With experimental mode on, some also change them by writing the EEPROM: a long press on `1`, `2`, `3` and `5` switches the setting (FT-817: `2` only), and on the FT-857/897 a long press on `4` and `6` opens the low or high cut entry. The NR level, NB level and CW filter width are a double press of `1`, `2` and `5`; a press then a hold opens their entry. `7` long picks AGC: `0` off, `1` auto, `2` fast, `3` slow, then `Enter`. On the FT-817, `0` long switches the antenna jack. Without experimental mode a long press says "experimental".

`A`, `B` and `C` act on the last of these keys used (see the [keypad map](../keypad-map.md)): `B` and `C` read the value once and open its entry one step away, ending in "please"; more `B` and `C` presses step with no radio traffic, and `Enter` writes it. They need experimental mode too.

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `ANT?`: antenna jack of the current band (menu 07) | free (auto notch on `3`) |
| `1` short | not available (no DSP) | `NR?` (DSP noise reduction) |
| `1` double click | free | `NRLEVEL?` (menu 49, 1–16) |
| `2` short | `NB?` | `NB?` |
| `2` double click | free | `NBLEVEL?` (menu 63, 0–100) |
| `3` short | not available (no DSP) | `NOTCH?` (DSP auto notch) |
| `4` short | not available (no DSP) | `HPF?`: low cut (menu 46, DSP HPF cutoff) |
| `5` short | not available (no DSP) | `DBF?` (DSP bandpass filter) |
| `5` double click | free | `BPF?`: CW filter width (menu 45, DSP bandpass width, 60, 120 or 240 Hz) |
| `6` short | not available (no DSP) | `LPF?`: high cut (menu 47, DSP LPF cutoff) |
| `7` short | `AGC?` | `AGC?` |
| `7` long | `AGC` select (experimental) | `AGC` select (experimental) |
| `8` short | not available | `IPO?`: preamplifier, on when IPO is off (HF and 6 m) |
| `9` short | not available | `ATT?`: attenuator (HF and 6 m) |

The TX equalizer (`MICEQ?`, menu 48) is on Bank 4 `4`-`6`.

IPO and ATT are those of the current band and VFO, also outside the amateur bands (HF and 6 m only). The menu settings are read as the radio last saved them. The CW settings are on Bank 7. FM narrow has no key; the console command `NAR?` reads it. On the FT-857/897, `PROC?` and `PROCLEVEL?` read the speech processor and its level (menu 74). `MICGAIN?` reads the mic gain of the current mode: on the FT-857/897 SSB menu 81, AM menu 5, FM menu 51 or DIG gain menu 37, not in CW and PKT; on the FT-817 SSB menu 46, AM menu 5, FM menu 29, DIG menu 25 or PKT menu 39 or 3 by the packet rate, not in CW. `VOX?`, `VOXGAIN?` and `VOXDELAY?` read VOX, its gain and its delay (FT-857/897 menus 88 and 87, FT-817 menus 51 and 50). These are on Bank 4.

## Bank 3 - VFO / Split

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `SPLIT?` | `SPLIT?` |
| `0` long | `SPLIT ON/OFF` | `SPLIT ON/OFF` |
| `0` double click | `TXFREQ?` | `TXFREQ?`: `FREQ?` when split is off, otherwise not available |
| `1` short | current `VFOA/VFOB?` | current `VFOA/VFOB?` |
| `1` long | `A/B` | `A/B` |
| `1` double click | current `VFOA/VFOB <MHz>`, then digits, then `Enter` | current `VFOA/VFOB <MHz>`, then digits, then `Enter` |
| `2` short | other `VFOA/VFOB?` | other `VFOA/VFOB?` |
| `2` long | `A=B` (copy the active VFO to the other) | not available |
| `2` double click | other `VFOA/VFOB <MHz>`, then digits, then `Enter` | other `VFOA/VFOB <MHz>`, then digits, then `Enter` |
| `3` short | `VFOA MODE?` | — |
| `3` long | `VFOA MODE <n>`, then digit, then `Enter` | — |
| `4` short | `SYNC VFOA` | — |
| `4` long | `SYNC VFOB` | — |
| `5` short | `RIT?` | `RIT?` |
| `5` long | `RIT TOGGLE` | `RIT TOGGLE` |
| `6` short | active `VFO A` | `PTT OFF` (RX) |
| `6` long | active `VFO B` | `PTT ON` (TX) |

FT-817 Bank 3 note:

- The FT-817 branch currently mixes a tracked `current/other VFO` workflow with explicit `SYNC VFOA/VFOB` and explicit active-`VFO A/B` selection.
- `1`/`2` double click lead into staged frequency entry for the tracked current/other VFO, `1` long toggles `A/B` and `2` long copies the active VFO to the other (`A=B`, also the console command `VFO A=B`), and `3` handles `VFOA MODE`, then returns to the VFO in use. VFO B's mode: `1` long, then Bank 1 `9`.
- The FT-817 cannot report its active VFO, so `4` sync is still important after any unknown front-panel A/B change.
- `5` works as on the FT-857/897: RIT is the short press of the radio's CLAR key; IF shift (the long press) is left alone. If RIT is on, asking switches it off and straight back on.

FT-857/897 Bank 3 note:

- `5` short switches RIT off and straight back on when RIT is on, since the radio cannot report RIT otherwise. If the knob was tuning IF shift at that moment, it tunes RIT afterwards (EEPROM `0x6A` bit 3). `5` long does not do this: it moves the knob to RIT only when it switches RIT on.

## Bank 4 - Phone Transmit

The FT-8x7 keeps these in its EEPROM: a short press reads, a hold sets with experimental mode on.
`B` and `C` open the entry one step away, and `D` writes it.

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | not available | not available |
| `1` short | not available | `PROC?`: speech processor |
| `1` long | free | processor on/off (experimental) |
| `1` double | free | `PROCLEVEL?`: processor level (menu 74) |
| `1` short, then hold | free | processor level entry (experimental) |
| `2` short | `VOX?` | `VOX?` |
| `2` long | VOX on/off (experimental) | VOX on/off (experimental) |
| `2` double | `VOXGAIN?`: VOX gain (menu 51) | `VOXGAIN?`: VOX gain (menu 88) |
| `2` short, then hold | VOX gain entry (experimental) | VOX gain entry (experimental) |
| `4` short | not available | TX low cut: the TX equalizer's HPF (menu 48) |
| `4` long | free | TX low cut on/off (experimental) |
| `5` short | not available | `MICEQ?`: TX equalizer (menu 48) |
| `5` long | free | TX equalizer select (experimental) |
| `6` short | not available | TX high cut: the TX equalizer's LPF (menu 48) |
| `6` long | free | TX high cut on/off (experimental) |
| `7` short | `MICGAIN?`: mic gain of the mode | `MICGAIN?`: mic gain of the mode |
| `7` long | mic gain entry (experimental) | mic gain entry (experimental) |
| `8` short | `RFPOWER?` | `RFPOWER?` |
| `8` long | RF power level select (experimental) | RF power entry (experimental) |
| `9` short | `VOXDELAY?`: VOX delay (menu 50) | `VOXDELAY?`: VOX delay (menu 87) |
| `9` long | VOX delay entry (experimental) | VOX delay entry (experimental) |

The mic gain is that of the mode in use when the key is pressed; an entry sets that mode's gain
even if the mode changes before `D`. `4` and `6` switch one filter of the TX equalizer and keep
the other.

## Bank 6 - Repeater / Tone

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | `RPT OFF` | `RPT OFF` |
| `0` long | `RPT MINUS` | `RPT MINUS` |
| `0` double click | `RPT PLUS` | `RPT PLUS` |
| `1` short | `RPTSHIFT` preset 1 (`rpt_offset_1`, default `0.600`) | `RPTSHIFT` preset 1 (`rpt_offset_1`, default `0.600`) |
| `1` long | `RPTSHIFT` preset 2 (`rpt_offset_2`, default `7.600`) | `RPTSHIFT` preset 2 (`rpt_offset_2`, default `7.600`) |
| `1` double click | `RPTSHIFT <kHz>`, then digits, then `Enter` | `RPTSHIFT <kHz>`, then digits, then `Enter` |
| `2` short | `TONE?`: the mode last set from HamTRC | `TONE?` (read from the radio) |
| `2` long | `TONE CTCSS` | `TONE CTCSS` |
| `2` double click | `TONE OFF` | `TONE OFF` |
| `2` short, then hold | `TONE DCS` | `TONE DCS` |
| `3` short | `CTCSS?` (speak only) | `CTCSS?` (speak only) |
| `3` long | `CTCSS <tone>`, then digits, then `Enter` | `CTCSS <tone>`, then digits, then `Enter` |
| `4` short | `DCS?` (speak only) | `DCS?` (speak only) |
| `4` long | `DCS <code>`, then digits, then `Enter` | `DCS <code>`, then digits, then `Enter` |
| `5` short | `TONEDIR?`: the mode last set from HamTRC | `TONEDIR?` (read from the radio) |
| `5` long | `TONEDIR NEXT` (CTCSS only) | `TONEDIR NEXT` |
| `6` short | not available | `SPLITTONE?` (read from the radio) |

Bank 6 tone handling:

- `2` short = tone type: off, CTCSS or DCS
- `2` long = CTCSS, `2` short then hold = DCS, `2` double click = off
- `5` short = TX (encode only: the tone is sent, the squelch does not wait for it), BOTH (encode
  and decode, tone squelch) or SPLIT (one type sent, the other awaited). RX (decode only) is said
  when CAT left it set; the FT-897 stores it but its squelch stays open, so HamTRC no longer sets it
- `5` long = switches between BOTH and TX, keeping the type; from SPLIT it goes to TX with the sent
  type, from RX to BOTH; with the tone off it changes nothing
- switching the type keeps TX or BOTH; from SPLIT it sets BOTH; from off it goes back to the
  direction the tone had before it was switched off, also after HamTRC restarts
- `6` short = split tone (menu 79), "split tone on" or "split tone off". With it on, the radio's
  own keys can send one type and wait for the other; `2` short then says the received type first,
  e.g. "tone ctcss rx dcs tx". The radio keeps split tone per band, and a mixed mode stays when
  split tone is switched off. CAT cannot switch it or set a mixed mode

The FT-857/897 reads the tone mode of the current band and VFO from its EEPROM, also after a
change on the radio. On a memory channel the tone is kept with the channel, where it cannot be
read, so there `2`, `5` and `6` short and `5` long say "not available". The FT-817 keeps it where
it cannot be read live, so there `2` and `5` short say the mode last set from HamTRC on the
current band, and "not available" before one was set or on another band. The FT-817 has no DCS
encode only: DCS is always BOTH there.

`3` short and `4` short do not read the radio: they speak the last CTCSS tone or DCS code set from the keypad, or the built-in default (`88.5`, `023`) when nothing has been set yet.

After `3` or `4`, `B` and `C` walk the CTCSS tones or DCS codes from that one and set each at
once (held: 5 at a time); these are CAT commands, so no experimental mode and no `D`.

## Bank 7 - CW

As Bank 4: a short press reads, a hold sets with experimental mode on. `B` and `C` open the entry
one step away, and `D` writes it. The FT-817/818 addresses are from the KA7OEI map, not yet tested.

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `0` short | not available | not available |
| `1` short | `KYR?`: keyer | `KYR?`: keyer |
| `1` long | keyer on/off (experimental) | keyer on/off (experimental) |
| `1` double | `CWSPEED?`: keyer speed (menu 21) | `CWSPEED?`: keyer speed (menu 30) |
| `1` short, then hold | keyer speed entry (experimental) | keyer speed entry (experimental) |
| `2` short | `BK?`: break-in | `BK?`: break-in |
| `2` long | break-in on/off (experimental) | break-in on/off (experimental) |
| `2` double | free | `QSK?`: QSK time (menu 28) |
| `2` short, then hold | free | QSK time entry (experimental) |
| `3` short | `SIDETONE?`: sidetone level (menu 44) | `SIDETONE?`: sidetone level (menu 29) |
| `3` long | sidetone entry (experimental) | sidetone entry (experimental) |
| `4` short | not available | `PADDLE?`: keyer type ELEKEY or MICKEY (menu 26) |
| `4` long | free | ELEKEY/MICKEY (experimental) |
| `5` short | `CWPITCH?`: CW pitch (menu 20) | `CWPITCH?`: CW pitch (menu 27) |
| `5` long | CW pitch entry (experimental) | CW pitch entry (experimental) |
| `6` short | `CWWEIGHT?`: CW weight (menu 22) | `CWWEIGHT?`: CW weight (menu 32) |
| `6` long | CW weight entry, `*` as the point (experimental) | CW weight entry, `*` as the point (experimental) |
| `7` short | `KEYREV?`: paddle reverse (menu 19) | `KEYREV?`: paddle reverse (menu 25) |
| `7` long | paddle reverse on/off (experimental) | paddle reverse on/off (experimental) |
| `8` short | `RFPOWER?` | `RFPOWER?` |
| `8` long | RF power level select (experimental) | RF power entry (experimental) |
| `9` short | `CWDELAY?`: break-in delay | `CWDELAY?`: break-in delay (menu 24) |
| `9` long | break-in delay entry (experimental) | break-in delay entry (experimental) |

`B` and `C` step the speed by 1 WPM, the pitch by the menu's step, the sidetone by 5, the weight by
0.1, the QSK time by 5 ms and the delay by 100 ms. On the FT-857/897 the step below the shortest
delay is FULL, and `0` typed in the delay entry sets FULL.

## Bank 8 - Radio Menu

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `2` short | `BAUD?` | `BAUD?` |
| `2` long | baud rate pick: `1` 4800, `2` 9600, `3` 38400, then `D` | baud rate pick: `1` 4800, `2` 9600, `3` 38400, then `D` |
| `7` short | `ROW?` (function row) | `ROW?` (soft key row) |
| `7` long | experimental: `ROW <n>`, then digits, then `Enter` (1–12) | experimental: `ROW <n>`, then digits, then `Enter` (1–17) |
| `8` short | `MENU?` (last menu item) | `MENU?` (last menu item) |
| `8` long | experimental: `MENU <n>`, then digits, then `Enter` (1–52, 1–57 with menu 52 EXTEND on) | experimental: `MENU <n>`, then digits, then `Enter` (1–88, 1–91 with menu 1 EXT MENU on) |
| `A` `B` `C` | focus: baud rate (4800, 9600, 38400), or row / menu item after `7` / `8` | focus: baud rate, or row / menu item after `7` / `8` |

`MENU?` and `ROW?` are saved only when the radio's menu is exited. Set the same baud rate in the
radio's CAT RATE menu.

`7` long and `8` long are experimental and work only with experimental mode on (Bank 9 `0` long);
otherwise they say "experimental". They write the radio's EEPROM, where it keeps the menu item
and row, and say the value read back, e.g. "menu 12". A number out of range says "error" and
nothing is written. Leave the radio's menu closed while setting them: the radio saves its own
values when the menu is exited.

## Bank 9 - Profile / System

| Key | FT-817 | FT-857/897 |
|---|---|---|
| `A` short | `PROFILE?` | `PROFILE?` |
| `A` long | `PROFILE SELECT` (slot number, one or two digits, then `Enter`) | `PROFILE SELECT` (slot number, one or two digits, then `Enter`) |
| `A` double | `PROFILE RESET` | `PROFILE RESET` |
| `B` short | `PROFILE NEXT` | `PROFILE NEXT` |
| `C` short | `PROFILE PREV` | `PROFILE PREV` |
| `0` short | `EXPERIMENTAL?` | `EXPERIMENTAL?` |
| `0` long | `EXPERIMENTAL TOGGLE` | `EXPERIMENTAL TOGGLE` |
| `1` short | `PROFILE?` | `PROFILE?` |
| `1` long | `PROFILE SELECT` (slot number, one or two digits, then `Enter`) | `PROFILE SELECT` (slot number, one or two digits, then `Enter`) |
| `1` double | `PROFILE RESET` | `PROFILE RESET` |
| `4` short | `TUNINGSPEECH?` | `TUNINGSPEECH?` |
| `4` long | `TUNINGSPEECH TOGGLE` | `TUNINGSPEECH TOGGLE` |
| `5` short | `VERBOSE?` | `VERBOSE?` |
| `5` long | `VERBOSE TOGGLE` | `VERBOSE TOGGLE` |
| `7` short | `VOLUME?` | `VOLUME?` |
| `7` long | volume entry | volume entry |
| `8` short | `SPEED?` | `SPEED?` |
| `8` long | next speed (slow, normal, fast) | next speed (slow, normal, fast) |

`A`, `B` and `C` act on the profile until `6`-`9` move the focus to the speech speed or the volume;
`1` brings it back to the profile.

## Known Limits

| Function | FT-817 | FT-857/897 |
|---|---|---|
| `Bank 6 repeater/tone writes` | expect best results only on `2 m` or `70 cm` and already in `FM`; other contexts can make valid CAT writes look unreliable | expect best results only on the intended `VHF/UHF` band and already in `FM`; other contexts can make valid CAT writes look unreliable |
| `RIT OFF` | replaced by `RIT?` / `RIT TOGGLE`; the CAT commands `05`/`85` switch RIT | replaced by `RIT?` / `RIT TOGGLE`; the CAT commands `05`/`85` switch RIT |
| `VFO A/B tracking` | usable with sync support; the radio has no readable VFO | read from the EEPROM |
| `manual front-panel A/B changes` | resync recommended | followed; no sync needed |
| FT-817 hidden background conditions | documented CAT commands can work well, but some success still appears to depend on not-yet-characterized radio state; more testing is needed | not the main current concern |
| `Bank 2 settings` | only antenna, NB and AGC (no DSP, IPO/ATT not read); NB and AGC can be set experimentally | DSP and AGC can be set experimentally, the TX equalizer from the console; IPO and ATT read only |
| `MEM READ/WRITE` | experimental | experimental |
| `VOL/SQL` | not cleanly validated | not cleanly validated |

## Notes

- `Enter` refers to the keypad confirmation key `D`.
- In frequency entry `*` is the decimal point: `145*500`, then `Enter`, tunes to 145.500 MHz.
- `double click` refers to a quick second press of the same key; a key with no double-click action runs its short action at once.
- `*` short speaks the current bank; `*` long, then a digit, selects a bank.
- `—` marks a key with no function on that radio.
