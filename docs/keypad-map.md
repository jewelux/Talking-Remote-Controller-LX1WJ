# Keypad Map

What every bank key does on each radio. One table per bank; each row is a key and a press, each
column a group of radios.

Columns:

- **Icom**: CI-V radios (IC-7300, IC-705, IC-706 and others)
- **FTDX10**: the FTDX10 family profile; most keys run a console command
- **Kenwood, Elecraft, other FTDX**: TS-480, KX2, KX3, FT-891 and other Yaesu FTDX radios
- **FT-817/818** and **FT-857/897**: the Yaesu FT-8x7 radios

The NanoVNA profile is not a radio and has a layout of its own; see
[NanoVNA](#nanovna-profile-30) at the end.

Besides the functions, the tables say:

- **free**: the key has no action on this bank for this radio; pressing it beeps
- **not available**: the key has an action, but the radio or profile cannot do it, and it says
  "not available". With verbose on the function is named first, e.g. "tuner not available"

A function that fails says "timeout" when the radio did not answer, and "error" when the radio
refused or the value typed is invalid.

Presses: **short**, **long** (hold), **double** (a quick second press). A key with no double
action answers at once on a short press. "Entry" means digits follow, then `D` to confirm; `#`
cancels. `*` (bank) and `D`/`#` work the same on every radio and every bank; see the
[user guide](../user-guide.md).

In the Kenwood column some keys depend on the profile: when the profile has no command for them
they say "not available". This is noted where it is always the case.

## Bank 1 - Status

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `0` short | frequency | frequency | frequency | frequency | frequency |
| `0` long | frequency entry | frequency entry | frequency entry | frequency entry | frequency entry |
| `0` double | round to 500 Hz | round to 500 Hz | round to 500 Hz | round to 500 Hz | round to 500 Hz |
| `1` short | RX or TX | RX or TX | RX or TX; Kenwood and Elecraft: not available | RX or TX | RX or TX |
| `2` short | TX frequency | TX frequency | TX frequency; Kenwood and Elecraft: not available | the frequency (the radio gives no TX frequency) | the frequency when split is off, otherwise not available |
| `3` short | lock | lock | lock | lock | lock |
| `3` long | lock on/off | lock on/off | lock on/off | lock on/off | lock on/off |
| `4` short | output power | output power | output power | output power | output power |
| `5` short | free | tuner | free | free | free |
| `5` long | free | tuner on/off | free | free | free |
| `5` double | free | tune | free | free | free |
| `6` short | RF power setting (IC-706: free) | preamp | free | RF power setting | RF power setting |
| `6` long | RF power entry | preamp on/off | free | RF power level select (experimental) | RF power entry (experimental) |
| `7` short | S-meter | S-meter | S-meter | S-meter | S-meter |
| `8` short | SWR | SWR | SWR | SWR | SWR |
| `9` short | mode | mode | mode | mode | mode |
| `9` long | mode select | mode select | mode select | mode select | mode select |

## Bank 2 - Receiver

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `0` short | free | free | free | antenna jack of the band | free |
| `1` short | noise reduction | noise reduction | noise reduction | not available | noise reduction |
| `1` long | noise reduction on/off | noise reduction on/off | noise reduction on/off | not available | noise reduction on/off (experimental) |
| `2` short | noise blanker | noise blanker | noise blanker | noise blanker | noise blanker |
| `2` long | noise blanker on/off | noise blanker on/off | noise blanker on/off | noise blanker on/off (experimental) | noise blanker on/off (experimental) |
| `3` short | notch | notch | notch | not available | notch |
| `3` long | notch on/off | notch on/off | notch on/off | not available | notch on/off (experimental) |
| `4` short | noise reduction level | AGC | free | not available | noise reduction level |
| `4` long | NR level up 10 % | AGC fast | free | free | NR level up 1 (experimental) |
| `4` double | NR level down 10 % | AGC slow | free | free | NR level down 1 (experimental) |
| `5` short | noise blanker level | radio power state | free | not available | noise blanker level |
| `5` long | NB level up 10 % | radio power off | free | free | NB level up 10 (experimental) |
| `5` double | NB level down 10 % | radio power on | free | free | NB level down 10 (experimental) |
| `6` short | PBT inner | radio status line | free | not available | DSP bandpass filter |
| `6` long | PBT inner up | free | free | free | DSP bandpass filter on/off (experimental) |
| `6` double | PBT inner down | free | free | free | free |
| `7` short | PBT outer | radio ID | free | not available | low cut |
| `7` long | PBT outer up | free | free | free | low cut up 60 Hz (experimental) |
| `7` double | PBT outer down | free | free | free | low cut down 60 Hz (experimental) |
| `8` short | filter shape | not available | free | not available | high cut |
| `8` long | filter shape soft/sharp | free | free | free | high cut up one step (experimental) |
| `8` double | free | free | free | free | high cut down one step (experimental) |
| `9` short | filter width | not available | free | not available | TX equalizer |
| `9` long | next filter | free | free | free | free |
| `9` double | previous filter | free | free | free | free |
| `A` short | free | free | free | AGC | AGC |
| `A` long | free | free | free | AGC select (experimental) | AGC select (experimental) |
| `B` short | free | free | free | not available | preamplifier |
| `B` double | free | free | free | not available | attenuator |

## Bank 3 - VFO and Split

On the FT-817 and FT-857/897, `1` is the VFO in use and `2` the other one.

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `0` short | split | split | split | split | split |
| `0` long | split on/off | split on/off | split on/off | split on/off | split on/off |
| `0` double | TX frequency | TX frequency | TX frequency; Kenwood and Elecraft: not available | the frequency when split is off, otherwise not available | the frequency when split is off, otherwise not available |
| `1` short | VFO A frequency | VFO A frequency | VFO A frequency | current VFO frequency | current VFO frequency |
| `1` long | select VFO A | select VFO A | select VFO A | switch VFO A/B | switch VFO A/B |
| `1` double | VFO A frequency entry | VFO A frequency entry | VFO A frequency entry; TS-480, KX2: not available | current VFO frequency entry | current VFO frequency entry |
| `2` short | VFO B frequency | VFO B frequency | VFO B frequency | other VFO frequency | other VFO frequency |
| `2` long | select VFO B | select VFO B | select VFO B | copy to the other VFO (A=B) | not available |
| `2` double | VFO B frequency entry | VFO B frequency entry | VFO B frequency entry; TS-480, KX2: not available | other VFO frequency entry | other VFO frequency entry |
| `3` short | free | free | free | VFO A mode | free |
| `3` long | free | free | free | VFO A mode select | free |
| `4` short | VFO A mode | VFO A mode | VFO A mode; Kenwood and Elecraft: not available | sync: VFO A is in use | free |
| `4` long | VFO A mode select | VFO A mode select | VFO A mode select; Kenwood and Elecraft: not available | sync: VFO B is in use | free |
| `5` short | VFO B mode | VFO B mode | VFO B mode; Kenwood and Elecraft: not available | RIT | RIT |
| `5` long | VFO B mode select | VFO B mode select | VFO B mode select; Kenwood and Elecraft: not available | RIT on/off | RIT on/off |
| `6` short | RX or TX | RX or TX | RX or TX; Kenwood and Elecraft: not available | select VFO A | PTT off (receive) |
| `6` long | free | free | free | select VFO B | PTT on (transmit) |
| `7` short | band stack 1 | not available | not available | not available | not available |
| `7` long | recall band stack 1 | not available | not available | not available | not available |
| `8` short | band stack 2 | not available | not available | not available | not available |
| `8` long | recall band stack 2 | not available | not available | not available | not available |
| `9` short | band stack 3 | not available | not available | not available | not available |
| `9` long | recall band stack 3 | not available | not available | not available | not available |

## Bank 4 - Tuner and Monitor

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `0` short | tuner | tuner | tuner | not available | not available |
| `0` long | tuner on/off | tuner on/off | tuner on/off | not available | not available |
| `0` double | tune | tune | tune | not available | not available |
| `1` short | monitor | not available | free | free | free |
| `1` long | monitor on/off | not available | not available | not available | not available |
| `2` short | monitor level | not available | not available | not available | not available |
| `2` long | monitor level up 10 % | not available | not available | not available | not available |
| `2` double | monitor level down 10 % | not available | not available | not available | not available |
| `3` short | transceive | not available | free | free | free |
| `3` long | transceive on/off | not available | not available | not available | not available |

## Bank 5 - RIT

RIT offsets work on Icom only; every other radio says "not available" on these keys. The FT-8x7
switches RIT on Bank 3 `5`.

| Key | Icom |
|---|---|
| `0` short | RIT on/off and offset |
| `0` long | RIT on/off |
| `0` double | offset to 0 |
| `1` short / long | offset down 10 Hz / 100 Hz |
| `2` short / long | offset up 10 Hz / 100 Hz |
| `3` short | offset to 0 |
| `3` long | RIT off |
| `4` short / long | offset down 1 Hz / 500 Hz |
| `5` short / long | offset up 1 Hz / 500 Hz |

## Bank 6 - Repeater and Tone

FT-8x7 only (FT-817/818 and FT-857/897); on every other radio
these keys are free.

| Key | FT-8x7 |
|---|---|
| `0` short | repeater shift off |
| `0` long | repeater shift minus |
| `0` double | repeater shift plus |
| `1` short | repeater offset preset 1 (0.6 MHz) |
| `1` long | repeater offset preset 2 (7.6 MHz) |
| `1` double | repeater offset entry in kHz |
| `2` short | tone type: off, CTCSS or DCS |
| `2` long | CTCSS |
| `2` double | tone off |
| `2` short, then hold | DCS |
| `3` short | CTCSS tone (the last set, or 88.5 Hz) |
| `3` long | CTCSS tone entry |
| `4` short | DCS code (the last set, or 023) |
| `4` long | DCS code entry |
| `5` short | encode only (TX), decode only (RX), encode and decode (both), or split |
| `5` long | step from both to TX, RX and back to both (FT-817/818: TX and both, CTCSS only) |
| `6` short | split tone on or off (FT-857/897 menu 79; FT-817/818: not available) |

## Bank 7

Empty on every radio: all keys are free.

## Bank 8 - Connection and Radio Menu

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `1` short | CI-V address | not available | not available | not available | not available |
| `1` long | CI-V address entry | not available | not available | not available | not available |
| `2` short | baud rate | baud rate | baud rate | baud rate | baud rate |
| `2` long | previous baud rate | previous baud rate | previous baud rate | previous baud rate | previous baud rate |
| `2` double | next baud rate | next baud rate | next baud rate | next baud rate | next baud rate |
| `7` short | free | free | free | function row | soft key row |
| `7` long | free | free | free | function row entry (experimental) | soft key row entry (experimental) |
| `8` short | free | free | free | last menu item | last menu item |
| `8` long | free | free | free | menu item entry (experimental) | menu item entry (experimental) |

`2` long and double step through the rates the radio offers (see the
[radio support matrix](radio-support-matrix.md)) and keep the choice for the profile. Set the same
rate on the radio. Bank 9 `A` double press puts the baud rate and CI-V address back to the
profile's defaults.

## Bank 9 - Profile and Speech

The same on every radio, and on the NanoVNA.

| Key | All radios |
|---|---|
| `0` short | experimental mode |
| `0` long | experimental mode on/off |
| `1` short | USB radio link |
| `4` short | tuning speech |
| `4` long | tuning speech on/off |
| `5` short | verbose |
| `5` long | verbose on/off |
| `6` short | speech speed |
| `6` long | next speech speed (slow, normal, fast) |
| `7` short / long | volume down / down fast |
| `8` short / long | volume up / up fast |
| `9` short | volume |
| `A` short | profile |
| `A` long | profile select |
| `A` double | reset the profile's baud rate and CI-V address to its defaults |
| `B` short | next profile |
| `C` short | previous profile |

`2` and `3` are free.

## NanoVNA (profile 30)

Bank 1 is the NanoVNA bank and Bank 9 is as above. Every other bank is free. Details in
[docs/radios/nanovna.md](radios/nanovna.md).

| Key | NanoVNA |
|---|---|
| `0` short | measure: best and worst SWR and their frequencies |
| `0` double | last result |
| `1` short | sweep start |
| `1` long | sweep start entry in MHz |
| `2` short | sweep stop |
| `2` long | sweep stop entry in MHz |
| `3` short | sweep range: band preset, or start and stop |
| `6` short | calibration slot |
| `6` long | calibration select: `0`–`6` slot, `A` auto |
| `9` short | connection test (firmware version) |
| `B` short | next band preset |
| `C` short | previous band preset |

`4`, `5`, `7`, `8` and `A` are free.
