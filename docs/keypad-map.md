# Keypad Map

What every bank key does on each radio. One table per bank; each row is a key and a press, each
column a group of radios.

Columns:

- **Icom**: CI-V radios (IC-7300, IC-705, IC-706 and others)
- **FTDX10**: the FTDX10 family profile; most keys run a console command
- **Kenwood, Elecraft, other FTDX**: TS-480, KX2, KX3, FT-891 and other Yaesu FTDX radios
- **FT-817/818** and **FT-857/897**: the Yaesu FT-8x7 radios

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

## Focus: A, B and C

On every bank, `A`, `B` and `C` act on the bank's **focus**: the setting of the
last key on that bank that has one. A key with nothing to move (a switch, a meter, the frequency)
leaves the focus as it was. Each bank keeps its own focus, and a profile change puts every bank
back to its starting focus.

| Bank | Keys that set the focus | Starting focus |
|---|---|---|
| 1 | `6` RF power, `9` mode | none |
| 2 | `1` NR level, `2` NB level, `3` notch (FT-857/897: auto notch), `4` low cut or PBT inner, `5` CW filter width or filter, `6` high cut or PBT outer, `7` AGC, `8` preamp, `9` attenuator | none |
| 3 | none yet | none |
| 4 | `1` processor level, `2` VOX gain, `3` monitor level (Icom), `5` TX equalizer, `7` mic gain, `8` RF power, `9` VOX delay | none |
| 5 | `1`-`5` RIT offset | RIT offset |
| 6 | `3` CTCSS tone, `4` DCS code (FT-8x7) | none |
| 7 | `1` keyer speed, `2` QSK time (FT-857/897) or break-in (Icom), `3` sidetone, `5` pitch, `6` weight, `8` RF power, `9` break-in delay | none |
| 8 | none yet | none |
| 9 | `7` volume, `8` speech speed | profile |

With no focus, `A`, `B` and `C` say "not available".

| Press | Does |
|---|---|
| `A` short | the focus, with its name (verbose off too) |
| `A` long | its entry or pick; speech speed steps to the next |
| `A` double | back to the centre: PBT 0, RIT offset 0, normal speech speed, profile reset. Only where there is one; elsewhere `A` answers at once |
| `B` / `C` short | one step up / down |
| `B` / `C` long | a big step: RF power 5 steps, NR and NB level 25 % on Icom and 5 steps elsewhere, PBT 50, low and high cut 5 steps, the Bank 4 levels and the sidetone 25, VOX delay, CW speed, pitch, weight, delay and QSK time 5 steps, RIT offset 100 Hz, CTCSS tone and DCS code 5, volume 2; a list (filter, notch, AGC, preamp, attenuator, CW filter width, TX equalizer, Icom break-in, speech speed) one step; the mode and the profile not available |

The first `B` or `C` after another key names the focus; the next ones give only the value. A step
past the end of the range says "maximum" or "minimum" and changes nothing.

RF power, the mode and the FT-8x7 settings kept in the radio's memory (the FT-8x7 levels and
numbers on Banks 2, 4 and 7, and its AGC) wait for `D`: `B` or `C` reads the value once and opens the entry or pick
one step away, ending in "please". More `B` and `C` presses step with no radio traffic (RF power
held: the big step, as before the entry opened), `D` sets,
`#` cancels. The FT-817/818 power steps through its levels and stops at the lowest and highest. The
mode walks the modes in the order of the mode select digits and goes round: after the last comes
the first. The FT-8x7 settings need experimental mode; without it `B` and `C` say "not available". The other
settings change at once.

In a pick of named choices (mode, AGC), `B` and `C` go round. Levels and numbers stop at the end
with "maximum" or "minimum".

## Help mode

`D` long turns help mode on or off ("help on", "help off"). While it is on, every bank key press
says what it would do and does nothing: the function's name, after "choose" for an entry or pick,
"next" for a step through a short list, "up" or "down" (held: "up fast", "down fast") for a step,
or "reset" for back to the centre, and before "on off" for a switch. A function the radio lacks
says "not available". Every press works: short, long, double and a press then a hold. `A`, `B` and
`C` tell what they would do with the bank's focus, which the keys pressed in help mode move as they
normally would. `*` still changes banks. A press with no function beeps as usual. `D` long or `#`
ends help mode.

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
| `5` short | ALC while transmitting | ALC while transmitting | ALC (FTDX101, FT-891); Kenwood and Elecraft: not available | ALC while transmitting, 0–15 | ALC while transmitting, 0–15 |
| `6` short | RF power setting (IC-706: free) | RF power setting | RF power setting (FTDX101MP: free) | RF power setting | RF power setting |
| `6` long | RF power entry | RF power entry | RF power entry | RF power level select (experimental) | RF power entry (experimental) |
| `7` short | S-meter | S-meter | S-meter | S-meter | S-meter |
| `8` short | SWR | SWR | SWR | SWR | SWR |
| `9` short | mode | mode | mode | mode | mode |
| `9` long | mode select | mode select | mode select | mode select | mode select |
| `A` `B` `C` | focus: RF power or mode | focus | focus | focus | focus |

The FTDX10 tuner is on Bank 4 `0`.

## Bank 2 - Receiver

Top row: the switches with a level. Middle row: the passband, low side, filter, high side. Bottom
row: AGC, preamp, attenuator.

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `0` short | auto notch | auto notch | auto notch (FTDX101, FT-891); Kenwood and Elecraft: not available | antenna jack of the band | free (auto notch on `3`) |
| `0` long | auto notch on/off | auto notch on/off | auto notch on/off | antenna jack front/rear (experimental) | free |
| `1` short | noise reduction | noise reduction | noise reduction | not available | noise reduction |
| `1` long | noise reduction on/off | noise reduction on/off | noise reduction on/off; TS-480 off, NR1, NR2 | not available | noise reduction on/off (experimental) |
| `1` double | NR level | NR level | NR level (FTDX101, FT-891); Kenwood and Elecraft: free | free | NR level |
| `1` short, then hold | NR level entry | NR level entry | NR level entry (FTDX101, FT-891) | free | NR level entry (experimental) |
| `2` short | noise blanker | noise blanker | noise blanker | noise blanker | noise blanker |
| `2` long | noise blanker on/off | noise blanker on/off | noise blanker on/off | noise blanker on/off (experimental) | noise blanker on/off (experimental) |
| `2` double | NB level | NB level | NB level (FTDX101, FT-891); Kenwood and Elecraft: free | free | NB level |
| `2` short, then hold | NB level entry | NB level entry | NB level entry (FTDX101, FT-891) | free | NB level entry (experimental) |
| `3` short | notch | notch | notch | not available | auto notch |
| `3` long | notch off, narrow, mid, wide | notch on/off | notch on/off | not available | auto notch on/off (experimental) |
| `4` short | PBT inner | not available | not available | not available | low cut |
| `4` long | PBT inner entry | not available | not available | not available | low cut entry (experimental) |
| `5` short | filter (1-3) | not available | not available | not available | DSP bandpass filter |
| `5` long | next filter | free | free | free | DSP bandpass filter on/off (experimental) |
| `5` double | filter shape | free | free | free | CW filter width |
| `5` short, then hold | filter shape soft/sharp | free | free | free | CW filter width select (experimental) |
| `6` short | PBT outer | not available | not available | not available | high cut |
| `6` long | PBT outer entry | not available | not available | not available | high cut entry (experimental) |
| `7` short | AGC | AGC | AGC | AGC | AGC |
| `7` long | AGC select | AGC select | AGC select | AGC select (experimental) | AGC select (experimental) |
| `8` short | preamp | preamp | preamp | not available | preamplifier (IPO) |
| `8` long | next preamp level | next preamp level | next preamp level | free | free |
| `9` short | attenuator | attenuator | attenuator | not available | attenuator |
| `9` long | next attenuator step | next attenuator step | next attenuator step | free | free |
| `A` `B` `C` | focus | focus | focus | focus: AGC | focus |

The NR and NB levels are in percent on Icom, in the radio's own steps elsewhere (FTDX10, FTDX101
and FT-891 NR 1-15, NB 0-10; FT-857/897 NR 1-16, NB 0-100). PBT is in the radio's steps, -128 to
127, with 0 the centre. The FT-857/897 CW filter width (DSP bandpass width) is 60, 120 or 240 Hz.

AGC select: `0` off, `1` auto, `2` fast, `3` slow, `4` mid, then `D`. Only the radio's own speeds
are taken: Icom fast, mid and slow; FTDX10, FTDX101 and FT-891 all five; TS-480 off, fast and
slow; KX2 fast and slow; FT-8x7 off, auto, fast and slow. The IC-706 and G106 say "not
available".

Preamp and attenuator: a hold goes to the next level and from the last one to off. Preamp: IC-7300,
IC-705, IC-7760, FTDX10 and FTDX101 off, 1, 2; FT-891, TS-480 and KX2 off and on. Attenuator:
IC-7300 and IC-705 20 dB; IC-7760, FTDX10 and FTDX101 6, 12 and 18 dB; KX2 10 dB; FT-891 and TS-480
on. The FT-857/897 preamp and attenuator are read only; the IC-706, G106 and FT-817 say "not
available".

The FT-857/897 TX equalizer is on Bank 4 `4`-`6`; the FTDX10 power state, ID and status line are on
Bank 8.

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
| `A` `B` `C` | focus (none yet) | focus (none yet) | focus (none yet) | focus (none yet) | focus (none yet) |

## Bank 4 - Phone Transmit

The twin of Bank 2. Top row: the switches with a level. Middle row: the TX passband, low side,
equalizer, high side. Bottom row: mic gain, RF power, VOX delay.

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `0` short | tuner | tuner | tuner | not available | not available |
| `0` long | tuner on/off | tuner on/off | tuner on/off | not available | not available |
| `0` double | tune | tune | tune | not available | not available |
| `1` short | speech processor (IC-706: not available) | speech processor | speech processor (FT-891, TS-480); others: not available | not available | speech processor |
| `1` long | processor on/off | processor on/off | processor on/off (FT-891, TS-480) | free | processor on/off (experimental) |
| `1` double | processor level | processor level | processor level (FT-891, TS-480) | free | processor level |
| `1` short, then hold | processor level entry | processor level entry | processor level entry (FT-891, TS-480) | free | processor level entry (experimental) |
| `2` short | VOX (IC-706: not available) | VOX | VOX (FTDX101, FT-891, TS-480); others: not available | VOX | VOX |
| `2` long | VOX on/off | VOX on/off | VOX on/off | VOX on/off (experimental) | VOX on/off (experimental) |
| `2` double | VOX gain | VOX gain | VOX gain | VOX gain | VOX gain |
| `2` short, then hold | VOX gain entry | VOX gain entry | VOX gain entry | VOX gain entry (experimental) | VOX gain entry (experimental) |
| `3` short | monitor (IC-706: not available) | not available | not available | not available | not available |
| `3` long | monitor on/off | free | free | free | free |
| `3` double | monitor level | free | free | free | free |
| `3` short, then hold | monitor level entry | free | free | free | free |
| `4` short | not available | not available | not available | not available | TX low cut (the equalizer's HPF) |
| `4` long | free | free | free | free | TX low cut on/off (experimental) |
| `5` short | not available | not available | not available | not available | TX equalizer |
| `5` long | free | free | free | free | TX equalizer select (experimental) |
| `6` short | not available | not available | not available | not available | TX high cut (the equalizer's LPF) |
| `6` long | free | free | free | free | TX high cut on/off (experimental) |
| `7` short | mic gain (IC-706: not available) | mic gain | mic gain (FTDX101, FT-891, TS-480); others: not available | mic gain of the mode | mic gain of the mode |
| `7` long | mic gain entry | mic gain entry | mic gain entry | mic gain entry (experimental) | mic gain entry (experimental) |
| `8` short | RF power setting (IC-706: free) | RF power setting | RF power setting (FTDX101MP: free) | RF power setting | RF power setting |
| `8` long | RF power entry | RF power entry | RF power entry | RF power level select (experimental) | RF power entry (experimental) |
| `9` short | VOX delay (IC-7300, IC-705) | VOX delay | VOX delay (FT-891, TS-480) | VOX delay | VOX delay |
| `9` long | VOX delay entry | VOX delay entry | VOX delay entry | VOX delay entry (experimental) | VOX delay entry (experimental) |
| `A` `B` `C` | focus | focus | focus | focus | focus |

The levels are 0-100 (TS-480 VOX gain 0-9, FT-8x7 VOX gain 1-100); `B`/`C` step them by 5. The
VOX delay steps by the radio's own step: 100 ms on Icom and the FT-8x7, the radio's table on the
FTDX10, 150 ms on the TS-480. The FT-8x7 mic gain is that of the mode in use when the key is
pressed, and an entry sets that mode's gain even if the mode changes before `D`.

TX equalizer select: `0` off, `1` high cut, `2` low cut, `3` both, then `D`. `4` and `6` switch one
of its two filters and keep the other.

Transceive is on Bank 8 `3`.

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
| `A` `B` `C` | focus: the RIT offset. `B`/`C` 10 Hz, long 100 Hz; `A` long offset entry, `*` first for minus; `A` double offset to 0 |

## Bank 6 - Repeater and Tone

The same keys on every radio that has the commands. Over CAT the tone is CTCSS only: no DCS and no
repeater offset. Kenwood, Elecraft, other FTDX: the FT-891 has the FTDX10's keys; the TS-480, KX2
and KX3 say "not available".

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-8x7 |
|---|---|---|---|---|
| `0` short | repeater shift off (IC-705; others: not available) | repeater shift off | repeater shift off | repeater shift off |
| `0` long | repeater shift minus (IC-705) | repeater shift minus | repeater shift minus | repeater shift minus |
| `0` double | repeater shift plus (IC-705) | repeater shift plus | repeater shift plus | repeater shift plus |
| `1` short | not available | not available | not available | repeater offset preset 1 (0.6 MHz) |
| `1` long | free | free | free | repeater offset preset 2 (7.6 MHz) |
| `1` double | free | free | free | repeater offset entry in kHz |
| `2` short | tone type: off or CTCSS (IC-706: not available) | tone type: off or CTCSS | tone type: off or CTCSS | tone type: off, CTCSS or DCS |
| `2` long | CTCSS | CTCSS | CTCSS | CTCSS |
| `2` double | tone off | tone off | tone off | tone off |
| `2` short, then hold | free | free | free | DCS |
| `3` short | CTCSS tone (IC-706: not available) | CTCSS tone | CTCSS tone | CTCSS tone (the last set, or 88.5 Hz) |
| `3` long | CTCSS tone entry | CTCSS tone entry | CTCSS tone entry | CTCSS tone entry |
| `4` short | not available | not available | not available | DCS code (the last set, or 023) |
| `4` long | free | free | free | DCS code entry |
| `5` short | encode only (TX) or encode and decode (both) | the same | the same | encode only (TX), decode only (RX), encode and decode (both), or split |
| `5` long | step between both and TX | step between both and TX | step between both and TX | step from both to TX, RX and back to both (FT-817/818: TX and both, CTCSS only) |
| `6` short | free | free | free | split tone on or off (FT-857/897 menu 79; FT-817/818: not available) |
| `A` `B` `C` | focus | focus | focus | focus |

`B`/`C` walk the CTCSS tones one at a time (long: 5 tones) and the DCS codes likewise, and set
each at once. A walk stops at the first and last tone or code.

## Bank 7 - CW

The twin of Bank 4. Top row: the keyer with its speed, break-in, the sidetone. Middle row: the
keyer type, pitch, weight. Bottom row: key reverse, RF power, break-in delay.

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `0` short | tuner | tuner | tuner | not available | not available |
| `0` long | tuner on/off | tuner on/off | tuner on/off | not available | not available |
| `0` double | tune | tune | tune | not available | not available |
| `1` short | not available | keyer | keyer (FTDX101, FT-891); TS-480, KX2: keyer speed | keyer | keyer |
| `1` long | free | keyer on/off | keyer on/off; TS-480, KX2: keyer speed entry | keyer on/off (experimental) | keyer on/off (experimental) |
| `1` double | free | keyer speed | keyer speed (FTDX101, FT-891) | keyer speed | keyer speed |
| `1` short, then hold | free | keyer speed entry | keyer speed entry (FTDX101, FT-891) | keyer speed entry (experimental) | keyer speed entry (experimental) |
| `2` short | break-in (IC-706: not available) | break-in | break-in (FTDX101, FT-891); Kenwood and Elecraft: not available | break-in | break-in |
| `2` long | next: off, semi, full | break-in on/off | break-in on/off | break-in on/off (experimental) | break-in on/off (experimental) |
| `2` double | free | free | free | free | QSK time |
| `2` short, then hold | free | free | free | free | QSK time entry (experimental) |
| `3` short | not available | not available | not available | sidetone level | sidetone level |
| `3` long | free | free | free | sidetone entry (experimental) | sidetone entry (experimental) |
| `4` short | not available | not available | not available | not available | keyer type: paddle (ELEKEY) or mic keys (MICKEY) |
| `4` long | free | free | free | free | keyer type ELEKEY/MICKEY (experimental) |
| `5` short | CW pitch (IC-706: not available) | CW pitch | CW pitch (FTDX101, FT-891) | CW pitch | CW pitch |
| `5` long | CW pitch entry | CW pitch entry | CW pitch entry | CW pitch entry (experimental) | CW pitch entry (experimental) |
| `6` short | not available | not available | not available | CW weight | CW weight |
| `6` long | free | free | free | CW weight entry (experimental) | CW weight entry (experimental) |
| `7` short | not available | not available | not available | key reverse | key reverse |
| `7` long | free | free | free | key reverse on/off (experimental) | key reverse on/off (experimental) |
| `8` short | RF power setting (IC-706: free) | RF power setting | RF power setting (FTDX101MP: free) | RF power setting | RF power setting |
| `8` long | RF power entry | RF power entry | RF power entry | RF power level select (experimental) | RF power entry (experimental) |
| `9` short | not available | break-in delay | break-in delay (FT-891, TS-480) | break-in delay | break-in delay |
| `9` long | free | break-in delay entry | break-in delay entry | break-in delay entry (experimental) | break-in delay entry (experimental) |
| `A` `B` `C` | focus | focus | focus | focus | focus |

`B`/`C` step the keyer speed by 1 WPM and the pitch by 10 Hz (FT-8x7: its menu step), the
break-in delay by the radio's own step, the sidetone by 5, the weight by 0.1 and the QSK time by
the radio's step. Icom break-in is off, semi or full: `2` long goes to the next and from full back
to off, and `B`/`C` walk it. A delay of 0 is full break-in (TS-480, FT-857/897); on the FT-857/897
the step below the shortest delay is FULL. The weight is typed with `*` as its point: `3*5` is
1:3.5.

## Bank 8 - Connection and Radio Menu

| Key | Icom | FTDX10 | Kenwood, Elecraft, other FTDX | FT-817/818 | FT-857/897 |
|---|---|---|---|---|---|
| `1` short | CI-V address | not available | not available | not available | not available |
| `1` long | CI-V address entry | not available | not available | not available | not available |
| `2` short | baud rate | baud rate | baud rate | baud rate | baud rate |
| `2` long | previous baud rate | previous baud rate | previous baud rate | previous baud rate | previous baud rate |
| `2` double | next baud rate | next baud rate | next baud rate | next baud rate | next baud rate |
| `3` short | transceive (IC-7760: not available) | not available | free | free | free |
| `3` long | transceive on/off | not available | not available | not available | not available |
| `4` short | free | radio power state | free | free | free |
| `4` long | free | radio power off | free | free | free |
| `4` double | free | radio power on | free | free | free |
| `5` short | free | radio ID | free | free | free |
| `6` short | free | radio status line | free | free | free |
| `7` short | free | free | free | function row | soft key row |
| `7` long | free | free | free | function row entry (experimental) | soft key row entry (experimental) |
| `8` short | free | free | free | last menu item | last menu item |
| `8` long | free | free | free | menu item entry (experimental) | menu item entry (experimental) |
| `A` `B` `C` | focus (none yet) | focus (none yet) | focus (none yet) | focus (none yet) | focus (none yet) |

`2` long and double step through the rates the radio offers (see the
[radio support matrix](radio-support-matrix.md)) and keep the choice for the profile. Set the same
rate on the radio. Bank 9 `1` double press puts the baud rate and CI-V address back to the
profile's defaults.

## Bank 9 - Profile and Speech

The same on every radio.

| Key | All radios |
|---|---|
| `0` short | experimental mode |
| `0` long | experimental mode on/off |
| `1` short | profile |
| `1` long | profile select |
| `1` double | reset the profile's baud rate and CI-V address to its defaults |
| `4` short | tuning speech |
| `4` long | tuning speech on/off |
| `5` short | verbose |
| `5` long | verbose on/off |
| `7` short | volume |
| `7` long | volume entry |
| `8` short | speech speed |
| `8` long | next speech speed (slow, normal, fast) |
| `A` `B` `C` | focus: the profile, until `7` or `8` moves it to the volume or speech speed; `1` brings it back |

With the profile as the focus: `A` short profile, `A` long profile select, `A` double reset the
profile's baud rate and CI-V address to its defaults, `B` next profile, `C` previous profile.

`2`, `3`, `6` and `9` are free. `B` and `C` step the volume once `7` has made it the focus.
