# Talking Remote Controller LX1WJ User Guide

This guide is for operators and helpers.
It explains practical use in simple terms and keeps technical details out of the way.

## What The Controller Does

You operate the radio with a `4x4` keypad.
The controller speaks the result, so you do not need to read the radio display.

## Keypad Basics

Key layout:

```text
1 2 3 A
4 5 6 B
7 8 9 C
* 0 # D
```

Global rules:

- Short press: ask or read something
- Long press: change something
- `D`: confirm
- `#`: cancel and leave the current entry
- An entry or selection left for 30 seconds without a key ends and says "timeout"
- `*` short: speak the current bank
- `*` long, let go, then a digit `1`–`9`: change to that bank at once ("bank 3")
- `*` long and, still holding `*`, a digit `1`–`9`: the next key only acts on that bank, then you
  are back on your bank. Only the digit is spoken ("3"). Let go of `*` before that next key; `#`
  cancels
- Press one key at a time. If a second key goes down while another is still held, the radio beeps
  and ignores both keys until you let go of all of them; nothing is run or cancelled
- `*` while entering a frequency: the decimal point (see Bank 1 `0` long below)

Frequencies are spoken in full: the megahertz digits, "point", then the remaining digits with
trailing zeros dropped (always at least one decimal) — for example 7.12345 MHz is spoken "seven
point one two three four five", 14.1 MHz is spoken "one four point one", and a whole 7 MHz is
spoken "seven point zero".

## Two Things To Remember

- A new key press stops the current speech immediately.
- The spoken result depends on the selected radio profile.

## Shorter Answers

Every answer first says what it is, then the value: "power 50 watts", "noise blanker 40 percent",
"volume five ok". Once you know the keys, you can turn the names off and hear only the values:
"50 watts", "40 percent", "five". Units stay, and the "ok" after a change is dropped.

- Bank 9, `5` short: says "verbose on" or "verbose off"
- Bank 9, `5` long: switches between the two

The setting is kept after power off. Prompts such as "frequency please" and messages such as
"not available" stay the same. Console: `VERBOSE?` and `VERBOSE ON | OFF | TOGGLE`.

## Practical Everyday Use

These actions are the normal starting point on many profiles:

- Bank 1, `0` short: speak current frequency
- Bank 1, `0` long, digits, `D`: set frequency. The number is MHz and `*` is the decimal point,
  with up to 5 digits after it (10 Hz steps): `14` `D` = 14 MHz, `14*1` `D` = 14.1 MHz,
  `14*12345` `D` = 14.12345 MHz.
- Bank 1, `0` double: round the current frequency to the nearest 500 Hz
  (e.g. 7.1437 → 7.1435, 7.1438 → 7.144); the controller then speaks the new frequency
- Bank 1, `9` short: speak mode
- Bank 1, `9` long, mode digit, `D`: change mode
- Bank 1, `7` short: speak S-meter if supported
- Bank 1, `8` short: speak SWR if supported

Mode digits:

- `1` LSB
- `2` USB
- `3` CW
- `4` AM
- `5` FM
- `6` DATA/DIGI
- `7` RTTY
- `8` CW-R
- `9` RTTY-R

## CI-V Connection Setup

For CI-V profiles, Bank 8 can adjust the stored connection settings for the selected profile.

- Bank 8, `1` short: speak the current CI-V address
- Bank 8, `1` long, digits, `D`: set CI-V address as a decimal value from `0` to `255`
- Bank 8, `2` short: move to the next baud rate
- Bank 8, `2` long: move to the previous baud rate

## FTDX10 Family First Test

For `FTDX10`, `FTDX101D`, and `FTDX101MP`, the current practical test path is intentionally small and clear.

Use these keypad functions first:

- Bank 1, `0` short or long: read or set frequency
- Bank 1, `3` short or long: read or toggle lock
- Bank 1, `4` short: power meter
- Bank 1, `5` short, long, or double: tuner query, tuner toggle, tune
- Bank 1, `6` short or long: preamp query or toggle
- Bank 1, `7` short: S-meter
- Bank 1, `8` short: SWR
- Bank 1, `9` short or long: read or set mode
- Bank 2, `1`, `2`, `3` short or long: NR, NB, notch query or toggle
- Bank 2, `4` short, long, or double: AGC query, fast, slow
- Bank 2, `5` short, long, or double: power-state query, off, on
- Bank 2, `6` short: IF info
- Bank 2, `7` short: radio ID
- Bank 3, `0` short or long: split query or toggle
- Bank 3, `1` short or long: VFO-A frequency or select VFO-A
- Bank 3, `2` short or long: VFO-B frequency or select VFO-B
- Bank 3, `4` short or long: VFO-A mode read or set
- Bank 3, `5` short or long: VFO-B mode read or set
- Bank 3, `6` short: RX or TX state

Keep the first field test simple:

1. Select the FTDX10 family profile.
2. Check frequency.
3. Check mode.
4. Check S-meter.
5. Try lock on and off.
6. Try VFO-A, VFO-B, and split.
7. Try NR, NB, and notch.
8. Try tuner query, toggle, and tune.

## When Something Feels Wrong

- Press `#` to cancel the current action.
- Ask the current frequency again with Bank 1, `0` short.
- Ask the current bank again with `*` short.
- A key with no action on the current bank gives a short beep. A key whose feature the radio or
  profile lacks says "not available", for example the tuner keys on an FT-817 or keys hidden on
  FTDX10. With verbose on, the function's name comes first: "tuner not available", "split
  timeout", "frequency error".

## More Help

- Short setup path: [QUICKSTART.md](QUICKSTART.md)
- Every key on every radio: [docs/keypad-map.md](docs/keypad-map.md)
- Current per-radio support: [docs/radio-support-matrix.md](docs/radio-support-matrix.md)
- FTDX10 family helper list: [docs/radios/yaesu-ftdx10-blind-test-list.txt](docs/radios/yaesu-ftdx10-blind-test-list.txt)
