# FTDX10 First Test Plan

This is the first serial-monitor validation plan for the Yaesu `FTDX10` family.

Target profiles:
- `PROFILE 11` = `FTDX10`
- `PROFILE 12` = `FTDX101D`
- `PROFILE 13` = `FTDX101MP`

The current implementation is based on the documented Yaesu ASCII CAT command set used by the `FTDX10` reference manual.

## Goal

Verify the first safe and useful CAT feature block before moving anything to dedicated keypad behavior.

Included in this first block:
- frequency read and set
- mode read and set
- S-meter
- power meter
- SWR meter
- NR
- NB
- notch
- lock
- tuner on/off/tune
- preamp query and set
- AGC query and set
- power state query and set

## Preparation

1. Connect the radio with CAT enabled.
2. Use the correct serial speed for the profile.
3. Open the serial monitor.
4. Select the profile:

```text
PROFILE 11
PROFILE?
```

Expected:
- active profile is `Yaesu FTDX-10`

## Core Radio Test

```text
FREQ?
MODE?
SM?
PO?
SWR?
MODE USB
MODE?
```

Expected:
- all queries return a reply
- `MODE USB` is accepted
- `MODE?` reports the changed mode

## Lock And Tuner Test

```text
LOCK?
LOCK ON
LOCK?
LOCK OFF
LOCK?
TUNER?
TUNER ON
TUNER?
TUNE
TUNER OFF
TUNER?
```

Expected:
- lock state changes are visible in replies and, if practical, on the radio
- tuner state changes are accepted
- `TUNE` starts tuner action

## DSP Toggle Test

```text
NR?
NR ON
NR?
NR OFF
NB?
NB ON
NB?
NB OFF
NOTCH?
NOTCH ON
NOTCH?
NOTCH OFF
```

Expected:
- each feature toggles and reads back consistently

## Front-End And AGC Test

```text
PA?
PA ON
PA?
PA OFF
PA?
GT?
GT FAST
GT?
GT SLOW
GT?
```

Expected:
- preamp/IPO state changes are accepted
- AGC replies change in a meaningful way

## Power State Test

```text
PS?
PS OFF
```

Expected:
- the radio powers down

Then power it back on manually if needed, or test the documented CAT power-on path carefully:

```text
PS ON
PS?
```

Note:
- on `FTDX10`, CAT power-on may require the documented double-send timing behavior
- the firmware now applies that timing for the ASCII `PS ON` path

## Report Format

Please return results in a short form like this:

```text
FREQ? works
MODE? works
SM? works
LOCK ON/OFF works
TUNER ON works
TUNE works
NR read/write works
NB read/write works
NOTCH read/write works
PA read/write works
GT read/write unclear
PS OFF works
PS ON no reply
```

That is enough for the next implementation step.
