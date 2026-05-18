# Yaesu FTDX10 Family Test Plan

This is the short technical serial test plan for the current **V3.5.8** FTDX10 family block.

Target profiles:

- `PROFILE 11` = `FTDX10`
- `PROFILE 12` = `FTDX101D`
- `PROFILE 13` = `FTDX101MP`

## Confirm First

```text
PROFILE 11
PROFILE?
ID?
IF?
FREQ?
MODE?
```

Expected:

- the intended profile loads
- `ID?` and `IF?` reply
- frequency and mode reply

## Safe Core Test

```text
FREQ 14100
FREQ?
MODE USB
MODE?
SM?
PO?
SWR?
```

## VFO And Split

```text
VFO A
VFOA?
VFO B
VFOB?
VFOA MODE?
VFOB MODE?
SPLIT?
SPLIT ON
SPLIT?
SPLIT OFF
```

## DSP And Front-End

```text
NR?
NR ON
NB?
NB ON
NOTCH?
NOTCH ON
PA?
PA ON
GT?
GT FAST
GT SLOW
```

## Lock, Tuner, Power

```text
LOCK?
LOCK ON
LOCK OFF
TUNER?
TUNER ON
TUNE
TUNER OFF
PS?
PS OFF
PS ON
```

## Report Format

Use short tags:

- `works`
- `wobbles`
- `fails`

Example:

```text
FREQ set works
VFOA MODE works
GT FAST works
PS ON wobbles
```