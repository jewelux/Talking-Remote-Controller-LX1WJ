# NanoVNA

## Status

**Experimental. Tested with a NanoVNA-F V2** (firmware 0.6.0): the console commands, a full
scan, and the keys and their speech. Profile 70 (`NanoVNA`) measures the SWR of an antenna over a
band and says the best and worst match:

> "sweep" … "best match 1 point 2 1 at 3 point 6 1 2, worst match 3 point 4 0 at 3 point 8 0 0"

It was built from the NanoVNA-F V2 user guide (SYSJOINT, V2.0, chapter 7 "Console Command") and
cross-checked with DiSlord's NanoVNA-D firmware source. The other models are not tested.

Supported: the NanoVNAs with the text shell (`ch>` prompt): NanoVNA, -H, -H4, -F and -F V2. The
NanoVNA V2 / V2Plus4 (S-A-A-2) uses a binary protocol and does not work with this profile.

A NanoVNA is not a radio. The profile has no radio function, and nothing is ever sent to the
NanoVNA unless a key or a console command asks for it.

## Connecting

The NanoVNA's USB-C cable goes into the controller's **native USB socket**, as for the QMX (see
`docs/radios/qrp-labs-qmx.md`, Connecting):

- the PC console moves to the second USB socket (the USB-to-serial chip), where `HAMTRC?`,
  `USB?` and the `VNA` commands below answer;
- the NanoVNA runs on its own battery, so the native socket does not need to supply 5 V;
- choosing another profile restarts the controller, and the native socket goes back to the PC
  console. The QMX profile (21) restarts it only while the NanoVNA is connected, so that the QMX's
  CAT never reaches the NanoVNA.

## Keys

Bank 1 is the NanoVNA bank. Bank 8 `0` says the USB state, and Bank 9 works as on every radio:
profile, volume and speech. Every other key is free (a beep). Bank 2 is kept for calibration later.

| Key | Press | What it does |
|---|---|---|
| `0` | short | measure: "sweep", then "best match … at …, worst match … at …" |
| `0` | double | the last result again |
| `1` | short | start frequency |
| `1` | long | start frequency entry, in MHz (`*` is the point, `D` confirms) |
| `2` | short | stop frequency |
| `2` | long | stop frequency entry |
| `3` | short | the range: "band 8 0 meters" for a preset, otherwise "start … stop …" |
| `4` | short | the band, as `3` |
| `4` | long | band entry: the wavelength, then `D` (see below) |
| `5` | short | IARU region: "region 1" |
| `5` | long | region select: `1`, `2` or `3`, then `D`: "region 2 ok" |
| `6` | short | calibration: "calibration 3", or "calibration auto" |
| `6` | long | calibration select: `0`–`6` a saved slot, `A` auto, then `D` |
| `7` | short | test: "version 1 point 0 …", or "ok" |
| `8` | short | battery voltage: "battery 4 point 2 6 volts" |
| `9` | short | serial number: "s n", then its characters one by one (older models: "error") |
| `A` | short / long | the focus: say it / choose it |
| `B` | short | the focus up: next band, next region, next calibration slot |
| `C` | short | the focus down |

`4`, `5` and `6` make the band, region or calibration the focus of `A`, `B` and `C`. Until one of
them is pressed, the focus is the band.

- Band entry: type the band as it is said.
  - `8` `0` `D` is 80 m, `2` `D` is 2 m, `1` `6` `0` `D` is 160 m.
  - `7` `0` `D` is 70 cm and `2` `3` `D` is 23 cm: there is no 70 m or 23 m band.
  - `1` `*` `2` `5` `D` is 1.25 m, in Region 2 only.
  - A band the region does not have says "band error", and the range stays.
- Bands: the IARU band plan edges of the region, in MHz.

  | Band | Region 1 | Region 2 | Region 3 |
  |---|---|---|---|
  | 160 m | 1.81–2.0 | 1.8–2.0 | 1.8–2.0 |
  | 80 m | 3.5–3.8 | 3.5–4.0 | 3.5–3.9 |
  | 60 m | 5.3515–5.3665 | 5.3515–5.3665 | 5.3515–5.3665 |
  | 40 m | 7.0–7.2 | 7.0–7.3 | 7.0–7.2 |
  | 30 m | 10.1–10.15 | 10.1–10.15 | 10.1–10.15 |
  | 20 m | 14.0–14.35 | 14.0–14.35 | 14.0–14.35 |
  | 17 m | 18.068–18.168 | 18.068–18.168 | 18.068–18.168 |
  | 15 m | 21.0–21.45 | 21.0–21.45 | 21.0–21.45 |
  | 12 m | 24.89–24.99 | 24.89–24.99 | 24.89–24.99 |
  | 10 m | 28.0–29.7 | 28.0–29.7 | 28.0–29.7 |
  | 6 m | 50–52 | 50–54 | 50–54 |
  | 2 m | 144–146 | 144–148 | 144–148 |
  | 1.25 m | – | 222–225 | – |
  | 70 cm | 430–440 | 420–450 | 430–440 |
  | 23 cm | 1240–1300 | 1240–1300 | 1240–1300 |

  - `B` after 23 cm goes back to 160 m, and `C` before 160 m goes to 23 cm. From a range of your
    own, `B` goes to the first band that starts above its start, and `C` to the last band that
    starts at or below it.
  - Older NanoVNAs do not reach 23 cm; the NanoVNA then says "error" for the measurement.
- Region: Region 1 at first.
  - When the region changes, a range that is a band preset becomes the same band of the new region,
    and the band is said after "region 2 ok". 80 m in Region 1 (3.5–3.8) becomes 3.5–4.0 in Region 2.
  - A range of your own stays as it is, and so does 1.25 m when the region is no longer 2.
- Region: `B` and `C` go round, 3 to 1 and 1 to 3, also while choosing it with `5` long.
- Calibration: `B` and `C` stop at the ends (auto and slot 6) with the usual limit sound.
- Held, `B` and `C` say "not available" for the band, region and calibration.
- The start and stop can be set in either order. A stop below the start is swept from the lower
  to the higher frequency; the start and stop stay as typed. The same start and stop says "error".
- The range, the region and the calibration slot are kept when the controller is switched off. The
  first default is 80 m in Region 1 with auto calibration.
- SWR is said with two decimals below 10 ("1 point 2 1"), with one decimal below 100, and as
  "99 plus" above that, e.g. for an open port. The best and worst frequencies are said in MHz to
  1 kHz, the start and stop as they were typed.

### Calibration

"Auto" uses the calibration the NanoVNA has loaded, as on its screen. A slot (`0`–`6` on the F V2,
`0`–`4` on older models) is recalled before every measurement, so the measurement always uses
that calibration, even if it was changed on the screen in between. Choosing a slot also recalls it
at once when the NanoVNA is connected: a slot the device does not have says "calibration error",
and the slot chosen before stays.

Recalling a slot also loads the screen settings saved with it. The calibration itself (open,
short, load) is done on the NanoVNA for now.

### What else you hear

| Spoken | Meaning |
|---|---|
| "nano v n a" | profile 70 at boot or when chosen |
| "usb on" / "usb off" | the NanoVNA was plugged in or switched on / unplugged or switched off |
| "busy" | a key that needs the NanoVNA while a measurement is still running |
| "usb off" after a key | the NanoVNA is not connected |
| "error" | the NanoVNA refused a command, the scan gave no data, or the range is wrong |
| "timeout" | the NanoVNA did not answer |

## What the measurement does

The measure key runs four shell commands, each ended by CR. The controller waits for the echo of
each command, then for the `ch> ` prompt.

1. `recall <slot>`, only when a slot is chosen;
2. `sweep <start> <stop>`, so the NanoVNA's own screen shows the range just measured;
3. `scan <start> <stop> 101 3`: 101 points, each line the frequency and S11 (real, imaginary);
4. `resume`, so the NanoVNA sweeps again. It is sent after a failed scan too, and after a scan
   that timed out, without waiting for it. A NanoVNA that refuses it keeps the result.

SWR = (1 + |S11|) / (1 − |S11|) for each point. The lowest is the best match, the highest the
worst. Before every job the controller sends a bare CR and waits for the prompt, so whatever is
left on the shell line goes first. Anything before a command's echo is the late reply of a command
that timed out and is skipped; between jobs, what the NanoVNA sends is dropped.

## Console (second USB socket)

| Command | What it does |
|---|---|
| `VNA?` | connection, range, calibration and the last result |
| `VNASCAN` | a measurement, as Bank 1 `0` |
| `VNAREGION?` | the IARU region of the band presets: `VNAREGION: 1` |
| `VNAREGION 1` / `2` / `3` | sets the region, as Bank 1 `5` long |
| `VNA <command>` | any shell command, as typed, and its reply. Examples: `VNA version`, `VNA info`, `VNA cal`, `VNA vbat`, `VNA SN`, `VNA help` |

Each ends with `VNA -> ok` or what went wrong. `[VNA]` lines show the commands sent, the
measurement summary and any unexpected reply. `USB?` shows the USB connection.

## Test plan

On the second USB socket's console, with the NanoVNA-F V2 on the native socket:

1. Choose profile 70 (Bank 9 `1` long, `7` `0`, `D`): "nano v n a". Switch the NanoVNA on:
   "usb on".
2. `VNA version` and Bank 1 `7`: the version is printed and said. `VNA SN` and Bank 1 `9`: the
   serial number. `VNA info` prints the device
   information.
3. `VNA cal`: note what the calibration status looks like, for the calibration bank later.
   Bank 1 `8`: the battery voltage, as `VNA vbat` prints it in mV.
4. Bank 1 `3`: "band 8 0 meters". Bank 1 `B` and `C` step through the bands. Note how a band
   sounds.
5. With a 50 Ω load on port 1, Bank 1 `0`: "sweep", then a best and worst match near 1.0.
   `VNA?` shows the same result, and the NanoVNA's screen shows the band.
6. With port 1 open: a high SWR, maybe "99 plus".
7. With an antenna: compare the best match and its frequency with the NanoVNA's screen
   (SWR format, a marker at the minimum).
8. Bank 1 `1` long, `3` `*` `6` `D`; Bank 1 `2` long, `3` `*` `7` `D`; Bank 1 `3`: "start 3 point 6
   stop 3 point 7". Measure.
9. Bank 1 `6` long, `0`, `D`: "calibration 0 ok" if slot 0 holds a calibration. Measure. Then
   `6` long, `A`, `D`: "calibration auto". `B` and `C` now step the calibration.
10. Bank 1 `4` long, `8` `0` `D`: "band 8 0 meters". `4` long, `2` `3` `D`: "band 2 3
    centimeters". `4` long, `1` `*` `2` `5` `D`: "band error" (Region 1).
11. Bank 1 `5`: "region 1". `5` long, `2`, `D` on 80 m: "region 2 ok, band 8 0 meters", and
    `VNA?` shows 3500000 - 4000000 Hz. `4` long, `1` `*` `2` `5` `D`: "band 1 point 2 5 meters".
    `VNAREGION?` prints `VNAREGION: 2`; restart the controller, and it is still 2. `B` steps to
    region 3, `B` again goes round to region 1. `VNAREGION 1` at the end.
12. Bank 1 `0`, and `7` while the measurement runs: "busy".
13. Unplug the NanoVNA during a measurement: "usb off". Plug it in again and measure.
14. Choose another profile: the controller restarts, and the PC console is back on the native
    socket.

## Developer notes

- Profile: slot 70 in `firmware/radio_profile_table.cpp`, protocol `PROTO_NANOVNA`, port
  `RadioPort::UsbHost`.
  - It has no caps, and experimental mode leaves them off (`radio_catalog.cpp`). The frequency and
    S-meter polls, and the AI0 and frequency read when the port opens, never reach the shell.
    They once piled up on its shell line and hung the NanoVNA until a power cycle.
- `firmware/nanovna_codec.*` (no Arduino, host tests in `tests/nanovna`) holds:
  - the reply reader (lines and the `ch> ` prompt), echo and error recognition;
  - scan lines and SWR;
  - the best and worst summary;
  - the band presets and the command lines.
- `firmware/nanovna_client.*` runs the jobs from `pollNanoVna()` in the main loop:
  - It never blocks; a scan reply is read 512 bytes per loop.
  - Timeouts: 1 s for the sync CR, 2 s per command, 20 s for the scan.
  - It talks through `transport_serial`, which passes CR to the NanoVNA. CR and LF are dropped
    only for ASCII CAT radios such as the QMX.
  - The setup is kept in NVS (`vnastart`, `vnastop`, `vnacal`), the region apart from it
    (`vnaregion`), so a setup saved before the region still loads.
- The band tables of the three regions are in `nanovna_codec.cpp`. A band is held by its wavelength
  in cm (16000 for 160 m, 125 for 1.25 m) and named in metres from 1 m up.
- `firmware/ui_keypad_nanovna.cpp`: the Bank 1 actions and the speech. `KeypadLayout::NanoVna` in
  `keypad_keymap.*`; tests in `tests/keypad`.
- USB host: a scan reply is about 3 KB at once. The USB read waits for room in its 2 KB buffer
  instead of dropping bytes, and the NanoVNA is NAKed meanwhile (`serviceRx` in
  `transport_usb_host.cpp`).
- Shell facts:
  - The shell echoes the command line, ends lines with CR LF and prints `ch> ` when ready.
  - An unknown command answers `<command>?`, on the F V2 `Command not recognised.`. Errors start
    with `usage:` or `Err`, or say `invalid` or `exceeds`.
  - `scan` leaves the sweep paused on DiSlord's firmware, so `resume` follows it.
  - `recall` takes 0–6 on the F V2, 0–4 on older models.
  - The F V2 takes 11–201 scan points; 101 is used, which every model takes.
  - `vbat` replies `4256 mV`; Bank 1 `8` says it in volts to 10 mV.
  - On the F V2 (firmware 0.6.0, checked with `help`), `version` replies `0.6.0`, `cal`
    replies `load open short cal'ed`, and `SN` (upper case) replies 16 hex digits.
