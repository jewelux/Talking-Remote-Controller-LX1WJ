# Xiegu X6100

## Status

**Experimental, not yet tested on a radio.** Profile 22 (`Xiegu X6100`) connects to the X6100's
**DEV** USB port. It was built from:

- the CI-V table in the Radioddity Extended Manual for the X6100 (v1.1.8, 2024), chapter 15, and
  its firmware release notes, chapter 12;
- Hamlib's X6100 driver (`rigs/icom/xiegu.c`);
- the USB descriptors of the X6100's DEV port, read on a PC.

The X6100 speaks a subset of Icom's CI-V. HamTRC offers only the commands that the Radioddity
table lists, that Hamlib uses the same way, and that HamTRC sends the same way as on the Icoms.
The X6100 answers a level it does not have with 0 instead of an error, so a guessed command would
report a wrong value instead of "not available".

| Function | Keys | CI-V | Notes |
|---|---|---|---|
| Frequency read and set | Bank 1 `0` | `03`, `05` | |
| Mode read and set | Bank 1 `9` | `04`, `06` | LSB, USB, AM, CW, FM, CW-R. No RTTY: mode select offers only these |
| S-meter | Bank 1 `7` | `15 02` | Icom's scale (S9 = 120) from firmware 1.1.8 |
| SWR | Bank 1 `8` | `15 12` | Icom's scale from firmware 1.1.8 |
| RX or TX | Bank 1 `1`, Bank 3 `6` | `1C 00` | |
| Dial lock | Bank 1 `3` | `16 50` | |
| RF power | Bank 1 `6`, Bank 4 and 7 `8` | `14 0A` | in watts, 0 to 10 W; the radio takes it as a share of 10 W |
| Noise reduction, its level | Bank 2 `1` | `16 40`, `14 06` | level in percent |
| Noise blanker, its level | Bank 2 `2` | `16 22`, `14 12` | level in percent |
| AGC | Bank 2 `7` | `16 12` | off, fast, mid, slow |
| Preamp | Bank 2 `8` | `16 02` | off and on |
| VFO A and B, their frequency and mode | Bank 3 `1`, `2`, `4`, `5` | `07`, `25`, `26` | firmware 1.1.6 or later |
| Tuner, tune | Bank 4 and 7 `0` | `1C 01` | the built-in ATU |
| Speech processor (COMP), its level | Bank 4 `1` | `16 44`, `14 0E` | |
| VOX, its gain | Bank 4 `2` | `16 46`, `14 16` | |
| Mic gain | Bank 4 `7` | `14 0B` | |
| RIT | Bank 5 | `21 00`, `21 01` | firmware 1.1.6 or later |
| CI-V address | Bank 8 `1` | | A4 |
| USB connection state | Bank 8 `0`, console `USB?` | | |

These say "not available" on the X6100, and why:

- **Attenuator**: the X6100's `11` switches the attenuator over whatever it is sent, so "off"
  could switch it on.
- **Split**: the table has no read, and Hamlib found that the X6100 refuses split on.
- **TX frequency, output power and ALC meters**: no TX frequency command; the meters have no
  documented scale.
- **Notch and auto notch**: the X6100's DNF is a manual notch, not Icom's auto notch.
- **PBT, filter, monitor, break-in, CW pitch, VOX delay, transceive, band stack, tone squelch and
  repeater shift**: not in the X6100's table, or in another format.

## Connecting

The X6100's **DEV** port carries a USB hub with a sound card and a dual serial chip (CH342). The
chip has two ports: **SERIAL-A** and **SERIAL-B**. CAT is on **SERIAL-B**, and HamTRC opens that
one.

- The X6100's USB cable (DEV port) goes into the controller's **native USB socket**.
- The PC console then uses the **second USB socket** (the USB-to-serial chip on UART0). While the
  USB host runs, it answers only `HAMTRC?`, `USB?`, `SM?` and `CIVRAW`.
- The controller must put **5 V on the native socket**: the hub in the X6100 does not show itself
  without it. See `docs/hardware/v1-hamtrc/hardware_wiring.md`, section 6.
- The serial chip passes the baud on to the radio, so it matters here: **19200**, as the manual
  gives for CAT. The profile offers only that rate.
- HamTRC keeps **DTR and RTS off**. The X6100 keys CW on RTS.

On the X6100:

- Use firmware **APP 1.1.8 or later**. Older firmware gives the S-meter and SWR on another scale,
  and before 1.1.6 the VFO B and RIT commands are missing.
- The CI-V address is **A4**, as in Hamlib; the manual gives none. If the X6100 does not answer,
  see the test plan, step 3.
- The CAT echo setting can be on or off: HamTRC ignores its own frames.

## What you hear

| Situation | Spoken |
|---|---|
| Profile 22 at boot or when chosen | "xiegu x 6 1 0 0" |
| The X6100's CAT port is ready | "usb on" |
| The X6100 is unplugged or switched off | "usb off" |
| A USB device without SERIAL-B (another radio, a sound card alone) | "usb error", after 3 s if no port opens meanwhile |

The sound card in the X6100 is seen as a device without a serial port. It does not cause "usb
error" when SERIAL-B opens within 3 seconds.

Choosing another profile restarts the controller while the X6100 is connected. The restart
happens also between profile 22 and the IC-705 on USB (23), which use another port and baud.

## Test plan

A dummy load on the antenna jack for steps 9 and 10. The console commands go to the **second USB
socket**.

1. `USB?` says `USB host: on`, `serial port: none` and `opens: CDC port 1 (counted from 0),
   19200 baud, DTR off, RTS off`.
2. Connect the X6100's DEV port and switch it on: "usb on". `USB?` shows `device: 1A86:55D2`
   and `port: CDC port 1 (counted from 0), interface 3, 19200 baud, DTR off, RTS off`. Nothing
   heard: Bank 8 `0` says what the controller saw; "usb off zero" means that no device showed
   itself, which is usually 5 V missing on the socket.
3. Bank 1 `0`: the frequency. No answer ("timeout"): `CIVRAW? 03` prints every reply. A reply
   `from=` another address than A4 gives the X6100's address; set it with Bank 8 `1` long. No
   reply at all: turn the X6100's VFO knob during `CIVRAW? 03` and note what comes.
4. Tune the X6100 by hand: HamTRC says the new frequency. Set one with Bank 1 `0` long.
5. Mode select (Bank 1 `9` long): LSB, USB, AM, CW, FM, CW-R are offered, nothing else. Set each
   and read it back with Bank 1 `9`.
6. S-meter: `SM?` prints the raw value and the S value. No antenna: near S0; a strong station:
   S9 or above. Compare with the X6100's display.
7. Bank 2: NR and NB on and off, their levels; AGC off, fast, mid, slow (Bank 2 `7` long, `0`,
   `2`, `4`, `3`); preamp on and off. Check each on the X6100's display. For the preamp, also note
   what `CIVRAW? 16 02` prints with the preamp on (`payload=02 01` expected).
8. Bank 3 `1` long and `2` long select VFO A and B; `1` and `2` say their frequencies, `4` and `5`
   their modes. Bank 5: RIT on, a few steps, off.
9. RF power (Bank 4 `8`): set 5 W, check on the X6100. Tuner (Bank 4 `0` double): it tunes.
10. Transmit into the dummy load: Bank 1 `1` says TX, `8` the SWR (near 1.0). For the meters
    not yet offered, note what these print while transmitting at a known power: `CIVRAW? 15 11`
    (power) and `CIVRAW? 15 13` (ALC).
11. For the attenuator not yet offered: `CIVRAW? 11` with the attenuator off, then on. Note both
    replies.
12. Bank 4: processor, VOX and its gain, mic gain: read and set each, check on the display.
13. Unplug: "usb off". Plug in again: "usb on", and the frequency works.
14. Choose profile 23 (Bank 9 `1` long, `2` `3`, `D`): the controller restarts.

## Developer notes

- Profile: `kX6100Caps`, `kX6100Settings`, `kX6100Modes`, `kX6100Bauds` in
  `firmware/radio_profile_table.cpp`; protocol `CI-V`, port `UsbHost`, model `Generic`. The
  comments there name the source of each command and why the others are left out.
- `link.usbSerialIndex = 1` opens the second CDC-ACM port (`findCdcSerialPort()`,
  `firmware/usb_cdc_descriptor.*`, tests in `tests/usb`); `link.usbDtr` and `usbRts` false;
  the line coding carries the profile's baud.
- USB descriptors read on a PC (2026-10-31): a GL850G hub `05E3:0608`, a CM108 sound card
  `0D8C:0012` (audio and HID), and a CH342 `1A86:55D2` with an interface association per port:
  interfaces 0/1 port A, 2/3 port B, bulk OUT 32 bytes, IN 64 bytes.
- `RadioProfile::civModes` limits mode select to the X6100's modes (table 2-2: 00, 01, 02, 03,
  05, 07); `AgcEncoding::CivWithOff` adds AGC off (`16 12 00`).
- The X6100's CI-V address may also be read with `19 00` (Hamlib: `00 A4` or `61 00`) and its
  Xiegu ID with `1D 19`.
- Not yet used, worth a test with `CIVRAW`: `1A 06` data mode (U-DIG, L-DIG), `14 0C` key speed,
  `14 0F` QSK time, `14 15` monitor level, `14 0D` DNF frequency, `15 15` supply voltage
  (0 = 0 V, 75 = 5 V, 241 = 16 V).
