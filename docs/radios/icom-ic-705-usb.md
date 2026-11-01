# Icom IC-705 over USB

## Status

**Experimental, not yet tested on a radio.** Profile 23 (`Icom IC-705 USB`) drives the IC-705
through its own USB port instead of the CI-V jack (profile 17). It sends the same CI-V commands as
profile 17, so every function and key on the IC-705 works the same. See
`docs/radios/icom-ic-7300.md` for the commands and keys, and the
[radio support matrix](../radio-support-matrix.md) for the IC-705's settings.

What is new is only the connection:

- the IC-705's USB port shows two serial ports (Icom's driver calls them "Serial Port A (CI-V)"
  and "Serial Port B"); HamTRC opens **port A**;
- HamTRC keeps **DTR and RTS off**, so the IC-705 cannot be keyed by them whatever its USB SEND
  and USB Keying menus say;
- the baud goes out with the port setup; it matters only when the IC-705's CI-V USB Baud Rate is
  not Auto.

## Connecting

- The IC-705's USB cable goes into the controller's **native USB socket**.
- The PC console then uses the **second USB socket** (the USB-to-serial chip on UART0). While the
  USB host runs, it answers only `HAMTRC?`, `USB?`, `SM?` and `CIVRAW`.
- The controller most likely has to put **5 V on the native socket**: the IC-705 is not expected to
  show itself on USB without it. See `docs/hardware/v1-hamtrc/hardware_wiring.md`, section 6.

On the IC-705, in its CI-V settings:

| Menu item | Setting | Why |
|---|---|---|
| CI-V Address | A4 (the default) | profile 23 uses A4; Bank 8 `1` long sets another |
| CI-V USB Baud Rate | Auto (the default) | any other rate: set the same with Bank 8 `2` |
| CI-V USB Port | Unlink from [REMOTE] | keeps the USB port apart from the CI-V jack; linked works too |
| CI-V USB Echo Back | either | HamTRC ignores its own frames |
| CI-V Transceive | either | HamTRC hears the frequency and mode the IC-705 sends when it changes |

Elsewhere on the IC-705:

- **USB Power Input**: set it **OFF**. Left on (the default), the IC-705 takes power from the
  USB port to run and charge its battery, and the controller's 5 V is not made for that.
- **USB SEND** and **USB Keying (CW, RTTY)**: OFF is the safest. HamTRC keeps DTR and RTS off,
  but another program on a PC would not.

## What you hear

| Situation | Spoken |
|---|---|
| Profile 23 at boot or when chosen | "icom 7 0 5 usb" |
| The IC-705's CI-V port is ready | "usb on" |
| The IC-705 is unplugged or switched off | "usb off" |
| A USB device without a serial port | "usb error", after 3 s if no port opens meanwhile |

Choosing another profile restarts the controller while the IC-705 is connected. A change of the
baud on Bank 8 `2` restarts it too, so that the port opens with the new rate.

## Test plan

Follow the IC-705 checks of profile 17, with these first. The console commands go to the
**second USB socket**.

1. `USB?` says `USB host: on`, `serial port: none` and `opens: CDC port 0 (counted from 0),
   115200 baud, DTR off, RTS off`.
2. Connect the IC-705 and switch it on: "usb on". `USB?` shows `device: 0C26:0036` and `port: CDC
   port 0`. Nothing heard: Bank 8 `0` says what the controller saw; "usb off zero" means that no
   device showed itself, which is usually 5 V missing on the socket.
3. Bank 1 `0`: the frequency. No answer ("timeout"): `CIVRAW? 03` prints every reply; a reply
   `from=` another address than A4 gives the IC-705's address. No reply at all: port A may be the
   wrong one; note what `USB?` prints.
4. Bank 1 `7`, `SM?`: the S-meter. Mode select, NR, the tuner, RF power (0 to 10 W): as on
   profile 17.
5. With a dummy load on, set the IC-705's USB SEND to USB (A) DTR, then unplug and plug in the
   USB cable: "usb on", and the IC-705 stays in receive. Set USB SEND back to OFF afterwards.
6. Unplug: "usb off". Plug in again: "usb on", and the frequency works.
7. Choose profile 17 (Bank 9 `1` long, `1` `7`, `D`): the controller restarts and the PC console is
   back on the native socket.

## Developer notes

- Profile: slot 23 in `firmware/radio_profile_table.cpp`, the IC-705's caps and settings
  (`kIc705Caps`, `kIc7300Settings`), model `Ic705` (its menu addresses), port `UsbHost`,
  `usbSerialIndex = 0`, `usbDtr` and `usbRts` false, 115200 by default from `kCivBauds`.
- USB `0C26:0036`; Icom's Windows driver is a CDC-ACM INF (`icom_usb_cdcacm.inf`) with CI-V on
  interface 0 (`MI_00`). The order of its two ports has not been seen from HamTRC yet.
