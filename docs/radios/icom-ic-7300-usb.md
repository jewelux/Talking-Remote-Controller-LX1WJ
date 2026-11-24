# Icom IC-7300 over USB

## Status

**Experimental, not yet tested on a radio.** Profile 24 (`Icom IC-7300 USB`) drives the IC-7300
through its own USB port instead of the CI-V jack (profile 1). It sends the same CI-V commands as
profile 1, so every function and key on the IC-7300 works the same. See
`docs/radios/icom-ic-7300.md` for the commands and keys.

What is new is the connection:

- the IC-7300's USB port is a **Silicon Labs CP210x** USB-UART bridge (`10C4:EA60`), not a
  CDC-ACM port like the IC-705's. The USB host recognises it by its VID:PID and sets it up with the
  CP210x vendor requests (UART on, 8N1, baud, DTR/RTS), see `transport_usb_host.cpp`;
- HamTRC keeps **DTR and RTS off**, so the IC-7300 cannot be keyed by them whatever its USB SEND
  and USB Keying menus say;
- the CP210x's UART runs at the profile's baud, **19200** by default. That works with the IC-7300's
  defaults whether the CI-V USB Port is linked to [REMOTE] (then the CI-V Baud Rate applies, up to
  19200) or unlinked (then the CI-V USB Baud Rate applies). Bank 8 `2` sets another rate.

## Connecting

- The IC-7300's USB cable (USB-B on the radio) goes into the controller's **native USB socket**.
- The PC console then uses the **second USB socket** (UART0). While the USB host runs, it answers
  only `HAMTRC?`, `USB?`, `SM?` and `CIVRAW`.
- The native socket needs **5 V** (USB-OTG bridge on the ESP32-S3 board), as for the IC-705. See
  `docs/hardware/v1-hamtrc/hardware_wiring.md`, section 6.

On the IC-7300, MENU > SET > Connectors > CI-V:

| Menu item | Setting | Why |
|---|---|---|
| CI-V Address | 94h (the default) | profile 24 uses 94; Bank 8 `1` long sets another |
| CI-V Baud Rate | Auto (the default) | used while CI-V USB Port is linked to [REMOTE] |
| CI-V USB Port | either | linked: up to 19200; unlinked: up to 115200 |
| CI-V USB Baud Rate | Auto (the default) | used while CI-V USB Port is unlinked |
| CI-V USB Echo Back | either | HamTRC ignores its own frames |
| CI-V Transceive | either | HamTRC hears the frequency and mode the IC-7300 sends when it changes |

MENU > SET > Connectors: **USB SEND**, **USB Keying (CW)** and **USB Keying (RTTY)**: OFF is the
safest. HamTRC keeps DTR and RTS off, but another program on a PC would not.

## What you hear

| Situation | Spoken |
|---|---|
| Profile 24 at boot or when chosen | "icom 7 3 0 0 usb" |
| The IC-7300's CI-V port is ready | "usb on" |
| The IC-7300 is unplugged or switched off | "usb off" |
| A USB device without a serial port | "usb error", after 3 s if no port opens meanwhile |

## Test plan

The console commands go to the **second USB socket**.

1. Choose profile 24 (Bank 9 `1` long, `2` `4`, `D`): "icom 7 3 0 0 usb". `USB?` says
   `USB host: on`, `serial port: none` and `opens: CDC or CP210x port 0 (counted from 0), 19200
   baud, DTR off, RTS off`.
2. Connect the IC-7300 and switch it on: "usb on". The console shows
   `device 10C4:EA60 ...` and `CP210x serial port: interface 0, ...`, then `serial port open`.
   `USB?` shows `port: CP210x port 0`. Lines `line setup refused` mean the CP210x refused a vendor
   request: note which ones. Other devices in the radio (a hub, the USB audio codec) are listed and
   skipped.
3. Bank 1 `0`: the frequency. No answer ("timeout"): `CIVRAW? 03` prints every reply; check the
   CI-V Address and that the CI-V (USB) Baud Rate is Auto or 19200.
4. Mode, VFO knob, S-meter (Bank 1 `7`, `SM?`) with an antenna, Bank 2 and 3 keys: as on profile 1.
5. With a dummy load or an antenna: RF power, PO, SWR (Bank 1 `4`, `8`), transmit state (Bank 1
   `1`).
6. With a dummy load on, set USB SEND to USB(A) DTR, unplug and plug in the USB cable: "usb on",
   and the IC-7300 stays in receive. Set USB SEND back to OFF afterwards.
7. Unplug: "usb off". Plug in again: "usb on", and the frequency works.
8. Optional: CI-V USB Port unlinked, CI-V USB Baud Rate 115200, Bank 8 `2` to 115200: the
   controller restarts, and the frequency works at the higher rate.
9. Choose profile 1 (Bank 9 `1` long, `1`, `D`), with the IC-7300 unplugged: the controller
   restarts and the PC console is back on the native socket.

## Developer notes

- Profile: slot 24 in `firmware/radio_profile_table.cpp`, the IC-7300's caps and settings
  (`kIc7300Caps`, `kIc7300Settings`), model `Ic7300`, port `UsbHost`, `usbSerialIndex = 0`,
  `usbDtr` and `usbRts` false, 19200.
- CP210x: `isCp210x()` and `findCp210xSerialPort()` in `usb_cdc_descriptor.*` (host tests in
  `tests/usb`); the vendor requests IFC_ENABLE (00h, 0001h), SET_LINE_CTL (03h, 0800h = 8N1),
  SET_BAUDRATE (1Eh, 4 bytes) and SET_MHS (07h, 0300h plus DTR/RTS bits) from Silicon Labs AN571.
  A CDC-ACM port is always tried first, so the IC-705, QMX, X6100 and NanoVNA are unchanged.
- The same path should take other radios and cables with a CP210x (the IC-9700, some USB CAT
  cables). Other USB-UART chips (FTDI, CH34x without CDC-ACM) still need their own setup.
