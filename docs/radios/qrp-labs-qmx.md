# QRP Labs QMX and QMX+

## Status

**Experimental, not yet tested on a radio.** Profile 21 (`QRP Labs QMX`) was built from the QMX
CAT manual (firmware 1.04_003) and operating manual (firmware 1.02_000). The QMX itself has not
been connected yet.

| Function | QMX command | Notes |
|---|---|---|
| Frequency read and set | `FA` | VFO A, 11 digits |
| Mode read and set | `MD` | LSB 1, USB 2, CW 3, AM 5, CW-R 7, DIGI 9 (USB; 6, Digi on LSB, is read as DIGI too) |
| S-meter (Bank 1 `7`, console `SM?`) | `SM` | in dB; said as S0 to S9, then S9 plus tens of dB. `SM?` also answers on the second socket, with the raw reply |
| Output power (Bank 1 `4`, console `PO?`) | `PC` | in tenths of a watt, said as watts ("power 4.5 watts"); "power rx" in receive. Also on the second socket |
| SWR (Bank 1 `8`, console `SWR?`) | `SW` | in hundredths, said as "swr 1.21"; "swr rx" in receive. Also on the second socket |
| Radio ID and model (console `ID?`, `OM?`, `IF?`) | `ID`, `OM`, `IF` | `ID020` (TS-480), `OMQC` |
| USB connection state (console `USB?`, Bank 9 `1`) | | also on the second USB socket |

While the QMX is connected, the full console is not reachable (see Connecting): use the keypad.
`ID?`, `OM?` and `IF?` are for a later setup with the console on the second socket.

Not yet: VFO B, split, RIT, PTT and the QMX menu commands.

## Connecting

The QMX has no RS-232 or CI-V port. Its USB-C socket carries a USB sound card and up to three
virtual COM ports; the first one is the CAT port. HamTRC talks to it as a **USB host** on the
ESP32-S3's native USB socket:

- the QMX's USB cable goes into the controller's **native USB socket**, the one that is normally
  the PC console (USB JTAG/serial, VID 303A);
- the PC console then uses the **second USB socket**, the USB-to-serial chip (a CH343 or CP210x)
  wired to UART0. While the USB host runs, it answers only `HAMTRC?`, `USB?`, `SM?`, `PO?` and
  `SWR?`;
- flashing works through the second socket as usual.

The QMX has its own supply, so the native socket does not need to supply 5 V; see
`docs/hardware_wiring.md`, section 6.

The QMX ignores the baud rate on its virtual COM port; the profile lists 9600 only. It keys
nothing on DTR or RTS. HamTRC never sends a carriage return or line feed to it: a carriage return
switches the QMX port into its terminal mode, which ends CAT until the QMX is switched off and on.
Mode 8 is never sent either: on the QMX it is SWR tune mode, not RTTY-R.

## What you hear

| Situation | Spoken |
|---|---|
| Profile 21 at boot or when chosen | "q r p q m x" |
| The QMX serial port is ready | "usb on" |
| The QMX is unplugged or switched off | "usb off" |
| A USB device without a serial port (sound card, CP210x adapter) | "usb error", after 3 s if no serial port opens meanwhile |
| The controller restarted after a crash with this profile | "usb error"; the USB host stays off until profile 21 is chosen again |

### Bank 9 `1`: the USB state, without a computer

| Spoken | Meaning | What to try |
|---|---|---|
| "usb on", "tx" bytes, "rx" bytes | the serial port is open; bytes sent to and received from the QMX since the start | press Bank 1 `0` (frequency), then Bank 9 `1` again: "tx" grows; if "rx" stays zero, the QMX does not answer CAT (check that its first COM port is the CAT port; switch it off and on) |
| "usb off zero" | the host runs, but no USB device has reached the controller | check the cable, and that the QMX is switched on |
| "usb off" and a number above zero | devices came, but none is open now: no CDC serial port, or it was unplugged | unplug the QMX and plug it in again; listen for "usb on" |
| "usb error" | the host could not start, or was held after a crash restart | choose profile 21 again (Bank 9 `A` long, `2` `1`, `D`) |
| "usb not available" | the active profile does not use USB | choose profile 21 |

Choosing another profile restarts the controller, so the native socket goes back to the PC
console. The new profile is announced after the restart. The NanoVNA profile (30) also restarts
it while the QMX is connected, so that nothing meant for the NanoVNA reaches the QMX.

## Test plan

1. On the second USB socket's console, `USB?` says `USB host: on` and `serial port: none`.
2. Plug in the QMX and switch it on: "usb on". `USB?` shows the device and `serial port: open`.
   If nothing is heard, `USB?` lists what the controller saw, and Bank 9 `1` says it without a
   computer (see the table above). With no device at all, the `port:` line of `USB?` tells
   whether the QMX shows itself on the cable (`D+=1`); see the developer notes below.
3. Bank 1 `0` short: the frequency. Tune the QMX: the new frequency is announced.
4. Mode select: LSB, USB, CW, DIGI; Bank 3 mode query after each.
5. S-meter: `SM?` on the second socket prints the raw reply and the S value, and says it; Bank 1
   `7` says it from the keypad. With no antenna it should be near S0, with a strong signal S9
   plus. Note the raw value for the developer notes below.
6. Power and SWR in receive: `PO?` and `SWR?` on the second socket print `RX (not transmitting)`;
   Bank 1 `4` says "power rx", Bank 1 `8` "swr rx".
7. Power and SWR while transmitting, into a dummy load (key down, or a Digi tone from the PC):
   `PO?` prints the watts and `SWR?` the SWR; on a dummy load the SWR should be near 1.0. If the
   QMX answers in receive with a number instead of the bare reply, note it for the developer
   notes below.
8. Unplug the QMX: "usb off". Plug it in again: "usb on", and the frequency query works.
9. Choose another profile: the controller restarts and the PC console is back on the native socket.

## Developer notes

- Profile: `kQmxCaps`, `kQmxCommands`, `kQmxModes` in `firmware/radio_profile_table.cpp`, protocol
  `KENWOOD_ASCII`, port `RadioPort::UsbHost`, model `RadioModel::Qmx`.
- S-meter: the CAT manual says only that `SM;` returns the S-meter value in dB. The operating
  manual calibrates the meter at 6 dB per S unit, S0 at -127 dBm, S9 at -73 dBm, up to S9+36 dB
  (-37 dBm). Until a reply has been seen from a radio, `qmxDecodeSMeter()` (`firmware/qmx_codec.*`,
  tests in `tests/qmx`) reads a negative value as dBm and zero and above as dB over S0. Whole S
  units below S9, tens of dB above, as for the FT-847.
- Power and SWR: `PC;` returns the measured output in tenths of a watt (`PC45;` is 4.5 W), `SW;`
  the SWR in hundredths (`SW121;` is 1.21). The CAT manual says `SW;` comes back bare in receive;
  Hamlib takes a bare `PC;` the same way. `queryQmxPowerTenths()` and `queryQmxSwrHundredths()`
  (`firmware/radio_protocol.cpp`) report a bare reply as receive, through `qmxParseTxMeter()`
  (`firmware/qmx_codec.*`, tests in `tests/qmx`); the console's `PO?` and `SWR?` take this path
  for `RadioModel::Qmx`, before the generic TS-480 meter readings.
- USB host and CDC-ACM: `firmware/transport_usb_host.*`, on the ESP-IDF USB Host Library of the
  ESP32 core (no extra library). It opens the first CDC data interface with a bulk IN and OUT
  endpoint, sends SET_LINE_CODING and SET_CONTROL_LINE_STATE (DTR on) to the ACM control interface
  before it, and moves bytes through two stream buffers. External hubs are supported by the core.
- The ESP32 core is built with `CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK`. With it, the USB Host
  Library cancels every device after SET_ADDRESS unless `usb_host_config_t.enum_filter_cb` is set,
  and says so only in a warning the core does not print. `acceptEveryDevice()` takes every device
  in its first configuration. Before it, no device was ever seen, which looked like missing 5 V.
- While the host runs, `USB?` also prints the registers: the root port (`port:` connected,
  enabled, D+ and D- levels, speed), the PHY routing (`phy:` must say USB host), the host channels
  (`channel n:` address, endpoint, `chena=1` while a transfer is still running, e.g. a device that
  keeps refusing data) and the devices the USB Host Library knows. `D+=1` with no device seen means
  the device is there but was not enumerated; `D+=0` means it does not show itself.
- `transport_serial.*` routes the radio link to it when the active port is `UsbHost`, and drops CR
  and LF on the way.
- `pollRadioLinkEvents()` in `firmware/radio_monitor.cpp` speaks the events, sends `AI0;` and
  refreshes the frequency and mode when the port opens. Auto-info goes off because a PC program
  may have left it on, and its `IF` lines would then answer our queries; Hamlib does the same.
- Cross-checked with Hamlib's QMX backend (`qrplabs_qmx_caps`, `rigs/kenwood/ts480.c`, October
  2026): it is TS-480 Kenwood CAT and an emulation, so digital USB is `MD9` and digital LSB `MD6`,
  as in this profile. Its `SW` and `PC` readings (hundredths, tenths of a watt, empty while
  receiving) match the ones here.
