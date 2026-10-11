# Hardware Wiring Reference (ESP32 Talking Remote Controller - V3.5.8)

> [!IMPORTANT]
> **Planning to build one? Please get in touch first.**
> The hardware is still evolving, so this documentation is deliberately kept minimal and may lag behind the bench.
> Before you order parts or PCBs, [open a build enquiry](https://github.com/jewelux/Talking-Remote-Controller-LX1WJ/issues/new?template=build-enquiry.yml) to get the current state of the board,
> recent changes and build tips. Feedback from builders is very welcome – every question helps to improve the documentation.

Schematic: [`circuit-diagram-V1.pdf`](circuit-diagram-V1.pdf) · PCB layout: [`pcb-layout-V1.pdf`](pcb-layout-V1.pdf) · Prototype photos: [`proto-pcbs/`](../proto-pcbs/)

> Looking for the compact V2 board (Mini-HamTRC, in development)? See [`hardware_wiring_v2.md`](../v2-mini-hamtrc/hardware_wiring_v2.md) and [`bom_v2.md`](../v2-mini-hamtrc/bom_v2.md).

This document describes the hardware wiring used by the current modular repository state based on
`firmware/TalkingRemoteControllerLX1WJ_V3_5_8.ino`.

All pin assignments listed below are taken from the current firmware files.

---

## 1. ESP32-S3 to 4x4 Matrix Keypad

### Pin Assignment

- Rows: `KP_ROW_PINS = {4, 8, 15, 16}`
- Columns: `KP_COL_PINS = {1, 2, 13, 21}`

### Wiring

| Pin | Keypad Signal | ESP32-S3 GPIO |
|---|---|---|
|1| ROW1 | GPIO 4 |
|2| ROW2 | GPIO 8 |
|3| ROW3 | GPIO 15 |
|4| ROW4 | GPIO 16 |
|5| COL1 | GPIO 1 |
|6| COL2 | GPIO 2 |
|7| COL3 | GPIO 13 |
|8| COL4 | GPIO 21 |

Note:
GPIO 1 and GPIO 2 can be special-purpose pins on some ESP32-S3 boards.
If the keypad works reliably on your hardware, this configuration is acceptable.

---

## 2. ESP32-S3 to I2S Audio Output

Example target: MAX98357A or a comparable I2S audio module.

### Wiring

| I2S Module Pin | ESP32-S3 GPIO |
|---|---|
| BCLK | GPIO 5 |
| LRC / WS | GPIO 6 |
| DIN | GPIO 7 |
| GND | GND |
| VIN | board supply, depending on the module |
| SD / EN | optional HIGH or open, depending on the module |

---

## 3. ESP32-S3 to ICOM CI-V Interface

### Firmware Pin Assignment

- `CIV_TX_PIN = GPIO17`
- `CIV_RX_PIN = GPIO18`
- Baud rate: set per radio profile (see `docs/radio-support-matrix.md`)

The ICOM CI-V interface uses a shared single-wire open-collector bus.
A transistor stage is used to interface the ESP32-S3 safely with the CI-V line.

### Transmit Stage

- GPIO17 -> 4.7 kOhm resistor -> BC548 base
- BC548 emitter -> GND
- BC548 collector -> CI-V line tip

### Receive Stage

- CI-V line tip -> 1 kOhm resistor -> GPIO18
- GPIO18 -> 3.3 V Zener diode -> GND

### Ground

- ESP32 GND <-> CI-V sleeve

Important note:
Directly connecting the CI-V line to an ESP32 GPIO pin without protection is not recommended.

---

## 4. ESP32-S3 to RS-232 Interface

### Wiring

- ESP32 TX GPIO10 <-> MAX3232 Pin named TX path
- ESP32 RX GPIO9 <- MAX3232 Pin named RX path
- ESP32 GND <-> MAX3232 GND <-> radio GND
- ESP32 5V -> MAX3232 VCC

---

## 5. ESP32-S3 to TTL CAT Interface

### Firmware Pin Assignment

- `CAT_TX_PIN = 11`
- `CAT_RX_PIN = 12`
- Baud rate: set per radio profile (see `docs/radio-support-matrix.md`)

### Wiring

- ESP32 TX (GPIO 11) -> 1 kOhm resistor -> radio RX
- ESP32 RX (GPIO 12) <- 1 kOhm resistor <- radio TX
- GND <-> GND

Series resistors provide basic protection and can improve signal robustness.

---

## 6. ESP32-S3 Native USB as USB Host (QRP Labs QMX, experimental)

A radio with only a USB connection (profile 21, QRP Labs QMX) plugs into the ESP32-S3's native
USB socket, GPIO 19 (D-) and GPIO 20 (D+). HamTRC is then the USB host. No wiring is needed
beyond the radio's USB cable.

On a board with two USB-C sockets (ESP32-S3-DevKitC-1 and copies such as the YD-ESP32-S3):

| Socket | Chip | Normal profiles | USB radio profile |
|---|---|---|---|
| native USB ("USB", "OTG") | ESP32-S3 itself | PC console and flashing | the radio |
| USB-to-serial ("COM", "UART") | CH343 or CP210x on UART0 | flashing | flashing, `HAMTRC?` and `USB?` only |

### 5 V on the native socket

A USB host supplies 5 V to the device. Most ESP32-S3 boards do not put 5 V out on the native
socket, because a diode blocks it. A radio that takes its USB power from the cable is then not
seen at all. The QMX has its own supply, but may still wait for 5 V on its USB socket.

- On the YD-ESP32-S3 and similar boards, close the solder jumper marked `USB-OTG`. It bypasses the
  diode, so the board's 5 V reaches the native socket.
- With that jumper closed, **never connect both sockets to a PC at the same time**: the two 5 V
  supplies would feed into each other.
- Without soldering, a USB-C OTG adapter with a power input can supply the 5 V instead.

A board with only the native USB socket works too, but then the PC console is gone while the USB
radio profile is active; flash it by holding BOOT while plugging it in.

---

## General Notes

- All interfaces share a common ground.
- Power is typically supplied via USB to the ESP32-S3.
- This document reflects the current V3.5.8 repository state.
- Future firmware versions may add profiles, interfaces, or alternative pin mappings.
- The hardware setup is still a development and test platform rather than a finished certified product.
