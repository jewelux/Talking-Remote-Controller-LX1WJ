# Hardware Wiring Reference – HamTRC V2 (Mini-HamTRC)

> [!IMPORTANT]
> **This page describes the current Mini-HamTRC V2 (`development` branch).**
> The `main` branch still contains the first prototype (V1, with SD card). Do not use `main` for a V2 build.
> A step-by-step build page written for screen readers is at [lx1wj.eu – Build a Mini-HamTRC](https://lx1wj.eu/hamtrc/mini-review/index-en.html)
> (also in [German](https://lx1wj.eu/hamtrc/mini-review/index.html) and [French](https://lx1wj.eu/hamtrc/mini-review/index-fr.html)).
> Questions are welcome: [open a build enquiry](https://github.com/jewelux/Talking-Remote-Controller-LX1WJ/issues/new?template=build-enquiry.yml).

> **Status: current V2 build (2026-11-25).** V2 is the compact successor of the original board.
> This page describes the V2 as it is built now. The schematic shows the protection of GPIO 9 with R5, 12 kΩ (variant B, section 5,
> added 2026-11-25), the external 5 V input J5 and the supply capacitors C5 and C6. The KiCad source files are in
> [`MiniHamTRC-V2-KiCad/`](MiniHamTRC-V2-KiCad/). For the V1 board see [`hardware_wiring.md`](../v1-hamtrc/hardware_wiring.md).

| Source | File |
|---|---|
| Schematic | [`circuit-diagram-V2.pdf`](circuit-diagram-V2.pdf) – KiCad 10.0.1, Rev V3.5, 2026-09-01 |
| KiCad project | [`MiniHamTRC-V2-KiCad/`](MiniHamTRC-V2-KiCad/) – schematic, PCB and libraries (KiCad 10.0) |
| PCB layout | [`pcb-layout-V2.pdf`](pcb-layout-V2.pdf) |
| Bill of materials | [`bom_v2.md`](bom_v2.md) |
| Firmware pin map | [`firmware/config_pins.h`](../../../firmware/config_pins.h) |

All GPIO assignments below were read from the V2 schematic and cross-checked against
`firmware/config_pins.h`. They are **identical to V1**, so the current firmware runs on V2 without changes.

---

## What changed from V1

| Topic | V1 | V2 (Mini-HamTRC) |
|---|---|---|
| Supply | 12 V input, DC-DC module (U2), power switch (J11), 1000 µF buffer (C6) | external 5 V on J5; ESP32 USB-C ports only for flashing; no DC-DC module, no switch |
| SD card module | J5 SD module (SPI) | **removed** (the firmware does not use it) |
| CI-V jack | 3-pin header / 3.5 mm | 3.5 mm jack J6 (Tip = CI-V, Ring unused) |
| CAT-TTL | 3-pin header J3 | 3.5 mm jack footprint J3 |
| Board | larger single-sided board | compact single-sided board, 4 mounting holes |
| GPIO mapping | – | unchanged |

---

## 1. Power Supply

The Mini-HamTRC is powered from an **external 5 V supply** through its own power input.
The USB-C ports of the ESP32-S3 board are **not** used to power the Mini.

| Connector | Pin | Signal |
|---|---|---|
| J5 "VCC In 5V" | 1 | +5 V |
| J5 | 2 | GND |

- **Power input:** J5 takes the external 5 V. Which socket or cable the builder fits on the box and wires to J5
  is up to them.
- **Power supply:** a good, stable 5 V supply, at least 1 A.
- The **5 V rail** connects J5 pin 1, U1 pin 21 `5V0` (5V pin of the ESP32 board), the MAX98357A (J4 pin 1 `Vin`)
  and the MAX3232 module (J2 pin 1 `VCC`).
- **C6 470 µF** electrolytic (≥ 10 V) with **C5 100 nF** ceramic in parallel decouple the 5 V rail to GND.
  Minus of C6 to GND (pad 2 on the PCB).
- **D1 position on the PCB: wire link.** No diode is needed.
- The ESP32-S3 board's own 3V3 output (U1 pins 1, 2) feeds the MAX98357A `SD` pin.
- **Do not close the USB-OTG solder bridge** on the ESP32-S3 board. USB control of radios comes with V3.

**Flashing:** the ESP32-S3 board's USB-C ports are used only to flash the firmware from a computer.
While flashing, the board is powered by the computer and the Mini does not control a radio.
After flashing, disconnect the computer and power the Mini again from its external 5 V input.

**Why not USB power:** on the ESP32-S3 board, diodes separate the USB ports from its 5V pin. Powered only
through USB, the two MAX modules did not get a proper supply: the 5 V rail sat at about 2.5 V, the speech
crackled and the RS-232 interface misbehaved (issues #29 and #31).

---

## 2. ESP32-S3 to 4x4 Matrix Keypad (J8, 2x4 header)

Firmware: `KP_ROW_PINS = {4, 8, 15, 16}`, `KP_COL_PINS = {1, 2, 13, 21}`

| J8 Pin | Keypad Signal | ESP32-S3 GPIO |
|---|---|---|
| 1 | ROW1 | GPIO 4 |
| 2 | ROW2 | GPIO 8 |
| 3 | ROW3 | GPIO 15 |
| 4 | ROW4 | GPIO 16 |
| 5 | COL1 | GPIO 1 |
| 6 | COL2 | GPIO 2 |
| 7 | COL3 | GPIO 13 |
| 8 | COL4 | GPIO 21 |

J8 pins 1…8 map 1:1 to the keypad's 8-pin ribbon connector (pins 1–8).

---

## 3. ESP32-S3 to I2S Audio (J4, MAX98357A breakout)

| J4 Pin | MAX98357A | Connection |
|---|---|---|
| 1 | Vin | +5 V rail |
| 2 | GND | GND |
| 3 | SD | ESP32 3V3 (amplifier permanently enabled) |
| 4 | GAIN | not connected (module default gain) |
| 5 | DIN | GPIO 7 (`I2S_DOUT_PIN`) |
| 6 | BCLK | GPIO 5 (`I2S_BCLK_PIN`) |
| 7 | LRC / WS | GPIO 6 (`I2S_LRCLK_PIN`) |

The speaker connects to the screw terminal of the MAX98357A breakout.
`AMP_SD_PIN = -1` in the firmware, which matches the hard-wired SD pin.

**Supply decoupling:** C6 **470 µF** electrolytic (≥ 10 V) and C5 **100 nF** ceramic in parallel, from the 5 V rail
(`Vin`) to GND, close to the module, minus of C6 to GND.

---

## 4. ESP32-S3 to ICOM CI-V (J6, 3.5 mm jack)

Firmware: `CIV_TX_PIN = GPIO17`, `CIV_RX_PIN = GPIO18`

| J6 Contact | Signal |
|---|---|
| Tip (1) | CI-V bus |
| Ring (2) | not connected |
| Sleeve (3) | GND |

**Transmit (open collector)**
- GPIO17 → R2 **4.7 kΩ** → Q1 **BC547** base
- Q1 emitter → GND
- Q1 collector → CI-V Tip

**Receive**
- CI-V Tip → R1 **1 kΩ** → GPIO18
- GPIO18 → ZD1 **3.3 V Zener** (cathode on GPIO18) → GND

Note: the transmit stage inverts the signal; the firmware handles this (`kCivJackPins` in `transport_serial.cpp`).
A BC548 or similar small-signal NPN works equally well.

---

## 5. ESP32-S3 to RS-232 (J2, MAX3232 TTL↔RS-232 module)

The module is soldered onto the two large pads at the top edge of the PCB (see section 8).

Firmware: `RS232_RX_PIN = 9`, `RS232_TX_PIN = 10`

| J2 Pin | MAX3232 module | Connection |
|---|---|---|
| 1 | VCC | +5 V rail |
| 2 | RXD (TTL out) | GPIO 9 through **R5 12 kΩ** (or variant A), see below |
| 3 | TXD (TTL in) | GPIO 10 |
| 4 | GND | GND |

**Protection of GPIO 9 (R5 in the schematic, variant B):** the module's RXD output gives about 4.6 V, more than the
ESP32-S3 input tolerates. Do not connect RXD directly to GPIO 9. Two tested variants:

| Variant | Parts | Measured at GPIO 9 | Note |
|---|---|---|---|
| A | 4.7 kΩ from RXD to GPIO 9, 10 kΩ from GPIO 9 to GND | ≈ 3.2 V | independent of the firmware |
| B | 12 kΩ from RXD to GPIO 9 | ≈ 3.2 V | one part fewer; relies on the load of the ESP32 input, so the voltage depends on how the firmware configures the pin |

The DB9 connector on the module goes to the radio's RS-232 CAT port.

| Radio | Cable |
|---|---|
| Yaesu FTDX10 | null modem cable or adapter (the radio is wired like a modem) |
| Yaesu FT-847 | straight cable |

---

## 6. ESP32-S3 to TTL CAT (J3)

Firmware: `CAT_TX_PIN = 11`, `CAT_RX_PIN = 12`

| J3 Pin | Signal | Connection |
|---|---|---|
| 1 | Radio RX | GPIO 11 via R3 **1 kΩ** |
| 2 | Radio TX | GPIO 12 via R4 **1 kΩ** |
| 3 | GND | GND |

The series resistors limit current if the radio's TTL level does not match exactly.
Check the radio's TTL level (3.3 V vs. 5 V) before connecting.

---

## 7. Unused ESP32-S3 GPIOs

Marked "no connect" in the V2 schematic: GPIO 0, 3, 14, 19, 20, 35–48 and RST.
They are available for future extensions (keep strapping pins GPIO 0, 3, 45, 46 in mind).

---

## 8. PCB Notes (from `pcb-layout-V2.pdf`)

- Single-sided board: copper and ground pour on the bottom layer (blue in the PDF).
- **6 wire bridges** on the component side (red straight links in the PDF), fitted before the modules.
- 4 mounting holes in the corners.
- The ESP32-S3 dev board, MAX98357A and MAX3232 modules plug into female headers.
- **Two large rectangular pads at the top edge**: solder pads for mounting the **RS-232 (MAX3232) module**
  (mechanical fixing, no electrical net).
- **D1 footprint** (left edge): fit a wire link, see section 1.

---

## General Notes

- All interfaces share a common ground.
- Only one radio interface (CI-V, RS-232 or CAT-TTL) is used at a time, selected by radio profile
  (see [`radio-support-matrix.md`](../../radio-support-matrix.md)).
- This is a development and test platform, not a certified product.
