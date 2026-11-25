# Bill of Materials – HamTRC V2 (Mini-HamTRC)

> [!IMPORTANT]
> **This page describes the current Mini-HamTRC V2 (`development` branch).**
> The `main` branch still contains the first prototype (V1, with SD card). Do not use `main` for a V2 build.
> A step-by-step build page written for screen readers is at [lx1wj.eu – Build a Mini-HamTRC](https://lx1wj.eu/hamtrc/mini-review/index-en.html)
> (also in [German](https://lx1wj.eu/hamtrc/mini-review/index.html) and [French](https://lx1wj.eu/hamtrc/mini-review/index-fr.html)).
> Questions are welcome: [open a build enquiry](https://github.com/jewelux/Talking-Remote-Controller-LX1WJ/issues/new?template=build-enquiry.yml).

> **Status: current V2 build (2026-11-25).** Derived from [`circuit-diagram-V2.pdf`](circuit-diagram-V2.pdf)
> and [`pcb-layout-V2.pdf`](pcb-layout-V2.pdf), plus the parts added since (marked \*). Wiring details: [`hardware_wiring_v2.md`](hardware_wiring_v2.md).
> KiCad source files: [`MiniHamTRC-V2-KiCad/`](MiniHamTRC-V2-KiCad/).

## Modules

| Ref | Qty | Part | Notes |
|---|---|---|---|
| U1 | 1 | ESP32-S3-N16R8 dev board (DevKitC-1 style, 2×22 pins, dual USB-C) | 16 MB flash / 8 MB PSRAM required by firmware |
| J4 | 1 | MAX98357A I2S class-D amplifier breakout (7 pins) | e.g. Adafruit #3006 or clone |
| J2 | 1 | MAX3232 TTL ↔ RS-232 module with DB9 (4 pins: GND, RXD, TXD, VCC) | |
| J8 | 1 | 4×4 membrane matrix keypad, 8-pin connector | |
| – | 1 | Speaker 4–8 Ω, ≥ 1 W | to the MAX98357A screw terminal |

## Discrete components

| Ref | Qty | Value | Part / Package | Function |
|---|---|---|---|---|
| R1 | 1 | 1 kΩ | resistor ¼ W, axial THT | CI-V receive series resistor |
| R2 | 1 | 4.7 kΩ | resistor ¼ W, axial THT | CI-V transmit base resistor |
| R3 | 1 | 1 kΩ | resistor ¼ W, axial THT | CAT-TTL TX series resistor |
| R4 | 1 | 1 kΩ | resistor ¼ W, axial THT | CAT-TTL RX series resistor |
| ZD1 | 1 | 3.3 V | Zener diode, e.g. BZX55C3V3 / BZX79C3V3, DO-35 | CI-V RX clamp |
| D1 | – | – | wire link (PCB footprint only, no diode) | 5 V from J5 to the rail |
| Q1 | 1 | BC547 | NPN, TO-92 (BC548 / BC549 also fine) | CI-V open-collector driver |
| C5 | 1 | 100 nF | ceramic capacitor, THT, 5 mm pitch | 5 V rail decoupling, in parallel with C6 |
| C6 | 1 | 470 µF | electrolytic capacitor, ≥ 10 V, radial, 5 mm pitch | 5 V rail and MAX98357A `Vin` buffer; minus to GND (pad 2) |
| R5 | 1 | 12 kΩ | resistor ¼ W, axial THT (variant B, as in the schematic) | GPIO 9 protection: RXD → GPIO 9 |
| \* | (1 + 1) | 4.7 kΩ + 10 kΩ | resistors ¼ W (variant A, instead of R5) | GPIO 9 protection: 4.7 kΩ RXD → GPIO 9, 10 kΩ GPIO 9 → GND |

## Connectors

| Ref | Qty | Part | Function |
|---|---|---|---|
| J5 | 1 | 2-pin header or terminal, 2.54 mm | 5 V power input |
| J6 | 1 | 3.5 mm stereo jack, PCB mount | CI-V (Tip = bus, Sleeve = GND) |
| J3 | 1 | 3.5 mm stereo jack, PCB mount | CAT-TTL |
| J8 | 1 | 2×4 pin header, 2.54 mm | keypad |
| – | 2 | female header 1×22, 2.54 mm | socket for U1 |
| – | 1 | female header 1×7, 2.54 mm | socket for J4 (MAX98357A) |
| – | 1 | female header 1×4, 2.54 mm | socket for J2 (MAX3232) |

## PCB and mechanics

| Qty | Item | Notes |
|---|---|---|
| 1 | PCB HamTRC V2, single-sided | per `pcb-layout-V2.pdf` |
| 6 | wire bridges (tinned copper wire) | red links in the layout PDF (+1 for D1 if no diode) |
| 4 | M3 spacers + screws | corner mounting holes |
| 1 | 5 V supply, at least 1 A | good, stable plug-in supply; socket or cable to J5 as the builder prefers |
| 1 | USB data cable for the ESP32 board | only for flashing the firmware from a computer |
| – | cables: 3.5 mm stereo cable (CI-V / CAT), DB9 cable (RS-232) | depending on radio |

## Notes

- The Mini is powered from an external 5 V supply on J5. The USB-C ports of the
  ESP32-S3 board are only used to flash the firmware; powered through them, the MAX modules do not get a proper
  supply (issues #29, #31). See `hardware_wiring_v2.md`, section 1.
- D1 is not in the schematic any more; the PCB keeps its footprint. Fit a wire link.
- Do not close the USB-OTG solder bridge on the ESP32-S3 board; USB control of radios comes with V3.
- RS-232 cable: Yaesu FTDX10 needs a null modem cable or adapter, Yaesu FT-847 a straight cable.
- The MAX3232 module is soldered onto the two large pads at the top edge of the PCB.
- The SD-card module of V1 is no longer part of V2.
