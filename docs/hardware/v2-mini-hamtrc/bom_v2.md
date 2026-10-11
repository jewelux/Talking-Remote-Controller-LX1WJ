# Bill of Materials – HamTRC V2 (Mini-HamTRC)

> [!IMPORTANT]
> **Planning to build one? Please get in touch first.**
> The hardware is still evolving, so this documentation is deliberately kept minimal and may lag behind the bench.
> Before you order parts or PCBs, [open a build enquiry](https://github.com/jewelux/Talking-Remote-Controller-LX1WJ/issues/new?template=build-enquiry.yml) to get the current state of the board,
> recent changes and build tips. Feedback from builders is very welcome – every question helps to improve the documentation.

> **Status: in development.** Derived from [`circuit-diagram-V2.pdf`](circuit-diagram-V2.pdf)
> and [`pcb-layout-V2.pdf`](pcb-layout-V2.pdf). Wiring details: [`hardware_wiring_v2.md`](hardware_wiring_v2.md).

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
| D1 | 0–1 | – | **optional** (PCB footprint only): wire link, or diode 1N5817 Schottky (preferred) / 1N400x, DO-41 | diode only with separate 5 V supply on J5 |
| Q1 | 1 | BC547 | NPN, TO-92 (BC548 / BC549 also fine) | CI-V open-collector driver |
| C5 | 1 | 100 nF | ceramic capacitor, THT | 5 V rail decoupling |

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
| 1 | 5 V supply (≥ 1 A) | USB-C to the ESP32 board, or a separate 5 V supply on J5 (then fit D1 as diode) |
| – | cables: 3.5 mm stereo cable (CI-V / CAT), DB9 cable (RS-232) | depending on radio |

## Notes

- D1 is not in the schematic any more; the PCB keeps its footprint. Fit a wire link for USB supply,
  or a diode when a separate 5 V supply is used on J5 (Schottky: ≈ 0.3 V drop, 1N400x: ≈ 0.7 V).
- The MAX3232 module is soldered onto the two large pads at the top edge of the PCB.
- The SD-card module of V1 is no longer part of V2.
