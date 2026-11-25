# MiniHamTRC-V2-KiCad – KiCad project of the Mini-HamTRC V2

The KiCad source files of the Mini-HamTRC V2: schematic, PCB and the project's own symbol and footprint
libraries. Made with **KiCad 10.0**.

| File | Content |
|---|---|
| `MiniHamTRC-V2-KiCad.kicad_pro` | project – open this one in KiCad |
| `MiniHamTRC-V2-KiCad.kicad_sch` | schematic, the source of [`../circuit-diagram-V2.pdf`](../circuit-diagram-V2.pdf) |
| `MiniHamTRC-V2-KiCad.kicad_pcb` | PCB, see *Status* below |
| `sym-lib-table`, `fp-lib-table` | point KiCad to the libraries in `libraries/` |
| `libraries/_MyLibraryLX1WJ.kicad_sym` | symbols used by the schematic |
| `libraries/footprints/` | footprints used by the PCB (suffix `_CNC`: pads sized for a milled board) |

The folder is self-contained: the library tables use `${KIPRJMOD}/libraries/...`, so the project opens on any
computer without installing extra libraries. From KiCad's standard libraries come only the connector symbols of
J4, J5 and J8 (also stored in the schematic), the TO-92 footprint of Q1 and the 3D models.

## Status

- **Schematic:** current V2, as in [`../circuit-diagram-V2.pdf`](../circuit-diagram-V2.pdf), including R5, 12 kΩ,
  between the MAX3232's RXD and GPIO 9. [`../hardware_wiring_v2.md`](../hardware_wiring_v2.md) describes every
  connection in words, and [`../bom_v2.md`](../bom_v2.md) lists the parts.
- **Power:** J5 "VCC In 5V" is the external 5 V input, pin 1 +5 V, pin 2 GND. Which socket or cable goes to J5 on
  the box is up to the builder. C6, 470 µF electrolytic, and C5, 100 nF ceramic, are in parallel on the 5 V rail.
  C6 uses the plain capacitor symbol and the 5 mm disc footprint, which has no polarity mark: minus goes to pad 2, GND.
- **PCB:** a two-layer layout routed with Freerouting. It has not been made or tested yet. The boards built so far
  are the single-sided layout in [`../pcb-layout-V2.pdf`](../pcb-layout-V2.pdf).
- **ERC:** three "power pin not driven" errors at U1 (3V3, 5V0, GND), because the schematic has no `PWR_FLAG`
  symbols. The supply comes from J5 and the ESP32 board, so they are harmless.
- **DRC:** no unconnected items, schematic and PCB in parity; warnings only (mostly silkscreen over copper).

The GPIO mapping is the same as in [`firmware/config_pins.h`](../../../../firmware/config_pins.h).
Questions are welcome: [open a build enquiry](https://github.com/jewelux/Talking-Remote-Controller-LX1WJ/issues/new?template=build-enquiry.yml).
