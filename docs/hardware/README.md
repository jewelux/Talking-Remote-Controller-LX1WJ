# Hardware

> [!IMPORTANT]
> **The current reference build is the Mini-HamTRC V2, documented on the `development` branch.**
> The `main` branch still contains the first prototype (V1, with SD card).
> A step-by-step build page written for screen readers is at [lx1wj.eu – Build a Mini-HamTRC](https://lx1wj.eu/hamtrc/mini-review/index-en.html).
> Questions are welcome: [open a build enquiry](https://github.com/jewelux/Talking-Remote-Controller-LX1WJ/issues/new?template=build-enquiry.yml).

| Folder | Board | Status |
|---|---|---|
| [`v1-hamtrc/`](v1-hamtrc/) | HamTRC V1 – original board (12 V input, DC-DC module, SD module) | current |
| [`v2-mini-hamtrc/`](v2-mini-hamtrc/) | HamTRC V2 – Mini-HamTRC, compact board, external 5 V via USB-C power socket | current reference build (`development` branch) |
| [`line-out/`](line-out/) | Optional 3.5 mm line out for demo units (MAX98357A bridged output) | proposed, not yet built |
| [`proto-pcbs/`](proto-pcbs/) | Photos of early prototype boards | archive |

The hardware is evolving. Each board folder contains the schematic and PCB layout as PDF
and a wiring description; V2 also has a bill of materials.
The GPIO mapping is defined in [`firmware/config_pins.h`](../../firmware/config_pins.h).
