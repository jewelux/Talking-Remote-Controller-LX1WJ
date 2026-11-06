# Talking Remote Controller LX1WJ Builder Guide

This guide is the short technical entry point for builders, contributors, and test helpers.

## Current Firmware Line

The repository now documents and ships the modular firmware line **V3.5.8**.
The current focus is a clean first Yaesu **FTDX10 family** field-test block on top of the existing multi-radio platform.

## Builder Path

Use this repository if you need to:

- build the ESP32-S3 controller
- load the current firmware
- add or adjust radio profiles
- extend CAT command handling
- document real-radio behavior

## Main Technical References

- [QUICKSTART.md](QUICKSTART.md)
- [docs/hardware/v1-hamtrc/hardware_wiring.md](docs/hardware/v1-hamtrc/hardware_wiring.md)
- [docs/radio-support-matrix.md](docs/radio-support-matrix.md)
- [docs/radios/yaesu-ftdx10-family.md](docs/radios/yaesu-ftdx10-family.md)
- [docs/radios/yaesu-ftdx10-test-plan.md](docs/radios/yaesu-ftdx10-test-plan.md)

## Firmware Structure

`firmware/TalkingRemoteControllerLX1WJ_V3_5_8.ino` is the main sketch entry.
The project is split into modular source files for:

- protocol handling
- keypad and speech UI
- runtime and live radio state
- the built-in radio profiles

## Online Update Notes

The V3.5.8 browser update on `lx1wj.eu` uses the ESP Web Tools manifest
`firmware/v3-5-7/manifest.json` and the matching factory image
`hamtrc-v3-5-7.factory.bin`.

The checked update flow first opens the selected serial port at 115200 baud and
sends `HAMTRC?`. V3.5.8 answers with `LX1WJ-HAMTRC;protocol=1;chip=esp32;version=V3.5.8`.
If that signature is not present, the website does not start flashing on that
selected COM port.

The GitHub Actions workflow `.github/workflows/firmware.yml` builds the updater
package on every push and pull request (download it from the run's artifacts),
and attaches it to a GitHub Release when a `v*` tag is pushed. It contains:

- `hamtrc-<version>.factory.bin`: bootloader, partition table, boot_app0 and app
  merged for offset `0x0`, not padded; saved settings (NVS) are reset on
  purpose, so a new version never starts from settings written by an older one
- `hamtrc-<version>.voices.bin`: the voice pack (all spoken words) for the
  `voices` partition at `0x810000`
- `manifest.json`: ESP Web Tools manifest (`ESP32-S3`, factory image at offset 0,
  voice pack at offset `0x810000`)
- `SHA256SUMS`

Upload `manifest.json`, the factory image and the voice pack to the updater site.

The same workflow also publishes the package to GitHub Pages (branch `gh-pages`),
so a flash page can load it directly without manual uploads:

- `https://jewelux.github.io/Talking-Remote-Controller-LX1WJ/firmware/main/manifest.json`
- `https://jewelux.github.io/Talking-Remote-Controller-LX1WJ/firmware/development/manifest.json`
- `https://jewelux.github.io/Talking-Remote-Controller-LX1WJ/firmware/<tag>/manifest.json`
  for each `v*` tag, with dots turned into dashes (tag `v3.5.9` is published as `v3-5-9`)

The `main` channel appears with the first push to `main` that contains this
workflow, and tag channels only for tags pushed since then. Pushes that change only
documentation neither rebuild nor republish the firmware.

`firmware/index.json` lists all published channels with version, commit and date.
On `main` and `development` the manifest version reads like
`V3.5.8 development eda5ca1 2026-10-29`, so the flash page tells which build it is.
The images there carry the commit in their names
(`hamtrc-v3-5-8-eda5ca1.factory.bin`), so a browser never mixes images of two
builds. The previous build's images stay until the next publish. A tag channel
holds exactly the files of its GitHub Release.

A `v*` tag must match `HAMTRC_FIRMWARE_VERSION`: for `V3.5.9` that is `v3.5.9`,
or a longer tag such as `v3.5.9-rc1`. Otherwise the build fails and nothing is
published or released. GitHub starts no run when more than three tags are pushed
at once. Push tags one at a time; to publish a missed tag (or any channel) again,
use Run workflow on that tag or branch.

Boards that do not answer `HAMTRC?` (new boards, firmware older than V3.5.7, boards
in bootloader mode) are flashed with the manual update that skips the HAMTRC check.

## Current FTDX10 Family Scope

The present documented scope is the first practical CAT block:

- frequency read and set
- mode read and set
- VFO A and VFO B select
- VFO-A and VFO-B frequency read and set
- VFO-A and VFO-B mode read and set
- split read and set
- S-meter, power, and SWR
- NR, NB, notch
- lock
- tuner query, toggle, and tune
- preamp query and toggle
- AGC query plus fast and slow presets
- power-state query plus on and off path
- IF and ID query

Anything beyond that should be treated as later expansion unless documented otherwise.

## Documentation Rule

Keep user-facing operation notes separate from engineering detail.
If a function is still experimental, say so clearly and do not present it as normal keypad behavior.
