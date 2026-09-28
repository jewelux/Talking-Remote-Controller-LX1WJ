# Talking Remote Controller LX1WJ

The Talking Remote Controller is a voice-first keypad controller for amateur radio transceivers.
It is intended to make practical radio operation possible without relying on a display.

The repository now tracks the current modular firmware line **V3.5.8**.
The newest work in this state keeps the Yaesu **FTDX10 / FTDX101D / FTDX101MP** support block current, lets CI-V profiles adjust address and baud rate from Bank 8, and adds the HAMTRC service response used by the online updater to identify the correct COM port before flashing.

## Start Here

- If you want to use the controller in practice, read [user-guide.md](user-guide.md).
- If you want to build, adapt, or test the project, read [builder-guide.md](builder-guide.md).
- If you want the short setup path, read [QUICKSTART.md](QUICKSTART.md).

## Online Update

The matching HAMTRC V3.5.8 browser update is published on `lx1wj.eu`.
The firmware answers `HAMTRC?` with an `LX1WJ-HAMTRC` signature and `V3.5.8`,
which lets the checked update path avoid flashing a wrong COM port.

## Current Radio Notes

- General support overview: [docs/radio-support-matrix.md](docs/radio-support-matrix.md)
- Yaesu FTDX10 family status: [docs/radios/yaesu-ftdx10-family.md](docs/radios/yaesu-ftdx10-family.md)
- Assisted blind test list: [docs/radios/yaesu-ftdx10-blind-test-list.txt](docs/radios/yaesu-ftdx10-blind-test-list.txt)
- Technical serial test plan: [docs/radios/yaesu-ftdx10-test-plan.md](docs/radios/yaesu-ftdx10-test-plan.md)

## What Is In The Repo

```text
firmware/
  TalkingRemoteControllerLX1WJ_V3_5_8.ino
  modular source files
  SDCard/*.ini radio profiles

docs/
  user-facing and technical notes
```

The firmware keeps the spoken operating concept but now uses modular protocol, UI, and profile code.

## Project Direction

- practical accessibility for blind and visually impaired operators
- short, predictable keypad workflows
- spoken feedback instead of display dependency
- separate end-user and technical documentation
- real-radio testing before broader command expansion

## Collaboration

This project grows through practical collaboration between developers, testers, and radio amateurs who contribute their experience with different transceivers and accessibility requirements.

### Development

- Jean Weber, LX1WJ – Project initiator, hardware, firmware development, testing and documentation
- Jan Hegr, OK1TE – Developer and collaborator, software architecture, firmware development and radio support

### Testing and Support

Special thanks to the radio amateurs who support the project with practical testing, accessibility feedback, radio-specific experience, and documentation:

- Richard DO9RE
- Stefan DK7STJ
- Tom OK1ICQ
- Damian SP9QLO

### Development Workflow

The project uses a simple development model:

- `main` contains the current stable version.
- `development` is used to integrate and test ongoing development.
- Larger additions and radio-specific work can be developed in separate feature branches before being merged into `development`.
- After testing and stabilization, changes from `development` are merged into `main`.

New contributors: start with the [Developer Cookbook](docs/developer-cookbook.md), which covers the architecture, how to add a key, feature or radio, and the project conventions.


## Safety

This project is experimental and educational.
You are responsible for safe wiring, correct CAT connections, RF safety, and compliance with local regulations.

## License

See `LICENSE` for code and `LICENSE-docs` for documentation.
