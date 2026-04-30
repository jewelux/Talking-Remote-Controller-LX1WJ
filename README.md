# Talking Remote Controller LX1WJ

The Talking Remote Controller is a voice-first keypad controller for amateur radio transceivers.
It is intended to make practical radio operation possible without relying on a display.

The repository now tracks the current modular firmware line **V3.5.5**.
The newest work in this state is the first practical Yaesu **FTDX10 / FTDX101D / FTDX101MP** support block for assisted field testing.

## Start Here

- If you want to use the controller in practice, read [user-guide.md](user-guide.md).
- If you want to build, adapt, or test the project, read [builder-guide.md](builder-guide.md).
- If you want the short setup path, read [QUICKSTART.md](QUICKSTART.md).

## Current Radio Notes

- General support overview: [docs/radio-support-matrix.md](docs/radio-support-matrix.md)
- Yaesu FTDX10 family status: [docs/radios/yaesu-ftdx10-family.md](docs/radios/yaesu-ftdx10-family.md)
- Assisted blind test list: [docs/radios/yaesu-ftdx10-blind-test-list.txt](docs/radios/yaesu-ftdx10-blind-test-list.txt)
- Technical serial test plan: [docs/radios/yaesu-ftdx10-test-plan.md](docs/radios/yaesu-ftdx10-test-plan.md)

## What Is In The Repo

```text
firmware/
  TalkingRemoteControllerLX1WJ_V3_5_5.ino
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

This project grows through practical support from radio amateurs helping with design, testing, and documentation.

- Richard DO9RE
- Stefan DK7STJ
- Tom OK1ICQ
- Jan OK1TE
- Damian SP9QLO

## Safety

This project is experimental and educational.
You are responsible for safe wiring, correct CAT connections, RF safety, and compliance with local regulations.

## License

See `LICENSE` for code and `LICENSE-docs` for documentation.