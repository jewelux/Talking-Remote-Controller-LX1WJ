# Changelog

All notable changes to this project are documented in this file.

The versioning prior to 1.0 consisted of multiple experimental and integration stages.
Version 1.0 marks the first stable core release.

---

## [1.0] – Initial Stable Release

### Added
- Stable speech output architecture based on pre-recorded voice tokens
- Deterministic keypad interaction model using short and long presses
- Immediate interruption of speech output on new user input
- Numeric input with spoken confirmation
- Spoken output for:
  - Frequency
  - Operating mode
  - S-meter
  - SWR
  - Output power
  - Active bank and radio profile
- Modular radio profile concept
- Non-volatile storage of user preferences
- Toggle for automatic frequency announcements during tuning

### Supported Radio Profiles
- ICOM IC-7300 (CI-V)
- ICOM IC-706MKIIG (CI-V and RS-232)
- Xiegu G106

### Notes
- Versions prior to 1.0 were iterative development versions and are not considered stable releases.
- Version 1.0 establishes a stable foundation for further functional and profile extensions.
