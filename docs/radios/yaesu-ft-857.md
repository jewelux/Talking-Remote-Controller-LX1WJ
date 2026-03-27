# Yaesu FT-857

## Overview

This page documents the current radio-specific command layout for the Yaesu
FT-857 profile in firmware `V3.5`.

The current `ft857.ini` profile uses protocol type `YAESU_FT8X7` and currently
supports frequency query/set, mode query/set, and S-meter query.
Power and SWR are not currently declared in the profile capabilities.

---

## Currently Supported Core Functions

### Bank 1

- `0` short: query frequency
- `0` long: enter and set frequency in kHz, then confirm with `D`
- `7` short: query S-meter
- `9` short: query mode
- `9` long: select mode, then confirm with `D`

Current mode mapping from the active profile:

- `1` = LSB
- `2` = USB
- `3` = CW
- `4` = AM
- `5` = FM
- `6` = DIGI
- `8` = CWR

### Bank 2 - FT-8x7 Specific Actions

The current firmware includes a Yaesu FT-8x7 specific action block for this
profile family.

- `4` short: query Yaesu status
- `4` long: toggle VFO
- `5` short: select VFO A
- `5` long: select VFO B
- `6` short: turn clarifier on
- `6` long: turn clarifier off
- `7` short: turn split on
- `7` long: turn split off
- `8` short: turn PTT on
- `8` long: turn PTT off
- `9` short: query the full Yaesu status block

---

## Notes

- This page reflects the current `V3.5` firmware and `ft857.ini` profile state.
- The FT-857 and FT-817 currently share the same protocol family and broadly
  the same command coverage in this repository.
- Hardware behavior should still be verified on a live radio.
