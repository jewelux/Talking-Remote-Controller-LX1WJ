# Radio Support Matrix

## Purpose

This document tracks the current real support state of radio profiles in the project.
It is intentionally focused on the present status, not on deadlines or roadmap promises.

Suggested status wording:

- `implemented`
- `partial`
- `planned`
- `tested on hardware`
- `not recently verified`

---

## Current Support Overview

| Radio / Profile | Protocol / Interface | Profile File | Frequency | Mode | Power | S-Meter | SWR | Profile Load | Hardware Test Status | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| ICOM IC-7300 | CI-V | `ic7300.ini` | implemented | implemented | implemented | implemented | implemented | implemented | tested on hardware | Extended beyond basic CAT: RX/TX state, TX frequency, NR/NB levels, notch width, PBT1/PBT2, filter shape/width, tuner, VFO A/B, split, RIT, monitor, transceive, lock, and band stacking are documented in `docs/radios/icom-ic-7300.md`. |
| ICOM IC-7300 (RS-232) | RS-232 | `ic7300_rs232.ini` | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | Intended to expose the same IC-7300 command families as the CI-V profile, but the RS-232 transport path should still be rechecked on live hardware. |
| ICOM IC-706 (CI-V) | CI-V | `ic706_civ.ini` | implemented | implemented | partial | partial | partial | implemented | not recently verified | Legacy support path carried into the modular structure. |
| ICOM IC-706 (RS-232) | RS-232 | `ic706_rs232.ini` | partial | partial | partial | partial | partial | implemented | not recently verified | Needs confirmation against live hardware in V3.5 structure. |
| Xiegu G106 | CAT | `g106.ini` | implemented | partial | partial | partial | partial | implemented | not recently verified | Existing support should be re-checked command by command. |
| Kenwood TS-480 | ASCII / CAT | `ts480.ini` | partial | partial | planned | planned | planned | implemented | not recently verified | Good candidate for incremental command expansion. |
| Elecraft KX2 | ASCII / CAT | `kx2.ini` | partial | partial | partial | planned | planned | implemented | not recently verified | Important evolving profile with ongoing command work. |
| Yaesu FT-817 | CAT | `ft817.ini` | implemented | implemented | partial | implemented | partial | implemented | tested on hardware | V3.5.1 fixes FT8x7 VFO A/B selection, split status decoding, safe keypad write verification, and adds Bank 6 repeater/tone defaults; keypad `RXTX?` remains intentionally disabled on FT-817 because practical readings are still unstable. |
| Yaesu FT-818 | CAT | `ft818.ini` | implemented | implemented | partial | implemented | partial | implemented | not recently verified | Mirrors the FT-817 profile variant including the V3.5.1 FT8x7 fixes and Bank 6 defaults, but still needs direct on-radio verification. |
| Yaesu FT-857 | CAT | `ft857.ini` | implemented | implemented | partial | implemented | partial | implemented | tested on hardware | V3.5.1 enables split query/set in the shipped profile, corrects split bit decoding, improves keypad write verification, and keeps the documented FT8x7 family notes in `docs/radios/yaesu-ft8x7-keypad.md` and `docs/radios/yaesu-ft8x7-status.md`. |
| Yaesu FT-897 | CAT | `ft897.ini` | partial | partial | partial | partial | partial | implemented | not recently verified | Uses the FT-857/897 variant logic and now ships with split query/set plus Bank 6 repeater/tone defaults, but hardware verification is still pending. |
| Yaesu FTDX10 | ASCII CAT | `ftdx10.ini` | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | First documented bring-up now includes VFO and split test coverage; see `docs/radios/yaesu-ftdx10-test-plan.md`. |
| Yaesu FTDX101D | ASCII CAT | `ftdx101d.ini` | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | Shares the same documented first-block plus VFO/split bring-up as FTDX10; hardware confirmation still needed. |
| Yaesu FTDX101MP | ASCII CAT | `ftdx101mp.ini` | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | Shares the same documented first-block plus VFO/split bring-up as FTDX10; hardware confirmation still needed. |


---

## Important Note

This matrix is a working engineering overview.
It does not replace the project disclaimer, operating precautions, or hardware responsibility notes described elsewhere in the repository.


