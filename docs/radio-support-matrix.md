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
| ICOM IC-7300 | CI-V | `ic7300.ini` | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | One of the core reference radios in the project history. |
| ICOM IC-7300 (RS-232) | RS-232 | `ic7300_rs232.ini` | partial | partial | partial | partial | partial | implemented | not recently verified | Verify actual command coverage against current firmware behavior. |
| ICOM IC-706 (CI-V) | CI-V | `ic706_civ.ini` | implemented | implemented | partial | partial | partial | implemented | not recently verified | Legacy support path carried into the modular structure. |
| ICOM IC-706 (RS-232) | RS-232 | `ic706_rs232.ini` | partial | partial | partial | partial | partial | implemented | not recently verified | Needs confirmation against live hardware in V3.5 structure. |
| Xiegu G106 | CAT | `g106.ini` | implemented | partial | partial | partial | partial | implemented | not recently verified | Existing support should be re-checked command by command. |
| Kenwood TS-480 | ASCII / CAT | `ts480.ini` | partial | partial | planned | planned | planned | implemented | not recently verified | Good candidate for incremental command expansion. |
| Elecraft KX2 | ASCII / CAT | `kx2.ini` | partial | partial | partial | planned | planned | implemented | not recently verified | Important evolving profile with ongoing command work. |
| Yaesu FT-817 | CAT | `ft817.ini` | partial | partial | planned | planned | planned | implemented | not recently verified | Verify exact CAT coverage. |
| Yaesu FT-857 | CAT | `ft857.ini` | partial | partial | planned | planned | planned | implemented | not recently verified | Verify exact CAT coverage. |
| Yaesu FT-897 | CAT | `ft897.ini` | partial | partial | planned | planned | planned | implemented | not recently verified | Verify exact CAT coverage. |
| Yaesu FTDX10 | CAT | `ftdx10.ini` | partial | partial | partial | planned | planned | implemented | not recently verified | Newer profile family in the SD-card architecture. |
| Yaesu FTDX101D | CAT | `ftdx101d.ini` | partial | partial | partial | planned | planned | implemented | not recently verified | Needs command-by-command confirmation. |
| Yaesu FTDX101MP | CAT | `ftdx101mp.ini` | partial | partial | partial | planned | planned | implemented | not recently verified | Needs command-by-command confirmation. |


---

## Important Note

This matrix is a working engineering overview.
It does not replace the project disclaimer, operating precautions, or hardware responsibility notes described elsewhere in the repository.
