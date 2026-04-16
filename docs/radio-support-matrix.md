# Radio Support Matrix

This table tracks the current documented support state.
It is meant as a short engineering overview, not as a promise list.

| Radio / Profile | Protocol | Frequency | Mode | Power | S-Meter | SWR | Profile Load | Hardware Status | Notes |
|---|---|---|---|---|---|---|---|---|---|
| ICOM IC-7300 | CI-V | implemented | implemented | implemented | implemented | implemented | implemented | tested on hardware | See `docs/radios/icom-ic-7300.md` for the extended command map. |
| ICOM IC-7300 RS-232 | RS-232 | implemented | implemented | implemented | implemented | implemented | implemented | not recently verified | Intended to mirror the CI-V path. |
| ICOM IC-706 CI-V | CI-V | implemented | implemented | partial | partial | partial | implemented | not recently verified | Legacy support carried into the modular branch. |
| ICOM IC-706 RS-232 | RS-232 | partial | partial | partial | partial | partial | implemented | not recently verified | Needs recheck on live hardware. |
| Xiegu G106 | CAT | implemented | partial | partial | partial | partial | implemented | not recently verified | Existing support should be rechecked command by command. |
| Kenwood TS-480 | ASCII / CAT | partial | partial | planned | planned | planned | implemented | not recently verified | Good candidate for future expansion. |
| Elecraft KX2 | ASCII / CAT | partial | partial | partial | planned | planned | implemented | not recently verified | Evolving profile. |
| Yaesu FT-817 | CAT | implemented | implemented | partial | implemented | partial | implemented | tested on hardware | FT8x7 family notes are documented separately. |
| Yaesu FT-818 | CAT | implemented | implemented | partial | implemented | partial | implemented | not recently verified | Mirrors the FT-817 family path. |
| Yaesu FT-857 | CAT | implemented | implemented | partial | implemented | partial | implemented | tested on hardware | Split and VFO handling are already documented and tested. |
| Yaesu FT-897 | CAT | partial | partial | partial | partial | partial | implemented | not recently verified | Shares the FT-857/897 family handling. |
| Yaesu FTDX10 | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Current `V3.5.4` block includes VFO, VFO mode, split, lock, tuner, preamp, AGC, IF, ID, and power-state paths. See `docs/radios/yaesu-ftdx10-family.md`. |
| Yaesu FTDX101D | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Uses the same first-block documentation as FTDX10. |
| Yaesu FTDX101MP | ASCII CAT | implemented | implemented | implemented | implemented | implemented | implemented | assisted field test pending | Uses the same first-block documentation as FTDX10. |