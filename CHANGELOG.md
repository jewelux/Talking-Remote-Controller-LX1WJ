# Changelog

## Unreleased

### Keypad

- in an entry (frequency, RF power, repeater shift, CTCSS, DCS, CI-V address) and in bank,
  profile and mode select, a key acts as soon as it is pressed; holding or releasing it does
  nothing more
- bank select (`*` long): pressing `1`–`9` switches to that bank at once and says its number;
  `D` is no longer needed
- mode select ("mode please") takes every key as a mode digit, also keys that do something else on
  the current bank (e.g. Bank 3 `6`–`9`), and holding a key does not run its long action. A key
  that picks no mode, or a mode the radio profile cannot set, beeps and mode select stays. The
  picked mode waits for `D` to apply it; another mode digit replaces it, `#` cancels it and any
  other key beeps. Before, an invalid digit ended mode select, keys ran their bank actions, a
  mode the profile cannot set was spoken as picked and then failed silently, and a picked mode left
  behind could be applied by a later `D`, e.g. at the end of a frequency entry
- fixed mode select: `1`/`2` were ignored after a Bank 3 VFO A/B mode select (`3`/`4`/`5` long), and
  Bank 1 `9` long could set VFO A or B instead of the current VFO
- `D` with nothing typed or chosen beeps and the entry or selection stays. Before, some entries said
  "error" and ended, some failed silently, and bank and profile select ended with a beep
- `#` cancels any entry, selection, picked mode, command waiting for `D` or key waiting for a double
  press, and says "cancel" (it said "ok"; cancelling bank or profile select was silent). With
  nothing to cancel it beeps
- holding a key that has no long action beeps once the hold time is reached, and releasing it does
  nothing. Before, the release ran the key's short action. This includes `D` and `#`
- a key that waits for a possible double press (e.g. Bank 1 `0`, `1`, `2`) answers as soon as
  another key is pressed, before that key. Before, the two answers came in the wrong order, or the
  first press was lost
- pressing `#` while another key is held no longer runs that key when it is released
- a key that does nothing gives a short beep instead of silence: keys with no action on the current
  bank, keys whose feature the radio lacks, and keys an entry or selection does not take
- choosing a profile number with no stored profile says "not available"

### Frequency

- frequencies are announced and shown to 10 Hz (e.g. `7.12345`, `14.1`, `7.0`) instead of being cut
  to kHz
- frequency entry is in MHz with `*` as the decimal point: `14*1` is 14.1 MHz, `14*12345` is
  14.12345 MHz
- Bank 1 `0` double press rounds the current frequency to the nearest 500 Hz
- tuning is announced only when the dial stops, and only once it is at least 100 Hz away from the
  last frequency you heard or entered. Moving the dial cuts off a readout that is out of date
  instead of queueing another one, and the next stop is always announced

### Speech and sounds

- any key press stops what is being spoken, so answers no longer queue up behind earlier ones
- new voice (Piper `en_US-lessac-medium`) for all clips
- "timeout" when the radio does not answer, and "not available" for features the radio or profile
  cannot provide (e.g. keys hidden on FTDX10, an empty profile slot); both used to say "error" or
  nothing
- the S-meter says dB over S9 as a word ("S meter nine plus twenty")
- WFM is said as "wfm" instead of being spelled out
- the RX/TX query (Bank 1 `1`, and Bank 3 `6` on radios other than the FT-8x7 and FTDX10) says
  "transceiver rx" or "transceiver tx" instead of "transceiver off" or "on"
- Bank 9 says what a value is: the volume keys say "volume five" (plus "ok" after a change), the
  profile query says "profile" before the radio and no longer ends with "ok", and profile select asks "profile please" instead of
  "choose please"

### Console commands

- new commands for what only the keypad could do: `ROUND [<Hz>]`, `NRLEVEL`, `NBLEVEL`,
  `MONLEVEL`, `PBT1`, `PBT2`, `RIT` and `VOLUME STEP <+-n>`; `FILWIDTH NEXT | PREV`; `TOGGLE` for
  `MONITOR`, `TRANSCEIVE`, `RIT`, `FILSHAPE` and `TUNINGSPEECH`; `CIVADDR? | <hex>` and
  `BAUD? | <rate>` (CI-V); on FT-8x7 `CTCSS? | <Hz>`, `DCS? | <code>` and `VFO SYNC A | B`; on
  FT-817 `VFO A=B`
- `FREQHZ`, `VFOAHZ` and `VFOBHZ` set a frequency in Hz
- `BANK <n>`, `BANK NEXT` and `BANK PREV` cover banks 1–9 like the keypad (they stopped at 3)
- `NR`, `NB` and `NOTCH` behave like the Bank 2 keys: `TOGGLE` starts from the radio's known state,
  on the TS-480 `NR TOGGLE` steps off → 1 → 2 → off, and on CI-V `NOTCH TOGGLE` steps off → NAR →
  MID → WIDE → off
- `EXPERIMENTAL ON | OFF` and `EXPERIMENTAL?`, for testing: every feature counts as supported by
  the radio profile, also those the profile turns off. It is not saved, so after a restart the
  profile applies again
- FT-817/818/857/897: `YEEPROM! <addr> <byte> <byte>` writes two bytes of the radio's EEPROM, at
  the address and the one after it, the counterpart of `YEEPROM?`. **Use with caution:** a wrong address or value
  can wipe the radio's memories and calibration. It is refused while transmitting

### Radios

- FT-817/818/857/897: the frequency is polled in the background again. When the radio is off or
  disconnected, polling slows down so the keypad stays responsive
- FT-817/818/857/897: fixed the S-meter always reading S0
- FT-857/897: fixed the dial lock turning on when HamTRC starts with the radio already on
- FT-817/818/857/897: `LOCK?` (Bank 1 `3`) reads the lock from the radio, also a lock set on its front
  panel, and `3` long toggles from that state. Before, HamTRC said the lock it last set itself, or
  "error" if it had not set one yet. VFO A/B switching and reading the other VFO say "lock on"
  when the radio is locked
- FT-817/818/857/897: RTTY and RTTY-R beep in mode select and are not listed by `MODE LIST`, since
  these radios have no such mode. Before, RTTY-R switched the radio to FM and RTTY sent WFM
- FT-817: Bank 3 `1` long toggles VFO A/B and `2` long copies the active VFO to the other (A=B)
- FT-817/818/857/897: the SWR reading no longer sends the radio a command that writes to its
  internal memory, and `ALC?` no longer answers with a fixed memory value
- FT-817/818/857/897: `PO?` and `SWR?` (Bank 1 `4` and `8`) work while transmitting. Power is said in
  meter bars from 0 to 15, SWR as a value from 1.0 to 10, with "high" when the radio flags a high
  SWR ("swr high 3.7"). In receive they say "power rx" and "swr rx". `ALC?` prints 0–15
- FT-817/818/857/897: split status is read from the radio, also after split was changed on the
  radio's front panel. Before, it was remembered from HamTRC's own split commands, or read with the
  on/off meaning reversed, and the FT-857/897 often said it could not tell
- FT-857/897: Bank 3 `0` works like on the other radios: short says the split state, long toggles
  it and a double press says the TX frequency. It used to set split off, on, or on and then off,
  and the `SPLIT CAL` console command is gone
- FT-857/897: Bank 2 reads the radio settings a HamPod reads: DSP noise reduction (`1`), noise
  blanker (`2`), auto notch (`3`), AGC (`4`, "a g c auto"), IPO and ATT of the current band (`5`,
  long for ATT), DSP bandpass filter (`6`), break-in and keyer (`7`, long for keyer) and FM narrow
  (`8`), the menu item the radio's menu was last left on (`9`, "menu 7 6") and the soft key row
  saved with it (`9` long). Bank 1 `6` says the menu 75 power of the current band ("power 10
  watts"). The same as console commands `NR?`, `NB?`, `NOTCH?`, `AGC?`, `IPO?`, `ATT?`, `DBF?`,
  `BK?`, `KYR?`, `NAR?`, `MENU?`, `ROW?` and `RFPOWER?`. They only read these settings; HamTRC
  cannot change them
- FT-857/897: the Bank 3 VFO keys read which VFO is active from the radio, also after A/B was
  pressed on its front panel, so "vfo a" and "vfo b" are always right. Before, HamTRC assumed VFO
  A at start and needed Bank 3 `4` (sync) after a front panel change; that key is now unassigned.
  IPO, ATT, FM narrow and the RIT or IF shift offset are read for the active VFO the same way
- FT-857/897: the console command `YSETTINGS?` lists more settings read from the radio: VOX,
  PROC, lock, fast tuning, the chosen filter, CW speed, the mic and data gains, VOX gain and
  delay, NR, NB and PROC levels, the DSP filter widths and an offset of RIT or IF shift (which
  one is not yet confirmed). No keys speak them yet
- FT-857/897: Bank 3 `5` short says whether RIT is on ("rit on"), long toggles it. It follows
  the radio's own CLAR key too. Before, the key sent "clarifier on" and "clarifier off" without
  knowing the state; those CAT commands switch RIT on these radios. If RIT is on, asking switches
  it off and straight back on, and hands the tuning knob to RIT if IF shift had it. The same
  as console commands `RIT?`, `RIT ON`, `RIT OFF` and `RIT TOGGLE`
- FT-817/818/857/897: the console command `IFSHIFT?` prints whether IF shift (a long press of the
  radio's CLAR key) is on; it is not spoken. HamTRC can read it but not switch it. `YSETTINGS?`
  also lists IF shift, and on the FT-857/897 whether the RF/SQL knob is squelch or RF gain (menu
  80) and the mic EQ (menu 48)
- FT-817/818/857/897: `RXTX?` reads the radio's PTT state. Before, it read part of the power meter
  and could report receive while transmitting. On the FT-817 Bank 1 `1` says it too, instead of
  "transceiver not available"
- FT-817/818: Bank 3 `5` short says whether RIT is on, long toggles it, as on the FT-857/897 (RIT is
  the clarifier, a short press of the radio's CLAR key; IF shift, the long press, is left alone). It
  replaces VFO B mode on that key: for VFO B's mode, switch with Bank 3 `1` long and use Bank 1 `9`
- FT-817/818: Bank 2 reads the radio's settings like on the FT-857/897: noise blanker (`2`), AGC
  (`4`, "a g c auto"), and break-in and keyer (`7`, long for keyer), also when they were changed on
  the radio. The same as console commands `NB?`, `AGC?`, `BK?` and `KYR?`; `YSETTINGS?` also lists
  VOX, lock, fast tuning and IF shift. The FT-817 has no DSP, so noise reduction and notch (`1` and `3`) say
  "not available"
- FT-817/818: Bank 1 `6` says the TX power setting ("power 2.5 watts"; on the FT-818 the levels
  are 6, 5, 2.5 and 1 watts), and Bank 2 `9` the menu
  item the radio's menu was last left on, long the function row. The same as console commands
  `RFPOWER?`, `MENU?` and `ROW?`
- FT-817/818: VFO A mode (Bank 3 `3`) and the console commands `VFOA?`, `VFOB?`, `VFOA MODE` and
  `VFOB MODE` switch back to the VFO you were on. Before, they left the radio on the VFO they
  read or set, and `VFOA?` and `VFOB?` could time out
- TS-480: `NR 0`, `NR 1` and `NR 2` set the NR level, and the Bank 2 `1` key and `NR?` say it
  ("noise reduction two") instead of only on or off
- NR, NB and notch keys say so when the radio rejects the change or does not answer; this was silent
- FTDX10: frequency entry and 500 Hz rounding announce the new frequency once and report a write
  the radio rejects. Bank 2 `8` and `9` say "not available" like the other hidden keys
- Bank 6 on radios other than the FT-8x7 family is empty: its keys beep like any unassigned key
  (they said "BANK6 reserved")

### Firmware updates

- FT-817/818/857/897: copy the new `ft817.ini`, `ft818.ini`, `ft857.ini` and `ft897.ini` from
  `firmware/SDCard` to the SD card. With the old files, power, SWR, TX power and the noise
  blanker (on the FT-857/897 also noise reduction and notch) stay "not available", the FT-817's
  RX/TX state too, and the FT-857/897 VFO keys do not read the active VFO from the radio
- every push builds the firmware as a factory image with a manifest for the online updater;
  version tags attach them to a GitHub release

### For testers: serial trace

- a keypad `CMD` line names the key and gesture, e.g. `CMD BANK3 2 LONG -> A=B`, also where it used
  a placeholder (`BANK3 FT857 -> SPLIT ON` is now `BANK3 0 LONG -> SPLIT ON`)
- a rejected key names the mode it was pressed in: `FREQ A -> unassigned`,
  `MODE SELECT 0 -> unassigned`, `BANK3 5 LONG -> unassigned`

### For developers

- keypad handling is one state machine (`firmware/keypad_input.{h,cpp}`) with a keymap per bank
  and radio (`firmware/keypad_keymap.cpp`); host unit tests in `tests/keypad` run in CI
- `generate_voices.py`, `say.py` and `setup_venv.ps1` default to `en_US-lessac-medium` with both
  noise scales at 0, so regenerating the clips gives identical files

## V3.5.8 FTDX10 and Keypad Refinement

The repository is now aligned to the local `V3.5.8` firmware state.

Main points:

- synced the current local V3.5.8 firmware sources into `firmware/`
- renamed the main sketch entry to `TalkingRemoteControllerLX1WJ_V3_5_8.ino`
- updated `firmware_version.h` with `HAMTRC_FIRMWARE_VERSION` set to `V3.5.8`
- refreshed the FTDX10 / FTDX101D / FTDX101MP CAT command notes and keypad target layout
- updated Yaesu CAT handling, radio preferences, monitoring, speech, console, and keypad code from the local build
- added Bank 8 keypad setup for CI-V profile connection overrides: CI-V address on `T1` and baud rate on `T2`
- added the current V3.5.8 German operation guide and Jan's FT-897D testing note under `docs/`

## V3.5.7 Firmware and Online Update Prep

The repository is now aligned to the local `V3.5.7` firmware state.

Main points:

- synced the current local firmware sources into `firmware/`
- renamed the main sketch entry to `TalkingRemoteControllerLX1WJ_V3_5_7.ino`
- added `firmware_version.h` with `HAMTRC_FIRMWARE_VERSION` set to `V3.5.7`
- added the HAMTRC service response for online updater port detection via `HAMTRC?`
- added the `HAMTRC_BOOTLOADER` service command for controlled updater handoff
- refreshed SD card profiles, including the IC-7760 profile
- added the current V3.5.7 German operation guide under `docs/`

## V3.5.5 Voice and FTDX10 Test Refresh

The repository is now aligned to the local `V3.5.5` firmware state.

Main points:

- synced the current local firmware sources into `firmware/`
- refreshed generated `voice_data.h` and the matching `voice_assets` clips
- added the current FTDX10 planning and regression notes from the local project folder
- renamed the main sketch entry to `TalkingRemoteControllerLX1WJ_V3_5_5.ino`

## V3.5.4 FTDX10 Family Test Update

The repository is now aligned to the local `V3.5.4` firmware state.

Main points:

- synced the newer FTDX10 family firmware files and profiles into the repository
- documented the first practical FTDX10 / FTDX101D / FTDX101MP keypad block for assisted testing
- added VFO mode, IF, ID, AGC, power-state, and preamp details to the published FTDX10 family notes
- tightened the user-facing documentation so it stays short and practical
- separated user help from specialist notes more clearly

## V3.5.1 FT8x7 Reliability Update

This repository now tracks the local `V3.5.1` firmware build and its matching radio documentation.

Main points:

- fixed FT8x7 VFO A/B selection so FT-817 family VFO operations no longer fail silently
- corrected FT8x7 split-status decoding to use the proper TX-status bit instead of the former ALC bit mix-up
- disabled the unsafe pseudo memory-write path that could unintentionally switch CTCSS/DCS settings on FT8x7 radios
- changed FT8x7 keypad write flows so frequency and mode writes are verified more carefully and rejected while the radio is in TX
- added clearer FT8x7 lock-state caching, poll suppression, and serial trace handling for more reliable keypad interaction
- updated `ft817.ini`, `ft818.ini`, `ft857.ini`, and `ft897.ini` with the current FT8x7 capability and Bank 6 defaults
- added SD profile boot-status reporting in the main firmware startup path
- refreshed FT8x7 radio pages and the support matrix so the published per-radio descriptions match the current code

## V3.5 Transition

- moved from the older single-sketch public state to the modular `V3.5` firmware structure
- introduced the parallel short name `HamTRC-LX1WJ`
- added SD card based profile handling and modular protocol code
