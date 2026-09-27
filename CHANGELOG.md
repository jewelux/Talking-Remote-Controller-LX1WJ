# Changelog

## Unreleased — Keypad state machine

- console commands for what only the keypad could do, taking the keypad's fixed value as an
  argument: `ROUND [<Hz>]` (Bank 1 `0` double rounds to 500 Hz); `NRLEVEL`, `NBLEVEL`,
  `MONLEVEL`, `PBT1`, `PBT2`, `RIT` and `VOLUME STEP <+-n>`; `FILWIDTH NEXT | PREV`; `TOGGLE` for
  `MONITOR`, `TRANSCEIVE`, `RIT`, `FILSHAPE` and `TUNINGSPEECH`; `CIVADDR? | <hex>` and
  `BAUD? | <rate>` (CI-V); on FT-8x7 `CTCSS? | <Hz>`, `DCS? | <code>` and `VFO SYNC A | B`
- `BANK <n>`, `BANK NEXT` and `BANK PREV` on the console now cover banks 1–9 like the keypad
  (they stopped at 3)
- keypad input is now one state machine (`firmware/keypad_input.{h,cpp}`) with a declarative
  keymap per bank (`firmware/keypad_keymap.cpp`); the bank actions moved to
  `ui_keypad_bank1..9.cpp` and the entries to `ui_keypad_entry.cpp`. Host unit tests in
  `tests/keypad` (`make -C tests/keypad`) cover the state machine and every key of every bank for
  each radio family, and run in CI
- the keymap alone decides what a key does on each radio: `KeypadTraits` names one
  `KeypadLayout` (generic, CI-V, FTDX10, FT-8x7, FT-817, FT-857/897) and the bank actions no
  longer check which radio they run on. The radio branches inside actions became their own
  actions, and the FTDX10 console-command keys are listed in the keymap. Branches no key could
  reach were removed: the FT-857/897 Bank 1 `2` TX frequency read by toggling the VFO (replaced
  earlier by the "not available" answer while split is on), plus FT-857/897 and FT-817 fallbacks
  in Bank 3 actions those radios never reach. The host tests gained an "FT-8x7 without variant"
  family
- the serial `CMD` line of a bank key now always names the key and gesture that ran it, taken from
  the keymap (`CMD BANK3 2 LONG -> A=B`); the actions no longer spell out their key. Lines that
  used a placeholder name the real key now: `BANK3 FT857 -> SPLIT ON` becomes `BANK3 0 LONG ->
  SPLIT ON`, `BANK6 FT8X7 -> RPT OFF` becomes `BANK6 0 SHORT -> RPT OFF` (printed once, not
  twice), `BANK5 STEP -> +10 Hz` becomes `BANK5 2 SHORT -> RIT STEP +10 Hz`, and a hidden FTDX10
  key reports e.g. `BANK4 1 SHORT hidden on FTDX10`. A follow-up query inside an action (e.g. the
  VFO A read after VFO A select) now shows the key that was pressed instead of the query's own key.
  An FTDX10 key that sends a console command prints the command itself (`LOCK TOGGLE`, was
  `LOCK`; likewise NR, NB, NOTCH, SPLIT and Bank 4 TUNER)
- one table in `keypad_input.cpp` (`EntrySpec`) holds the digit rules of bank select, profile
  select and every entry: label, length, whether a digit replaces a full entry, leading zero,
  digits after the point and the unit shown while typing. A new entry is an `InputMode`, a table
  row and its commit. Mode select is one mode with an optional picked mode (`ModeStaged` is
  gone). The beep for a key an entry rejects now names the entry: `FREQ A` (was `ENTRY A`),
  `RPTSHIFT 9` / `CTCSS D` / `DCS D` (was `BANK6 ENTRY ...`)
- the staged command (a Bank 1 query kept for Enter when `AUTO_SEND_BANK1_QUERIES` is off) now
  lives in `KeypadInput` next to the staged mode, instead of in statics in `ui_keypad.cpp` that
  the state machine asked about through the listener; behaviour is unchanged and host tests now
  cover it
- FT-817 Bank 3 `1` long toggles VFO A/B and `2` long copies the active VFO's frequency and mode
  to the other (A=B, spoken "a equals b") again; both had become unreachable when the long press
  was given the same frequency entry as the double press. New console command `VFO A=B` (FT-817)
- Bank 6 on radios other than the FT-8x7 family is now empty like any unassigned key
  ("BANK6 k -> unassigned" beep, a hold beeps at the hold time); before, each key beeped
  "BANK6 reserved", and `0`-`2` first waited for a double press
- FT-857/897 Bank 3 `6` short (PTT off) no longer prints the "BANK3 6 SHORT -> RXTX?" line first
- fixed `1`/`2` being ignored after a Bank 3 VFO A/B mode select (`3`/`4`/`5` long): the next
  press, e.g. mode digit 1 (LSB), was swallowed
- fixed FT-817 Bank 3 `1`/`2` long: releasing the held digit typed it into the new frequency entry,
  and a later short `1`/`2` did nothing
- fixed Bank 1 `9` long mode select applying the mode to VFO A or B when an earlier Bank 3 VFO mode
  select had ended on an invalid digit; it now always sets the current VFO's mode
- pressing `#` while another key is held no longer runs that key's short action on its release
- a short press waiting for a possible double press (e.g. Bank 1 `0`, `1`, `2`) now answers as
  soon as another key goes down, before that key's own answer. Before, a key pressed within the
  220 ms wait answered first and the waiting key afterwards, and a second waiting key, a hold or
  bank select dropped the first key's press entirely. `#` still cancels the waiting press
- fixed mode select from Bank 3 (`3`/`4`/`5` long): digits `6`–`9` ran the bank's RX/TX, band stack
  or VFO actions instead of picking the mode. The same applied to other keys with a short action
  on the current bank (Bank 5 `1`–`5`, Bank 6 `3`/`4`, Bank 8 `1`, Bank 9 `B`/`C`). Every key is
  now taken as the mode digit
- mode select ("mode please") now waits for a mode digit and only `#` cancels it. Holding a key no
  longer runs its long action there (e.g. Bank 1 `0` long started a frequency entry whose first
  digit was then taken as the mode). A key that picks no mode, `*` or `D` beeps and mode select
  stays active. Before, an invalid digit ended mode select (and dropped a mode staged earlier),
  `*` said or selected the bank, and `D` could apply an earlier staged mode or send a staged
  command
- a mode picked in mode select now waits for `D` to apply it or `#` to cancel it. Another mode
  digit replaces it; any other key beeps. Before, the other keys kept working and the staged mode
  stayed behind, so a later `D`, e.g. one pressed after finishing a frequency entry, applied the
  old mode
- holding a bank key that has no long action now beeps once the hold time is reached
  ("BANKn k LONG -> unassigned") and the release does nothing. Before, the release ran the key's
  short action, or beeped only then when it had none
- holding `D` or `#` beeps the same way ("ENTER LONG" / "CLEAR LONG") and the release does
  nothing; before, the release entered or cleared. In entries and bank, profile and mode select,
  holds are still ignored and the release acts
- `#` with nothing to cancel (no entry, selection, staged command or waiting key) beeps
  ("CLEAR -> unassigned") instead of saying "cancel"
- `D` with nothing typed or chosen beeps and the entry or selection stays, the same in every
  entry and in bank, profile and mode select; only `#` cancels. Before, frequency, RF power and
  CI-V address entry said "error" and ended, repeater offset, CTCSS and DCS entry failed silently,
  and bank and profile select ended with a "no selection" beep
- choosing a profile number with no stored profile in profile select says "not available" instead
  of beeping
- FTDX10: Bank 2 `8` and `9` now say "not available" like the other keys hidden on FTDX10,
  instead of the "unassigned" beep

## Unreleased — Firmware CI

- GitHub Actions workflow `.github/workflows/firmware.yml` builds the ESP32-S3 firmware on every
  push/PR and packages `hamtrc-<version>.factory.bin` + ESP Web Tools `manifest.json` for the
  online updater; `v*` tags attach them to a GitHub Release

## Unreleased — Voice clips

- all voice clips regenerated with Piper voice `en_US-lessac-medium` (was lessac-high) with both
  noise scales at 0, so regenerating gives identical clips; these are now the defaults of
  `generate_voices.py`, `say.py` and `setup_venv.ps1`
- added voice clips "cancel", "not available" and "timeout"
- pressing `#` to cancel an input now says "cancel" instead of "ok"; cancelling bank or profile
  selection, which was silent, says "cancel" too
- when the radio does not answer a keypad action or serial command, the device now says "timeout";
  it used to say "error" or nothing. Failures for other reasons (unsupported, rejected) keep their
  old behaviour
- features the radio or profile cannot provide now say "not available": keys hidden on FTDX10, RF
  power set, TX frequency and VFO B / VFO mode on FT-857/897, RX/TX state on FT-817, CI-V address and
  baud setup, an empty profile slot, and serial commands answered "unsupported" or "hidden on
  FTDX10". Before, these said "error" or nothing
- a key that does nothing now gives a short beep instead of silence: keys with no action on the
  current bank or profile, keys whose feature the profile's protocol lacks (e.g. on FT-8x7: NR, NB,
  notch, tuner, monitor, transceive, band stack, RIT), BANK6 on non-FT-8x7 profiles, `D` with
  nothing to enter, and keys ignored during an entry or bank/profile selection (extra digits, a
  second `*`, letter keys, an invalid mode digit, a leading `0` in profile select)
- WFM mode is now spoken as "wfm" using its own clip; the clip was already in `voice_data.h` but
  missing from the voice table, so it was spelled out as "w f m"

## Unreleased — FT8x7 S-meter

- fixed the FT-817/818/857/897 S-meter always reading S0: the RX-status byte carries the meter in
  its low nibble (S0..S9, then S9+10..+60 dB) and is now decoded per protocol; the console shows
  e.g. `S9+20dB`
- dB over S9 is spoken as a word ("S meter nine plus twenty") using new voice clips ten..sixty;
  falls back to digits if a clip is missing

## Unreleased — FT8x7 frequency polling

- re-enabled background frequency polling for the Yaesu FT-817/857/897 family (disabled in V3.5.8)
- FT8x7 polls every 700 ms with a 300 ms timeout; other radios keep 400 ms / 80 ms
- after 3 failed polls in a row (radio off or disconnected) polling backs off to every 3 s so the
  blocking timeout does not starve the keypad
- after a CAT timeout the next FT8x7 command waits for the line to go quiet, so a late reply is not
  read as the start of the next one; commands are spaced at least 20 ms apart
- FT8x7 frequency/mode frames with non-BCD digits or an out-of-range frequency are rejected; a
  polled frequency change is accepted on the first reading (confirmation by two identical readings
  is available but off)
- the first FT8x7 command after the CAT port is opened now keeps the same minimum gap as between
  commands
- fixed FT-857/897 dial lock turning on when HamTRC starts with the radio already on: the RS232 TX
  line was held low (a break) from power-up until the radio profile was applied, because the early
  `digitalWrite(HIGH)` was ignored while the pin was not yet set up as GPIO; the radio read the end
  of the break as a stray byte, which shifted the first poll so it was executed as LOCK ON

## Unreleased — Frequency precision and entry

- added a `RadioFrequency` class (`firmware/radio_frequency.{h,cpp}`) that owns frequency parsing,
  formatting and rounding
- frequency is now announced and displayed to 10 Hz resolution as `MHz` "point" fractional digits
  with trailing zeros dropped but always at least one decimal (e.g. `7.12345`, `14.1`, `7.0`)
  instead of being truncated to kHz; the voice dictionary is unchanged
- keypad frequency entry now reads a plain number as MHz with `*` as the decimal point
  (`14*1` → 14.1 MHz, `14*12345` → 14.12345 MHz), replacing the previous kHz-integer entry
- added Bank 1 `0` double-press to round the current frequency to the nearest 500 Hz (the serial
  monitor reports old -> new; speech reports the new frequency the same way as a tuning
  announcement, without the "frequency" prefix)
- added Hz-argument console commands `FREQHZ`, `VFOAHZ`, `VFOBHZ`
- FTDX10 keypad frequency entry and rounding set the frequency directly instead of through console
  commands, so the new frequency is announced once and a rejected write is reported as failed
- tuning is announced only once the frequency is at least 100 Hz (`FREQ_SPEAK_MIN_STEP_HZ`) away
  from the last frequency the user heard or entered: a tuning announcement, a Bank 1 `0` query,
  a keypad/console frequency entry or a 500 Hz rounding
- any key press now stops speech in progress and cancels a pending tuning announcement, so
  answers no longer queue up behind earlier announcements; key actions only append their
  label and value
- tuning announcements no longer queue up while the dial is moving: moving the dial at least
  100 Hz away from the frequency being read out stops that readout immediately (other speech such
  as mode or key answers keeps playing), and only the frequency where tuning stops is announced;
  the fixed 5 s minimum interval between tuning announcements is replaced by a 500 ms gap
  (`FREQ_SPEAK_MIN_GAP_MS`) measured from the end of the previous announcement

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
