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
- `#` cancels any entry, selection, picked mode or key waiting for a double press, and says
  "cancel" (it said "ok"; cancelling bank or profile select was silent). With nothing to cancel it
  beeps
- holding a key that has no long action beeps once the hold time is reached, and releasing it does
  nothing. Before, the release ran the key's short action. This includes `D` and `#`
- a key that waits for a possible double press (e.g. Bank 1 `0`, `1`, `2`) answers as soon as
  another key is pressed, before that key. Before, the two answers came in the wrong order, or the
  first press was lost
- on such a key, a press quickly followed by a hold beeps. Before, it ran the long action
- pressing a key while another is held beeps, and both are ignored until all keys are let go;
  digits already typed stay. This includes `#`
- bank select with `*` still held: the digit picks the bank of the next key only, then your bank is
  back. Only the digit is spoken
- an entry or bank, profile or mode select left for 30 seconds without a key ends and says
  "timeout". Before, it waited forever, and tuning on the radio was not announced meanwhile
- a key that does nothing gives a short beep instead of silence: keys with no action on the current
  bank, and keys an entry or selection does not take
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

- speech speed slow, normal or fast: Bank 9 `6`, console `SPEED`
- no long pause between a number and its unit or "ok" ("power 50 watts")
- verbose off says only the value: "50 watts" instead of "power 50 watts", "five" instead of
  "volume five ok". Units stay; the name and the "ok" after a change are left out. Bank 9 `5`
  short says "verbose on" or "verbose off", `5` long switches it; console `VERBOSE?` and
  `VERBOSE ON | OFF | TOGGLE`. Verbose is on by default and kept after power off
- any key press stops what is being spoken, so answers no longer queue up behind earlier ones
- new voice (Piper `en_US-lessac-medium`) for all clips
- "timeout" when the radio does not answer, and "not available" for features the radio or profile
  cannot provide (e.g. keys hidden on FTDX10, an empty profile slot); both used to say "error" or
  nothing
- a frequency or mode the radio did not take says "error" on Icom, Kenwood, Elecraft and FTDX
  radios too. Before, only the FT-8x7 said it; the others stayed silent
- TS-480 and KX2: the Bank 3 VFO A/B frequency and mode set keys say "not available" at once
  instead of asking for the entry and then staying silent
- FT-8x7: a CTCSS tone or DCS code that is not a standard one says "error". Before, it was silent.
- a key whose radio command fails always answers: "timeout" when the radio does not reply, "error"
  when it refuses or answers wrongly, and "not available" when the radio or profile has no such
  command. Before, many keys (VFO, split, RIT, tuner, PBT and others) stayed silent unless it was a
  timeout, and keys for a feature the radio lacks (tuner, monitor, RIT, band stack) beeped
- with verbose on, "not available", "timeout" and "error" from a key start with the function's
  name, e.g. "tuner not available", "split timeout", "c t c s s error"
- a CI-V address with the hex digit E (e.g. E0) says "e" instead of "error"
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
  `MONITOR`, `TRANSCEIVE`, `RIT`, `FILSHAPE` and `TUNINGSPEECH`; `CIVADDR? | <hex>` (CI-V) and
  `BAUD? | <rate>`; on FT-8x7 `CTCSS? | <Hz>`, `DCS? | <code>` and `VFO SYNC A | B`; on
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
- FT-817/818/857/897: every Bank 2 and Bank 8 key, Bank 1 `6` and Bank 3 `5` has a console command
  (`NB?`, `AGC?`, `MENU?`, `ROW?`, `RFPOWER?`, `RIT?` …; on the FT-817/818 also `ANT?`; on the FT-857/897 also `NR?`, `NOTCH?`,
  `NRLEVEL?`, `NBLEVEL?`, `HPF?`, `LPF?`, `MICEQ?`, `IPO?`, `ATT?` and `DBF?`). `BK?`, `KYR?` and,
  on the FT-857/897, `NAR?` read break-in, keyer and FM narrow. `IFSHIFT?` prints whether IF shift is on,
  and `YSETTINGS?` lists more settings read from the radio, e.g. VOX, lock, fast tuning, NB,
  break-in, keyer and IF shift, on the FT-857/897 also PROC, CW speed, the gains and the DSP filter
  widths. `CLAR ON | OFF` is gone, use `RIT ON | OFF`; `CLAR OFFSET` is now `RIT OFFSET`
- `PROC?` says whether the speech processor is on ("processor on") and `PROCLEVEL?` says its level
  ("processor level 50"), on the FT-857/897 (menu 74), Icom radios (COMP) and the FTDX10. They
  only read; the processor cannot be changed from HamTRC yet
- `MICGAIN?` says the mic gain ("mic gain 50") on Icom radios, the FTDX10, the FT-817/818 and
  the FT-857/897. The FT-817 and FT-857/897 have one for each mode, so it says the one of the
  current mode: SSB, AM, FM, DIG, and on the FT-817 also packet. In CW, and in packet on the
  FT-857/897, it says "not available". It only reads for now

### Radio profiles

- the radio profiles are built into the firmware: no SD card or SD card reader is needed, and a
  firmware update brings every profile up to date. The profile numbers are those of the SD card's
  `slots.ini`: 1 IC-7300, 2 IC-706, 3 IC-7300 RS-232, 4 IC-706 RS-232, 5 G106, 6 KX2, 7 TS-480,
  8 FT-817, 9 FT-857, 10 FT-897, 11 FTDX10, 12 FTDX101D, 13 FTDX101MP, 14 FT-818, 15 FT-891,
  17 IC-705, 18 IC-7760
- the baud rate can be set on every radio, not only on CI-V radios. Bank 8 `2` short says it
  ("baud 4 8 0 0"), long and double press step down and up through the rates the radio offers
  (FT-817/818/857/897: 4800, 9600, 38400; FTDX10, FTDX101 and FT-891: 4800 to 38400; TS-480:
  4800 to 57600; KX2: 4800 to 38400). Before, `2` short and long stepped up and down. Console
  `BAUD <rate>` refuses other rates and lists them. The choice is kept for each profile, and
  `PROFILE?` lists the rates
- Bank 9 `A` double press resets the profile: its baud rate and CI-V address go back to the
  defaults, and HamTRC says "profile reset". Console `PROFILE RESET`, and `PROFILE RESET ALL`
  for every profile
- the six Icom profiles HamTRC offered when the SD card did not load are gone, and with them
  Bank 9 `1`–`9` picking a profile directly; on Bank 9, `1`–`3` and `6` beep

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
- FT-817/818/857/897: Bank 2 reads the radio's settings, also those changed on the radio: noise
  blanker (`2`) and AGC (`A` double press, "a g c auto"). The FT-857/897 also reads DSP noise
  reduction (`1`), auto notch (`3`), the NR level (`4`, "noise reduction level 8"), the NB level
  (`5`), the DSP bandpass filter (`6`), the low cut and high cut (`7` and `8`), the TX equalizer
  (`9`) and the preamplifier and attenuator (`A`, long for the attenuator; the preamplifier is on
  when IPO is off); the FT-817 has no DSP, so these say "not available".
  HamTRC only reads these settings; it cannot change them
- FT-817/818: Bank 2 `0` says which antenna jack the current band uses (menu 07), "antenna front"
  or "antenna rear". The console command is `ANT?`
- FT-817/818/857/897: Bank 8 `8` says the menu item the radio's menu was last left on ("menu 7 6"),
  `7` the soft key or function row
- FT-817/818/857/897: Bank 1 `6` says the TX power: on the FT-857/897 the menu 75 power of the
  current band ("power 10 watts"), on the FT-817/818 the power setting (on the FT-818 6, 5, 2.5
  and 1 watts)
- FT-817/818/857/897: Bank 3 `5` short says whether RIT is on ("rit on"), long toggles it, also
  after RIT was switched with the radio's CLAR key. Before, the key switched RIT on or off
  without knowing the state. If RIT is on, asking switches it off and straight back on; on the
  FT-857/897 the knob then tunes RIT even if it was tuning IF shift.
  On the FT-817/818 this replaces VFO B mode: for VFO B's mode, switch with Bank 3 `1`
  long and use Bank 1 `9`
- FT-857/897: the Bank 3 VFO keys read which VFO is active from the radio, also after A/B was
  pressed on its front panel, so "vfo a" and "vfo b" are always right. Before, HamTRC assumed VFO
  A at start and needed Bank 3 `4` (sync) after a front panel change; that key is now unassigned
- FT-817/818/857/897: `RXTX?` reads the radio's PTT state. Before, it read part of the power meter
  and could report receive while transmitting. On the FT-817 Bank 1 `1` says it too, instead of
  "transceiver not available"
- FT-817/818: VFO A mode (Bank 3 `3`) and the console commands `VFOA?`, `VFOB?`, `VFOA MODE` and
  `VFOB MODE` switch back to the VFO you were on. Before, they left the radio on the VFO they
  read or set, and `VFOA?` and `VFOB?` could time out
- TS-480: `NR 0`, `NR 1` and `NR 2` set the NR level, and the Bank 2 `1` key and `NR?` say it
  ("noise reduction two") instead of only on or off
- NR, NB and notch keys say so when the radio rejects the change or does not answer; this was silent
- FTDX10: frequency entry and 500 Hz rounding announce the new frequency once and report a write
  the radio rejects. Bank 2 `8` and `9` say "not available" like the other hidden keys
- FTDX10: setting the current VFO's mode says it once (it was said twice), and a failed VFO A/B
  mode set says "error" or "timeout" instead of the mode name
- Bank 6 on radios other than the FT-8x7 family is empty: its keys beep like any unassigned key
  (they said "BANK6 reserved")

### Yaesu FT-847 (new, tested on one radio)

- new built-in profile 16 with its own protocol `YAESU_FT847` (baud 4800, 9600 or 57600, like
  the radio's menu 37). The FT-847 shares the
  5-byte CAT frame of the FT-817/857/897 but not all of its commands (`0x80` is CAT OFF, not
  lock off), so no FT-8x7 code runs for it
- frequency and mode read and set, S-meter (5-bit, 0..31 dots), RX/TX status. HamTRC sends CAT ON
  before the first command and again after the radio did not answer, e.g. after it was switched
  off and on
- while an FT-847 profile is active, only FT-847 CAT frames reach the radio; any other write (e.g.
  a console command for another radio) is dropped and reported, because its bytes could be read
  as an FT-847 command such as PTT ON
- console commands for testing: `F847?`, `F847CAT ON | OFF`, `F847GAP`, `F847RX?`, `F847TX?`,
  `F847MODE?`, `F847TRACE`, `F847RAW`, `F847RAW1?`, `F847RAW5?`; see
  `docs/radios/yaesu-ft-847.md`. Host tests in `tests/ft847`
- tested on Richard's FT-847 (serial 8H09.., June 1998), every console command: frequency and
  mode read and set, RX/TX status, S-meter, PO meter, narrow (AM-N, CW-N), CAT OFF/ON, CAT ON
  after a power cycle, FM guard. Not yet: keypad, S-meter against a real signal
- FM is not supported: in FM this FT-847 goes into transmit on every frequency/mode read. HamTRC
  never selects FM; if the radio is switched to FM on its front panel, HamTRC sends PTT OFF at
  once, says "FM not available" and stops polling until a query finds another mode. Details and
  the tests in `docs/radios/yaesu-ft-847.md`. Console `F847POLL OFF | ON` pauses the poll
- `PO?` and Bank 1 `4` say the PO/ALC meter (0..31) while transmitting, "power rx" in receive
- `NAR? | ON | OFF | TOGGLE` switch the narrow filter in CW, CW-R and AM, like the radio's NAR
  key ("cw n"); no key yet
- not yet: PTT, satellite mode, repeater shift, CTCSS and DCS

### Firmware updates

- every push builds the firmware as a factory image with a manifest for the online updater;
  version tags attach them to a GitHub release
- the spoken words are a separate voice pack, installed by the online update; without it the
  controller says "voice pack missing"

### For testers: serial trace

- a keypad `CMD` line names the key and gesture, e.g. `CMD BANK3 2 LONG -> A=B`, also where it used
  a placeholder (`BANK3 FT857 -> SPLIT ON` is now `BANK3 0 LONG -> SPLIT ON`)
- a rejected key names the mode it was pressed in: `FREQ A -> unassigned`,
  `MODE SELECT 0 -> unassigned`, `BANK3 5 LONG -> unassigned`

### For developers

- building needs the ESP32 Arduino core 3.x or newer; an older core stops with a message saying so
- keypad handling is one state machine (`firmware/keypad_input.{h,cpp}`) with a keymap per bank
  and radio (`firmware/keypad_keymap.cpp`); host unit tests in `tests/keypad` run in CI
- the radio profiles are one table, `kProfiles` in `firmware/radio_profile_table.cpp`, checked
  at build time and by host unit tests in `tests/profiles`; a new radio is one entry. The SD
  card loader and its ini files are gone. The host unit tests build as C++20
- the FT-8x7 CAT fields and EEPROM map (`firmware/ft8x7_*`) are plain C++ with host unit tests in
  `tests/ft8x7`; the on/off settings each model keeps in its EEPROM are one table per model
- `generate_voices.py`, `say.py` and `setup_venv.ps1` default to `en_US-lessac-medium` with both
  noise scales at 0, so regenerating the clips gives identical files
- `voice_data.h` is replaced by `voices.bin` (`build_voice_pack.py`), flashed to the `voices`
  partition at `0x810000`

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
