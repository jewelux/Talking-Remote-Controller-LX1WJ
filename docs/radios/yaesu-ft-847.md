# Yaesu FT-847

## Status

First step, **not yet tested on the radio**: frequency and mode (read and set), S-meter and
RX/TX status. Not yet: PTT, satellite mode, repeater shift, CTCSS and DCS.

The FT-847 has its own protocol, `YAESU_FT847` (`firmware/protocol_ft847.*`,
`firmware/ft847_codec.*`). It uses the same 5-byte CAT frame as the FT-817/857/897, but several
commands mean something else, so it never runs FT-8x7 code:

| Bytes | FT-817/857/897 | FT-847 |
|---|---|---|
| `00 00 00 00 00` | lock on | **CAT ON** (required before anything else) |
| `00 00 00 00 80` | lock off | **CAT OFF** |
| `00 00 00 00 13` | read a meter, 1 byte | read the satellite RX VFO, 5 bytes |
| `00 00 00 00 E7` | S-meter in 4 bits | S-meter in **5 bits** (0..31 dots) |
| `0x81`, `0x05`, `0x0F`, `0xBB` | VFO A/B, clarifier, power, EEPROM | do not exist |

Sources: FT-847 operating manual, pp. 91-93 (CAT System Programming), and Hamlib
`rigs/yaesu/ft847.c`. The manual's status bit charts on p. 92 have their titles swapped; see
`ft847_codec.h`.

## Safety

- While an FT-847 profile is active, only FT-847 CAT frames reach the radio. Any other write,
  for example a console command meant for another radio, is dropped and the console prints
  `[F847] blocked a write that is not an FT-847 CAT frame`. Without this, stray bytes could be
  read as an FT-847 command, e.g. `08` = PTT ON.
- `F847RAW` sends any frame you type, including `0000000008` (PTT ON, the radio transmits).
- Test with a dummy load on the antenna jack.

## Before the first test

1. **Serial number.** FT-847s built before the `8G05` production run (May 1998) cannot report
   frequency, mode or status over CAT; HamTRC would only be able to send. The serial number is
   on the rear panel, e.g. `8G051234` = 1998, May, run 05. Note it in the test report.
2. **Cable.** The FT-847 CAT jack is a 9-pin RS-232 port with its own level converter. Use a
   **null-modem (crossed)** cable to HamTRC's RS-232 port (MAX3232, GPIO 9/10), not a straight
   one. This differs from other Yaesu radios.
3. **Tuner.** CAT does not work while an FC-20 tuner is connected to the TUNER jack. Unplug the
   FC-20 control cable.
4. **Profile.** The FT-847 is built into the firmware as profile 16; no SD card is needed.
   Choose it with the keypad profile select or console `PROFILE 16`. `PROFILE?` must show
   `protocol: YAESU_FT847_CAT`.
5. **Baud rate.** HamTRC starts at 4800, the radio's default in menu 37. If the radio is set to
   9600 or 57600, set the same in HamTRC: Bank 8 `2` long or double press, or console
   `BAUD 9600`. HamTRC keeps it for profile 16; `PROFILE RESET` goes back to 4800.

## Test steps (serial console, 115200 baud)

Turn the trace on first, so every frame shows: `F847TRACE ON`.

| # | Command | Expected | If not |
|---|---|---|---|
| 1 | `F847?` | `CAT on (sent automatically), byte gap 50 ms` | |
| 2 | `F847RAW5? 0000000003` | 5 bytes, e.g. `01 42 50 00 01` = 14.250 MHz USB. The CAT icon on the radio's display flashes | No reply: check cable (null-modem), baud, FC-20, serial number. Try `F847CAT ON`, then again |
| 3 | `FREQ?` | the radio's frequency | |
| 4 | `F847MODE?` | the radio's mode; try CW-N or FM-N on the radio: `(narrow)` | |
| 5 | `F847RX?` | S-meter dots and S units, squelch open/closed. Compare with the radio's meter at a few signal levels | Note dots vs. the radio's display |
| 6 | `F847TX?` | `RX` | |
| 7 | `RXTX?` | `RX` | |
| 8 | `FREQ 14074` (or the keypad frequency entry) | the radio goes to 14.074 MHz | |
| 9 | `MODE LIST`, then `MODE USB`, `MODE CW` … (or keypad mode select) | the radio follows; RTTY and DIGI are rejected (the FT-847 has none) | |
| 10 | Turn the dial | HamTRC announces the new frequency after about a second | |
| 11 | Switch the radio off and on, then `FREQ?` | works again (HamTRC re-sends CAT ON after a missed reply) | `F847CAT ON` |
| 12 | `F847GAP 0`, then steps 2-5 again | Same results. If so, polling can be faster; report it | Set back with `F847GAP 50` |
| 13 | Keypad: Bank 1 keys (frequency, mode, S-meter) | spoken answers; keys for features the FT-847 lacks say "not available" | |

Please send back: the serial number, which steps worked, and the console output (with trace) of
any step that did not.
