# Yaesu FT-847

## Status

Tested on Richard's FT-847 (serial 8H0901.., June 1998) by Jean LX1WJ on 5-6 Oct 2026, with a
Mini-HamTRC (no SD card) on profile 16:

All serial console commands were tested on 6 Oct 2026 (dummy load, no antenna):

| Works | Console | Notes |
|---|---|---|
| Frequency read and set | `FREQ?`, `FREQ <kHz>` | 10 Hz steps, HF to 70 cm (7.074 and 145.500 MHz tried); turning the dial is announced |
| Mode read and set | `MODE?`, `MODE LIST`, `MODE <name>`, `F847MODE?` | LSB, USB, CW, CW-R, AM; narrow variants read as their mode; `MODE FM` is rejected |
| RX/TX status | `F847TX?`, `RXTX?` | RX and TX (MOX) both read correctly |
| S-meter | `SM?`, `F847RX?` | read in SSB and AM; in FM the radio does not answer it. Not yet compared with a real signal |
| Power while transmitting: the radio's PO/ALC meter, 0..31 ("power 12"; "power rx" in receive) | `PO?` | keypad Bank 1 `4` |
| Narrow filter in CW, CW-R and AM, like the radio's NAR key ("cw n" when on; "not available" in SSB). CW-N needs the optional YF-115C filter and menu 33 on | `NAR?`, `NAR ON`, `NAR OFF`, `NAR TOGGLE` | AM-N and CW-N tried. No keypad key yet (to agree with Jan) |
| CAT OFF and ON by hand | `F847CAT OFF`, `F847CAT ON` | no reply while off, works again after on |
| CAT ON after the radio was switched off and on | | automatic |
| FM guard | | radio switched to FM: short transmit, then receive and "FM not available" |
| Test aids | `F847?`, `F847TRACE`, `F847POLL`, `F847GAP`, `F847RAW…`, `BAUD?`, `HELP` | |

Not yet tested: the keypad (the test board has none), and the S-meter against a real signal on
an antenna.

**FM is not supported** on this radio, see below. Not yet: PTT, satellite mode, repeater shift,
CTCSS and DCS.

Two things to know when operating:

- Choose AM with HamTRC (`MODE AM` or the keypad), not with the radio's FM/AM key. That key
  passes through FM on its way to AM, and if a poll catches the radio in FM, the FM guard
  trips (short transmit, "FM not available", poll stopped until the next query).
- `F847RAW5? 0000000003` is a raw frame and has no FM guard. In FM it keys the radio, which
  then keeps transmitting until `F847RAW 0000000088` (PTT OFF).

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

## FM is not supported: the radio transmits on a frequency/mode read

In FM, every frequency/mode read (`00 00 00 00 03`) puts this FT-847 into transmit. The radio
still answers the read correctly (mode byte `08`), and its TX status (`F7`) then says TX. It
stays in transmit until PTT OFF (`00 00 00 00 88`) or power off.

How it was narrowed down (dummy load, trace on):

| Test | Result |
|---|---|
| Cable unplugged, switch to FM | receive |
| HamTRC connected, polling once a second, switch to FM | **transmit** within about a second |
| `F847CAT OFF`, then FM | receive |
| `F847CAT OFF` while transmitting | keeps transmitting |
| `PTT OFF` (`F847RAW 0000000088`) while polling | receive for a moment, transmit again at the next poll |
| 3-wire cable (pins 2, 3, 5 only) | **transmit**: the RS-232 control lines are not the cause |
| Byte gap 50 ms instead of 0 | **transmit**: timing is not the cause |
| CAT on, background poll paused (`F847POLL OFF`), then FM | receive |
| then one read, `F847RAW5? 0000000003` | **transmit** |
| one read with other parameter bytes, `F847RAW5? FFFFFFFF03` | **transmit** |
| TX status read (`F847TX?`, `F7`) in FM | receive, answer RX |
| RX status read (`F847RX?`, `E7`) in FM | receive, no answer |
| SSB, CW, AM with polling, for minutes | receive |

So it is the radio's handling of the read in FM, probably in the early firmware of this serial
range (two-way CAT came with run 8G05 in May 1998; this unit is 8H09). Satellite programs poll the
FT-847 the same way, so later units presumably do not do this. Not yet tried: the satellite VFO
reads `13` and `23` in FM.

What HamTRC does about it:

- The FT-847 profile has no FM mode, so HamTRC never selects FM (`MODE FM` and the keypad mode
  select say "not available").
- If a read finds the radio in FM (switched on the radio's front panel), HamTRC sends PTT OFF at
  once, says "FM not available", prints a note on the console and stops the background poll. The
  radio has then transmitted for about a quarter of a second.
- The poll restarts when a query the user starts (keypad or console `FREQ?`) finds another mode.
  While the radio is still in FM, that query causes the same short transmit, which HamTRC ends the
  same way.

Richard uses a TYT MD-9600 with OpenGD77 speech for FM on 2 m and 70 cm, and the FT-847 with
HamTRC for SSB, CW and AM on all bands.

## Safety

- While an FT-847 profile is active, only FT-847 CAT frames reach the radio. Any other write,
  for example a console command meant for another radio, is dropped and the console prints
  `[F847] blocked a write that is not an FT-847 CAT frame`. Without this, stray bytes could be
  read as an FT-847 command, e.g. `08` = PTT ON.
- `F847RAW` sends any frame you type, including `0000000008` (PTT ON, the radio transmits).
  `F847RAW 0000000088` is PTT OFF.
- Test with a dummy load on the antenna jack.

## Before the first test

1. **Serial number.** FT-847s built before the `8G05` production run (May 1998) cannot report
   frequency, mode or status over CAT; HamTRC would only be able to send. The serial number is
   on the rear panel, e.g. `8G051234` = 1998, May, run 05. Note it in the test report.
2. **Cable.** The FT-847 CAT jack is a 9-pin RS-232 port with its own level converter. It needs
   its pin 2 and 3 crossed to the PC or HamTRC side. With a HamTRC whose DB-9 is wired like a PC,
   use a **null-modem (crossed)** cable; Jean's Mini-HamTRC crosses them itself, so it uses a
   **straight** cable. Pins 2, 3 and 5 (ground) are all that is needed.
3. **Tuner.** CAT does not work while an FC-20 tuner is connected to the TUNER jack. Unplug the
   FC-20 control cable.
4. **Profile.** The FT-847 is built into the firmware as profile 16; no SD card is needed.
   Choose it with the keypad profile select or console `PROFILE 16`. `PROFILE?` must show
   `protocol: YAESU_FT847_CAT`.
5. **Baud rate.** HamTRC starts at 4800, the radio's default in menu 37. If the radio is set to
   9600 or 57600, set the same in HamTRC: Bank 8 `2` long or double press, or console
   `BAUD 9600`. HamTRC keeps it for profile 16; `PROFILE RESET` goes back to 4800.
6. **Not in FM.** Keep the radio in SSB, CW or AM while HamTRC is connected.

## Test steps (serial console, 115200 baud)

Turn the trace on first, so every frame shows: `F847TRACE ON`.

| # | Command | Expected | If not |
|---|---|---|---|
| 1 | `F847?` | `CAT on (sent automatically), byte gap 50 ms, background poll running` | |
| 2 | `F847RAW5? 0000000003` | 5 bytes, e.g. `01 42 50 00 01` = 14.250 MHz USB. The CAT icon on the radio's display flashes | No reply: check cable, baud, FC-20, serial number. Try `F847CAT ON`, then again |
| 3 | `FREQ?` | the radio's frequency | |
| 4 | `F847MODE?` | the radio's mode | |
| 5 | `F847RX?` | S-meter dots and S units, squelch open/closed. Compare with the radio's meter at a few signal levels | Note dots vs. the radio's display |
| 6 | `F847TX?` | `RX` | |
| 7 | `RXTX?` | `RX` | |
| 8 | `FREQ 14074` (or the keypad frequency entry) | the radio goes to 14.074 MHz | |
| 9 | `MODE LIST`, then `MODE USB`, `MODE CW` … (or keypad mode select) | the radio follows; FM, RTTY and DIGI are rejected | |
| 10 | Turn the dial | HamTRC announces the new frequency after about a second | |
| 11 | Switch the radio off and on, then `FREQ?` | works again (HamTRC re-sends CAT ON after a missed reply) | `F847CAT ON` |
| 12 | FM guard: switch the radio to FM on its front panel | a short transmit, then receive; HamTRC says "FM not available" and the console shows the FM note; `F847?` says the poll is stopped | Radio keeps transmitting: `F847RAW 0000000088`, report it |
| 13 | Back to USB on the radio, then `FREQ?` | the frequency; `F847?` says the poll is running again | |
| 14 | Keypad: Bank 1 keys (frequency, mode, S-meter) | spoken answers; keys for features the FT-847 lacks say "not available" | |
| 15 | `PO?` in receive | `PO: RX (not transmitting)`, spoken "power rx" | |
| 16 | Press MOX on the radio (dummy load), `PO?`, release MOX | `PO: n of 31`, spoken "power n"; compare with the radio's meter | |
| 17 | Radio in AM: `NAR ON`, `NAR?`, `NAR OFF` | the radio's NAR icon follows; "am n", then "am" | |
| 18 | Radio in USB: `NAR ON` | `NAR -> not available in USB` | |

Test aids: `F847POLL OFF | ON` pauses the background poll (CAT stays on); `F847GAP <ms>` changes the
pause between the five bytes (0 worked for reads in SSB).

Please send back: the serial number, which steps worked, and the console output (with trace) of
any step that did not.
