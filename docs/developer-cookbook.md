# HamTRC Developer Cookbook

A guide for anyone who wants to change the firmware: how the pieces fit
together, plus step-by-step recipes for the most common jobs (a new key, a new
radio feature, a new radio, a new spoken word).

The other docs, which this one does not repeat:

- [README](../README.md): project goals and the branch model (`main`, `development`, feature branches)
- [QUICKSTART](../QUICKSTART.md): flashing, the SD card, using the device
- [builder-guide](../builder-guide.md): hardware, online updates, and the documentation rule
- [hardware_wiring](hardware_wiring.md): pins and cables
- [voice_assets/piper/README](../firmware/voice_assets/piper/README.md): setting up voice generation

Contents:

1. [The map](#1-the-map)
2. [Build, run, test](#2-build-run-test)
3. [The keypad state machine](#3-the-keypad-state-machine)
4. [Recipes](#4-recipes)
5. [Best practices](#5-best-practices)
6. [Reference cards](#6-reference-cards)

---

## 1. The map

HamTRC is a single cooperative Arduino `loop()` on an ESP32-S3. It has no RTOS
tasks of its own; audio runs in the background. A key press runs its action
to completion, and that action talks to the radio synchronously.

```
 loop()                                    TalkingRemoteControllerLX1WJ_*.ino
  ├─ pollKeypadUi() ──► KeypadInput        keypad_input.*      pure C++: what the key means
  │                       │ keymapLookup   keypad_keymap.*     pure C++: which action, per radio
  │                       ▼
  │                    bank actions        ui_keypad_bankN.cpp, ui_keypad_entry.cpp
  ├─ processCommand() ─ console            ui_console.cpp
  │                       │
  │                       ▼
  │                    shared features     radio_features.* (+ ui_features.* for text/speech)
  │                       ▼
  │                    runtime             radio_runtime.*     applyXAndTrack / refreshLiveX
  │                       ▼
  │                    protocol façade     radio_protocol.*    setNb() → civSetNb / asciiSetNb / …
  │                       ▼
  │                    protocol ops        protocol_ops_{civ,ascii,yaesu}.*
  │                       ▼
  │                    framing / packets   protocol_*.*, packet_*.*
  │                       ▼
  │                    serial              transport_serial.*
  │
  ├─ pumpIncoming()      CI-V transceive frames ─┐  engine_civ.cpp
  └─ pollFrequencyIfDue() ────────────────────────┴► live (radio_state.*)  radio_monitor.cpp
```

| Area | Files | Owns |
|---|---|---|
| Keypad logic | `keypad_input.*`, `keypad_keymap.*`, `keypad_actions.h`, `keypad_digit_buffer.h` | Gestures, modes, entries; the key → action table. No `Arduino.h`. |
| Keypad glue and actions | `ui_keypad.cpp`, `ui_keypad_common.*`, `ui_keypad_bankN.cpp`, `ui_keypad_entry.cpp` | Carrying out the decision: radio calls, serial trace, speech |
| Console | `ui_console*.cpp` | Serial commands (`HELP` lists them) |
| Shared features | `radio_features.*`, `ui_features.*` | NR, NB and notch, used by both keypad and console |
| Radio API | `radio_protocol.*`, `radio_runtime.*`, `radio_state.*` | Protocol-independent calls, and the tracked `live` state |
| Protocols | `protocol_ops_*`, `protocol_*`, `packet_*`, `transport_serial.*` | CI-V, Kenwood/Elecraft/FTDX ASCII, Yaesu FT-8x7 5-byte CAT |
| FT-8x7 fields | `ft8x7_codec.*`, `ft8x7_eeprom_map.*`, `ft8x7_model.h` | CAT frame fields, the EEPROM map per model, the model from the variant. No `Arduino.h`; `protocol_ft8x7_eeprom.*` reads the EEPROM with them |
| Profiles | `radio_types.h`, `sd_profile_parser.*`, `profile_loader.*`, `sd_slots.*`, `radio_profile.*`, `radio_catalog.*` | What a radio is: connection, protocol, capabilities, command strings |
| Speech | `ui_speech.*`, `voice_data.h` (generated) | Clips, tokens, the audio queue |
| Settings | `radio_prefs.*` | NVS: profile, volume, tuning speech, CI-V address and baud per slot |

**Five concepts to learn first:**

- **Profile** (`StoredProfile`): one radio in one slot (1–24). Slots come
  from `SDCard/slots.ini`, with built-in CI-V profiles as the fallback.
- **Protocol** (`ProtocolType`): `PROTO_CIV`, `PROTO_KENWOOD_ASCII`,
  `PROTO_ELECRAFT_ASCII`, `PROTO_YAESU_FTDX_ASCII`, `PROTO_YAESU_FT8X7`.
- **Capabilities** (`RadioCapabilities`, `sp.caps.getX` / `setX`): what this
  profile may do. Set from the ini's `[capabilities]` section.
- **`live`** (`LiveState`): what we last knew about the radio, with a
  `xValid` flag for each value.
- **Bank**: one page of keypad functions, from 1 to 9. The same key does
  different things in different banks.

---

## 2. Build, run, test

### Firmware

The CI settings in `.github/workflows/firmware.yml` are the reference:

```sh
arduino-cli config init --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.11
arduino-cli lib install Keypad

# arduino-cli needs the sketch folder to be named like the .ino, so stage a copy:
INO=TalkingRemoteControllerLX1WJ_V3_5_8
mkdir -p build/$INO && cp firmware/*.ino firmware/*.cpp firmware/*.h firmware/partitions.csv build/$INO/
arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:FlashSize=16M,FlashMode=dio,CDCOnBoot=cdc,USBMode=hwcdc,PartitionScheme=custom \
  build/$INO
```

- **All sources are flat in `firmware/`.** Arduino does not compile
  subfolders, so a new `.cpp` goes next to the others and is picked up
  automatically.
- **Use the full FQBN.** Without `CDCOnBoot=cdc` there is no USB console,
  and without `FlashSize=16M` the 8 MB app partition does not boot.
- **In the Arduino IDE**, use the settings in
  `docs/Arduino IDE Tools Settings.txt`.

### Host tests (no hardware)

The keypad logic and the FT-8x7 frame and EEPROM decoding are plain C++ and
are tested with g++:

```sh
make -C tests/keypad               # build and run everything
make -C tests/keypad FILTER=hold   # only tests whose name contains "hold"
make -C tests/ft8x7                # FT-817/857/897 CAT fields and EEPROM map
```

A source under test must not include `Arduino.h`; its suite's `Makefile`
lists it in `FIRMWARE_SRCS`. The runner is shared in `tests/common`.

CI runs these on every push. On Windows, use any g++ with `make`, such as
MSYS2 or WSL.

### The serial console is your debugger

Open the USB port at 115200 baud. Every key prints a trace, so a key press
can be read without listening to it:

```
CMD BANK2 3 LONG -> NOTCH
NOTCH ON
```

Useful commands:

| Command | What it shows |
|---|---|
| `HELP` | Every command, for the active protocol |
| `PROFILE?`, `SLOTS?` | Active profile details (source, protocol, variant, capabilities), all slots |
| `MODE LIST` | Modes the keypad mode select accepts on this profile |
| `STATUS?` | A status summary of the radio and device |
| `VOICE <name>`, `LISTVOICES`, `TEST` | Play one clip, list all clips, play every clip |
| `BANK <n>` | Switch the keypad bank from the console |

Any console command can be typed exactly as a key sends it, so keypad
behaviour can be reproduced without touching the keypad. `DBG_PRINT` /
`DBG_PRINTLN` (`debug_log.h`) are compiled out by default; set
`ENABLE_DEBUG_LOG` to true for extra monitor logging.

### Radio simulator

`firmware/tools/ftdx10_simulator.py` is an FTDX-10 CAT simulator (Tkinter plus
`pyserial`).

1. Connect a USB-UART adapter (TTL level) to the radio port. `ftdx10.ini`
   uses UART2, pins 9/10.
2. Start it: `pip install pyserial`, then `python firmware/tools/ftdx10_simulator.py`.
3. Pick the COM port at 38400 baud and select the FTDX-10 slot on HamTRC.

You can edit the simulated radio state (VFOs, mode, split, NR/NB/notch,
meters) in the GUI. Custom exact or prefix replies let you fake errors or
commands that are not implemented yet. For another ASCII radio, copy it and
change the handler table.

---

## 3. The keypad state machine

### What the user does

| Key | Short | Long (700 ms) |
|---|---|---|
| `*` | Say the bank | Bank select: "bank please", then one digit; a digit while `*` is still held picks the bank of the next key only |
| `0`–`9`, `A`–`C` | Bank function | Bank function (hold) |
| `D` | Enter: commit an entry or selection | – |
| `#` | Clear: cancel whatever is pending | – |

Gestures:

- A **short** press acts on release.
- A **long** press acts at the hold. The release that follows is swallowed.
- A **double** press is two presses of the same key within 220 ms. Only keys
  that wait for a double press notice it; the rest act on the first release
  straight away.
- A **double hold** is a short press followed, within the same 220 ms, by a
  hold of the same key. It is a gesture of its own: it runs the double-hold
  action, and with none assigned it beeps. It never falls back to the long
  action, and the short it follows does not run. Only keys that wait for a
  double press can have it; the rest act on the first release, so their next
  hold is a plain long press. While the second press is down on a waiting key,
  its short is held back until the key is released or the hold fires.

### The three pieces

```
 Keypad library ──► keypadEvent() ──► KeypadInput::onKey(key, gesture, ms)
                   (ui_keypad.cpp)           │
                                             │ "which action?"
                                             ├──────────► listener.keyBinding(bank, key)
                                             │                 └► keymapLookup(traits, bank, key)
                                             │
                                             │ "run it" / "entry got a digit" / "commit" / "reject"
                                             └──────────► KeypadUiListener (ui_keypad.cpp)
                                                               └► bank action / entry commit / beep
```

1. **`KeypadInput`** (`keypad_input.*`) *decides*. It tracks the mode, the
   digits typed, the pending double click and the swallowed releases. It never
   talks to the radio or the speaker; everything goes out through
   `KeypadInputListener`.
2. **The keymap** (`keypad_keymap.cpp`) answers "what does this key do on this
   radio?" by returning a `KeyBinding {short, hold, double, doubleHold, waitsForDouble}`.
3. **`KeypadUiListener`** (`ui_keypad.cpp`) *carries it out*. It interrupts
   speech on a press, builds `KeypadTraits` from the active profile, and routes
   entries to `ui_keypad_entry.cpp`.

Pieces 1 and 2 include no `Arduino.h`, which is why they can be host-tested.
Keep it that way.

### Modes

```
                ┌─────────── '#' (Clear) from any mode ─────────────┐
                ▼                                                    │
            ┌────────┐  '*' hold                ┌─────────────┐      │
            │ Normal │ ───────────────────────► │ BankSelect  │ ─ digit commits at once
            └────────┘                          └─────────────┘
                 │  action calls keypadBeginEntry / BeginModeSelect / BeginProfileSelect
                 ├──────────────────────────► ProfileSelect   (1–2 digits, D)
                 ├──────────────────────────► ModeSelect      (a mode key, D)
                 └──────────────────────────► FreqEntry, RfPowerEntry, CivAddrEntry,
                                              RptOffsetEntry, CtcssEntry, DcsEntry  (digits, D)
```

- **Normal** mode waits for the release so it can tell short, long and double
  apart.
- **Every other mode** acts on the *press*, ignores holds, and swallows the
  release.
- **A key the mode does not take** beeps and the mode stays; only `#` leaves.
- **Enter with nothing typed** beeps and stays.
- **30 s with no key event** (`kEntryTimeoutMs`) ends any mode other than
  Normal: `poll()` calls the listener's `onEntryTimeout`.

How each entry takes digits is one table row in `keypad_input.cpp`:

```cpp
constexpr EntrySpec kEntries[] = {
    // mode                     name           len  commits zero  fraction unit
    {InputMode::BankSelect,     "BANK SELECT", 1,   true,   false, 0,      ""},
    {InputMode::FreqEntry,      "FREQ",        12,  false,  true,  5,      ""},
    {InputMode::RfPowerEntry,   "RFPOWER",     3,   false,  true,  0,      " W"},
    ...
};
```

### The keymap

```cpp
KeyBinding bank2(const KeypadTraits& t, char key) {
  const bool civ = t.layout == L::Civ;
  const bool ftdx10 = t.layout == L::Ftdx10;
  switch (key) {
    case '3': return bind(queryBank2Notch, toggleBank2Notch);        // short, long
    case '4':
      if (civ) {
        return bind(queryBank2NrLevel, [] { adjustBank2NrLevel(10); },
                   [] { adjustBank2NrLevel(-10); });                 // short, long, double
      }
      if (ftdx10) return bind(SEND("GT?"), SEND("GT FAST"), SEND("GT SLOW"));
      return {};                                                      // unassigned: beep
    ...
```

- `bind(short, hold, double, doubleHold)`: pass `nullptr` for no action. A key
  with a double or double-hold action waits 220 ms before running its short
  action. A key with only a double-hold action passes `waitOnly` as its double.
- `waitOnly` as the double action means "wait anyway, but do nothing on a
  double press". Use it when a key next to a double-click key should feel the
  same.
- `SEND("CMD")` makes the key run the console command `CMD`. Most FTDX10 keys
  work this way.
- A captureless lambda carries an argument: `[] { adjustBank5Rit(-10); }`.
- `KeypadTraits` is everything the keymap may look at: `layout` (`Generic`,
  `Civ`, `Ftdx10`, `Ft8x7`, `Ft817`, `Ft857`) plus a few flags
  (`supportsMonitor`, `canGetRfPower`, …).

> **The golden rule** (`keypad_actions.h`): the keymap picks the action for
> the radio's layout, and the action does not check the layout again. An
> action still checks what the radio *can do* (capabilities, protocol
> support), and may work around protocol limits.

### Walkthrough: Bank 2, long press on `3` (notch)

1. The key is held for 700 ms. `KeypadInput::held()` looks up the binding
   `bind(queryBank2Notch, toggleBank2Notch)` and runs the hold action with the
   active key named `"BANK2 3 LONG"`.
2. The action, in `ui_keypad_bank2.cpp`:

   ```cpp
   void toggleBank2Notch() {
     printKeypadAction("NOTCH");                 // "CMD BANK2 3 LONG -> NOTCH"
     prepareKeypadSpeechResponse();              // hold polling and tuning speech
     NotchState state;
     if (keypadReportFeatureFailure(notchToggle(state), "NOTCH")) return;
     printKeypadStatus("{}", notchStateText(state).c_str());  // "NOTCH ON"
     speakNotchState(state);                     // "notch filter" "on"
   }
   ```

3. The shared operation `notchToggle()` (`radio_features.cpp`) checks
   `caps.setNotch`, refreshes `live` if it is stale, and calls
   `applyNotchAndTrack()`. That calls `setNotch()` (`radio_protocol.cpp`),
   which dispatches to `civSetNotch()` or `asciiSetNotch()`.
4. The console command `NOTCH TOGGLE` calls the same `notchToggle()`, so the
   key and the command behave the same.

### Walkthrough: Bank 1, long press on `0` (frequency entry)

1. `beginBank1FrequencySet()` calls
   `keypadBeginEntry(InputMode::FreqEntry, TargetVfo::Current)` and says
   "frequency please".
2. The user types `1 4 * 2 5`. `KeypadInput` checks each key against the
   `FREQ` row (up to 12 characters, 5 digits after the `*` point). For each
   accepted key the listener calls `keypadEntryDigit()`, which prints
   `FREQ STAGE: 14*25` and speaks the digit.
3. `D` commits. `KeypadInput` returns to Normal and calls
   `onCommit(FreqEntry, "14*25", Current)`. `keypadEntryCommit()` then calls
   `commitFrequency()`, which parses the digits with
   `RadioFrequency::parseEntry`, writes with `keypadApplyFrequencyHz`, and
   speaks the result.

---

## 4. Recipes

The recipes share one running example: the **preamp**. Today it exists only
as ASCII console commands (`PA?`, `PA ON`), so turning it into a real
cross-radio feature with a key exercises every layer.

### 4.1 Add a key action

Suppose Bank 4 key `4` should be "preamp?" on a short press and "toggle
preamp" on a long press, on CI-V radios.

**1. Declare** the actions in `keypad_actions.h`, under the bank. Use plain
types only.

```cpp
// ---- Bank 4 ----
void queryBank4Preamp();
void toggleBank4Preamp();
```

**2. Implement** them in `ui_keypad_bank4.cpp`. Every action has the same
shape:

```cpp
void toggleBank4Preamp() {
  printKeypadAction("PREAMP");                  // trace first, always
  prepareKeypadSpeechResponse();
  bool on = false;
  if (keypadReportFeatureFailure(preampToggle(on), "PREAMP")) return;  // beep / "timeout" / "error"
  printKeypadStatus("PREAMP {}", on ? "ON" : "OFF");
  speakTokenState("preamp", on);                // needs a "preamp" clip, see 4.7
}
```

**3. Bind** the actions in `keypad_keymap.cpp`:

```cpp
    case '4':
      if (t.layout == L::Civ) return bind(queryBank4Preamp, toggleBank4Preamp);
      return {};
```

**4. Update the tests.** The keymap is checked against a table of what every
key does for every radio family.

- `tests/keypad/test_keymap.cpp`: add a recording stub for each new action.

  ```cpp
  void queryBank4Preamp() { record("queryBank4Preamp"); }
  void toggleBank4Preamp() { record("toggleBank4Preamp"); }
  ```

- `tests/keypad/keymap_expectations.inc`: add rows that together cover all
  families. This is enforced: a key assigned for one family needs an explicit
  row for every family.

  ```
  KEYMAP_ROW(CIV,      4, '4', "queryBank4Preamp", "toggleBank4Preamp", "-")
  KEYMAP_ROW(NOT(CIV), 4, '4', "unassigned",       "none",              "-")
  ```

  The columns are short, long, double. The sentinels are `unassigned` (short
  press beeps), `none` (no hold action), `-` (does not wait for a double
  press), and `none` in the double column (waits, but has no double action).

**5. Run** `make -C tests/keypad`, then build the firmware.

**6. Document** the key in the radio's page under `docs/radios/`, and add a
user-facing line to `CHANGELOG.md`.

### 4.2 Make a key behave differently on one radio

Branch in the keymap, never in the action:

```cpp
case '2':
  if (t.layout == L::Ft857) return bind(queryBank1Ft857TxFrequency, nullptr, waitOnly);
  return bind(queryBank1TxFrequency, nullptr, waitOnly);
```

- **Whole radio family behaves differently:** add a `KeypadLayout` value.
- **One capability differs:** add a `bool` to `KeypadTraits` (see
  `canGetRfPower`).
- **Either way, fill it in two places.** `currentKeypadTraits()` in
  `ui_keypad.cpp` sets it on the device. `traitsFor()` in
  `tests/keypad/test_keymap.cpp` sets it in the tests; add a new family to
  `keymap_expectations.h` if one is needed.
- **Naming:** a radio-specific action carries the radio in its name, e.g.
  `queryBank1Ft857TxFrequency`.

### 4.3 Add a numeric entry

Example: typing a CW keyer speed.

1. Add `InputMode::KeyerSpeedEntry` to the enum in `keypad_input.h`.
2. Add a row to `kEntries` in `keypad_input.cpp`:

   ```cpp
   {InputMode::KeyerSpeedEntry, "KEYSPEED", 2, false, false, 0, " WPM"},
   ```

   The fields, in order:

   | Field | Meaning |
   |---|---|
   | `name` | Label used in beeps and the trace |
   | `maxLen` | Maximum characters, the point included |
   | `commitsWhenFull` | The last digit commits without Enter |
   | `leadingZero` | `0` may be the first digit |
   | `maxFraction` | Digits allowed after the `*` point; 0 means no point |
   | `unit` | Shown after the digits while typing |

3. In `ui_keypad_entry.cpp`, write `commitKeyerSpeed(const char* digits)` and
   add its `case` to `keypadEntryCommit()`.
4. Write the action that starts the entry:

   ```cpp
   void beginBank4KeyerSpeedEntry() {
     printKeypadAction("KEYSPEED");
     keypadBeginEntry(InputMode::KeyerSpeedEntry);
     // "speed" needs a clip first (4.7)
     if (g_speechEnabled) { speakToken("speed"); playSilenceMs(80); speakToken("please"); }
   }
   ```

5. Bind the action (recipe 4.1). Add tests to
   `tests/keypad/test_keypad_input.cpp` covering the digits the entry refuses.
   The existing `input_*` tests show the `tap()`, `hold()` and `typeKeys()`
   helpers.

### 4.4 Add a radio feature, layer by layer

Work bottom-up. NB (noise blanker) is the smallest complete example to copy;
every step below names its NB counterpart.

**1. Capability.** The NB counterpart is `getNb` / `setNb`.

- `radio_types.h`: add `getPreamp` and `setPreamp` to `RadioCapabilities`.
- `profile_loader.cpp`: set them to false in the defaults, and to true for
  the built-in full-feature CI-V profile if it applies.
- `sd_profile_parser.cpp`: parse the ini keys.

  ```cpp
  else if (key == "get_preamp") sp.caps.getPreamp = parseIniBool(val, sp.caps.getPreamp);
  ```

- `radio_profile.cpp`: add the new caps to the capability print, so
  `PROFILE?` shows them.

**2. Command strings (ASCII protocols only).** The NB counterparts are
`nbGet`, `nbOnCmd` and `nb_get`. Preamp already has these:
`sp.ascii.preampGet`, `preampOnCmd`, `preampOffCmd` and
`preampReplyPrefix`, with ini keys `preamp_get`, `preamp_on`, `preamp_off`
and `preamp_prefix`. For a new command, add the fields to
`AsciiCommandProfile`, give them defaults in `profile_loader.cpp`, and add
`[commands]` / `[responses]` keys in the parser.

**3. Protocol ops.** Each op takes the profile and checks the capability
first. The CI-V ops are hard-coded bytes; the ASCII ops are driven by the
profile's strings.

```cpp
// protocol_ops_civ.cpp — 0x16 0x02 is the CI-V preamp
bool civQueryPreamp(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getPreamp) return false;
  return civQueryToggleSub(0x02, onOut, timeoutMs);
}

// protocol_ops_ascii.cpp — asciiQueryPreamp exists; add the caps check like asciiQueryNb
bool asciiQueryPreamp(const StoredProfile& sp, bool& onOut, uint32_t timeoutMs) {
  if (!sp.caps.getPreamp || !sp.ascii.preampGet[0] || !sp.ascii.preampReplyPrefix[0]) return false;
  ...
```

Write ops that verify: CI-V set functions read the value back
(`civSetNb`). FT-8x7 CAT writes are write-only, so never read back there.

The FT-8x7 CAT cannot read most settings, but the radio's EEPROM has them.
An on/off setting is an `Ft8x7Flag` with one row per model in
`ft8x7_eeprom_map.cpp`, read with `yaesuFt8x7QueryFlag`; a model without
the row reports it unsupported. Add a test row in `tests/ft8x7` with the
address and bit you measured on the radio.

**4. Façade.** Add one dispatch per protocol in `radio_protocol.cpp`, and
declare it in the header:

```cpp
bool setPreamp(bool on) {
  ProtocolType pt = currentProtocolType();
  const StoredProfile& sp = currentStoredProfile();
  if (pt == PROTO_CIV) return civSetPreamp(sp, on);
  if (pt == PROTO_KENWOOD_ASCII || pt == PROTO_ELECRAFT_ASCII || pt == PROTO_YAESU_FTDX_ASCII) return asciiSetPreamp(sp, on);
  return false;
}
```

If only some protocols can ever do it, also add `protocolSupportsPreamp()`
next to `protocolSupportsTuner()`, and keep it in sync with the dispatch.

**5. Tracked state (optional).** The NB counterparts are `nbOn`, `nbValid`,
`rememberLiveNb`, `refreshLiveNb` and `applyNbAndTrack`. Do this when a toggle
should start from what we know rather than reading the radio every time.

- `LiveState`: add `preampOn` and `preampValid`.
- `radio_state.cpp`: `rememberLivePreamp()`, and a reset in
  `resetLiveRadioState()`.
- `radio_runtime.cpp`: `refreshLivePreamp()` and `applyPreampAndTrack()`.

**6. Shared operation.** Put it in `radio_features.*` so the key and the
console cannot drift apart:

```cpp
FeatureStatus preampToggle(bool& on) {
  if (!currentStoredProfile().caps.setPreamp) return FeatureStatus::Unsupported;
  if (!live.preampValid && !refreshLivePreamp()) return failure(FeatureStatus::NoReply);
  const bool next = !live.preampOn;
  if (!applyPreampAndTrack(next)) return failure(FeatureStatus::Failed);
  on = next;
  return FeatureStatus::Ok;
}
```

`failure()` turns a result into `Timeout` when the radio said nothing
(`g_radioReplyTimedOut`). Put the text and speech (`preampStateText`,
`speakPreampState`) in `ui_features.*`.

**7. Console.** In `ui_console.cpp`, add `PREAMP?`, `PREAMP ON`, `PREAMP OFF`
and `PREAMP TOGGLE` next to `handleConsoleFeatureCommand()`, calling the same
functions, and list them in `printHelp()`. Existing console commands (here
the FTDX `PA?`) should move onto the shared operation rather than keep a
second implementation.

**8. Key.** Follow recipe 4.1.

**9. Profiles and docs.**

- Add `get_preamp=1` / `set_preamp=1` to each ini whose radio has a preamp.
- Update `docs/radio-support-matrix.md` and the radio pages.
- Add a user-facing entry to the `CHANGELOG.md`.

### 4.5 Add a radio that speaks an existing protocol

This needs only an ini file, no code.

**1. Copy the closest profile** in `firmware/SDCard/`:

| Family | Template |
|---|---|
| Icom CI-V | `ic7300.ini` |
| Kenwood | `ts480.ini` |
| Elecraft | `kx2.ini` |
| Yaesu new ASCII | `ftdx10.ini` |
| Yaesu FT-817/857/897 | `ft817.ini` / `ft857.ini` |

**2. Edit it.** Mind the order:

```ini
[profile]
name=Icom IC-9700          ; up to 31 chars, shown and logged
voice_vendor=icom          ; spoken: xiegu, icom, kenwood, yaesu, elecraft
voice_digits=9700          ; spoken digit by digit
;variant=ic7760            ; only if an existing quirk path applies (see below)

[connection]
civ_addr=0xA2
baud=19200
uart_num=1
rx_pin=18
tx_pin=17
tx_invert=1
rx_invert=0

[protocol]                 ; MUST come before the sections below
type=CIV                   ; CIV | KENWOOD_ASCII | ELECRAFT_ASCII | YAESU_FTDX_ASCII | YAESU_FT8X7

[capabilities]             ; list only what the radio really does
get_freq=1
set_freq=1
...

[modes]                    ; ASCII / FT-8x7 only: the COMPLETE list of the radio's modes
lsb=1
usb=2
```

Three traps:

- **`type=` resets the profile to that protocol's defaults.** Any
  capabilities, commands or modes above `[protocol]` are lost.
- **`[modes]` is the full list.** A mode you leave out is gone, and it
  disappears from the keypad mode select and from `MODE LIST`. Without a
  `[modes]` section, the protocol defaults apply.
- **Unknown keys are silently ignored.** A typo means the feature is off.
  Check the result with `PROFILE?`.

**3. Register the slot** in `SDCard/slots.ini` using a free number, e.g.
`23=ic9700.ini`. The comments in `slots.ini` say which ranges are reserved
for which family.

**4. Verify it on the console.** Run `SLOTS?`, then `PROFILE?` (it shows
protocol, variant and capabilities), then `MODE LIST`, `FREQ?`, `MODE?`, and
one feature such as `NR?`. Then try every bank on the keypad.

**5. Document it.**

- Add a row to `docs/radio-support-matrix.md`.
- Add a page under `docs/radios/`, following `icom-ic-7300.md`.
- Say honestly what was tested on hardware.

**About `variant=`.** It opts into code paths for one radio:

- `ft817`, `ft818` and `ft857_897` select the FT-8x7 model (`ft8x7_model.h`):
  its EEPROM map, VFO tracking and keypad layout. Code asks
  `currentFt8x7Model()`, `currentIsFt817Family()` or `currentIsFt857Family()`,
  never the string.
- `ic7760` selects the CI-V main/sub handling.

A new variant name does nothing until code checks it. Prefer a variant or a
capability to matching the radio's `name`. The TS-480 name check in `radio_features.cpp` is legacy.

### 4.6 Add a new protocol

This is needed only when no existing protocol can talk to the radio. Work
through this checklist:

1. **Protocol type.** In `radio_types.h`, add `PROTO_X` to `ProtocolType`. In
   `radio_profile.cpp`, add it to `protocolTypeToString`.
2. **Parser.** In `sd_profile_parser.cpp`, map `type=X` to it.
3. **Defaults.** In `profile_loader.cpp`, add a branch to
   `setProtocolDefaults` with the capabilities every radio of this protocol
   has.
4. **Framing.** Add `packet_x.*` and `protocol_x.*` on top of
   `transport_serial.h`.
   - Set `g_radioReplyTimedOut` when the radio stays silent; the UI's
     "timeout" depends on it.
   - Flush stale input before each request.
5. **Ops.** Add `protocol_ops_x.*`. Each function has the form
   `xQueryFoo(const StoredProfile& sp, …)` and checks `sp.caps` first.
6. **Dispatch.** In `radio_protocol.cpp`, add a `PROTO_X` line to every
   function it supports, plus `canSetMode`, `sMeterFromRaw` and the
   `protocolSupports*` helpers.
7. **Serial.** In `serialTransportApplyProfile`
   (`transport_serial.cpp`), set the frame format (8N1/8N2) and idle level.
   Put any line-open handshake in `applyProfile` (`radio_profile.cpp`); the
   Elecraft `AI0;` is an example.
8. **Polling.** In `radio_monitor.cpp`, add a `freqPollPolicyFor()` case if
   the default interval or timeout does not suit the radio. If the radio sends
   updates on its own, add a pump like `engine_civ.cpp`.
9. **Keypad.** The radio gets the `Generic` layout. Add a `KeypadLayout` only
   if it needs its own keys (recipe 4.2).
10. **Simulator.** Before touching real hardware, write one, starting from
    `ftdx10_simulator.py`.

### 4.7 Add a spoken word

Clips are looked up by **name** at run time. There is no enum.

1. Add the phrase to `firmware/voice_assets/piper/voice_phrases.txt`:

   ```
   preamp | | pre amp
   ```

   The format is `phrase | symbol | say`. The symbol defaults to the phrase
   without spaces, and `say` fixes pronunciation.

2. Generate the clip and merge it into the header. The Piper setup is in its
   README.

   ```powershell
   cd firmware\voice_assets\piper
   .\.venv\Scripts\python generate_voices.py --only preamp --header
   ```

   This writes `voice_clips/voice_preamp.wav` and appends `voice_preamp[]` to
   `firmware/voice_data.h`. The `HAS_VOICE_voice_preamp` line that comes with
   it only marks the block for the merge script; firmware doesn't check it.

3. Register it in `kVoiceClips[]` in `ui_speech.cpp`, in alphabetical
   position:

   ```cpp
     VOICE_CLIP(preamp),
   ```

   Every clip in the table is required, so the build fails if the clip is
   missing from `voice_data.h`. Words spoken as a sequence of existing clips,
   such as `"pa"` (p, a), go in `kVoiceAliases[]` instead; they need no new
   clip.

4. Speak it: `speakToken("preamp")` or `speakTokenState("preamp", on)`.
   Check it by ear with `VOICE preamp`. An unknown token plays the error
   sound, so a typo is audible.

5. Commit the phrase, the `.wav`, `voice_data.h` and `ui_speech.cpp` together.

Use tokens (`speakToken`) rather than the raw arrays (`voice_x`,
`voice_x_len`) in new code; naming a clip's array in a new file can add
another copy of it to flash. Keep to the speech style in
`firmware/voice_assets_required_current_software.txt`: digits one by one,
abbreviations spelled out, a function word then on/off.

---

## 5. Best practices

**The user cannot see the device.** Every key must give audible feedback,
and the same situation must always sound the same:

| Situation | Feedback | Helper |
|---|---|---|
| Key has no action here, or the entry does not take the key | beep | `keypadReportUnassigned` (the state machine calls it) |
| The profile lacks the feature | beep | `keypadReportIfUnsupported`, `FeatureStatus::Unsupported` |
| The radio did not answer | "timeout" | `keypadReportIfTimedOut` |
| The radio or protocol cannot do it at all | "not available" | `speakNotAvailable` |
| Invalid value, or a write was rejected | "error" | `speakError` |
| `#` cancelled something | "cancel" | the listener's `onClear` |
| An entry or selection waited 30 s for a key | "timeout" | the listener's `onEntryTimeout` |
| Asking for input | "<thing> please" | e.g. "frequency please" |

- **Build the answer by appending speech.** Only a new key press interrupts
  speech (`silenceSpeechForKeyPress`), so never call `audioAbortNow()` from an
  action.
- **Guard speech with `g_speechEnabled`.** Speech can be turned off; the
  serial trace must still say everything.
- **Say a value's name with `speakLabel`** (it adds the gap) and the "ok"
  after a changed value with `speakValueOk`. Verbose off (Bank 9 `5`) drops
  both, so the user hears only the value. `speakTokenState` and
  `speakTokenPercent` already do this.
- **Trace first.** Start every action with `printKeypadAction("WHAT")`.
  Testers and blind users debug from that line.
- **Format traces with `{}`, not `String`.** The `printKeypad*` helpers take a
  literal with `{}` placeholders and its arguments, e.g.
  `printKeypadStatus("VFO{}: {} MHz", which, RadioFrequency::fromHz(hz))`.
  They format on the stack, so a trace never touches the heap. Arguments are
  integers, `char`, `const char*` and `RadioFrequency`; anything else, or a
  wrong `{}` count, does not compile. Text that is not a literal goes through
  `"{}"`, and a `String` passes `.c_str()`.

**Blocking and the radio bus.**

- An action blocks `loop()` while it waits for the radio. Pass explicit
  timeouts (800 ms is the convention) and never loop without a bound.
- Call `prepareKeypadSpeechResponse()` (or `prepareKeypadRadioWrite()` when
  the key writes to the radio) so background polling does not talk over your
  exchange.
- Read before you toggle: use the tracked `live` value when it is valid,
  otherwise query. After a write, read back where the protocol allows it.

**Capabilities, not names.**

- Check `sp.caps` in the ops and in `radio_features`.
- The keymap alone decides the layout.
- New radio-specific behaviour needs a capability, a `variant`, or a
  `KeypadTraits` flag, never `name.indexOf(...)`.

**Keep the pure core pure.** `keypad_input.*` and `keypad_keymap.*` must not
include `Arduino.h` or `String`. Every change to the state machine or keymap
comes with tests; CI fails the build otherwise.

**Share, don't duplicate.** If a key and a console command do the same
thing, both call one function in `radio_features`, which returns a
`FeatureStatus`. See NR, NB and notch.

**Code style.** Match the file you are in.

- **Headers:** `#pragma once`. New code uses `enum class X : uint8_t`.
- **Case:** types in `PascalCase`, functions in `camelCase`.
- **Prefixes and suffixes:** globals `g_`, file statics `s_`, constants and
  tables `k` (pins and macros are `UPPER_SNAKE`), private members end in
  `_`.
- **File-local code:** an anonymous `namespace {}` in new files, `static` in
  older ones.
- **Formatting:** 2-space indent, braces on the same line.
- **Action names:** `<verb>Bank<N><Feature>`, where the verb is `query`,
  `toggle`, `set`, `adjust`, `cycle`, `begin…Entry`, `select` or `report…`.
- **Comments:** plain present-tense sentences about what something does or
  means, often quoting the serial trace (`// "RIT OFF", RIT off or "RIT ->
  unsupported"`). No Doxygen, and no comments that only repeat the code.

**Commits and changelog.**

- Branch from `development`, and keep one topic per branch.
- Commit subjects are imperative, in sentence case, with no prefix or
  trailing period, and describe behaviour: *"Keep only the modes a profile
  lists"*. The body is short prose explaining why.
- `CHANGELOG.md` is for people using the radio. Group entries by topic
  (Keypad, Speech, Radios, …) and describe what they will notice, quoting keys
  in backticks and spoken words in quotes. Internals go under "For
  developers", if anywhere.
- Label experimental features as experimental (see the documentation rule in
  `builder-guide.md`).

**Flash is finite.** `voice_data.h` is several megabytes. Reuse clips and
aliases before generating new ones.

**Known rough edges** (improve them when you're nearby, but don't copy the
pattern):

- Most console commands still have their own implementation beside the keypad
  one. Only NR, NB and notch are shared so far.
- `isFtdx10KeypadProfile()` recognises the FTDX10 by its `voice_vendor` /
  `voice_digits`, not by a variant.
- `radio_api.*` is an empty umbrella header; include the specific `radio_*.h`
  instead.
- The header of `keymap_expectations.inc` refers to function names from
  before the keypad refactor. It explains where the matrix came from.

---

## 6. Reference cards

### Where does it go?

| I want to… | Touch |
|---|---|
| Change what a key does | `keypad_keymap.cpp`, then the tests |
| Change how an action behaves | `ui_keypad_bankN.cpp` |
| Change what Enter does for an entry | `ui_keypad_entry.cpp` |
| Change gestures, timing, Enter/Clear rules | `keypad_input.cpp`, then `test_keypad_input.cpp` |
| Add a console command | `ui_console.cpp` (plus `printHelp`) |
| Share an operation between key and console | `radio_features.*` + `ui_features.*` |
| Talk to the radio in a new way | `protocol_ops_*` + `radio_protocol.*` |
| Add a capability flag | `radio_types.h`, `profile_loader.cpp`, `sd_profile_parser.cpp`, `radio_profile.cpp` |
| Support a new radio | `firmware/SDCard/*.ini` + `slots.ini` |
| Add a spoken word | `voice_phrases.txt` → `generate_voices.py` → `kVoiceClips[]` |
| Change pins or timing constants | `config_pins.h`, `radio_types.h` |

### `FeatureStatus` → feedback

| Status | Keypad (`keypadReportFeatureFailure`) | Console |
|---|---|---|
| `Ok` | caller prints and speaks the state | same |
| `Unsupported` | beep, `"X -> unsupported"` | "not available" |
| `Timeout` | "timeout" | "timeout" |
| `NoReply`, `Failed` | "error", `"X -> no reply"` / `"X -> failed"` | printed only, no sound |

### Ini sections

| Section | Holds | Notes |
|---|---|---|
| `[profile]` | `name`, `voice_vendor`, `voice_digits`, `variant` | Survives `type=` |
| `[connection]` | `civ_addr`, `baud`, `uart_num`, `rx_pin`, `tx_pin`, `tx_invert`, `rx_invert` | Survives `type=` |
| `[protocol]` | `type` | Resets the sections below to the protocol defaults |
| `[capabilities]` | `get_*`, `set_*`, `start_tune`, `rf_power_max_watts` | Booleans accept `1/0`, `yes/no`, `on/off` |
| `[commands]` | ASCII command strings, e.g. `nb_on=NB01;`; set formats use `%llu` / `%s` | ASCII protocols only |
| `[responses]` | Reply prefixes, e.g. `nb_prefix=NB0` | ASCII protocols only |
| `[modes]` | `lsb usb am cw rtty fm cwr rttyr digi` → the radio's code | The complete list; FT-8x7 codes are hex |
| `[bank6]` | `rpt_offset_1`, `rpt_offset_2`, `ctcss_default`, `dcs_default` | FT-8x7 repeater keys |

The authoritative key list is `loadSingleProfileIni` in `sd_profile_parser.cpp`.
