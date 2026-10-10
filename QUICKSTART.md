# Talking Remote Controller LX1WJ QuickStart

This is the shortest technical path for the current firmware line **V3.5.8**.

## Basic Setup

1. Open `firmware/TalkingRemoteControllerLX1WJ_V3_5_8.ino` in Arduino IDE.
2. Build for the intended ESP32-S3 target with the ESP32 Arduino core 3.x or newer
   (Boards Manager: esp32 by Espressif Systems).
3. Flash the voice pack (the spoken words) once, and again after the clips change or after an
   upload with "Erase all flash" enabled:
   `python firmware/voice_assets/build_voice_pack.py`, then
   `esptool --chip esp32s3 -p <port> write-flash 0x810000 firmware/voice_assets/voices.bin`.
   Without it the controller says "voice pack missing" at power on.

The radio profiles are built into the firmware; no SD card is needed.

## Browser Update

The matching online update package is published as HAMTRC V3.5.8 on `lx1wj.eu`.
Current V3.5.8 firmware answers the updater probe `HAMTRC?` with an `LX1WJ-HAMTRC`
signature and the firmware version, so the checked update path can reject a wrong
COM port before flashing starts.

## Operating Model

- The keypad works in banks.
- A short press usually asks or reads.
- A long press usually changes something.
- `D` confirms.
- `#` cancels.
- New key presses interrupt speech immediately.

## If You Need More

- Practical user help: [user-guide.md](user-guide.md)
- Technical build and test notes: [builder-guide.md](builder-guide.md)
- Current radio support state: [docs/radio-support-matrix.md](docs/radio-support-matrix.md)
