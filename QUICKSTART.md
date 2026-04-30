# Talking Remote Controller LX1WJ QuickStart

This is the shortest technical path for the current firmware line **V3.5.5**.

## Basic Setup

1. Open `firmware/TalkingRemoteControllerLX1WJ_V3_5_5.ino` in Arduino IDE.
2. Build for the intended ESP32-S3 target.
3. Keep `voice_data.h` in the firmware folder.
4. Copy the contents of `firmware/SDCard` to the controller SD card.
5. Insert the SD card before normal operation.

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