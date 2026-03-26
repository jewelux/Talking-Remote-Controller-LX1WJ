# Hardware Wiring Reference (ESP32 Talking Remote Controller - V3.5)

This document describes the hardware wiring used by the current modular repository state based on
`firmware/TalkingRemoteControllerLX1WJ_V3_5.ino`.

All pin assignments listed below are taken from the current firmware files.

---

## 1. ESP32-S3 to 4x4 Matrix Keypad

### Pin Assignment

- Rows: `KP_ROW_PINS = {4, 8, 15, 16}`
- Columns: `KP_COL_PINS = {1, 2, 13, 21}`

### Wiring

| Keypad Signal | ESP32-S3 GPIO |
|---|---|---|
|1| ROW1 | GPIO 4 |
|2| ROW2 | GPIO 8 |
|3| 3=ROW3 | GPIO 15 |
|4| ROW4 | GPIO 16 |
|5| COL1 | GPIO 1 |
|6| COL2 | GPIO 2 |
|7| COL3 | GPIO 13 |
|8| COL4 | GPIO 21 |

Note:
GPIO 1 and GPIO 2 can be special-purpose pins on some ESP32-S3 boards.
If the keypad works reliably on your hardware, this configuration is acceptable.

---

## 2. ESP32-S3 to I2S Audio Output

Example target: MAX98357A or a comparable I2S audio module.

### Wiring

| I2S Module Pin | ESP32-S3 GPIO |
|---|---|
| BCLK | GPIO 5 |
| LRC / WS | GPIO 6 |
| DIN | GPIO 7 |
| GND | GND |
| VIN | board supply, depending on the module |
| SD / EN | optional HIGH or open, depending on the module |

---

## 3. ESP32-S3 to SD Card Module

### Wiring

| SD Signal | ESP32-S3 GPIO |
|---|---|
| GND | GND |
| MISO | GPIO 37 |
| SCK | GPIO 36 |
| MOSI | GPIO 35 |
| CS | GPIO 14 |
| VIN | board supply, typically 3.3 V depending on the module |

---

## 4. ESP32-S3 to ICOM CI-V Interface

### Firmware Pin Assignment

- `CIV_TX_PIN = GPIO17`
- `CIV_RX_PIN = GPIO18`
- Baud rate: `9600`

The ICOM CI-V interface uses a shared single-wire open-collector bus.
A transistor stage is used to interface the ESP32-S3 safely with the CI-V line.

### Transmit Stage

- GPIO17 -> 4.7 kOhm resistor -> BC548 base
- BC548 emitter -> GND
- BC548 collector -> CI-V line tip

### Receive Stage

- CI-V line tip -> 1 kOhm resistor -> GPIO18
- GPIO18 -> 3.3 V Zener diode -> GND

### Ground

- ESP32 GND <-> CI-V sleeve

Important note:
Directly connecting the CI-V line to an ESP32 GPIO pin without protection is not recommended.

---

## 5. ESP32-S3 to RS-232 Interface

### Wiring

- ESP32 TX GPIO10 -> MAX3232 TX path -> radio RX
- ESP32 RX GPIO9 <- MAX3232 RX path <- radio TX
- ESP32 GND <-> MAX3232 GND <-> radio GND
- ESP32 5V -> MAX3232 VCC

---

## 6. ESP32-S3 to TTL CAT Interface

### Firmware Pin Assignment

- `CAT_TX_PIN = 11`
- `CAT_RX_PIN = 12`
- Baud rate: `19200`

### Wiring

- ESP32 TX (GPIO 11) -> 1 kOhm resistor -> radio RX
- ESP32 RX (GPIO 12) <- 1 kOhm resistor <- radio TX
- GND <-> GND

Series resistors provide basic protection and can improve signal robustness.

---

## General Notes

- All interfaces share a common ground.
- Power is typically supplied via USB to the ESP32-S3.
- This document reflects the current V3.5 repository state.
- Future firmware versions may add profiles, interfaces, or alternative pin mappings.
- The hardware setup is still a development and test platform rather than a finished certified product.
