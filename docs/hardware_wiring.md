# Hardware Wiring Reference (ESP32 Talking Remote Controller – V1.0)

This document describes the hardware wiring based on  
`ESP32S3_TalkingRemote_V1.0.ino`.

All pin assignments listed below are taken directly from the firmware.

---

## 1. ESP32-S3 ↔ 4×4 Matrix Keypad

### Pin Assignment
- Rows: `KP_ROW_PINS = {4, 8, 15, 16}`
- Columns: `KP_COL_PINS = {1, 2, 13, 21}`

### Wiring

| Keypad Signal | ESP32-S3 GPIO |
|--------------|---------------|
| 1 - ROW1 | GPIO 4 |
| 2 - ROW2 | GPIO 8 |
| 3 - ROW3 | GPIO 15 |
| 4 - ROW4 | GPIO 16 |
| 5 - COL1 | GPIO 1 |
| 6 - COL2 | GPIO 2 |
| 7 - COL3 | GPIO 13 |
| 8 - COL4 | GPIO 21 |

**Note:**  
GPIOs 1 and 2 are considered “special” on some ESP32-S3 boards (USB/UART/boot related).  
If the keypad operates reliably on your breadboard setup, this configuration is acceptable.

---

## 2. ESP32-S3 ↔ I2S Audio Output (e.g. MAX98357A)

### Pin Assignment (from firmware)

### Wiring (MAX98357A)

| I2S Module Pin | ESP32-S3 GPIO |
|---------------|---------------|
| BCLK | GPIO 5 |
| LRC / WS | GPIO 6 |
| DIN | GPIO 7 |
| GND | GND |
| VIN | board supply (often 3.3–5V depending on breakout; check your module) |
| SD / EN | SD/EN optional HIGH / open |

---

### SD-Module ### 

| SD-Signal	| ESP32-S3 GPIO |
|-----------|---------------|
| GND | GND |
| MISO	| GPIO37 |
| SCK | GPIO36 |
| MOSI	| GPIO35 |
| CS	| GPIO14 |
| VIN | board supply (3.3V; check your module) |

---

## 3. ESP32-S3 ↔ ICOM CI-V Interface

### Firmware Pin Assignment ###

CIV_TX_PIN = GPIO17

CIV_RX_PIN = GPIO18

Baud rate  = 9600

### CI-V Interface Wiring ###

The ICOM CI-V interface uses a single-wire open-collector bus.
A transistor stage is used to safely interface the ESP32-S3 to the CI-V line.

### Transmit stage (open collector driver) ###

GPIO17 -> 4.7kΩ -> BC548 Base

BC548 Emitter -> GND

BC548 Collector -> CI-V line (3.5 mm jack TIP)

Description:
- GPIO17 drives the base of an NPN transistor (BC548) through a 4.7 kΩ resistor
- The transistor acts as an open collector driver
- The collector connects to the CI-V data line (3.5 mm jack TIP)
- The emitter is connected to GND

### Receive stage (level protection) ###

CI-V line (TIP) -> 1kΩ -> GPIO18

GPIO18 -> 3.3 V Zener diode -> GND

Description:
- The CI-V line is connected to GPIO18 through a 1 kΩ resistor
- A 3.3 V Zener diode to ground protects the ESP32 input from higher bus voltages
- This allows safe reception of CI-V signals

### Ground connection ###

ESP32 GND <-> CI-V Sleeve

### Important Note ###

CI-V is a shared open-collector bus.
Directly connecting the CI-V line to an ESP32 GPIO pin without protection is not recommended.
The transistor driver and input protection network ensure reliable and safe operation.

### Compact wiring summary ###

GPIO17 -> 4k7 -> BC548 Base

BC548 Emitter -> GND

BC548 Collector -> CI-V Tip

CI-V Tip -> 1k -> GPIO18

GPIO18 -> 3.3V Zener -> GND

CI-V Sleeve -> GND

---

## 5. ESP32-S3 ↔ TTL-CAT Interface (Xiegu G106)

### Pin Assignment (from firmware)

- `CAT_TX_PIN = 11`
- `CAT_RX_PIN = 12`
- Baud rate: **19200**

### Wiring

- ESP32 TX (GPIO 11) → **1 kΩ resistor** → G106 RX (tip)
- ESP32 RX (GPIO 12) ← **1 kΩ resistor** ← G106 TX (ring)
- GND ↔ GND

**Note:**  
Series resistors provide basic protection and improve signal robustness.

---

## General Notes

- All interfaces share a **common ground**.
- Power is typically supplied via USB (5 V) to the ESP32-S3.
- This wiring description reflects the current firmware configuration (V1.0).
- Future firmware versions may introduce additional options or pin changes.
- The current hardware setup is implemented as a breadboard prototype for development and testing.
- Formal schematics and PCB designs (e.g. KiCad) are planned for future releases.


