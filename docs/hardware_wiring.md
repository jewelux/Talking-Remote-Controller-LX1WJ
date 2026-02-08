# Hardware Wiring Reference (ESP32 Talking Remote Controller – V1.0)

This document describes the hardware wiring based on  
`ESP32S3_TalkingRemote_V1.0.ino`.

All pin assignments listed below are taken directly from the firmware.

---

## 1. ESP32-S3 ↔ 4×4 Matrix Keypad

### Pin Assignment
- Rows: `KP_ROW_PINS = {4, 8, 15, 16}`
- Columns: `KP_COL_PINS = {1, 2, 3, 21}`

### Wiring

| Keypad Signal | ESP32-S3 GPIO |
|--------------|---------------|
| ROW1 | GPIO 4 |
| ROW2 | GPIO 8 |
| ROW3 | GPIO 15 |
| ROW4 | GPIO 16 |
| COL1 | GPIO 1 |
| COL2 | GPIO 2 |
| COL3 | GPIO 3 |
| COL4 | GPIO 21 |

**Note:**  
GPIOs 1, 2, and 3 are considered “special” on some ESP32-S3 boards (USB/UART/boot related).  
If the keypad operates reliably on your breadboard setup, this configuration is acceptable.

---

## 2. ESP32-S3 ↔ I2S Audio Output (e.g. MAX98357A)

### Pin Assignment (from firmware)

- `I2S_BCLK_PIN = 5`
- `I2S_LRCLK_PIN = 6`
- `I2S_DOUT_PIN = 7`
- `AMP_SD_PIN = -1` (no amplifier shutdown control used)

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

## 3. ESP32-S3 ↔ ICOM CI-V Interface (TTL-UART)

### Pin Assignment (from firmware)

- `CIV_TX_PIN = 17`
- `CIV_RX_PIN = 18`
- Baud rate: **9600**

### Wiring

- ESP32 TX (GPIO 17) → CI-V Data  
  *(via open-collector transistor stage)*
- ESP32 RX (GPIO 18) ← CI-V Data  
  *(via resistor divider and 3.3 V Zener diode)*
- GND ↔ GND

**Note:**  
CI-V is a single-wire, open-collector bus.  
Direct TTL connection is not recommended without proper level protection.

---

## 4. ESP32-S3 ↔ RS-232 Interface (via MAX3232)

### Pin Assignment (from firmware)

- `RS232_TX_PIN = 10`
- `RS232_RX_PIN = 9`

### Wiring

- ESP32 TX (GPIO 10) → MAX3232 **T1IN**
- ESP32 RX (GPIO 9) ← MAX3232 **R1OUT**
- MAX3232 RS-232 side:
  - **T1OUT / R1IN** → DB9 connector or radio RS-232 port
- GND ↔ GND
- VCC → **5 V**

---

## 5. ESP32-S3 ↔ TTL-CAT Interface (Xiegu G106)

### Pin Assignment (from firmware)

- `CAT_TX_PIN = 11`
- `CAT_RX_PIN = 12`
- Baud rate: **19200**

### Wiring

- ESP32 TX (GPIO 11) → **1 kΩ resistor** → G106 RX
- ESP32 RX (GPIO 12) ← **1 kΩ resistor** ← G106 TX
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


