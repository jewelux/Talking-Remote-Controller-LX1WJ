# Talking Remote Controller LX1WJ – User Guide (English)

**Firmware:** `TalkingRemoteControllerLX1WJ_V1_0.ino`
**Board:** ESP32-S3 DevKitC-1 (N16R8)

---

## Keypad Layout (4×4 Matrix)

Assumed labeling:

   1  2  3  A

   4  5  6  B

   7  8  9  C

 \*  0  \#  D   (star, zero, hash, delta)



---

## Global Keys (Work in All Banks)

### \* (STAR)
- **Short press:** Speaks the current bank number.
- **Long press:** Switches to next bank (Bank 1 → 2 → 3 → 1) and speaks it.

### D (delta-ENTER)
- Applies staged actions (e.g., staged volume, staged mode).
- Confirms frequency entry (see below).

### \# (hash-CLEAR / CANCEL)
- Cancels any active "modal" flow (frequency entry, mode set flow, profile select).
- Clears any staged command.
- Speaks **"OK"**.

---

## Banks (Layers)

The controller uses **3 banks (layers)**. Use **\* (star-long press)** to cycle banks.

---

### Bank 1 – Live Radio Queries + Tuning Tools

**NOTE:** These keys execute **immediately** (no ENTER needed).

#### 0 (Short Press)
- Queries frequency from the radio (`FREQ?`).
- If speech is enabled: speaks **"frequency"** and then the frequency.

#### 0 (Long press) – Frequency Entry
- Starts frequency entry in **kHz**.
- **Example:** Type `14070` and press **D** → sets **14.070 MHz**.
- During entry: each digit is spoken for confirmation.
- **D** applies / **#** cancels.

#### 4 (Short Press)
- Queries power output (`PO?`).

#### 7 (Short Press)
- Queries S-meter once (`SM?`).

#### 8 (Short Press)
- Queries SWR once (`SWR?`).

#### 9 (Short Press)
- Queries mode once (`MODE?`).

#### 9 (long press) – "Mode Set Flow"
- Starts mode selection.
- After long press **9**, press a digit **1..9** to stage a mode, then press **D (ENTER)** to apply.
- **Mapping (digit → mode):**
  - `1` = LSB
  - `2` = USB
  - `3` = CW
  - `4` = AM
  - `5` = FM
  - `6` = DIGI (currently maps to USB)
  - `7` = RTTY
  - `8` = CWR
  - `9` = RTTYR
- **#** cancels.

#### A (Volume, Direct)
- **Short press:** Volume down (level -1).
- **Long press:** Volume up (level +1).
- **Levels:** 0 (very low) to 3 (loud).

---

### Bank 2 – Reserved
Currently **reserved** in this firmware (no default actions).

---

### Bank 3 – System Options / Profile Selection / Volume Staging

#### A
- **Short press:** Speaks current profile.
- **long press:** Profile select mode:
  - Press:
    - `1` = IC-7300 (CI-V)
    - `2` = Icom 706 (CI-V)
    - `3` = Icom 706 (RS232)
    - `4` = Xiegu G106 (CAT)
  - **#** cancels.

#### B
- **Short press:** Toggles **"tuning frequency announcements"** ON/OFF (so the radio/VFO changes do not always talk).
- **long press** Speaks current state (frequency announcements ON/OFF).

#### Volume Staging (Then ENTER to Apply)
- `0` → Stage volume level **0** (very low).
- `7` → Stage volume level **1** (low).
- `8` → Stage volume level **2** (medium).
- `9` → Stage volume level **3** (loud).
- **D** applies / **#** cancels.

---



