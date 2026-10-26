# Line out for demo units (optional add-on)

An optional 3.5 mm line output for HamTRC units used at fairs, club evenings and demos, where the
small internal speaker cannot compete with the noise in the hall. It feeds a mixer, an active
speaker or a portable speaker. It is an add-on, not part of the standard V1 or V2 board, and it
needs no firmware change.

Status: proposed circuit, not yet built and measured. Component values are starting points.

## Why not just wire a jack to the speaker terminals?

The MAX98357A is a filterless class-D amplifier with a bridged (BTL) output:

- **Neither speaker terminal is ground.** Both outputs swing in opposite directions around half
  the supply voltage (about 2.5 V DC). If the sleeve of a jack is tied to `OUT−` and the other
  device shares ground with the HamTRC (same USB supply, PC, cable shield), one half of the
  amplifier is shorted. Result: silence, crackling or heavy distortion while it goes into
  protection.
- **DC on the input.** About 2.5 V DC reaches the input of the other device.
- **Switching frequency.** Without a speaker coil the output is a PWM signal of several hundred
  kHz, which can overload a mixer input and cause noise or whistling.
- **Level and hum.** Speaker level is well above line level, and the double ground connection
  can cause a hum loop.

A battery-powered portable speaker usually works because it has no second ground connection to the
HamTRC. A mixer or mains-powered active speaker usually does not.

**Rule: never connect `OUT−` (or `OUT+`) of the MAX98357A to ground.**

## Circuit

```
 MAX98357A                                    T1 1:1               J1 3.5 mm jack
 OUT+ ──┬── R1 1k ──┬── C2 4.7µ ──┐          600 Ω : 600 Ω
        │           │             │  ║ ║                        ┌── tip
   speaker          C1 22n       pri║ ║sec ── R3 2.2k ──┬───────┤
   (via S1)         │             │  ║ ║                 │      └── ring
 OUT− ──┴── R2 1k ──┴─────────────┘  ║ ║ ───────────────┼──────────── sleeve
                                                       RV1 1k
                                                       (trimmer, wiper to R3 side)
```

The add-on is connected in parallel to the speaker, directly at the screw terminal of the
MAX98357A breakout. Nothing on the secondary side of T1 is connected to the HamTRC ground.

| Part | Function |
|---|---|
| R1, R2, C1 | Differential low-pass that removes the switching frequency. With the load of the transformer the corner is around 6 kHz; speech at 8 kHz sample rate only reaches 4 kHz. R1 and R2 also limit the current, so the amplifier can never be shorted |
| C2 | Blocks DC left over from unequal output offsets |
| T1 | Galvanic isolation: no short, no DC and no hum loop towards the other device |
| R3, RV1 | Divider from speaker level down to line level. Set RV1 on the bench |
| J1 | Tip and ring together, so both channels of a stereo input get the sound |
| S1 | Optional switch in one speaker lead to mute the internal speaker |

Rough level: at full volume the MAX98357A (5 V, default gain 9 dB) gives about 3.5 V rms across
the speaker. With the values above, about 0.7 V rms reaches the jack with RV1 at maximum, which
suits line inputs. The fine adjustment is the HamTRC volume (Bank 9 `7`).

## Bill of materials

| Ref | Qty | Part | Note |
|---|---|---|---|
| R1, R2 | 2 | 1 kΩ, 0.25 W | |
| R3 | 1 | 2.2 kΩ, 0.25 W | |
| RV1 | 1 | 1 kΩ trimmer | |
| C1 | 1 | 22 nF film | |
| C2 | 1 | 4.7 µF bipolar electrolytic or film | |
| T1 | 1 | Audio isolation transformer 1:1, 600 Ω : 600 Ω | small PCB type is enough |
| J1 | 1 | 3.5 mm stereo jack, panel mount | |
| S1 | 1 | Small toggle switch (optional) | mutes the internal speaker |

**Quick version for a demo:** a ready-made "ground loop isolator" with 3.5 mm plugs (car audio
accessory) contains T1. Put R1, R2 and C1 in front of it, and use the volume control of the
mixer or speaker instead of R3 and RV1.

## Limits

- **Line out, not a headphone output.** R1, R2 and R3 are too high for 32 Ω headphones; they
  would be very quiet. Use the headphone output of the mixer or the active speaker.
- **Internal speaker.** A switched jack only works if its switch contacts are isolated from tip and
  ring, which is rare on cheap jacks; S1 is simpler. Do not mute by pulling the MAX98357A `SD`
  pin low – that also silences the line out.
- Not yet measured. Results and corrected values from the bench are welcome in an issue.

## Possible later alternative

A PCM5102A I2S DAC in parallel to the MAX98357A on GPIO 5/6/7 would give a ground-referenced line
out. To check first: the firmware sends MSB-justified (left-justified) 16-bit mono on the left slot
at 8 kHz without MCLK, so `FMT` must be high, the left output goes to tip and ring, and it is not
known yet whether the PCM5102A PLL locks at this low bit clock. It also gives no galvanic isolation.
