# Tuning Guide

How to tune the robot for a new track or different hardware.

## Quick Start

1. Connect via WiFi, verify with `$PING`
2. Place robot on track, send `$START`
3. Observe behavior, send `$STOP` to halt
4. Adjust parameters with `$SET:KEY=VAL,...`
5. Test again, repeat until satisfied
6. Save with `$SAVE`

## PID Speed Tuning

Parameters: `KP`, `KI`, `KD`

| Symptom | Fix |
|---------|-----|
| Slow to reach target speed | Increase KP |
| Oscillating around target | Decrease KP, increase KD |
| Steady-state speed error | Increase KI |
| Motor buzzing / jerky | Decrease KI, increase KD |

### PID tune (on-track, recommended for drive mode)

`$TEST:pidtune` runs open-loop ESC steps on the **track** (reactive steering on),
fits a simple FOPDT model per step, then prints **IMC** and **PI** gains over WiFi.
Use **Apply PI** in the web UI for a calmer low-speed loop, or **IMC** if you want
derivative from the tuner. Always verify on the ground and **`$SAVE`** when happy.

Defaults in flash were seeded from a real pidtune log; **re-run after ESC, wheel,
or gearing changes**.

### Auto-tune (bench)

Run `$TEST:autotune` with the robot elevated (wheels off ground).
This performs a Ziegler–Nichols-style step test and suggests gains (different
method than pidtune).

### Manual procedure

1. Set KI=0, KD=0
2. Increase KP until speed oscillates steadily (note this as Ku)
3. Measure oscillation period Tu
4. Set: KP = 0.6×Ku, KI = 1.2×Ku/Tu, KD = 0.075×Ku×Tu

## Steering Tuning

### Servo Limits (MNP, NTP, XNP)

If the robot turns more to one side:
1. Send `$SRV:90` — wheels should point straight
2. If not, adjust NTP until they do
3. Set MNP/XNP to the mechanical limits (don't force beyond stops)

Use `$TEST:servo` to sweep the full range and verify.

Set `SVR=1` if the servo is mounted in reverse orientation.

### Steering Aggressiveness (COE1, COE2)

- `COE1`: coefficient when path is clear — lower = smoother straights
- `COE2`: coefficient when blocked — higher = sharper turns

Defaults (~**0.28** / **0.65**) match low-speed cruise; increase COE2 if the car
misses apexes, decrease COE1 if it weaves.

## Speed Tuning

### Target Speeds (SPD1, SPD2)

- `SPD1`: target speed when the path ahead is **clear** (m/s)
- `SPD2`: target when **front obstacle** logic selects “slow” (m/s)

Factory defaults are conservative (~**0.48** / **0.32** m/s) for bring-up; raise
gradually for faster laps once PID and steering are stable.

### Setpoint slew (SLW)

`SLW` limits how fast the **PID speed setpoint** can move (m/s per second).
**0** = instant step (old behavior). A value around **0.8–1.2** softens starts
and SPD1↔SPD2 transitions. Feedforward follows the slewed setpoint.

### Start kick (KOP, KOM)

Short **extra ESC pulse** when commanding forward from rest:

- `KOP`: percent of **(XSP − MSP)** span added as µs (capped in firmware)
- `KOM`: duration in **ms**; ends early when measured speed reaches ~78% of setpoint
- `KOP = 0` disables kick

Use a small kick if the car is lazy off the line; reduce if it lurches.

### ESC Limits (MSP, XSP, BSP)

- `MSP`: minimum forward pulse (µs) — clears motor dead zone with feedforward
- `XSP`: maximum forward pulse (µs) — top speed cap for PID
- `BSP`: reverse pulse (µs)

If the motor doesn't start at low speeds, **increase MSP** slightly. If top speed
is too high, **decrease XSP**. Kick strength scales with **(XSP − MSP)**.

## Obstacle Detection

### Distance Thresholds (FOD, SOD, ACD, CFD)

| Parameter | Purpose | Increase if... | Decrease if... |
|-----------|---------|----------------|----------------|
| FOD | Front obstacle — triggers slowdown | Braking too late | Braking too early |
| SOD | Side open — wall-follow bias | Missing wide openings | False open detection |
| ACD | All-close — emergency zone | — | Too cautious |
| CFD | Close front — stuck detection | Not detecting walls | False stuck triggers |

Default numeric values are in `settings.h` (`DEFAULT_FOD`, etc.); compare readings
with `$SNS` when tuning.

## Race Direction

Set `RCW=1` for clockwise, `RCW=0` for counter-clockwise.
This affects wrong-direction detection (`WDD` threshold).

## Stuck Recovery

- `STK` (default 25): iterations before escape triggers (25 × 40 ms = 1 s)
- Decrease for faster escape reaction, increase to avoid false triggers

## Track Learning

For optimal lap times on a known track:

1. `$TRK:CLEAR` — erase previous data
2. `$TRK:START` then `$START` — drive one learning lap at moderate speed
3. `$TRK:STOP` then `$STOP` — stop recording
4. `$TRK:STATUS` — verify recorded points
5. `$TRK:RACE` then `$START` — race using learned profile

The learned profile stores speed and steering at each 10 cm segment,
allowing anticipatory braking before turns.

## Diagnostics

| Command | When to use |
|---------|-------------|
| `$DIAG` | Overall health check |
| `$SNS` | Verify sensor readings make sense |
| `$IMU` | Check gyro drift after calibration |
| `$PID` | Monitor PID output during driving |
| `$BAT` | Check battery before a run |
| `$LOG:ON` | Enable real-time debug messages |

## Battery

- `BEN`: battery monitoring **on/off** (default **off** in NVS defaults — enable for packs)
- `BML`: voltage divider multiplier — **calibrate with a multimeter** on your divider
- `BLV`: low-voltage cutoff (V) — sustained low voltage triggers stop after ~10 s

In the web UI, battery fields are under **Settings → ⚡ Battery** (after **Read** / `$GET`).
