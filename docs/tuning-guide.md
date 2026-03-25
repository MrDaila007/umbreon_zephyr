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

### Auto-tune

Run `$TEST:autotune` with the robot elevated (wheels off ground).
This performs a Ziegler-Nichols step response test and suggests gains.

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

- `COE1` (default 0.3): coefficient when path is clear — lower = smoother
- `COE2` (default 0.7): coefficient when blocked — higher = sharper turns

If the robot clips walls on corners, increase COE2.
If it weaves on straights, decrease COE1.

## Speed Tuning

### Target Speeds (SPD1, SPD2)

- `SPD1` (default 2.7 m/s): speed on clear straights
- `SPD2` (default 0.8 m/s): speed when obstacles ahead

Start conservative (SPD1=1.5, SPD2=0.5), increase gradually.

### ESC Limits (MSP, XSP, BSP)

- `MSP` (default 1520 µs): minimum forward pulse (motor dead zone)
- `XSP` (default 1700 µs): maximum forward pulse (top speed cap)
- `BSP` (default 1460 µs): reverse pulse

If the motor doesn't start at low speeds, decrease MSP.
If top speed is too high, decrease XSP.

## Obstacle Detection

### Distance Thresholds (FOD, SOD, ACD, CFD)

| Parameter | Purpose | Increase if... | Decrease if... |
|-----------|---------|----------------|----------------|
| FOD (700) | Front obstacle — triggers slowdown | Braking too late | Braking too early |
| SOD (900) | Side open — wall-follow bias | Missing wide openings | False open detection |
| ACD (200) | All-close — emergency zone | — | Too cautious |
| CFD (300) | Close front — stuck detection | Not detecting walls | False stuck triggers |

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

- `BLV` (default 6.0 V): low-voltage cutoff — robot stops if sustained 10 s
- `BML` (default 2.8): voltage divider multiplier — calibrate with multimeter
- `BEN` (default 1): set to 0 to disable monitoring (bench testing)
