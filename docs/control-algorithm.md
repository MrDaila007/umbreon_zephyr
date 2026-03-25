# Control Algorithm

## Overview

The robot uses a reactive wall-following algorithm with PID speed control,
stuck detection, and wrong-direction recovery. The control loop runs at 25 Hz
(40 ms period) in a dedicated Zephyr thread (priority 2, 4 KB stack).

## Control Loop

```
┌─ Sense ──────────────────────────┐
│  sensors_poll()  — 6× VL53L0X    │
│  imu_update()    — gyro Z        │
│  taho_get_speed()— encoder       │
└──────────────────────────────────┘
              ↓
┌─ Decide ─────────────────────────┐
│  Wall-follow steering            │
│  Speed selection                 │
│  Stuck / wrong-dir detection     │
└──────────────────────────────────┘
              ↓
┌─ Actuate ────────────────────────┐
│  car_write_steer()               │
│  car_write_speed_ms()            │
│  car_pid_control()               │
└──────────────────────────────────┘
              ↓
┌─ Report ─────────────────────────┐
│  Telemetry CSV over WiFi         │
│  Watchdog feed                   │
└──────────────────────────────────┘
```

## Wall-Following Steering

The steering command is computed from the difference between right-side and
left-side sensor readings:

```
if both sides open (> SOD):
    diff = 800                      # bias toward right wall
else:
    diff = sensor[RIGHT] - sensor[LEFT]

# blend in hard-side sensors (25% weight)
diff += (sensor[HARD_RIGHT] - sensor[HARD_LEFT]) * 0.25

steer = diff * steering_coefficient
```

The steering coefficient adapts to the situation:
- **Clear path** (front distance ≥ FOD): `coef = COE1` (default 0.3) — gentle turns
- **Blocked path** (front distance < FOD): `coef = COE2` (default 0.7) — aggressive turns

## Speed Selection

| Condition | Target Speed | Description |
|-----------|-------------|-------------|
| Front clear (≥ FOD) | SPD1 (2.7 m/s) | Full speed straight |
| Front blocked (< FOD) | SPD2 (0.8 m/s) | Slow for turn |
| Track learning active | Learned profile | Anticipatory braking |

## PID Speed Controller

The PID controller runs every control loop iteration (40 ms):

```
error = target_speed - measured_speed

# EMA filter on speed (α = 0.7)
filtered_speed = 0.7 * current + 0.3 * previous

# PID terms
P = Kp * error
I = I_prev + Ki * error * dt    # clamped to ±50
D = Kd * (error - prev_error) / dt

# Feedforward: jump past motor dead zone
ff = (target > 0) ? min_esc_speed : neutral

output = ff + P + I + D
output = clamp(output, min_esc_speed, max_esc_speed)
```

Default gains: Kp=60, Ki=40, Kd=6.

The tachometer provides speed feedback via optical encoder pulses. Speed is
calculated from the interval between consecutive rising edges.

## Stuck Detection

The robot detects stuck conditions when:
1. Front sensors report close distance (< CFD), AND
2. Measured speed is near zero

A counter increments each loop iteration while stuck. When it exceeds `STK`
(default 25 ticks = 1 second), the escape sequence triggers:

```
1. Full stop (100 ms)
2. Short reverse at -150 steering (200 ms)
3. Long reverse at full opposite lock (1800 ms)
4. Resume forward driving
```

## Wrong-Direction Detection

The IMU integrates yaw rate to track cumulative heading change. If the robot
turns more than `WDD` degrees (default 120°) in the direction opposite to
`RCW` (race clockwise flag), it performs an emergency recovery:

```
1. Stop motors
2. Reset heading
3. Reverse with corrective steering (1500 ms)
4. Resume forward
```

## Manual Drive Mode

When enabled via `$DRVEN`, the control loop accepts direct steering and speed
commands via `$DRV:steer,speed`. Manual commands have a 500 ms timeout —
if no command arrives within that window, the robot stops (safety).

## Thread Architecture

| Thread | Priority | Stack | Period | Role |
|--------|----------|-------|--------|------|
| control | 2 | 4096 B | 40 ms | Main control loop |
| wifi_cmd | 5 | 2048 B | event-driven | Command parsing |
| battery | 10 | 1024 B | 500 ms | Voltage monitoring |
| Tachometer ISR | — | — | edge-triggered | Pulse counting |

Inter-thread communication:
- **Command queue** (`k_msgq`): wifi_cmd → control (start/stop/drive commands)
- **Atomic variables**: tachometer ISR → PID (pulse count, interval)
- **Shared config struct**: read by control, written by wifi_cmd under settings API
