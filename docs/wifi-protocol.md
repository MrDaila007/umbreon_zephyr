# WiFi Command Protocol

## Transport

- Physical: UART0 at 115200 baud (GP16 TX, GP17 RX)
- Bridge: ESP8266 with custom firmware (transparent serial-to-WiFi)
- Format: ASCII, prefix `$`, terminator `\n`
- Responses use the same format

## Command Reference

### Connection

| Command | Response | Description |
|---------|----------|-------------|
| `$PING` | `$PONG` | Connectivity check |
| `$HELP` | Command list | Print all available commands |

### Autonomous Control

| Command | Response | Description |
|---------|----------|-------------|
| `$START` | `$ACK` then `$STS:RUN` | Start autonomous mode (5 s countdown) |
| `$STOP` | `$ACK` then `$STS:STOP` | Stop all motors immediately |
| `$STATUS` | `$STS:RUN` or `$STS:STOP` | Query current state |

The 5-second countdown after `$START` allows the operator to place the robot
on the track. The robot begins driving when the countdown expires.

### Settings Management

| Command | Response | Description |
|---------|----------|-------------|
| `$GET` | `$CFG:KEY=VAL,...` | Dump all runtime parameters (see table below) |
| `$SET:KEY=VAL,...` | `$ACK` or `$NAK` | Set one or more parameters |
| `$SAVE` | `$ACK` | Persist current settings to NVS flash |
| `$LOAD` | `$ACK` | Reload settings from NVS |
| `$RST` | `$ACK` | Reset all settings to compile-time defaults |

Setting keys and defaults — see [Configuration Parameters](#configuration-parameters).

Example: `$SET:KP=80,KI=30,SPD1=3.0`

### Manual Drive

| Command | Response | Description |
|---------|----------|-------------|
| `$DRVEN` | `$ACK` | Enable manual drive mode |
| `$DRV:steer,speed` | — | Drive command: steer (-1000..+1000), speed (m/s) |
| `$DRVOFF` | `$ACK` | Disable manual drive, stop motors |
| `$SRV:angle` | `$ACK` | Direct servo angle (0–180 degrees) |
| `$ESC:us` | `$ACK` | Direct ESC pulse width (1000–2000 µs) |

Manual drive commands (`$DRV`) have a **500 ms safety timeout**. If no new
command arrives within that window, the motors stop automatically.

`$SRV` and `$ESC` bypass PID and directly control hardware — use with care.

### Diagnostics

| Command | Response | Description |
|---------|----------|-------------|
| `$DIAG` | Multi-line | System diagnostics (sensors, IMU, battery, tachometer) |
| `$SNS` | `$SNS:d0,d1,d2,d3,d4,d5` | Raw sensor distances (mm) |
| `$IMU` | `$IMU:yaw=...,hdg=...` | Yaw rate (°/s) and heading (°) |
| `$PID` | `$PID:kp=...,ki=...,kd=...,spd=...` | PID state and current speed |
| `$BAT` | `$BAT:voltage` | Battery voltage (V) |
| `$SYS` | Multi-line | System info (uptime, firmware version, settings) |
| `$LOG:ON` | `$ACK` | Enable debug log forwarding |
| `$LOG:OFF` | `$ACK` | Disable debug log forwarding |

When logging is enabled, debug messages from any module are forwarded over
WiFi with the `$L:` prefix: `$L:message text here`.

### Diagnostic Tests

| Command | Description |
|---------|-------------|
| `$TEST:lidar` | Continuous sensor readings for 10 s |
| `$TEST:servo` | Sweep servo through full range |
| `$TEST:taho` | Display tachometer readings for 5 s |
| `$TEST:esc` | Ramp ESC speed up and down |
| `$TEST:speed` | PID step response test |
| `$TEST:autotune` | PID auto-tuning (Ziegler–Nichols style, wheels up) |
| `$TEST:pidtune` | On-track FOPDT + IMC/PI suggestions (car on track, reactive steer) |
| `$TEST:reactive` | Reactive steering test (no PID) |
| `$TEST:cal` | ESC calibration sequence |

**Motor tests:** elevate the car for `esc`, `speed`, `autotune`. **`pidtune`** is
meant for the real track (low-speed steps, safety speed cut at ~0.5 m/s).

Tests run in the WiFi command thread context (`k_msleep` yields to other work).

### Track Learning

| Command | Response | Description |
|---------|----------|-------------|
| `$TRK:START` | `$ACK` | Begin recording track profile |
| `$TRK:STOP` | `$ACK` | Stop recording |
| `$TRK:RACE` | `$ACK` | Switch to race mode (use recorded profile) |
| `$TRK:STATUS` | `$TRK:...` | Recording status (points recorded, distance) |
| `$TRK:CLEAR` | `$ACK` | Erase recorded profile |

Track learning records the steering and speed at each 10 cm segment.
Maximum capacity: 900 points (~90 m of track).

### Telemetry Stream

When the robot is running (autonomous or manual), it streams CSV telemetry
at approximately 25 Hz:

```
#ms,s0,s1,s2,s3,s4,s5,steer,speed,target,yaw,heading
```

| Field | Unit | Description |
|-------|------|-------------|
| ms | ms | Timestamp (uptime) |
| s0–s5 | mm | Sensor distances (HR, FR, R, L, FL, HL) |
| steer | -1000..+1000 | Steering command |
| speed | m/s | Measured speed |
| target | m/s | Target speed |
| yaw | °/s | Yaw rate |
| heading | ° | Integrated heading |

### Boot Messages

The robot sends status messages during startup:

```
$BOOT:SNS=6,FW=2.0.0       # Sensor count and firmware version
$BOOT:READY,UP=4200         # Boot complete, uptime in ms
```

## Configuration Parameters

Compile-time defaults are in `src/settings.c` (`set_defaults()`). **`$LOAD`**
restores last **`$SAVE`** blob; **`$RST`** resets to these defaults. NVS format
version may bump when new fields are added — old saves are rejected until you
re-save.

| Key | Description | Typical default | Unit |
|-----|-------------|-----------------|------|
| FOD | Front obstacle threshold | 800 | sensor units (see `settings.h`; same scale as `$SNS`) |
| SOD | Side “open” threshold | 600 | sensor units |
| ACD | All-close threshold | 400 | sensor units |
| CFD | Close-front (stuck) threshold | 150 | sensor units |
| KP | PID proportional gain | *from pidtune* | — |
| KI | PID integral gain | *from pidtune* | — |
| KD | PID derivative gain | *from pidtune* | — |
| MSP | Min forward ESC (feedforward / PID floor) | 1540 | µs |
| XSP | Max forward ESC | 1600 | µs |
| BSP | Reverse ESC | 1460 | µs |
| MNP | Min servo angle | 60 | ° |
| XNP | Max servo angle | 120 | ° |
| NTP | Neutral servo angle | 90 | ° |
| ENH | Encoder holes per rev | 68 | — |
| WDM | Wheel diameter | 0.060 | m |
| LMS | Control loop period | 40 | ms |
| SPD1 | Speed (path clear) | 0.48 | m/s |
| SPD2 | Speed (path blocked) | 0.32 | m/s |
| SLW | Setpoint slew rate | 0.85 | m/s per s (`0` = off) |
| KOP | Start-kick strength | 18 | % of `(XSP−MSP)` span (`0` = off) |
| KOM | Start-kick duration | 300 | ms |
| COE1 | Steering coef (clear) | 0.28 | — |
| COE2 | Steering coef (blocked) | 0.65 | — |
| WDD | Wrong direction threshold | 120 | ° |
| RCW | Race clockwise | 1 | bool |
| STK | Stuck threshold | 25 | ticks |
| IMR | IMU use in navigation | 1 | bool |
| SVR | Servo reverse | 0 | bool |
| CAL | ESC calibrated | 0 | bool |
| BEN | Battery monitoring enabled | 0 | bool |
| BML | Battery voltage multiplier | 4.85 | — (calibrate for your divider) |
| BLV | Battery low voltage cutoff | 6.0 | V |

Read-only keys appended on `$GET` (not writable via `$SET`): `IMU`, `DBG`, `SNS`,
`SMX`, `FWV`.
