# Umbreon Zephyr — Autonomous Roborace Firmware

Firmware for the Umbreon autonomous racing robot running on Zephyr RTOS.
Ported from Arduino (arduino-pico) to Zephyr v4.1 for RP2350 (Raspberry Pi Pico 2).

## Hardware

| Component | Interface | Pins |
|-----------|-----------|------|
| 6x VL53L0X ToF | I2C1 100kHz | SDA=GP2, SCL=GP3, XSHUT=GP6-9,14,15 |
| MPU-6050 IMU | I2C0 400kHz | SDA=GP0, SCL=GP1 |
| ESP8266 WiFi | UART1 115200 | TX=GP4, RX=GP5 |
| Debug console | UART0 115200 | TX=GP16, RX=GP17 |
| Servo (steering) | PWM slice 5A | GP10 |
| ESC (motor) | PWM slice 5B | GP11 |
| Tachometer | GPIO IRQ RISING | GP13 |
| Battery | ADC ch0 | GP26 (18k/10k divider) |

### Sensor Layout

```
        [1:FR]  [4:FL]
  [0:HR]                [5:HL]
        [2:R]   [3:L]
      ────── FRONT ──────
```

## Building

### Prerequisites

- Zephyr SDK 0.17+ with ARM toolchain
- Zephyr v4.1 workspace (`~/zephyrproject`)

### Cherry-picks (required for Zephyr v4.1)

The RP2350 flash driver is broken in v4.1 out of the box. Two cherry-picks are required:

```bash
# Flash controller support for RP2350
cd ~/zephyrproject/zephyr
git cherry-pick --no-commit 5d36e85b99a

# flash_write_partial() for QMI controller
cd ~/zephyrproject/modules/hal/rpi_pico
git cherry-pick --no-commit 5d7744c
```

These fixes are included in Zephyr v4.3+. Drop the cherry-picks when upgrading.

### Build and Flash

```bash
cd ~/zephyrproject
source .venv/bin/activate
west build -b rpi_pico2/rp2350a/m33 --pristine always /path/to/umbreon_zephyr

# Hold BOOTSEL, connect USB
cp build/zephyr/zephyr.uf2 /media/$USER/RP2350/
```

## Architecture

### Threads

| Thread | Priority | Stack | Period | Purpose |
|--------|----------|-------|--------|---------|
| control | 2 | 4096B | 40ms | Sensors, PID, steering, detection |
| wifi_cmd | 5 | 2048B | event | UART command parsing |
| battery | 10 | 1024B | 500ms | Battery ADC monitoring |
| main | — | 4096B | — | Init, then sleeps forever |
| Tachometer ISR | ISR | — | edge | Pulse counting |

### Source Files

| File | Purpose |
|------|---------|
| `main.c` | Subsystem initialization, watchdog setup |
| `car.c/h` | PWM servo/ESC control, PID speed controller |
| `sensors.c/h` | 6x VL53L0X polling via Zephyr sensor API |
| `imu.c/h` | MPU-6050 gyro Z, calibration, heading integration |
| `tachometer.c/h` | GPIO ISR, speed calculation |
| `control.c/h` | Main control loop: wall-follow, stuck detection |
| `wifi_cmd.c/h` | UART1 command protocol (ESP8266 WiFi bridge) |
| `settings.c/h` | NVS storage for 31 configurable parameters |
| `battery.c/h` | ADC monitoring, low-voltage cutoff |
| `tests.c/h` | 8 diagnostic test routines |
| `track_learn.c/h` | Track profile recording and race replay |

## WiFi Protocol

All commands are ASCII over UART1 (ESP8266 bridge, GP4/GP5), prefixed with `$`, terminated with `\n`.

### Control

| Command | Response | Description |
|---------|----------|-------------|
| `$PING` | `$PONG` | Connection check |
| `$START` | `$ACK` + `$STS:RUN` | Start autonomous driving (5s countdown) |
| `$STOP` | `$ACK` + `$STS:STOP` | Stop all motors |
| `$STATUS` | `$STS:RUN/STOP` | Current state |

### Settings

| Command | Description |
|---------|-------------|
| `$GET` | Retrieve all parameters (`$CFG:...`) |
| `$SET:KEY=VAL,...` | Set parameters (e.g. `$SET:MNP=60,XNP=120`) |
| `$SAVE` | Persist to NVS flash |
| `$LOAD` | Load from NVS |
| `$RST` | Reset to compile-time defaults |

### Manual Drive

| Command | Description |
|---------|-------------|
| `$DRVEN` | Enable manual drive mode |
| `$DRV:steer,speed` | Steering (-1000..+1000) and speed (m/s) |
| `$DRVOFF` | Disable manual drive, stop motors |
| `$SRV:angle` | Direct servo angle (0-180 degrees) |
| `$ESC:us` | Direct ESC pulse width (1000-2000 us) |

### Diagnostics

| Command | Description |
|---------|-------------|
| `$BAT` | Battery voltage |
| `$TEST:name` | Run test (lidar, servo, taho, esc, speed, autotune, **pidtune**, reactive, cal) |
| `$DIAG` | Overall system diagnostics |
| `$SNS` | Raw sensor readings |
| `$IMU` | IMU state (yaw rate, heading) |
| `$PID` | PID parameters and current speed |
| `$SYS` | System info (uptime, settings) |
| `$LOG:ON/OFF` | Toggle debug log forwarding (`$L:...` prefix) |
| `$HELP` | List all available commands |

### Track Learning

| Command | Description |
|---------|-------------|
| `$TRK:START` | Start recording track profile |
| `$TRK:STOP` | Stop recording |
| `$TRK:RACE` | Race mode using recorded profile |
| `$TRK:STATUS` | Recording status |
| `$TRK:CLEAR` | Clear recorded track |

### Telemetry

CSV stream at ~25 Hz:
```
#ms,s0,s1,s2,s3,s4,s5,steer,speed,target,yaw,heading
```

## Configuration Parameters ($GET/$SET)

Authoritative defaults and new keys are in **`src/settings.c`** and
**[docs/wifi-protocol.md](docs/wifi-protocol.md)**. Summary:

| Key | Description | Default (see `settings.c`) |
|-----|-------------|----------------------------|
| FOD/SOD/ACD/CFD | Obstacle / stuck thresholds | `DEFAULT_*` in `settings.h` |
| KP/KI/KD | PID gains | From on-track pidtune seed |
| MSP/XSP/BSP | ESC µs limits | 1540 / 1600 / 1460 |
| MNP/XNP/NTP | Servo limits (°) | 60 / 120 / 90 |
| ENH / WDM | Encoder / wheel | 68 / 0.060 m |
| LMS | Loop period (ms) | 40 |
| SPD1 / SPD2 | Cruise speeds (m/s) | ~0.48 / 0.32 |
| SLW | Setpoint slew (m/s per s; 0=off) | 0.85 |
| KOP / KOM | Start kick % / ms (KOP 0=off) | 18 / 300 |
| COE1/COE2 | Steering gain clear/blocked | 0.28 / 0.65 |
| WDD / RCW / STK / IMR / SVR / CAL | Navigation & hardware flags | see `settings.c` |
| BEN / BML / BLV | Battery monitor / scale / low (V) | 0 / 4.85 / 6.0 |

## TODO

See [docs/TODO.md](docs/TODO.md) for the full backlog.
