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
| `$GET` | Retrieve all 31 parameters |
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
| `$TEST:name` | Run test (lidar, servo, taho, esc, speed, autotune, reactive, cal) |
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

| Key | Description | Default |
|-----|-------------|---------|
| FOD | Front obstacle distance (mm) | 700 |
| SOD | Side open distance (mm) | 900 |
| ACD | All-close distance (mm) | 200 |
| CFD | Close front distance (mm) | 300 |
| KP/KI/KD | PID gains | 60/40/6 |
| MSP/XSP | Min/max ESC speed | 1520/1700 |
| BSP | Min reverse speed | 1460 |
| MNP/XNP/NTP | Min/max/neutral servo angle | 40/140/90 |
| ENH | Encoder holes | 62 |
| WDM | Wheel diameter (m) | 0.060 |
| LMS | Control loop period (ms) | 40 |
| SPD1/SPD2 | Speed clear/blocked (m/s) | 2.7/0.8 |
| COE1/COE2 | Steering coefficient clear/blocked | 0.3/0.7 |
| WDD | Wrong direction threshold (degrees) | 120 |
| RCW | Race clockwise | 1 |
| STK | Stuck threshold (ticks) | 25 |
| SVR | Servo reverse | 0 |
| CAL | ESC calibration done | 0 |
| BEN/BML/BLV | Battery: enabled/multiplier/low threshold (V) | 1/2.8/6.0 |

## TODO — Not yet ported from Arduino

### Major

- [ ] **OLED menu + rotary encoder** — 10-screen state machine (dashboard, settings, tests, info) on SSD1306 128×64. Arduino: `menu.h` (943 lines). No display driver or encoder handling in Zephyr yet
- [ ] **Dual-core (Core 1)** — Arduino uses Core 1 for OLED/encoder (`core1.h`, `setup1()`/`loop1()`). Zephyr runs everything as threads on a single core, second RP2350 core is idle
- [ ] **TF-Luna 4× LiDAR** — 4 UART lidars via SerialPIO (`luna_car.h`). Zephyr only supports 6× VL53L0X, no flexible sensor selection

### Minor

- [ ] **Compile-time sensor config** — Arduino: `sensor_config.h` allows switching between 4×Luna and 6×VL53L0X via `SENSOR_CONFIG`. Zephyr hardcodes 6× VL53L0X
- [ ] **Competition mode** — Auto-start flag without menu/WiFi for races
- [ ] **I2C bus recovery (bit-bang)** — Manual I2C1 bus recovery via SCL toggling. Zephyr uses `i2c_recover_bus()` (less reliable)
- [ ] **Battery sustained low-voltage cutoff** — Arduino cuts motor after 10 seconds of sustained low voltage. Zephyr only does a one-shot check
