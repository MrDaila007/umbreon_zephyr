# Umbreon Zephyr — Autonomous Roborace Firmware

Firmware for the Umbreon autonomous racing robot running on Zephyr RTOS.
Targets Zephyr v4.4 (recommended) for RP2350 (Raspberry Pi Pico 2).

## Hardware

| Component | Interface | Pins |
|-----------|-----------|------|
| 6x VL53L0X ToF | I2C1 100kHz | SDA=GP2, SCL=GP3, XSHUT=GP6-9,14,15 |
| MPU-6050 IMU | I2C0 400kHz | SDA=GP0, SCL=GP1 |
| ESP8266 WiFi | UART1 115200 | TX=GP4, RX=GP5 |
| Debug console / commands | UART0 115200 | TX=GP16, RX=GP17 |
| Servo (steering) | PWM slice 5A | GP10 |
| ESC (motor) | PWM slice 5B | GP11 |
| Tachometer | GPIO IRQ RISING | GP13 |
| Battery | ADC ch0 | GP26 (18k/10k divider) |
| OLED SSD1306 128x64 | I2C0 400kHz | SDA=GP0, SCL=GP1, addr=0x3C |
| Menu encoder | GPIO active low | CLK=GP22, DT=GP12, button=GP19 |

### Sensor Layout

```text
             REAR

      [0 HR]           [5 HL]
      Hard-Right       Hard-Left

          [1 FR]   [4 FL]
          Front-R  Front-L

          [2 R]    [3 L]
          Right    Left

            FRONT
```

## Getting Started (from scratch)

### One-command setup

The `setup_zephyr.sh` script installs everything needed on a fresh Linux machine:

```bash
git clone <repo-url> && cd umbreon_zephyr
./setup_zephyr.sh
```

This will:
1. Install system packages (cmake, ninja, dtc, ccache, etc.)
2. Download and install [Zephyr SDK 1.0](https://github.com/zephyrproject-rtos/sdk-ng) with ARM toolchain
3. Initialize Zephyr v4.4 workspace at `~/zephyrproject-v4.4`
4. Create Python venv and install all dependencies
5. Clone u8g2 C sources into `modules/u8g2`

Run `./setup_zephyr.sh --help` for all options.

### Manual setup (step by step)

<details>
<summary>Click to expand</summary>

#### 1. System packages (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install -y git cmake ninja-build python3 python3-pip \
    python3-venv wget device-tree-compiler ccache dfu-util
```

#### 2. Zephyr SDK

```bash
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.0/zephyr-sdk-1.0.0_linux-x86_64_minimal.tar.xz
tar xf zephyr-sdk-1.0.0_linux-x86_64_minimal.tar.xz -C ~/
cd ~/zephyr-sdk-1.0.0
./setup.sh -t arm-zephyr-eabi -c
```

#### 3. Zephyr workspace

```bash
pip3 install --user west
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.4.0 ~/zephyrproject-v4.4
cd ~/zephyrproject-v4.4
west update --narrow -o=--depth=1
```

#### 4. u8g2 sources

```bash
git init modules/u8g2
git -C modules/u8g2 remote add origin https://github.com/olikraus/u8g2.git
git -C modules/u8g2 fetch --depth 1 origin cbceaa1cab22ad63e41c2df684e173cd5433766e
git -C modules/u8g2 checkout --detach FETCH_HEAD
```

#### 5. Python venv

```bash
python3 -m venv ~/zephyrproject-v4.4/.venv
source ~/zephyrproject-v4.4/.venv/bin/activate
pip install west
pip install -r ~/zephyrproject-v4.4/zephyr/scripts/requirements.txt
```

</details>

<details>
<summary>Legacy: Zephyr v4.1 (requires RP2350 flash patches)</summary>

The RP2350 flash driver is broken in v4.1. The setup script handles this automatically:

```bash
./setup_zephyr.sh --version v4.1.0 ~/zephyrproject-v4.1
```

Or apply patches manually to an existing v4.1 workspace:

```bash
./setup_zephyr.sh --patch-only ~/zephyrproject-v4.1
```

</details>

### Build

```bash
make build                  # standard build (UART0 console)
make build-usb              # USB CDC-ACM console instead
```

Or manually:

```bash
cd ~/zephyrproject-v4.4 && source .venv/bin/activate
west build -b rpi_pico2/rp2350a/m33 -d /path/to/umbreon_zephyr/build --pristine always /path/to/umbreon_zephyr
```

### Flash

1. Hold **BOOTSEL** button on Pico 2
2. Connect USB cable (Pico appears as mass storage `RP2350`)
3. Run:

```bash
make flash
```

Or manually: `cp build/zephyr/zephyr.uf2 /media/$USER/RP2350/`

### HIL bring-up (bare controller, no sensors)

Use this profile when the board is connected only to debugger/console wires.

#### Wiring

- ST-Link V2 `SWDIO` -> Pico 2 `SWDIO`
- ST-Link V2 `SWCLK` -> Pico 2 `SWCLK`
- ST-Link V2 `GND` -> Pico 2 `GND`
- Optional USB-TTL for UART0 logs:
  - USB-TTL `RX` -> Pico 2 `GP16` (UART0 TX)
  - USB-TTL `TX` -> Pico 2 `GP17` (UART0 RX)
  - USB-TTL `GND` -> Pico 2 `GND`

#### Build/flash/monitor

```bash
make build-hil                 # disables IMU + VL53 nodes
make flash-stlink              # via OpenOCD + ST-Link
make monitor-uart0             # default /dev/ttyUSB0 @115200
```

If your serial adapter uses a different port:

```bash
make monitor-uart0 UART0_PORT=/dev/ttyUSB1
```

If OpenOCD target/interface paths differ on your host:

```bash
make flash-stlink OPENOCD_IFACE=interface/stlink.cfg OPENOCD_TARGET=target/rp2350.cfg
```

### HIL transport without WiFi

For HIL where RP2350 has no WiFi module:

- `sim` is the data source / behavior model
- `dashboard` is only visualization + command UI
- `tools/hil_bridge.py` provides a unified TCP endpoint for dashboard

Modes:

- `real`: dashboard -> TCP -> bridge -> UART (`RP2350`)
- `sim`: dashboard -> TCP -> bridge -> `sim.py --bridge`
- `dual`: both endpoints in one process:
  - real endpoint (default `127.0.0.1:8023`)
  - sim endpoint  (default `127.0.0.1:8123`)

Quick start:

```bash
make hil-deps
make hil-real   HIL_SERIAL_PORT=/dev/ttyUSB0
make hil-sim
make hil-dual   HIL_SERIAL_PORT=/dev/ttyUSB0
```

Dashboard endpoint selection:

- connect dashboard to `127.0.0.1:8023` for real stream
- connect dashboard to `127.0.0.1:8123` for sim stream (or use second dashboard instance)

### HIL health tests on real hardware

`tools/hil_runner.py` drives the firmware command UART and fails with non-zero
exit if it sees reboot/fault markers, timestamp rollback, too many telemetry
gaps, low battery, or impossible tachometer speed spikes.

Safe smoke test (no motor):

```bash
make hil-smoke PROBE_UART=/dev/ttyACM0
```

Five-minute RUN endurance test:

```bash
make hil-endurance PROBE_UART=/dev/ttyACM0 HIL_DURATION=300
```

Bench motor test (wheels must be lifted):

```bash
make hil-motor PROBE_UART=/dev/ttyACM0
```

Logs are written to `/tmp` by default:

- `/tmp/umbreon_hil_smoke.log`, `/tmp/umbreon_hil_smoke.json`
- `/tmp/umbreon_hil_endurance.log`, `/tmp/umbreon_hil_endurance.json`
- `/tmp/umbreon_hil_motor.log`, `/tmp/umbreon_hil_motor.json`

Useful overrides:

```bash
make hil-endurance HIL_LOG_DIR=./hil-logs HIL_TGF=500 HIL_DURATION=600
```

### Monitor serial console

```bash
make monitor                # picocom on /dev/ttyACM0 @ 115200
```

### Run tests

```bash
make test                   # all tests (host + ztest)
make test-host              # host unit tests only (no hardware)
make test-ztest             # Zephyr ztest on native_sim
```

### Check display UI

On boot, the OLED shows a splash screen (team logo, firmware version, status) for
2.5 seconds (`CONFIG_APP_BOOTSCREEN`, `CONFIG_APP_BOOTSCREEN_DURATION_MS`), then the
dashboard. Disable with `CONFIG_APP_BOOTSCREEN=n` in `prj.conf`.

Firmware version shown on the boot splash and Info screen comes from
`src/version.h`. The built firmware appends the current Git commit, for example
`2.0.1 (e4c0dab)`, so the car's menu shows exactly which commit is running.
An asterisk after the hash, for example `2.0.1 (e4c0dab*)`, means the firmware
was built from a dirty worktree.

Install the repository hook once to bump the patch version automatically before
each commit:

```bash
make install-hooks
```

Manual version controls are still available:

```bash
make version-patch            # 2.0.0 -> 2.0.1
make version-minor            # 2.0.1 -> 2.1.0
make version-major            # 2.1.0 -> 3.0.0
make version-bump VERSION=2.2.0
make version-show
```

Renders the 128×64 dashboard at 4× scale using the project's own BDF fonts
and checks every pixel for zone overlaps. The source-screen pass also compiles
the real `src/screens/*.c` files against a small host stub harness, then checks
the actual `screen_*_draw()` output for blank frames, out-of-bounds draw calls,
and overlaps.

Saves `tools/sim_dashboard.png` for the legacy dashboard scenarios and
`tools/sim_screens.png` for the source-screen render grid.

```bash
make check-ui               # requires: pip install pillow
make check-ui-dashboard     # legacy dashboard scenarios only
make check-ui-screens       # real src/screens/*.c draw functions
```

Intentional overlaps (tick marks crossing the IMU baseline) are whitelisted and
shown in yellow. Real overlaps are red and block CI.

### Makefile targets reference

| Target | Description |
|--------|-------------|
| `make setup` | Run `setup_zephyr.sh` (full environment setup) |
| `make install-hooks` | Enable the pre-commit firmware version bump hook |
| `make build` | Build firmware (UART console) |
| `make build-usb` | Build firmware (USB console) |
| `make build-hil` | Build for bare HIL (IMU/VL53 disabled) |
| `make version-patch` | Bump firmware patch version in `src/version.h` |
| `make version-minor` | Bump firmware minor version in `src/version.h` |
| `make version-major` | Bump firmware major version in `src/version.h` |
| `make version-bump VERSION=x.y.z` | Set an explicit firmware version |
| `make version-show` | Print the current firmware version |
| `make flash` | Copy UF2 to Pico 2 in BOOTSEL mode |
| `make flash-stlink` | Flash ELF via ST-Link/OpenOCD |
| `make monitor` | Serial console (picocom) |
| `make monitor-uart0` | UART0 logs via USB-TTL (default `/dev/ttyUSB0`) |
| `make test` | Run all tests |
| `make test-host` | Host unit tests (gcc, no Zephyr) |
| `make test-ztest` | Zephyr ztest (native_sim) |
| `make clean` | Remove build artifacts |
| `make hil-deps` | Install Python deps for HIL bridge (`pyserial`) |
| `make hil-real` | Run bridge: dashboard <-> UART (RP2350) |
| `make hil-sim` | Run sim + bridge endpoint for dashboard |
| `make hil-dual` | Run real + sim endpoints in one bridge process |
| `make hil-smoke` | Safe real-hardware command/telemetry smoke test |
| `make hil-endurance` | RUN endurance test with reboot/fault detection |
| `make hil-motor` | Bench motor HIL test (requires lifted wheels) |
| `make check-ui` | Run both display UI checkers; saves `tools/sim_dashboard.png` and `tools/sim_screens.png` |

## Architecture

### Threads

| Thread | Priority | Stack | Period | Purpose |
|--------|----------|-------|--------|---------|
| control | 2 | 4096B | 40ms | Sensors, PID, steering, detection |
| display | 3 | 4096B | 120ms | SSD1306 screen rendering (dashboard, menu, settings, info, WiFi) |
| wifi_cmd | 5 | 2048B | event | UART command parsing, WiFi status polling |
| battery | 10 | 1024B | 50ms default | Battery ADC monitoring |
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
| `wifi_cmd.c/h` | UART1 command protocol (ESP8266 WiFi bridge), WiFi status state |
| `wifi_cipher.h` | Shared XOR+hex cipher for Pico↔ESP credential exchange |
| `settings.c/h` | NVS storage for 31 configurable parameters |
| `battery.c/h` | ADC monitoring, low-voltage cutoff |
| `tests.c/h` | 8 diagnostic test routines |
| `track_learn.c/h` | Track profile recording and race replay |
| `display.c/h` | Display thread, screen state machine (dashboard / menu) |
| `display_hal.c/h` | u8g2 HAL — Zephyr I2C0 to SSD1306 |
| `screens/screen_boot.c` | Boot splash: logo (XBM), firmware version, status line |
| `assets/umbreon_logo.c` | Monochrome logo bitmap for boot screen |
| `screens/screen_dashboard.c` | Main dashboard: battery, sensor bars, IMU scale, WiFi strip |
| `screens/screen_info.c` | Info screen: firmware version, sensor status |
| `screens/screen_wifi.c` | WiFi status screen: mode, SSID, IP, RSSI, connection status |

## WiFi Protocol

All commands are ASCII over UART1 (ESP8266 bridge, GP4/GP5), prefixed with `$`, terminated with `\n`.

The internal Pico↔ESP bridge protocol (`#WIFISTATUS` polling, `$WIFICFG` credential
provisioning, XOR+hex cipher) is documented in
[docs/wifi-protocol.md](docs/wifi-protocol.md#internal-bridge-protocol-pico--esp).

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
| `$WIFICFG:ACK` / `$WIFICFG:NAK` | ESP acknowledges credential update (internal) |

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
| KOP / KOM / CKU | Start kick %, duration, steering-load extra µs | 18 / 300 / 30 |
| COE1/COE2 | Steering gain clear/blocked | 0.28 / 0.65 |
| RBC/RDC/RBM/RDM | Short reverse escape command/timing | see `settings.c` |
| LBM/LDM/LFS/LFM | Wrong-direction recovery reverse/forward timing | see `settings.c` |
| WDD / RCW / STK / STL / IMR / SVR / CAL | Navigation & hardware flags | see `settings.c` |
| BEN / BML / BLV | Battery monitor / scale / low (V) | 0 / 4.85 / 6.0 |
| TGF | Tachometer glitch filter (µs) | 500 |

## TODO

See [docs/TODO.md](docs/TODO.md) for the full backlog.
