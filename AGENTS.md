# Umbreon Zephyr Agent Instructions

## Setup
- Full setup: `make setup` (runs `setup_zephyr.sh`)
- Manual setup: See README for Zephyr SDK 1.0 and Zephyr v4.4 Python venv

## Build
- Race / track (default, no UART0 debug): `make build` or `make build-race` → `./build`
- Bench debug (UART0 console, logs, $ commands on GP16/GP17): `make build-bench` → `./build-bench`
- USB console (bench): `make build-usb`
- HIL (no sensors): `make build-hil`
- Manual west build: `cd ~/zephyrproject-v4.4 && source .venv/bin/activate && west build -b rpi_pico2/rp2350a/m33 -d /path/to/umbreon_zephyr/build --pristine always /path/to/umbreon_zephyr`

## Flash
- UF2 (BOOTSEL): `make flash`
- CMSIS-DAP probe/OpenOCD: `make flash-probe`
- ST-Link/OpenOCD: `make flash-stlink`
- Manual UF2 copy: `cp build/zephyr/zephyr.uf2 /media/$USER/RP2350/`

## Monitor
- Default serial (ttyACM0): `make monitor`
- UART0 console (ttyUSB0): `make monitor-uart0`
- Override port: `make monitor-uart0 UART0_PORT=/dev/ttyUSB1`

## Debug Probe
- Hardware: Raspberry Pi Debug Probe / CMSIS-DAP connected to SWD, plus probe UART wired to target UART0 (`probe RX -> GP16`, `probe TX -> GP17`, common GND).
- Flash current build through the probe: `make flash-probe`.
- If OpenOCD says `unable to find a matching CMSIS-DAP device` or `libusb initialization failed`, rerun with escalated USB access.
- Probe UART usually appears as `/dev/ttyACM0`. The firmware console/logs use UART0 at 115200 by default.
- Read live logs:
  ```sh
  stty -F /dev/ttyACM0 115200 raw -echo
  timeout 12 cat /dev/ttyACM0
  ```
- Reset target and capture boot logs in one command:
  ```sh
  openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
    -c 'adapter speed 5000' \
    -c 'init; reset run; shutdown'
  stty -F /dev/ttyACM0 115200 raw -echo
  timeout 12 cat /dev/ttyACM0
  ```
- If `/dev/ttyACM0` is missing but `lsusb` shows `2e8a:000c Raspberry Pi Debug Probe`, check CDC binding:
  ```sh
  lsmod | grep cdc_acm
  find /sys/bus/usb/devices -path '*tty/ttyACM*' -print
  ```
- The first sandboxed read of `/dev/ttyACM0` may fail even when sysfs shows it. Use escalated access for host USB device reads.
- Useful smoke command after flashing and boot: send `$BEEP` over UART/WiFi to verify the GP18 piezo path.

## Test
- All tests: `make test`
- Host unit tests: `make test-host`
- Zephyr ztest (native_sim): `make test-ztest`

## HIL (Hardware-in-the-Loop)
- Install deps: `make hil-deps`
- Real bridge (UART <-> TCP): `make hil-real HIL_SERIAL_PORT=/dev/ttyUSB0`
- Sim bridge: `make hil-sim`
- Dual bridge (real + sim): `make hil-dual HIL_SERIAL_PORT=/dev/ttyUSB0`
- Dashboard connects to: 127.0.0.1:8023 (real) or 127.0.0.1:8123 (sim)

## Notes
- Zephyr dir: `~/zephyrproject-v4.4` (adjust in Makefile if needed)
- SDK dir: `~/zephyr-sdk-1.0.0` (adjust in setup_zephyr.sh if needed)
- Build dir: `./build` in the project root (configurable via Makefile)
- WiFi protocol: UART1 (GP4/GP5) with ESP8266, ASCII commands prefixed with `$`
- Source layout: `src/` contains main.c, car.c, sensors.c, imu.c, etc.
- Threads: control (40ms), wifi_cmd (event), battery (50ms default), main
- Settings: NVS storage, modify via `$SET:KEY=VAL` commands over WiFi/UART
