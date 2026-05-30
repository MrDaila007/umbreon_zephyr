# Umbreon Zephyr Agent Instructions

## Setup
- Full setup: `make setup` (runs `setup_zephyr.sh`)
- Manual setup: See README for Zephyr SDK 1.0 and Zephyr v4.4 Python venv

## Build
- Standard (UART0 console): `make build`
- USB console: `make build-usb`
- HIL (no sensors): `make build-hil`
- Manual west build: `cd ~/zephyrproject-v4.4 && source .venv/bin/activate && west build -b rpi_pico2/rp2350a/m33 -d /path/to/umbreon_zephyr/build --pristine always /path/to/umbreon_zephyr`

## Flash
- UF2 (BOOTSEL): `make flash`
- ST-Link/OpenOCD: `make flash-stlink`
- Manual UF2 copy: `cp build/zephyr/zephyr.uf2 /media/$USER/RP2350/`

## Monitor
- Default serial (ttyACM0): `make monitor`
- UART0 console (ttyUSB0): `make monitor-uart0`
- Override port: `make monitor-uart0 UART0_PORT=/dev/ttyUSB1`

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
