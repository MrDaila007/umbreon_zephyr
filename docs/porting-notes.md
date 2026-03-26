# Porting Notes: Arduino → Zephyr

## Motivation

The original firmware ran on RP2350 under the arduino-pico framework.
Persistent I2C bus hangs with VL53L0X sensors caused reliability issues that
were difficult to solve without OS-level I2C timeout and recovery support.

Zephyr RTOS provides:
- Hardware I2C timeouts and `i2c_recover_bus()` API
- Deterministic thread scheduling with priorities
- Built-in VL53L0X driver with XSHUT sequencing
- NVS flash storage (replaces EEPROM emulation)
- Watchdog support

## Source File Mapping

| Arduino Source | Zephyr Target | What Was Ported |
|---------------|---------------|-----------------|
| `luna_car.h` (PWM, PID) | `car.c` | Servo/ESC pulse, PID controller |
| `luna_car.h` (tachometer ISR) | `tachometer.c` | GPIO interrupt, speed calc |
| `luna_car.h` (IMU) | `imu.c` | Gyro calibration, heading integration |
| `luna_car.h` (VL53L0X init) | `sensors.c` | Replaced with Zephyr sensor API |
| `Umbreon_roborace.ino` (work loop) | `control.c` | Wall-follow, stuck detection |
| `Umbreon_roborace.ino` (commands) | `wifi_cmd.c` | UART protocol dispatch |
| `Umbreon_roborace.ino` (setup) | `main.c` | Init sequence |
| `eeprom_settings.h` | `settings.c` | NVS instead of EEPROM |
| `core1.h` | — | Removed (single-core with threads) |
| `tests.h` | `tests.c` | Diagnostic routines |
| `track_learn.h` | `track_learn.c` | Track recording/replay |

## API Translation

| Arduino | Zephyr | Notes |
|---------|--------|-------|
| `millis()` | `k_uptime_get()` | Returns int64_t ms |
| `micros()` | `k_cycle_get_32()` | Convert via `sys_clock_hw_cycles_per_sec()` |
| `delay(ms)` | `k_msleep(ms)` | Yields CPU to other threads |
| `delayMicroseconds(us)` | `k_busy_wait(us)` | Busy-wait, use sparingly |
| `analogRead(pin)` | `adc_read(dev, &seq)` | Requires ADC channel setup |
| `digitalWrite(pin, val)` | `gpio_pin_set_dt(&spec, val)` | Uses devicetree GPIO specs |
| `attachInterrupt()` | `gpio_pin_interrupt_configure()` | + `gpio_init_callback()` + `gpio_add_callback()` |
| `volatile` + `noInterrupts()` | `atomic_set()` / `atomic_get()` | Lock-free for ISR→thread |
| `Serial1.print()` | `uart_poll_out()` | Byte-by-byte output |
| `Serial1.available()` | UART IRQ callback | Event-driven via ring buffer |
| `Servo.write(angle)` | `pwm_set_pulse_dt()` | Angle → pulse conversion in code |
| `EEPROM.put()/get()` | `nvs_write()/nvs_read()` | Key-value flash storage |
| `rp2040.wdt_reset()` | `wdt_feed()` | Zephyr watchdog API |
| `rp2040.fifo.push()` | `k_msgq_put()` | Thread-safe message queue |
| `constrain(x, lo, hi)` | `CLAMP(x, lo, hi)` | Zephyr macro |

## Concurrency Model

### Arduino (before)
- Single-threaded `loop()` on core 0
- Core 1 ran IMU updates via `rp2040.fifo`
- `volatile` + `noInterrupts()` for ISR data
- WiFi commands processed in `loop()` between sensor reads

### Zephyr (after)
- **control thread** (priority 2): sensors, PID, steering — 40 ms period
- **wifi_cmd thread** (priority 5): UART parsing, blocks on semaphore
- **battery thread** (priority 10): ADC reads every 500 ms
- **Tachometer ISR**: atomic pulse counting
- Communication via `k_msgq` (commands) and `atomic_t` (tachometer)

## Key Differences

### VL53L0X Initialization
Arduino: Manual XSHUT toggle, I2C address reassignment with Adafruit library.
Zephyr: Declarative in devicetree — driver handles XSHUT and addresses automatically.
Requires `CONFIG_VL53L0X_RECONFIGURE_ADDRESS=y`.

### Distance Units
Zephyr sensor API returns distance in **meters** (`val1` = integer, `val2` = micro-meters).
Conversion: `mm = val1 * 1000 + val2 / 1000`.
The Arduino library returned millimeters directly.

### UART Numbering
On RP2350, GP16/GP17 map to **UART0** (not UART1 as in arduino-pico).
The devicetree overlay configures `&uart0` with these pins.

### Flash Storage
Arduino used EEPROM emulation (LittleFS-backed).
Zephyr uses NVS with a dedicated 64 KB flash partition.
Settings are not compatible — fresh calibration needed after porting.

### No OLED Menu
The Arduino version had an SSD1306 OLED with button-based menu.
This was dropped in the Zephyr port — all configuration is via WiFi commands.

## Zephyr Version Notes

**Recommended: Zephyr v4.3.0+** — RP2350 flash support is upstream, no patches needed.

### v4.1 → v4.3 migration (done 2026-03-26)

- Removed cherry-picks for RP2350 flash (`5d36e85b99a`, `5d7744c`) — now upstream
- DTS `partitions` node must explicitly declare `compatible = "fixed-partitions"`,
  `#address-cells = <1>`, `#size-cells = <1>` (optional in v4.1, required in v4.3)
- CI updated from v4.1 to v4.3, cherry-pick step removed

### Legacy: Build workarounds for Zephyr v4.1

The RP2350 flash driver is broken in Zephyr v4.1. Two cherry-picks were required:

1. **Zephyr commit `5d36e85b99a`**: QMI flash controller support for RP2350
2. **HAL commit `5d7744c`**: `flash_write_partial()` for QMI controller

These are applied automatically by `./setup_zephyr.sh --version v4.1.0`.

### Kconfig notes
- `CONFIG_FPU=y` is not needed — RP2350 M33 doesn't declare `CPU_HAS_FPU` in v4.1
- `CONFIG_CONSOLE=n` and `CONFIG_UART_CONSOLE=n` — UART0 is used for WiFi, not console
- `CONFIG_FLASH_MAP=y` — required for NVS flash area API
