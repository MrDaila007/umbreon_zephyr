# Hardware Reference

## MCU

Raspberry Pi Pico 2 (RP2350A, dual Cortex-M33 @ 150 MHz).
Only core 0 is used — Zephyr runs on the M33 target `rpi_pico2/rp2350a/m33`.

## Pin Map

| GPIO | Function | Notes |
|------|----------|-------|
| GP0 | I2C0 SDA | MPU-6050 IMU |
| GP1 | I2C0 SCL | 400 kHz |
| GP2 | I2C1 SDA | 6× VL53L0X ToF |
| GP3 | I2C1 SCL | 100 kHz |
| GP6 | XSHUT sensor 0 | Hard-Right |
| GP7 | XSHUT sensor 1 | Front-Right |
| GP8 | XSHUT sensor 2 | Right |
| GP9 | XSHUT sensor 3 | Left |
| GP10 | PWM5A — Servo | Steering, 50 Hz |
| GP11 | PWM5B — ESC | Motor, 50 Hz |
| GP13 | Tachometer | Rising-edge IRQ |
| GP14 | XSHUT sensor 4 | Front-Left |
| GP15 | XSHUT sensor 5 | Hard-Left |
| GP16 | UART0 TX | ESP8266 WiFi bridge |
| GP17 | UART0 RX | ESP8266 WiFi bridge |
| GP26 | ADC ch0 | Battery (18 kΩ / 10 kΩ divider) |

## Sensor Layout (top-down view)

```
           REAR
   [0:HR]        [5:HL]
       [1:FR]  [4:FL]
       [2:R]   [3:L]
         ── FRONT ──
```

Indices match the XSHUT power-on sequence and devicetree order.

## VL53L0X ToF Sensors

- 6 sensors on shared I2C1 bus at 100 kHz
- Default I2C address 0x29; Zephyr driver reassigns to 0x30–0x35 via XSHUT sequencing
- Requires `CONFIG_VL53L0X_RECONFIGURE_ADDRESS=y`
- Range: 30–2000 mm (values above 8190 mm treated as out-of-range)
- Internal units: mm (same scale as cm×10)

## MPU-6050 IMU

- I2C0 at 400 kHz, address 0x68
- Only gyroscope Z-axis is used (yaw rate)
- Full-scale range: ±500°/s
- Calibration: 200 samples at 5 ms intervals on startup

## Servo (Steering)

- PWM period: 20 ms (50 Hz)
- Pulse range: 700–2300 µs
- Angle mapping: 0° → 700 µs, 180° → 2300 µs
- Default limits: min 40°, neutral 90°, max 140°
- Steering input: -1000 (full left) to +1000 (full right)

## ESC (Motor)

- PWM period: 20 ms (50 Hz)
- Neutral: 1500 µs (motor off)
- Forward range: 1520–1700 µs (configurable via MSP/XSP)
- Reverse: 1460 µs (configurable via BSP)
- Calibration sequence on first boot: 2000 → 1000 → 1500 µs

## Tachometer

- Optical encoder on GP13
- 62 holes per revolution (configurable via ENH)
- Wheel diameter: 60 mm (configurable via WDM)
- Debounce: 200 µs minimum pulse interval
- Speed formula: `v = π × diameter / (holes × interval)`

## Battery Monitoring

- 2S LiPo (7.4 V nominal)
- Resistor divider: R1 = 18 kΩ, R2 = 10 kΩ → multiplier 2.8
- ADC: 12-bit, 3.3 V reference
- Low-voltage cutoff: 6.0 V (configurable, 10 s sustained)

## ESP8266 WiFi Bridge

- Connected via UART0 at 115200 baud
- Acts as transparent serial-to-WiFi bridge
- No AT commands — firmware on ESP handles TCP/WiFi
- All robot commands and telemetry pass through this link
