# WiFi Command Protocol

## Transport

- Primary physical link: UART1 at 115200 baud (GP4 TX, GP5 RX)
- Debug physical link: UART0 at 115200 baud (GP16 TX, GP17 RX)
- Bridge: ESP8266 with custom firmware on UART1 (transparent serial-to-WiFi)
- Format: ASCII, prefix `$`, terminator `\n`
- Responses use the same format and are mirrored to the debug UART

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

### RUN Sub-State Stream

During autonomous mode, the robot sends sub-state telemetry at 5 Hz (every
200 ms) plus on entry/exit of recovery maneuvers (`go_back`, wrong-direction):

```
$RUN:<state>,<stuck_time>,<turns>,<how_clear>,<diff>
```

| Field | Type | Description |
|-------|------|-------------|
| state | 0–6 | 0=CLEAR, 1=BLOCKED, 2=STUCK_WAIT, 3=REVERSE, 4=WRONG_DIR, 5=STALL, 6=SENSOR_RECOVERY |
| stuck_time | int | Stuck counter (cycles, threshold at `STK` setting) |
| turns | float | Integrated heading change (°), triggers wrong-dir at `WDD` |
| how_clear | 0–2 | Number of front sensors detecting an obstacle |
| diff | int | Steering decision value (after coefficient) |

### Boot Messages

The robot sends status messages during startup:

```
$BOOT:SNS=6,FW=<version>   # Sensor count and firmware version
$BOOT:READY,UP=4200         # Boot complete, uptime in ms
```

`FW` and read-only `$GET` key `FWV` both come from `src/version.h`. Use
`make version-patch`, `make version-minor`, `make version-major`, or
`make version-bump VERSION=x.y.z` before building visible firmware changes.

## Configuration Parameters

Compile-time defaults are in `src/settings.c` (`set_defaults()`). **`$LOAD`**
restores the last **`$SAVE`** blob; **`$RST`** resets to these defaults. NVS
format version may bump when new fields are added; firmware keeps migration code
for recent saved versions and fills new fields from defaults.

Example:

```
$SET:KP=30,KI=45,KD=0,TGF=400,CKU=45,LFS=0.12,LFM=400
$SAVE
```

Sensor distance units match `$SNS` output and current firmware constants
(`cm x 10`; `9999` means no wall / out of range in telemetry paths).

| Key | Description | Default | Unit / range |
|-----|-------------|---------|--------------|
| FOD | Front obstacle threshold; selects blocked speed/steering when a front sensor is closer | 800 | sensor units |
| SOD | Side-open threshold; if both sides are open, wall-follow bias is used | 600 | sensor units |
| ACD | All-close threshold; if all sensors are close, wall-follow bias is used | 400 | sensor units |
| CFD | Close-front threshold used by stuck detection | 100 | sensor units |
| KP | PID proportional gain | 66.4 | float |
| KI | PID integral gain | 243.5 | float |
| KD | PID derivative gain | 4.16 | float |
| MSP | Minimum forward ESC pulse; PID feedforward floor / motor dead-zone compensation | 1540 | µs, 1000-2000 |
| XSP | Maximum forward ESC pulse; final PID output clamp | 1600 | µs, 1000-2000 |
| BSP | Reverse ESC pulse used by direct `car_write_speed(-x)` mapping endpoint | 1460 | µs, 1000-2000 |
| MNP | Servo angle at full left command | 60 | degrees, 0-180 |
| XNP | Servo angle at full right command | 120 | degrees, 0-180 |
| NTP | Servo neutral angle | 90 | degrees, 0-180 |
| ENH | Encoder holes per wheel revolution | 68 | count |
| WDM | Wheel diameter used for speed/distance from tachometer | 0.060 | meters |
| LMS | Control loop period | 40 | ms, min 10 |
| SPD1 | Target speed when path ahead is clear | 0.48 | m/s |
| SPD2 | Target speed when front obstacle logic selects blocked/slow mode | 0.32 | m/s |
| SLW | PID setpoint slew limit; `0` disables smoothing | 0.85 | m/s per s |
| KOP | Start-kick strength; percent of forward ESC span `(XSP - 1500)` | 18.0 | %, 0-80 |
| KOM | Start-kick duration after forward command from rest | 300 | ms, 0-5000 |
| CKU | Extra launch ESC microseconds at full steering lock while speed is below ~0.12 m/s | 30 | µs, 0-120 |
| COE1 | Steering coefficient when path is clear | 0.28 | float |
| COE2 | Steering coefficient when front is blocked | 0.65 | float |
| WDD | Wrong-direction heading threshold before long recovery maneuver | 120.0 | degrees |
| RCW | Race direction flag: `1` clockwise, `0` counter-clockwise | 1 | bool |
| STK | Stuck counter threshold before short reverse escape | 25 | control ticks |
| STL | Stall counter threshold for stall state reporting | 50 | control ticks |
| RBC | Reverse brake command for ESC brake phase | -250 | raw speed cmd, -1000..0 |
| RDC | Reverse drive command after brake-neutral phase | -380 | raw speed cmd, -1000..0 |
| RBM | Reverse brake phase duration | 650 | ms, 0-5000 |
| RDM | Short reverse drive phase timeout | 2600 | ms, 0-5000 |
| LBM | Long/wrong-direction reverse brake phase duration | 1100 | ms, 0-5000 |
| LDM | Long/wrong-direction reverse drive phase timeout | 2400 | ms, 0-5000 |
| LFS | Long/wrong-direction final forward speed cap; actual target is `min(SPD2, LFS)` | 0.18 | m/s, 0-2 |
| LFM | Long/wrong-direction final forward phase duration | 550 | ms, 0-5000 |
| IMR | IMU rotate flag used by navigation orientation | 1 | bool |
| SVR | Reverse servo command direction | 0 | bool |
| CAL | ESC calibrated flag | 0 | bool |
| BEN | Battery monitoring enabled flag | 0 | bool |
| BML | Battery voltage multiplier for ADC divider calibration | 4.85 | float |
| BLV | Battery low voltage threshold | 6.0 | volts |
| TGF | Tachometer glitch reject threshold | 500 | µs, 1-500 |

Read-only keys appended on `$GET` (not writable via `$SET`): `IMU`, `DBG`, `SNS`,
`SMX`, `FWV`.

---

## Internal Bridge Protocol (Pico ↔ ESP)

These messages are exchanged directly between the RP2350 (Pico) and the
ESP8266 over UART1. They are **never forwarded** to TCP/WebSocket clients.

### Status Poll

The Pico sends `#WIFISTATUS\n` every 10 s (on the k_poll timeout). The ESP
intercepts it and replies with a multi-line block:

```
# Mode: STA
# SSID: <hex>
# IP: 192.168.1.42
# RSSI: -65
# Status: ready
```

In AP mode the `# RSSI:` line is omitted, `# AP Pass:` carries the XOR+hex
encrypted AP password (same cipher as SSID), and IP is the AP gateway address
(`192.168.4.1` by default).

The `# SSID:` value is XOR-encrypted with the shared PSK and hex-encoded (see
Cipher section below). The Pico decrypts it and stores the plain SSID in
`ws_ssid`. The IP is stored as-is in `ws_ip`.

Pico getters updated on every poll:

| Function | Returns |
|----------|---------|
| `wifi_status_is_ready()` | `true` when `# Status: ready` |
| `wifi_status_is_ap()` | `true` when `# Mode: AP` |
| `wifi_status_get_rssi()` | RSSI dBm (0 in AP mode) |
| `wifi_status_get_ssid()` | Decrypted SSID string |
| `wifi_status_get_ip()` | IP address string |
| `wifi_status_get_ap_pass()` | Decrypted AP password (AP mode only) |

### Credential Provisioning

To change the ESP WiFi credentials at runtime, call `wifi_cfg_set(ssid, pass)`
from the Pico (must be called from the `wifi_cmd` thread, e.g. inside
`dispatch_command`).

**Wire format:**

```
$WIFICFG:<hex>\n
```

where `<hex>` = `cfg_to_hex(cfg_xor("ssid\tpassword"))`.

The payload before encryption is `ssid` + tab (`\t`) + `password`, up to
96 bytes (32 SSID + 1 tab + 63 password). Maximum hex length: 192 chars.

**Retry behaviour:** The Pico retransmits on each 10 s timeout until it
receives `$WIFICFG:ACK`, up to `CFG_MAX_RETRIES` (5) attempts. On NAK or after
all retries the counter is cleared.

**ESP response:**

| Response | Meaning |
|----------|---------|
| `$WIFICFG:ACK\r\n` | Credentials saved to NVS; ESP restarts in ~300 ms |
| `$WIFICFG:NAK\r\n` | Payload malformed (invalid hex or missing tab separator) |

After restart the ESP reads the NVS override (`wifi_ovrd` namespace) on boot
with priority over compile-time credentials (`wifi_config.h`).

### Cipher (`wifi_cipher.h`)

The header `src/wifi_cipher.h` (identical copy in `umbreon_esp_web/main/`)
provides three static inline helpers used by both firmwares:

| Function | Description |
|----------|-------------|
| `cfg_xor(in, out, n)` | XOR stream with 16-byte PSK; same operation for encrypt/decrypt |
| `cfg_to_hex(data, n, out, sz)` | Encode bytes as lowercase hex |
| `cfg_from_hex(s, out, max)` | Decode hex string to bytes |

The PSK (`CFG_PSK[16]`) must be identical in both builds. Rotate by editing
both headers together.
