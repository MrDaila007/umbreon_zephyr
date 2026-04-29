# u8g2 Zephyr OLED UI — Design Spec

**Date:** 2026-04-29  
**Branch:** feature/zephyr-v4.4  
**Status:** Approved

---

## Problem

The existing SSD1306 display driver uses Zephyr's **CFB (Character Framebuffer)** API, which
provides only 6×8 text and basic geometry. The desired UI requires:

- Multiple font sizes (status labels, menu items, large values)
- Bitmap icons (battery, WiFi, mode indicators)
- Graphical sensor visualization (6× VL53L0X distance bars)
- IMU yaw deviation indicator (horizontal sliding scale)

---

## Solution

Replace CFB with **u8g2** (olikraus/u8g2). u8g2 includes a built-in SSD1306 I2C driver,
40+ font families, and full bitmap drawing primitives. It communicates directly with the
hardware over I2C, bypassing the Zephyr display subsystem entirely.

u8g2 is integrated as an **out-of-tree Zephyr module** (`modules/u8g2_zephyr/`) following
the same pattern as the existing `modules/vl53l0x_enhanced/`. The u8g2 source is cloned
into `modules/u8g2/` within the project tree.

The public `display.h` API (`display_init`, `display_notify_run_state`,
`display_notify_test_state`, `i2c0_mutex`, `menu_cmd_q`) is **unchanged** — callers in
`control.c`, `tests.c`, and `main.c` require no modification.

---

## Architecture

```
modules/
├── vl53l0x_enhanced/        existing module (reference)
├── u8g2/                    u8g2 source (git clone olikraus/u8g2)
└── u8g2_zephyr/             Zephyr module wrapper
    ├── zephyr/module.yml
    ├── Kconfig
    └── CMakeLists.txt       compiles 21 u8g2 sources, exposes csrc/ headers

src/
├── display.h                PUBLIC API — unchanged
├── display.c                coordinator: thread + input handling (~320 lines)
├── display_hal.h/.c         NEW: Zephyr I2C ↔ u8g2 HAL callbacks
├── display_internal.h       NEW: struct ui_state (internal only)
└── screens/                 NEW: one file per screen
    ├── screen_dashboard.c/.h
    ├── screen_settings.c/.h
    ├── screen_edit.c/.h
    ├── screen_tests.c/.h
    ├── screen_actions.c/.h
    ├── screen_confirm.c/.h
    └── screen_info.c/.h
```

---

## Approved Dashboard Layout (128×64)

```
┌────────────────────────────────┐ y=0..8   Status bar
│⚡▊▊▊▊ 83% 7.4V  [IDLE]  )))   │          Battery icon+bars+%+V, mode, WiFi
├────────────────────────────────┤ y=9..33  Sensor bars (24px height)
│L2 L1 L0          R0 R1 R2     │          6× VL53L0X vertical bars
│■  ■  ■            ■  ■  ■     │          bar height = dist/MAX_DIST × 24px
│■  ■  ░            ░  ■  ■     │
│■  ░  ░            ░  ░  ■     │
├────────────────────────────────┤ y=34..63 IMU horizontal scale
│-90° -45°   0°  +45° +90°      │          tick marks + labels (5x7 font)
│  |    |    |    |    |         │
│──────────█─────────────────── │          sliding marker at yaw position
│        -12°                    │          numeric value (9x15 font)
└────────────────────────────────┘
```

---

## Font Plan

| Usage | Font |
|-------|------|
| Status bar, tick labels, small text | `u8g2_font_5x7_tr` |
| Menu items, sensor labels | `u8g2_font_6x10_tr` |
| IMU numeric value, parameter values | `u8g2_font_9x15_tr` |
| Icons (8px: battery, WiFi) | `u8g2_font_open_iconic_all_1x_t` |

---

## HAL Design

Two callbacks bridge u8g2 to Zephyr:

### `u8x8_byte_zephyr_hw_i2c()`

```
START_TRANSFER  →  k_mutex_lock(&i2c0_mutex, K_MSEC(50))
                   on timeout: return 0 (skip frame, no crash)
BYTE_SEND       →  memcpy into i2c_buf[1026] with overflow guard
END_TRANSFER    →  i2c_write(dev, buf, len, addr >> 1)
                   k_mutex_unlock(&i2c0_mutex)
```

Address handling: u8g2 stores 8-bit write address (0x78), Zephyr `i2c_write` takes
7-bit (0x3C) → right-shift by 1 in END_TRANSFER.

### `u8x8_gpio_and_delay_zephyr()`

- `DELAY_MILLI` → `k_msleep()`
- `DELAY_I2C` → `k_busy_wait(2)` (2µs for 400 kHz)
- `GPIO_RESET` → no-op (RST pin not wired on this board)

### I2C Contention Analysis

| Thread | Mutex hold time | Period | Duty |
|--------|----------------|--------|------|
| display (priority 8) | ~10 ms | 120 ms | 8.3% |
| IMU/control (priority 2) | ~2 ms | 40 ms | 5.0% |

Non-overlapping by mutex exclusion. 50 ms timeout gives >20× IMU hold time margin.
No deadlock possible: neither thread nests the mutex.

---

## Screen Module Interface

```c
// display_internal.h — only included by display.c and screens/
struct ui_state {
    int   cur_scr, prev_scr;
    int   sel, scroll;
    int   grp_sel, param_idx;
    float edit_val;
    int   test_idx, action_idx;
    bool  confirm_yes;
    const char *confirm_msg;
    int   info_scroll;
};
```

Each screen module exposes one draw function taking `const struct ui_state *`.
`screen_dashboard_draw()` takes no arguments (reads sensor/IMU/battery directly).
All draw functions call `u8g2_SendBuffer()` at the end.

`display.c` owns navigation state mutation (`handle_input()`). Screen modules
are **read-only renderers** — no state side effects.

---

## Files Modified

| File | Change |
|------|--------|
| `CMakeLists.txt` | Add `modules/u8g2_zephyr` to ZEPHYR_EXTRA_MODULES, add new sources |
| `prj.conf` | Remove CONFIG_SSD1306/DISPLAY/CFB, add CONFIG_U8G2=y |
| `boards/rpi_pico2_rp2350a_m33.overlay` | Disable ssd1306 node, remove chosen zephyr,display |
| `src/display.c` | Full rewrite: replace CFB with u8g2, 930→~320 lines |
| `src/display.h` | **Unchanged** |

---

## Verification Checklist

- [ ] `west build` → zero errors, zero warnings
- [ ] `grep -r "cfb_" src/` → empty
- [ ] `grep -r "CONFIG_CHARACTER_FRAMEBUFFER\|CONFIG_SSD1306" prj.conf` → empty
- [ ] Hardware: encoder double-click → display wakes, dashboard renders
- [ ] Sensor bars respond to hand near VL53L0X sensors
- [ ] IMU slider moves when robot rotates
- [ ] Menu navigation: rotate/click/hold all work correctly
- [ ] Auto-sleep after 10s inactivity
- [ ] `$START` WiFi command → display sleeps (car_is_running=true)
