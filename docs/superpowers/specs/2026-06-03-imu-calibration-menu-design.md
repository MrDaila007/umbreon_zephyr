# IMU Calibration Menu — Design Spec

**Date:** 2026-06-03
**Branch:** feature/zephyr-v4.4

## Overview

Add gyroscope and accelerometer calibration options to the display menu. User selects a calibration from the Actions menu, waits 5 seconds (countdown shown), calibration runs (~1s), result is shown for 2 seconds, then returns to menu.

## Placement

Both options are added to the existing **Actions** menu (alongside Start/Stop/Save/Load/Reset). No confirm screen — calibrations do not involve motors. ACTION_COUNT increases from 5 to 7.

New entries:
- "Gyro Cal" → `ACT_GYRO_CAL`
- "Accel Cal" → `ACT_ACCEL_CAL`

## New Screen: `SCR_CALIBRATING`

Three sequential phases, tracked via `cal_phase` in `ui_state`:

| Phase | Duration | Display |
|-------|----------|---------|
| `CAL_COUNTDOWN` | 5 seconds | "Keep still" + countdown "5…4…3…2…1" |
| `CAL_RUNNING` | ~1s (blocking) | "Calibrating…" |
| `CAL_DONE` | 2 seconds | "Done!" + result line (e.g. "bias: -0.12/s") |

After `CAL_DONE` expires, `go_back()` returns to Actions. Click during any phase cancels and goes back.

Phase transitions are driven by `process_calibration()` called at the top of the display thread loop (before `handle_input`), using `k_uptime_get()` for elapsed time. The calibration call is blocking (~1s) and runs inline in the display thread — acceptable given the short duration.

## State Fields Added to `ui_state`

```c
int       cal_type;           /* 0=gyro, 1=accel */
int       cal_phase;          /* CAL_COUNTDOWN / CAL_RUNNING / CAL_DONE */
int64_t   cal_phase_start_ms; /* k_uptime_get() at phase entry */
char      cal_result[24];     /* result string shown on CAL_DONE */
```

`cal_phase` uses a local `enum cal_phase { CAL_COUNTDOWN, CAL_RUNNING, CAL_DONE }` defined in `display.c`.

## IMU Changes (`imu.h` / `imu.c`)

### New public functions

```c
/* Calibrate accelerometer: average N samples of SENSOR_CHAN_ACCEL_XYZ,
 * store per-axis bias. Same sample count and delay as imu_calibrate(). */
void imu_calibrate_accel(void);

/* Expose calibration results for display */
float imu_get_gyro_bias(void);
void  imu_get_accel_bias(float *x, float *y, float *z);
```

### `imu_calibrate_accel()` logic

- If `!mpu_ok`: return immediately
- Read `SENSOR_CHAN_ACCEL_XYZ` for `CAL_SAMPLES` (200) iterations with `CAL_DELAY_MS` (5ms) between — matches gyro calibration
- Store average per axis in `static float accel_bias[3]`
- Log result

Bias values are stored in RAM only (not in NVS/settings). They reset on reboot. This matches the existing gyro bias behaviour.

## Files

| File | Change |
|------|--------|
| `src/display_internal.h` | + `SCR_CALIBRATING` in `enum screen`; + cal fields in `struct ui_state`; `ACTION_COUNT` 5 → 7 |
| `src/display.c` | + 2 action items; `process_calibration()`; `SCR_CALIBRATING` in `screen_name()`, `go_back()`, `handle_input()`, `draw_current_screen()` |
| `src/imu.h` | + 3 function declarations |
| `src/imu.c` | + `accel_bias[3]`; implement `imu_calibrate_accel()`, `imu_get_gyro_bias()`, `imu_get_accel_bias()` |
| `src/screens/screen_calibrating.c` | new — renders countdown / running / done |
| `src/screens/screen_calibrating.h` | new — declares `screen_calibrating_draw()` |

## Out of Scope

- Persisting bias to NVS (can be added later)
- Using accel bias in `imu_update()` (accel not currently used in control)
- Confirm dialog before calibration start
