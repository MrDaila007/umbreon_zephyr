# IMU Calibration Menu Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add "Gyro Cal" and "Accel Cal" options to the Actions menu; selecting one shows a 5-second countdown, runs calibration, then shows the result for 2 seconds.

**Architecture:** Two new `enum action_id` entries route to a new `SCR_CALIBRATING` screen. A `process_calibration()` function called each display frame advances the countdown → running → done state machine using `k_uptime_get()`. The blocking calibration call (~1s) runs inline in the display thread. A new `src/screens/screen_calibrating.c` renders each phase. New imu functions expose calibration results for display.

**Tech Stack:** Zephyr RTOS, u8g2 display library, MPU-6050 via Zephyr sensor driver, host-based C unit tests (test_runner.h)

---

## File Map

| File | Change |
|------|--------|
| `src/imu.h` | + `imu_calibrate_accel()`, `imu_get_gyro_bias()`, `imu_get_accel_bias()` |
| `src/imu.c` | + `accel_bias[3]`, implement 3 new functions |
| `src/display_internal.h` | + `SCR_CALIBRATING`, `enum cal_phase`, cal fields in `ui_state`, `ACTION_COUNT` 5→7 |
| `src/display.c` | + 2 action items, `process_calibration()`, routing for new screen |
| `src/screens/screen_calibrating.h` | new — declares `screen_calibrating_draw()` |
| `src/screens/screen_calibrating.c` | new — renders countdown / calibrating / done |
| `tests/test_unit.c` | + 4 tests for imu accessor logic |

---

## Task 1: Add imu accessor functions and accel calibration

**Files:**
- Modify: `src/imu.h`
- Modify: `src/imu.c`
- Test: `tests/test_unit.c`

- [ ] **Step 1: Write failing tests for imu accessors**

Add this block near the top of `tests/test_unit.c`, after the existing extracted-functions section:

```c
/* ═══════════════════════════════════════════════════════════════════════════
 * IMU accessor logic (extracted static state + functions from imu.c)
 * ═══════════════════════════════════════════════════════════════════════════ */

static float _gyro_bias_test;
static float _accel_bias_test[3];

static float extracted_get_gyro_bias(void) { return _gyro_bias_test; }
static void  extracted_get_accel_bias(float *x, float *y, float *z)
{
    *x = _accel_bias_test[0];
    *y = _accel_bias_test[1];
    *z = _accel_bias_test[2];
}

TEST(test_imu_gyro_bias_zero) {
    _gyro_bias_test = 0.0f;
    ASSERT_FLOAT_EQ(extracted_get_gyro_bias(), 0.0f, 1e-6f);
}

TEST(test_imu_gyro_bias_negative) {
    _gyro_bias_test = -0.12f;
    ASSERT_FLOAT_EQ(extracted_get_gyro_bias(), -0.12f, 1e-5f);
}

TEST(test_imu_accel_bias_zero) {
    _accel_bias_test[0] = 0.0f;
    _accel_bias_test[1] = 0.0f;
    _accel_bias_test[2] = 0.0f;
    float x, y, z;
    extracted_get_accel_bias(&x, &y, &z);
    ASSERT_FLOAT_EQ(x, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(y, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(z, 0.0f, 1e-6f);
}

TEST(test_imu_accel_bias_values) {
    _accel_bias_test[0] =  0.10f;
    _accel_bias_test[1] = -0.20f;
    _accel_bias_test[2] =  9.81f;
    float x, y, z;
    extracted_get_accel_bias(&x, &y, &z);
    ASSERT_FLOAT_EQ(x,  0.10f, 1e-5f);
    ASSERT_FLOAT_EQ(y, -0.20f, 1e-5f);
    ASSERT_FLOAT_EQ(z,  9.81f, 1e-4f);
}
```

Also register them in `main()` of `tests/test_unit.c` (add after the last `RUN_TEST` block):

```c
    printf("\n--- IMU accessor ---\n");
    RUN_TEST(test_imu_gyro_bias_zero);
    RUN_TEST(test_imu_gyro_bias_negative);
    RUN_TEST(test_imu_accel_bias_zero);
    RUN_TEST(test_imu_accel_bias_values);
```

- [ ] **Step 2: Run tests — expect PASS (tests are self-contained)**

```bash
make -C /home/user/Documents/Robots/roborace/umbreon_zephyr/tests && ./tests/test_unit
```

Expected: all 4 new tests PASS (they only test extracted local functions, no Zephyr dependency).

- [ ] **Step 3: Add declarations to `src/imu.h`**

After `void imu_reset_heading(void);`, add:

```c
/* Calibrate accelerometer: average CAL_SAMPLES reads of all axes, store bias */
void  imu_calibrate_accel(void);

/* Expose calibration results for display */
float imu_get_gyro_bias(void);
void  imu_get_accel_bias(float *x, float *y, float *z);
```

- [ ] **Step 4: Add `accel_bias[3]` and implement the three functions in `src/imu.c`**

After the existing `static float heading;` line, add:

```c
static float accel_bias[3]; /* [0]=X, [1]=Y, [2]=Z — m/s² */
```

After `imu_calibrate()` (around line 95), add:

```c
void imu_calibrate_accel(void)
{
	if (!mpu_ok) {
		return;
	}

	LOG_INF("IMU: calibrating accel bias (~1s, keep still)...");

	float sum[3] = {0};
	int count = 0;

	for (int i = 0; i < CAL_SAMPLES; i++) {
		if (sensor_sample_fetch(mpu_dev) != 0) {
			k_msleep(CAL_DELAY_MS);
			continue;
		}
		struct sensor_value val[3];
		if (sensor_channel_get(mpu_dev, SENSOR_CHAN_ACCEL_XYZ, val) != 0) {
			k_msleep(CAL_DELAY_MS);
			continue;
		}
		sum[0] += sensor_value_to_float(&val[0]);
		sum[1] += sensor_value_to_float(&val[1]);
		sum[2] += sensor_value_to_float(&val[2]);
		count++;
		k_msleep(CAL_DELAY_MS);
	}

	if (count > 0) {
		accel_bias[0] = sum[0] / count;
		accel_bias[1] = sum[1] / count;
		accel_bias[2] = sum[2] / count;
	}

	LOG_INF("IMU: accel bias = (%.3f, %.3f, %.3f) m/s² (%d samples)",
		(double)accel_bias[0], (double)accel_bias[1],
		(double)accel_bias[2], count);
}

float imu_get_gyro_bias(void)
{
	return gyro_bias;
}

void imu_get_accel_bias(float *x, float *y, float *z)
{
	*x = accel_bias[0];
	*y = accel_bias[1];
	*z = accel_bias[2];
}
```

- [ ] **Step 5: Run tests again to verify still passing**

```bash
make -C /home/user/Documents/Robots/roborace/umbreon_zephyr/tests && ./tests/test_unit
```

Expected: all tests PASS (imu.c not compiled into host tests, no regression).

- [ ] **Step 6: Commit**

```bash
git add src/imu.h src/imu.c tests/test_unit.c
git commit -m "feat: add imu_calibrate_accel and gyro/accel bias accessors"
```

---

## Task 2: Extend `display_internal.h` with calibration types

**Files:**
- Modify: `src/display_internal.h`

- [ ] **Step 1: Add `SCR_CALIBRATING` to `enum screen`**

In `display_internal.h`, after `SCR_WIFI,` add:

```c
	SCR_CALIBRATING,
```

- [ ] **Step 2: Add `enum cal_phase`**

After the closing brace of `enum screen`, add:

```c
/* ─── Calibration phases ─────────────────────────────────────────────────── */
enum cal_phase { CAL_COUNTDOWN = 0, CAL_RUNNING = 1, CAL_DONE = 2 };
```

- [ ] **Step 3: Add calibration fields to `struct ui_state`**

At the end of `struct ui_state` (before the closing `}`), add:

```c
	/* Calibration screen state */
	int      cal_type;            /* 0=gyro, 1=accel */
	enum cal_phase cal_phase;
	int64_t  cal_phase_start_ms;
	char     cal_result[24];
```

- [ ] **Step 4: Update `ACTION_COUNT` from 5 to 7**

Change:

```c
#define ACTION_COUNT   5
```

to:

```c
#define ACTION_COUNT   7
```

- [ ] **Step 5: Commit**

```bash
git add src/display_internal.h
git commit -m "feat: add SCR_CALIBRATING, cal_phase enum, and cal fields to ui_state"
```

---

## Task 3: Add calibration actions and routing to `display.c`

**Files:**
- Modify: `src/display.c`

- [ ] **Step 1: Extend `enum action_id` in `src/display_internal.h`**

Find in `src/display_internal.h`:

```c
enum action_id { ACT_START, ACT_STOP, ACT_SAVE, ACT_LOAD, ACT_RESET };
```

Change to:

```c
enum action_id { ACT_START, ACT_STOP, ACT_SAVE, ACT_LOAD, ACT_RESET,
		 ACT_GYRO_CAL, ACT_ACCEL_CAL };
```

- [ ] **Step 2: Add two new entries to `actions[]`**

After `{"Reset Defaults", ACT_RESET, true},` add:

```c
	{"Gyro Cal",       ACT_GYRO_CAL,  false},
	{"Accel Cal",      ACT_ACCEL_CAL, false},
```

- [ ] **Step 3: Add `SCR_CALIBRATING` to `screen_name()`**

In `screen_name()`, after `case SCR_WIFI: return "wifi";` add:

```c
	case SCR_CALIBRATING:     return "calibrating";
```

- [ ] **Step 4: Add `SCR_CALIBRATING` to `go_back()`**

In `go_back()`, after `case SCR_WIFI: st.cur_scr = SCR_MAIN_MENU; break;` add:

```c
	case SCR_CALIBRATING:     st.cur_scr = SCR_ACTIONS;      break;
```

- [ ] **Step 5: Add `process_calibration()` function**

Add this function after `go_back()` and before `exec_action()`:

```c
static void process_calibration(void)
{
	int64_t now     = k_uptime_get();
	int64_t elapsed = now - st.cal_phase_start_ms;

	if (st.cal_phase == CAL_COUNTDOWN) {
		if (elapsed >= 5000) {
			st.cal_phase          = CAL_RUNNING;
			st.cal_phase_start_ms = k_uptime_get();
			if (st.cal_type == 0) {
				imu_calibrate();
				float bias = imu_get_gyro_bias();
				snprintf(st.cal_result, sizeof(st.cal_result),
					 "bias: %.2f deg/s", (double)bias);
			} else {
				float bx, by, bz;
				imu_calibrate_accel();
				imu_get_accel_bias(&bx, &by, &bz);
				snprintf(st.cal_result, sizeof(st.cal_result),
					 "%.2f %.2f %.2f", (double)bx, (double)by, (double)bz);
			}
			st.cal_phase          = CAL_DONE;
			st.cal_phase_start_ms = k_uptime_get();
		}
	} else if (st.cal_phase == CAL_DONE) {
		if (elapsed >= 2000) {
			go_back();
		}
	}
}
```

- [ ] **Step 6: Handle calibration actions in `SCR_ACTIONS` input handler**

In `handle_input()`, inside the `case SCR_ACTIONS:` block, after `exec_action(actions[ai].id); go_screen(SCR_DASHBOARD);` add handling for calibration actions. Replace the `else` branch of `if (actions[ai].confirm)` with:

```c
				} else if (actions[ai].id == ACT_GYRO_CAL ||
					   actions[ai].id == ACT_ACCEL_CAL) {
					st.cal_type           = (actions[ai].id == ACT_GYRO_CAL) ? 0 : 1;
					st.cal_phase          = CAL_COUNTDOWN;
					st.cal_phase_start_ms = k_uptime_get();
					st.cal_result[0]      = '\0';
					go_screen(SCR_CALIBRATING);
				} else {
					exec_action(actions[ai].id);
					go_screen(SCR_DASHBOARD);
				}
```

The full `if/else` for `actions[ai].confirm` becomes:

```c
				if (actions[ai].confirm) {
					st.confirm_msg = actions[ai].label;
					st.confirm_yes = false;
					st.prev_scr    = SCR_ACTIONS;
					go_screen(SCR_CONFIRM);
				} else if (actions[ai].id == ACT_GYRO_CAL ||
					   actions[ai].id == ACT_ACCEL_CAL) {
					st.cal_type           = (actions[ai].id == ACT_GYRO_CAL) ? 0 : 1;
					st.cal_phase          = CAL_COUNTDOWN;
					st.cal_phase_start_ms = k_uptime_get();
					st.cal_result[0]      = '\0';
					go_screen(SCR_CALIBRATING);
				} else {
					exec_action(actions[ai].id);
					go_screen(SCR_DASHBOARD);
				}
```

- [ ] **Step 7: Add `SCR_CALIBRATING` input case**

In `handle_input()`, after `case SCR_WIFI:` block, add:

```c
	case SCR_CALIBRATING:
		if (click) {
			go_back();
		}
		break;
```

- [ ] **Step 8: Call `process_calibration()` in the display thread loop**

In `display_thread_fn()`, inside the `if (!running)` block, before `handle_input()`, add:

```c
			if (st.cur_scr == SCR_CALIBRATING) {
				process_calibration();
			}
```

- [ ] **Step 9: Add `SCR_CALIBRATING` to `draw_current_screen()`**

After `case SCR_WIFI: screen_wifi_draw(&st); break;` add:

```c
	case SCR_CALIBRATING: screen_calibrating_draw(&st); break;
```

- [ ] **Step 10: Add `screen_calibrating.h` include at top of `display.c`**

After `#include "screens/screen_wifi.h"` add:

```c
#include "screens/screen_calibrating.h"
```

- [ ] **Step 11: Commit (display.c only — screen not yet compiled)**

```bash
git add src/display.c
git commit -m "feat: add calibration routing and process_calibration in display.c"
```

---

## Task 4: Create `screen_calibrating.c` and `screen_calibrating.h`

**Files:**
- Create: `src/screens/screen_calibrating.h`
- Create: `src/screens/screen_calibrating.c`

- [ ] **Step 1: Create `src/screens/screen_calibrating.h`**

```c
#pragma once

#include "../display_internal.h"

void screen_calibrating_draw(const struct ui_state *st);
```

- [ ] **Step 2: Create `src/screens/screen_calibrating.c`**

```c
/*
 * screen_calibrating.c — IMU calibration progress screen
 *
 * Phases (cal_phase field in ui_state):
 *   CAL_COUNTDOWN — 5s countdown, "Keep still…"
 *   CAL_RUNNING   — blocking cal call in display.c, show "Calibrating…"
 *   CAL_DONE      — show result for 2s, then go_back()
 */

#include "screen_calibrating.h"
#include "../display_hal.h"
#include "../display_internal.h"

#include <u8g2.h>
#include <zephyr/kernel.h>
#include <stdio.h>

void screen_calibrating_draw(const struct ui_state *st)
{
	const char *title = (st->cal_type == 0) ? "Gyro Cal" : "Accel Cal";

	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	int tw = (int)u8g2_GetStrWidth(&u8g2, title);
	u8g2_DrawStr(&u8g2, (SCR_W - tw) / 2, 11, title);
	u8g2_SetDrawColor(&u8g2, 1);

	switch (st->cal_phase) {
	case CAL_COUNTDOWN: {
		int64_t elapsed   = k_uptime_get() - st->cal_phase_start_ms;
		int remaining     = (int)((5000 - elapsed) / 1000) + 1;
		if (remaining < 1) remaining = 1;
		if (remaining > 5) remaining = 5;

		u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
		u8g2_DrawStr(&u8g2, 20, 30, "Keep still...");

		char nbuf[4];
		snprintf(nbuf, sizeof(nbuf), "%d", remaining);
		u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
		int nw = (int)u8g2_GetStrWidth(&u8g2, nbuf);
		u8g2_DrawStr(&u8g2, (SCR_W - nw) / 2, 52, nbuf);

		u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
		u8g2_DrawStr(&u8g2, 22, 62, "Click = Cancel");
		break;
	}
	case CAL_RUNNING:
		u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
		u8g2_DrawStr(&u8g2, 12, 38, "Calibrating...");
		break;

	case CAL_DONE:
		u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
		u8g2_DrawStr(&u8g2, 40, 32, "Done!");

		u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
		int rw = (int)u8g2_GetStrWidth(&u8g2, st->cal_result);
		u8g2_DrawStr(&u8g2, (SCR_W - rw) / 2, 48, st->cal_result);
		break;
	}

	u8g2_SendBuffer(&u8g2);
}
```

- [ ] **Step 3: Commit**

```bash
git add src/screens/screen_calibrating.h src/screens/screen_calibrating.c
git commit -m "feat: add screen_calibrating with countdown/running/done phases"
```

---

## Task 5: Build and verify

- [ ] **Step 1: Build the firmware**

```bash
cd /home/user/Documents/Robots/roborace/umbreon_zephyr && west build -b rpi_pico2/rp2350a/m33 2>&1 | tail -20
```

Expected: `Build successful` with no errors or warnings about undeclared symbols.

- [ ] **Step 2: Run host unit tests**

```bash
make -C /home/user/Documents/Robots/roborace/umbreon_zephyr/tests && ./tests/test_unit
```

Expected: all tests PASS (same count as before + 4 new IMU accessor tests).

- [ ] **Step 3: Commit build verification note (if any fixups were made)**

If no fixups: no commit needed. If fixups: commit with `fix: ...` message describing what was corrected.

---

## Verification Checklist

- [ ] Actions menu shows "Gyro Cal" and "Accel Cal" entries (scrollable — ACTION_COUNT=7, LIST_VISIBLE=5)
- [ ] Selecting "Gyro Cal" shows title "Gyro Cal", "Keep still...", countdown 5→1
- [ ] Selecting "Accel Cal" shows title "Accel Cal", same countdown
- [ ] After 5s: "Calibrating..." appears for ~1s
- [ ] After calibration: "Done!" + bias value shown
- [ ] After 2s: returns to Actions screen
- [ ] Click during countdown: immediately returns to Actions
- [ ] Long-hold during calibration: returns to dashboard (handled by existing held logic)
