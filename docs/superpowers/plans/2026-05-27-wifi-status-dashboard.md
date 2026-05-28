# WiFi Status Strip on Dashboard — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Parse `#WIFISTATUS` replies from the ESP module and render a WiFi signal icon + mode/RSSI strip at the bottom of the dashboard screen.

**Architecture:** Add state vars + `parse_wifi_status_line()` to `wifi_cmd.c`; expose 3 getters; switch `k_poll` to 10 s timeout for periodic re-polling; update `screen_dashboard.c` to shrink the IMU numeric font (9×15 → 5×7, drop tick labels) and draw the WiFi strip at y=57–63.

**Tech Stack:** Zephyr RTOS, u8g2 display library, RP2350 / Cortex-M33

---

## File Map

| File | Change |
|------|--------|
| `src/wifi_cmd.c` | +3 volatile state vars, `parse_wifi_status_line()`, call site, k_poll timeout, 3 getters |
| `src/wifi_cmd.h` | +3 getter declarations |
| `src/screens/screen_dashboard.c` | `draw_imu_scale()` — drop tick labels, 9×15→5×7 numeric; add `rssi_to_bars()` + `draw_wifi_strip()`; update `screen_dashboard_draw()` |
| `tests/test_unit.c` | Tests for `parse_wifi_status_line` and `rssi_to_bars` |

---

## Task 1 — WiFi state + parser in wifi_cmd.c

**Files:**
- Modify: `src/wifi_cmd.c`
- Modify: `src/wifi_cmd.h`

- [ ] **Step 1.1 — Add state vars to wifi_cmd.c**

  After the `static volatile bool log_on;` line (~line 74), add:

  ```c
  /* ─── WiFi connection state (parsed from ESP #WIFISTATUS replies) ────────── */
  static volatile bool ws_ready;
  static volatile bool ws_is_ap;
  static volatile int  ws_rssi;
  ```

- [ ] **Step 1.2 — Add parse_wifi_status_line() to wifi_cmd.c**

  Add the following static function directly before `dispatch_command()` (~line 89):

  ```c
  static void parse_wifi_status_line(const char *line)
  {
  	/* line format: "# Key:  value" (from wifi_manager_get_status on ESP) */
  	const char *rest = line + 2; /* skip leading "# " */
  	if (strncmp(rest, "Mode:", 5) == 0) {
  		const char *val = rest + 5;
  		while (*val == ' ') val++;
  		ws_is_ap = (strncmp(val, "AP", 2) == 0);
  	} else if (strncmp(rest, "RSSI:", 5) == 0) {
  		const char *val = rest + 5;
  		while (*val == ' ') val++;
  		ws_rssi = atoi(val);
  	} else if (strncmp(rest, "Status:", 7) == 0) {
  		const char *val = rest + 7;
  		while (*val == ' ') val++;
  		ws_ready = (strncmp(val, "ready", 5) == 0);
  	}
  }
  ```

- [ ] **Step 1.3 — Call parse_wifi_status_line() in the RX loop**

  In `wifi_cmd_thread()` (~line 800), find:

  ```c
  			cmd_buf[cmd_len] = '\0';
  			if (cmd_buf[0] == '$') {
  				dispatch_command(cmd_buf);
  			}
  			cmd_len = 0;
  ```

  Replace with:

  ```c
  			cmd_buf[cmd_len] = '\0';
  			if (cmd_buf[0] == '$') {
  				dispatch_command(cmd_buf);
  			} else if (cmd_buf[0] == '#' && cmd_len >= 2 &&
  				   cmd_buf[1] == ' ') {
  				parse_wifi_status_line(cmd_buf);
  			}
  			cmd_len = 0;
  ```

- [ ] **Step 1.4 — Switch k_poll to 10 s timeout for periodic re-poll**

  Find (~line 787):

  ```c
  	while (1) {
  		k_poll(poll_events, 2, K_FOREVER);

  		/* Reset poll event states */
  		poll_events[0].state = K_POLL_STATE_NOT_READY;
  ```

  Replace with:

  ```c
  	while (1) {
  		int poll_ret = k_poll(poll_events, 2, K_SECONDS(10));
  		if (poll_ret == -EAGAIN) {
  			wifi_cmd_send("#WIFISTATUS\n");
  			continue;
  		}

  		/* Reset poll event states */
  		poll_events[0].state = K_POLL_STATE_NOT_READY;
  ```

- [ ] **Step 1.5 — Add public getters at the end of wifi_cmd.c**

  After `wifi_cmd_init()`, add:

  ```c
  bool wifi_status_is_ready(void) { return ws_ready; }
  bool wifi_status_is_ap(void)    { return ws_is_ap; }
  int  wifi_status_get_rssi(void) { return ws_rssi; }
  ```

- [ ] **Step 1.6 — Declare getters in wifi_cmd.h**

  After the `bool wifi_log_enabled(void);` line, add:

  ```c
  /* WiFi connection state — updated from ESP #WIFISTATUS replies */
  bool wifi_status_is_ready(void);
  bool wifi_status_is_ap(void);
  int  wifi_status_get_rssi(void);
  ```

- [ ] **Step 1.7 — Verify build**

  ```bash
  make build 2>&1 | tail -5
  ```

  Expected: `[248/248] Linking C executable zephyr/zephyr.elf` (or similar, no errors)

- [ ] **Step 1.8 — Commit**

  ```bash
  git add src/wifi_cmd.c src/wifi_cmd.h
  git commit -m "feat: parse ESP #WIFISTATUS replies and expose wifi_status getters"
  ```

---

## Task 2 — Unit tests for parse_wifi_status_line and rssi_to_bars

**Files:**
- Modify: `tests/test_unit.c`

- [ ] **Step 2.1 — Write failing tests: add test functions to test_unit.c**

  Add the following block after the last TEST block (before `int main(void)`):

  ```c
  /* ═══════════════════════════════════════════════════════════════════════════
   * Tests: parse_wifi_status_line (copy from wifi_cmd.c)
   * ═══════════════════════════════════════════════════════════════════════════ */

  static volatile bool test_ws_ready;
  static volatile bool test_ws_is_ap;
  static volatile int  test_ws_rssi;

  static void test_parse_wifi_status_line(const char *line)
  {
  	const char *rest = line + 2;
  	if (strncmp(rest, "Mode:", 5) == 0) {
  		const char *val = rest + 5;
  		while (*val == ' ') val++;
  		test_ws_is_ap = (strncmp(val, "AP", 2) == 0);
  	} else if (strncmp(rest, "RSSI:", 5) == 0) {
  		const char *val = rest + 5;
  		while (*val == ' ') val++;
  		test_ws_rssi = atoi(val);
  	} else if (strncmp(rest, "Status:", 7) == 0) {
  		const char *val = rest + 7;
  		while (*val == ' ') val++;
  		test_ws_ready = (strncmp(val, "ready", 5) == 0);
  	}
  }

  /* From screen_dashboard.c — copy of rssi_to_bars */
  static int test_rssi_to_bars(int rssi)
  {
  	if (rssi >= -60) return 4;
  	if (rssi >= -70) return 3;
  	if (rssi >= -80) return 2;
  	if (rssi >= -90) return 1;
  	return 0;
  }

  TEST(test_wifi_parse_mode_sta)
  {
  	test_ws_is_ap = true;
  	test_parse_wifi_status_line("# Mode:  STA");
  	ASSERT_FALSE(test_ws_is_ap);
  }

  TEST(test_wifi_parse_mode_ap)
  {
  	test_ws_is_ap = false;
  	test_parse_wifi_status_line("# Mode:  AP");
  	ASSERT_TRUE(test_ws_is_ap);
  }

  TEST(test_wifi_parse_rssi_negative)
  {
  	test_ws_rssi = 0;
  	test_parse_wifi_status_line("# RSSI:  -65");
  	ASSERT_EQ(test_ws_rssi, -65);
  }

  TEST(test_wifi_parse_rssi_strong)
  {
  	test_ws_rssi = 0;
  	test_parse_wifi_status_line("# RSSI:  -42");
  	ASSERT_EQ(test_ws_rssi, -42);
  }

  TEST(test_wifi_parse_status_ready)
  {
  	test_ws_ready = false;
  	test_parse_wifi_status_line("# Status: ready");
  	ASSERT_TRUE(test_ws_ready);
  }

  TEST(test_wifi_parse_status_other)
  {
  	test_ws_ready = true;
  	test_parse_wifi_status_line("# Status: other");
  	ASSERT_FALSE(test_ws_ready);
  }

  TEST(test_wifi_parse_unknown_key_ignored)
  {
  	test_ws_ready = false;
  	test_ws_rssi  = -55;
  	test_ws_is_ap = true;
  	test_parse_wifi_status_line("# SSID:  umbreon");
  	ASSERT_FALSE(test_ws_ready);
  	ASSERT_EQ(test_ws_rssi, -55);
  	ASSERT_TRUE(test_ws_is_ap);
  }

  TEST(test_rssi_bars_excellent)
  {
  	ASSERT_EQ(test_rssi_to_bars(-55), 4);
  	ASSERT_EQ(test_rssi_to_bars(-60), 4);
  }

  TEST(test_rssi_bars_good)
  {
  	ASSERT_EQ(test_rssi_to_bars(-61), 3);
  	ASSERT_EQ(test_rssi_to_bars(-70), 3);
  }

  TEST(test_rssi_bars_fair)
  {
  	ASSERT_EQ(test_rssi_to_bars(-71), 2);
  	ASSERT_EQ(test_rssi_to_bars(-80), 2);
  }

  TEST(test_rssi_bars_poor)
  {
  	ASSERT_EQ(test_rssi_to_bars(-81), 1);
  	ASSERT_EQ(test_rssi_to_bars(-90), 1);
  }

  TEST(test_rssi_bars_none)
  {
  	ASSERT_EQ(test_rssi_to_bars(-91), 0);
  	ASSERT_EQ(test_rssi_to_bars(-100), 0);
  }
  ```

- [ ] **Step 2.2 — Add RUN_TEST calls to main() in test_unit.c**

  In `main()`, before `TEST_SUMMARY()`, add:

  ```c
  	printf("\n[parse_wifi_status_line]\n");
  	RUN_TEST(test_wifi_parse_mode_sta);
  	RUN_TEST(test_wifi_parse_mode_ap);
  	RUN_TEST(test_wifi_parse_rssi_negative);
  	RUN_TEST(test_wifi_parse_rssi_strong);
  	RUN_TEST(test_wifi_parse_status_ready);
  	RUN_TEST(test_wifi_parse_status_other);
  	RUN_TEST(test_wifi_parse_unknown_key_ignored);

  	printf("\n[rssi_to_bars]\n");
  	RUN_TEST(test_rssi_bars_excellent);
  	RUN_TEST(test_rssi_bars_good);
  	RUN_TEST(test_rssi_bars_fair);
  	RUN_TEST(test_rssi_bars_poor);
  	RUN_TEST(test_rssi_bars_none);
  ```

- [ ] **Step 2.3 — Run tests (expect PASS)**

  ```bash
  make -C tests test-host 2>&1
  ```

  Expected output includes:
  ```
  [parse_wifi_status_line]
    PASS  test_wifi_parse_mode_sta
    PASS  test_wifi_parse_mode_ap
    ...
  [rssi_to_bars]
    PASS  test_rssi_bars_excellent
    ...
  X/X tests passed
  ```

- [ ] **Step 2.4 — Commit**

  ```bash
  git add tests/test_unit.c
  git commit -m "test: add unit tests for parse_wifi_status_line and rssi_to_bars"
  ```

---

## Task 3 — Update screen_dashboard.c

**Files:**
- Modify: `src/screens/screen_dashboard.c`

- [ ] **Step 3.1 — Add wifi_cmd.h include**

  After the existing `#include` block (after `#include <math.h>`), add:

  ```c
  #include "../wifi_cmd.h"
  ```

- [ ] **Step 3.2 — Update IMU_VALUE_Y constant**

  Find:

  ```c
  #define IMU_VALUE_Y   52   /* numeric value y */
  ```

  Replace with:

  ```c
  #define IMU_VALUE_Y   53   /* numeric value baseline (5×7 font) */
  ```

- [ ] **Step 3.3 — Rewrite draw_imu_scale() — drop tick labels, use 5×7 font**

  Replace the entire `draw_imu_scale()` function with:

  ```c
  static void draw_imu_scale(void)
  {
  	float yaw = imu_get_heading();

  	if (yaw >  90.0f) yaw =  90.0f;
  	if (yaw < -90.0f) yaw = -90.0f;

  	/* Baseline */
  	u8g2_DrawHLine(&u8g2, 0, IMU_SCALE_Y, SCR_W);

  	/* Tick marks at -90, -45, 0, +45, +90 (no text labels — space for WiFi strip) */
  	const int ticks[] = {-90, -45, 0, 45, 90};
  	for (int i = 0; i < 5; i++) {
  		int tx = CENTER_X + (int)(ticks[i] * IMU_HALF_W / 90);
  		u8g2_DrawVLine(&u8g2, tx, IMU_SCALE_Y - 3, 6);
  	}

  	/* Sliding marker */
  	int mx = CENTER_X + (int)(yaw * IMU_HALF_W / 90.0f);
  	u8g2_DrawBox(&u8g2, mx - 2, IMU_MARKER_Y, 5, 5);

  	/* Numeric value — 5×7 font, baseline IMU_VALUE_Y */
  	char buf[12];
  	snprintf(buf, sizeof(buf), "%.1f", (double)yaw);
  	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
  	int vw = u8g2_GetStrWidth(&u8g2, buf);
  	u8g2_DrawStr(&u8g2, (SCR_W - vw) / 2, IMU_VALUE_Y, buf);
  }
  ```

- [ ] **Step 3.4 — Add rssi_to_bars() and draw_wifi_strip() before screen_dashboard_draw()**

  Insert the following two functions immediately before `screen_dashboard_draw()`:

  ```c
  static int rssi_to_bars(int rssi)
  {
  	if (rssi >= -60) return 4;
  	if (rssi >= -70) return 3;
  	if (rssi >= -80) return 2;
  	if (rssi >= -90) return 1;
  	return 0;
  }

  static void draw_wifi_strip(void)
  {
  	if (!wifi_status_is_ready()) {
  		return;
  	}

  	u8g2_DrawHLine(&u8g2, 0, 56, SCR_W);

  	bool is_ap = wifi_status_is_ap();
  	int  rssi  = wifi_status_get_rssi();
  	int  bars  = is_ap ? 4 : rssi_to_bars(rssi);

  	/* 4 signal bars, increasing height (2/3/5/7 px), anchored at y=63 */
  	static const uint8_t bar_h[] = {2, 3, 5, 7};
  	static const uint8_t bar_x[] = {2, 5, 8, 11};

  	for (int i = 0; i < 4; i++) {
  		int bh = bar_h[i];
  		int bx = bar_x[i];
  		int by = 64 - bh;
  		if (i < bars) {
  			u8g2_DrawBox(&u8g2, bx, by, 2, bh);
  		} else {
  			u8g2_DrawPixel(&u8g2, bx, by);
  		}
  	}

  	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
  	char buf[16];
  	if (is_ap) {
  		snprintf(buf, sizeof(buf), "AP");
  	} else {
  		snprintf(buf, sizeof(buf), "STA %d", rssi);
  	}
  	u8g2_DrawStr(&u8g2, 16, 63, buf);
  }
  ```

- [ ] **Step 3.5 — Call draw_wifi_strip() from screen_dashboard_draw()**

  Find:

  ```c
  void screen_dashboard_draw(void)
  {
  	draw_status_bar();
  	u8g2_DrawHLine(&u8g2, 0, 8, SCR_W);
  	draw_sensor_bars();
  	u8g2_DrawHLine(&u8g2, 0, IMU_Y - 2, SCR_W);
  	draw_imu_scale();

  	u8g2_SendBuffer(&u8g2);
  }
  ```

  Replace with:

  ```c
  void screen_dashboard_draw(void)
  {
  	draw_status_bar();
  	u8g2_DrawHLine(&u8g2, 0, 8, SCR_W);
  	draw_sensor_bars();
  	u8g2_DrawHLine(&u8g2, 0, IMU_Y - 2, SCR_W);
  	draw_imu_scale();
  	draw_wifi_strip();

  	u8g2_SendBuffer(&u8g2);
  }
  ```

- [ ] **Step 3.6 — Build**

  ```bash
  make build 2>&1 | tail -5
  ```

  Expected: no errors, `zephyr.elf` linked successfully.

- [ ] **Step 3.7 — Commit**

  ```bash
  git add src/screens/screen_dashboard.c
  git commit -m "feat: add WiFi status strip to dashboard bottom"
  ```

---

## Task 4 — Flash and verify

- [ ] **Step 4.1 — Flash**

  ```bash
  make flash-probe 2>&1 | tail -5
  ```

  Expected: `** Verified OK ** ... ** Resetting Target **`

- [ ] **Step 4.2 — Manual check**

  On the dashboard screen, verify:
  - Before WiFi connects (~first 10 s): strip is absent, IMU section looks normal.
  - After ESP sends `#WIFISTATUS` reply: strip appears with signal bars + `"STA -XX"` or `"AP"`.
  - IMU tick marks and sliding marker still work; numeric value shows heading in small font.
  - No visual overlap between IMU section and WiFi strip.
