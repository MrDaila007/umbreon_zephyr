/*
 * screen_dashboard.c — Main status screen with sensor bars and IMU scale
 *
 * Layout (128×64):
 *   y=0..8   Status bar: battery (icon + bars + % + V), mode, WiFi
 *   y=9..33  Sensor bars: 6× VL53L0X vertical bars (left group | right group)
 *   y=34..63 IMU horizontal scale: tick marks + sliding marker + numeric value
 */

#include "screen_dashboard.h"
#include "../display_hal.h"
#include "../display_internal.h"
#include "../battery.h"
#include "../sensors.h"
#include "../imu.h"
#include "../control.h"
#include "../settings.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ─── Layout constants ───────────────────────────────────────────────────── */
#define STATUS_Y       0
#define SENSOR_Y      10
#define SENSOR_H      22   /* bar height in pixels */
#define IMU_Y         36
#define IMU_SCALE_Y   40   /* tick line y */
#define IMU_MARKER_Y  41   /* marker top */
#define IMU_VALUE_Y   52   /* numeric value y */

#define BAR_W          8   /* each sensor bar width */
#define BAR_GAP        4   /* gap between bars */
#define LEFT_X         4   /* x start of left sensor group */
#define RIGHT_X       80   /* x start of right sensor group */
#define CENTER_X      64   /* display center for IMU marker */

/* IMU marker half-width in pixels (covers ±90°) */
#define IMU_HALF_W    58

/* ─── Battery percentage (2S LiPo: 8.4V=100%, 6.6V=0%) ─────────────────── */
static int bat_pct(float v)
{
	const float full = 8.4f;
	const float empty = 6.6f;
	if (v >= full)  return 100;
	if (v <= empty) return 0;
	return (int)((v - empty) / (full - empty) * 100.0f);
}

/* ─── Status bar ─────────────────────────────────────────────────────────── */
static void draw_status_bar(void)
{
	char buf[24];
	float vbat = battery_get_voltage();
	int pct = bat_pct(vbat);

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);

	/* Battery: filled blocks proportional to charge */
	const int blocks = (pct + 19) / 20; /* 0..5 filled blocks */
	buf[0] = '[';
	for (int i = 0; i < 5; i++) buf[1 + i] = (i < blocks) ? '#' : '.';
	buf[6] = ']';
	buf[7] = '\0';
	u8g2_DrawStr(&u8g2, 0, 7, buf);

	/* Voltage + percent */
	snprintf(buf, sizeof(buf), "%d%% %.1fV", pct, (double)vbat);
	u8g2_DrawStr(&u8g2, 38, 7, buf);

	/* Mode (right-aligned area) */
	const char *mode = control_is_running() ? "RUN " : "IDLE";
	u8g2_DrawStr(&u8g2, 96, 7, mode);
}

/* ─── Sensor bars ────────────────────────────────────────────────────────── */
static void draw_sensor_bars(void)
{
	const int *dist = sensors_get_distances();
	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);

	/*
	 * Physical layout: sensors 5,4,3 face left wall, sensors 0,1,2 face right.
	 * Labels (sns_short in display.c): {"HR","FR","R ","L ","FL","HL"}
	 * Left group bars from left to right: HL(5), FL(4), L(3)
	 * Right group bars from left to right: R(2), FR(1), HR(0)
	 */
	static const int left_idx[]  = {5, 4, 3};
	static const char *left_lbl[] = {"HL", "FL", "L "};
	static const int right_idx[] = {2, 1, 0};
	static const char *right_lbl[] = {"R ", "FR", "HR"};

	for (int i = 0; i < 3; i++) {
		int x = LEFT_X + i * (BAR_W + BAR_GAP);
		int d = dist[left_idx[i]];
		int h = (d >= MAX_SENSOR_RANGE) ? 0 : (d * SENSOR_H / MAX_SENSOR_RANGE);

		/* Bar outline */
		u8g2_DrawFrame(&u8g2, x, SENSOR_Y, BAR_W, SENSOR_H);
		/* Bar fill from bottom */
		if (h > 0) {
			u8g2_DrawBox(&u8g2, x + 1, SENSOR_Y + SENSOR_H - h,
				     BAR_W - 2, h);
		}
		/* Label below bar */
		u8g2_DrawStr(&u8g2, x, SENSOR_Y + SENSOR_H + 7, left_lbl[i]);
	}

	for (int i = 0; i < 3; i++) {
		int x = RIGHT_X + i * (BAR_W + BAR_GAP);
		int d = dist[right_idx[i]];
		int h = (d >= MAX_SENSOR_RANGE) ? 0 : (d * SENSOR_H / MAX_SENSOR_RANGE);

		u8g2_DrawFrame(&u8g2, x, SENSOR_Y, BAR_W, SENSOR_H);
		if (h > 0) {
			u8g2_DrawBox(&u8g2, x + 1, SENSOR_Y + SENSOR_H - h,
				     BAR_W - 2, h);
		}
		u8g2_DrawStr(&u8g2, x, SENSOR_Y + SENSOR_H + 7, right_lbl[i]);
	}
}

/* ─── IMU horizontal scale ───────────────────────────────────────────────── */
static void draw_imu_scale(void)
{
	float yaw = imu_get_heading();

	/* Clamp to ±90° for display */
	if (yaw >  90.0f) yaw =  90.0f;
	if (yaw < -90.0f) yaw = -90.0f;

	/* Baseline */
	u8g2_DrawHLine(&u8g2, 0, IMU_SCALE_Y, SCR_W);

	/* Tick marks at -90, -45, 0, +45, +90 */
	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	const int ticks[] = {-90, -45, 0, 45, 90};
	const char *tick_lbl[] = {"-90", "-45", "0", "45", "90"};

	for (int i = 0; i < 5; i++) {
		int tx = CENTER_X + (int)(ticks[i] * IMU_HALF_W / 90);
		u8g2_DrawVLine(&u8g2, tx, IMU_SCALE_Y - 3, 6);
		/* Center label under tick */
		int lw = u8g2_GetStrWidth(&u8g2, tick_lbl[i]);
		u8g2_DrawStr(&u8g2, tx - lw / 2, IMU_SCALE_Y + 10, tick_lbl[i]);
	}

	/* Sliding marker */
	int mx = CENTER_X + (int)(yaw * IMU_HALF_W / 90.0f);
	u8g2_DrawBox(&u8g2, mx - 2, IMU_MARKER_Y, 5, 5);

	/* Numeric value */
	char buf[12];
	snprintf(buf, sizeof(buf), "%.1f", (double)yaw);
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	int vw = u8g2_GetStrWidth(&u8g2, buf);
	u8g2_DrawStr(&u8g2, (SCR_W - vw) / 2, IMU_VALUE_Y + 10, buf);
}

/* ─── Public entry point ─────────────────────────────────────────────────── */
void screen_dashboard_draw(void)
{
	draw_status_bar();
	u8g2_DrawHLine(&u8g2, 0, 8, SCR_W);
	draw_sensor_bars();
	u8g2_DrawHLine(&u8g2, 0, IMU_Y - 2, SCR_W);
	draw_imu_scale();

	u8g2_SendBuffer(&u8g2);
}
