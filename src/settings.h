#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ─── Runtime-configurable parameters (31 total) ──────────────────────────── */
struct car_settings {
	/* Obstacle thresholds (cm×10) */
	int front_obstacle_dist;
	int side_open_dist;
	int all_close_dist;
	int close_front_dist;

	/* PID coefficients */
	float pid_kp;
	float pid_ki;
	float pid_kd;

	/* ESC limits (µs, 1000–2000, neutral 1500) */
	int min_speed;
	int max_speed;
	int min_bspeed;

	/* Steering limits (servo degrees) */
	int min_point;
	int max_point;
	int neutral_point;

	/* Tachometer */
	int encoder_holes;
	float wheel_diam_m;

	/* Control loop */
	int loop_ms;

	/* Speed/steer coefficients */
	float spd_clear;
	float spd_blocked;
	float coe_clear;
	float coe_blocked;

	/* Navigation */
	float wrong_dir_deg;
	bool race_cw;
	int stuck_thresh;

	/* IMU */
	bool imu_rotate;

	/* Servo */
	bool servo_reverse;

	/* Calibration */
	bool calibrated;

	/* Battery */
	bool bat_enabled;
	float bat_multiplier;
	float bat_low;
};

/* Global configuration instance */
extern struct car_settings cfg;

/* ─── Sensor constants ────────────────────────────────────────────────────── */
#define SENSOR_COUNT        6
#define IDX_HARD_RIGHT      0
#define IDX_FRONT_RIGHT     1
#define IDX_RIGHT           2
#define IDX_LEFT            3
#define IDX_FRONT_LEFT      4
#define IDX_HARD_LEFT       5
#define MAX_SENSOR_RANGE    2000  /* cm×10 (200 cm) */

/* Default thresholds (cm×10) — tighter for short-range VL53L0X */
#define DEFAULT_FOD         800
#define DEFAULT_SOD         600
#define DEFAULT_ACD         400
#define DEFAULT_CFD         150

/* ─── ESC constants ───────────────────────────────────────────────────────── */
#define NEUTRAL_SPEED       1500  /* µs — never changes */

/* ─── API ─────────────────────────────────────────────────────────────────── */
void settings_init(void);
bool settings_load(void);
bool settings_save(void);
void settings_reset(void);
