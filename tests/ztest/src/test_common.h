/*
 * test_common.h — Shared helpers for ztest unit tests
 *
 * Extracted copies of static inline functions from production code.
 * When source changes — update these or expose originals via #ifdef UNIT_TEST.
 */
#pragma once

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── From tachometer.c:37-41 ─────────────────────────────────────────────── */
static inline uint32_t cyc_delta_us(uint32_t from_cyc, uint32_t to_cyc)
{
	uint32_t delta_cyc = to_cyc - from_cyc;
	return (uint32_t)k_cyc_to_us_floor64(delta_cyc);
}

/* ─── From car.c:49-52 ────────────────────────────────────────────────────── */
static inline int map_val(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ─── Speed formula from tachometer.c:100-101 (extracted, no atomics) ─────── */
static inline float speed_formula(float wheel_diam_m, int encoder_holes,
				  uint32_t elapsed_us)
{
	if (elapsed_us == 0 || elapsed_us > 500000 || encoder_holes <= 0) {
		return 0.0f;
	}
	return ((float)M_PI * wheel_diam_m) /
	       ((float)encoder_holes * ((float)elapsed_us / 1e6f));
}

/* ─── cfg struct (from settings.h) ────────────────────────────────────────── */
#define SENSOR_COUNT     6
#define MAX_SENSOR_RANGE 2000
#define NEUTRAL_SPEED    1500

struct car_settings {
	int front_obstacle_dist;
	int side_open_dist;
	int all_close_dist;
	int close_front_dist;
	float pid_kp;
	float pid_ki;
	float pid_kd;
	int min_speed;
	int max_speed;
	int min_bspeed;
	int min_point;
	int max_point;
	int neutral_point;
	int encoder_holes;
	float wheel_diam_m;
	int loop_ms;
	float spd_clear;
	float spd_blocked;
	float coe_clear;
	float coe_blocked;
	float wrong_dir_deg;
	bool race_cw;
	int stuck_thresh;
	bool imu_rotate;
	bool servo_reverse;
	bool calibrated;
	bool bat_enabled;
	float bat_multiplier;
	float bat_low;
};

/* Global cfg instance — tests reset it in before() */
extern struct car_settings cfg;

/* ─── From wifi_cmd.c:208-256 ─────────────────────────────────────────────── */
static inline bool parse_set_pair(const char *pair)
{
	const char *eq = strchr(pair, '=');
	if (!eq) {
		return false;
	}

	char key[8];
	int klen = eq - pair;
	if (klen <= 0 || klen >= (int)sizeof(key)) {
		return false;
	}
	memcpy(key, pair, klen);
	key[klen] = '\0';
	const char *val = eq + 1;

	if      (strcmp(key, "FOD")  == 0) cfg.front_obstacle_dist = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "SOD")  == 0) cfg.side_open_dist      = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "ACD")  == 0) cfg.all_close_dist      = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "CFD")  == 0) cfg.close_front_dist    = CLAMP(atoi(val), 10, MAX_SENSOR_RANGE);
	else if (strcmp(key, "KP")   == 0) cfg.pid_kp              = strtof(val, NULL);
	else if (strcmp(key, "KI")   == 0) cfg.pid_ki              = strtof(val, NULL);
	else if (strcmp(key, "KD")   == 0) cfg.pid_kd              = strtof(val, NULL);
	else if (strcmp(key, "MSP")  == 0) cfg.min_speed           = CLAMP(atoi(val), 1000, 2000);
	else if (strcmp(key, "XSP")  == 0) cfg.max_speed           = CLAMP(atoi(val), 1000, 2000);
	else if (strcmp(key, "BSP")  == 0) cfg.min_bspeed          = CLAMP(atoi(val), 1000, 2000);
	else if (strcmp(key, "MNP")  == 0) cfg.min_point           = CLAMP(atoi(val), 0, 180);
	else if (strcmp(key, "XNP")  == 0) cfg.max_point           = CLAMP(atoi(val), 0, 180);
	else if (strcmp(key, "NTP")  == 0) cfg.neutral_point       = CLAMP(atoi(val), 0, 180);
	else if (strcmp(key, "ENH")  == 0) cfg.encoder_holes       = MAX(atoi(val), 1);
	else if (strcmp(key, "WDM")  == 0) cfg.wheel_diam_m        = strtof(val, NULL);
	else if (strcmp(key, "LMS")  == 0) cfg.loop_ms             = MAX(atoi(val), 10);
	else if (strcmp(key, "SPD1") == 0) cfg.spd_clear           = strtof(val, NULL);
	else if (strcmp(key, "SPD2") == 0) cfg.spd_blocked         = strtof(val, NULL);
	else if (strcmp(key, "COE1") == 0) cfg.coe_clear           = strtof(val, NULL);
	else if (strcmp(key, "COE2") == 0) cfg.coe_blocked         = strtof(val, NULL);
	else if (strcmp(key, "WDD")  == 0) cfg.wrong_dir_deg       = strtof(val, NULL);
	else if (strcmp(key, "RCW")  == 0) cfg.race_cw             = atoi(val) != 0;
	else if (strcmp(key, "STK")  == 0) cfg.stuck_thresh        = MAX(atoi(val), 0);
	else if (strcmp(key, "IMR")  == 0) cfg.imu_rotate          = atoi(val) != 0;
	else if (strcmp(key, "SVR")  == 0) cfg.servo_reverse       = atoi(val) != 0;
	else if (strcmp(key, "CAL")  == 0) cfg.calibrated          = atoi(val) != 0;
	else if (strcmp(key, "BEN")  == 0) cfg.bat_enabled         = atoi(val) != 0;
	else if (strcmp(key, "BML")  == 0) cfg.bat_multiplier      = strtof(val, NULL);
	else if (strcmp(key, "BLV")  == 0) cfg.bat_low             = strtof(val, NULL);
	else return false;

	return true;
}
