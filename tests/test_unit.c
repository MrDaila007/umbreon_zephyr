/*
 * test_unit.c — Host unit tests for umbreon_zephyr pure logic
 *
 * Build:  make -C tests
 * Run:    ./tests/test_unit
 *
 * Tests extracted static functions via copy-paste of their bodies.
 * When source changes — update these copies or expose via #ifdef UNIT_TEST.
 */

#include "zephyr_stubs.h"
#include "test_runner.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Extracted functions under test (copies of static inlines from src/)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* From tachometer.c:37-41 */
static inline uint32_t cyc_delta_us(uint32_t from_cyc, uint32_t to_cyc)
{
	uint32_t delta_cyc = to_cyc - from_cyc;
	return (uint32_t)k_cyc_to_us_floor64(delta_cyc);
}

/* From car.c:49-52 */
static inline int map_val(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ─── cfg struct for parse_set_pair tests ─────────────────────────────────── */
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
	float spd_slew;
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

static struct car_settings cfg;

/* From wifi_cmd.c:208-256 — copy of parse_set_pair */
static bool parse_set_pair(const char *pair)
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
	else if (strcmp(key, "SLW")  == 0) cfg.spd_slew           = strtof(val, NULL);
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

/* Speed formula from tachometer.c:100-101 (extracted, no atomics) */
static float speed_formula(float wheel_diam_m, int encoder_holes, uint32_t elapsed_us)
{
	if (elapsed_us == 0 || elapsed_us > 500000 || encoder_holes <= 0) {
		return 0.0f;
	}
	return ((float)M_PI * wheel_diam_m) /
	       ((float)encoder_holes * ((float)elapsed_us / 1e6f));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: cyc_delta_us()
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_cyc_delta_normal)
{
	/* 150 cycles at 150 MHz = 1 µs */
	uint32_t us = cyc_delta_us(0, 150);
	ASSERT_EQ(us, 1);
}

TEST(test_cyc_delta_larger)
{
	/* 15000 cycles = 100 µs */
	uint32_t us = cyc_delta_us(1000, 16000);
	ASSERT_EQ(us, 100);
}

TEST(test_cyc_delta_wraparound)
{
	/* Simulates cycle counter wrap: from near-max to near-zero */
	uint32_t from = 0xFFFFFFF0U;
	uint32_t to   = 0x00000096U;  /* 0x96 = 150 past wrap */
	/* delta_cyc = 0x96 - 0xFFFFFFF0 = 0xA6 = 166 cycles */
	uint32_t expected_cyc = to - from;  /* = 166 */
	uint32_t expected_us = (uint32_t)(expected_cyc / 150);
	uint32_t us = cyc_delta_us(from, to);
	ASSERT_EQ(us, expected_us);
}

TEST(test_cyc_delta_wraparound_large)
{
	/* Wrap with ~1 ms of cycles past wrap point */
	uint32_t from = 0xFFFF0000U;
	uint32_t to   = 0x00024B00U;  /* 150000 cycles past wrap + some */
	uint32_t delta_cyc = to - from;
	uint32_t expected_us = (uint32_t)(delta_cyc / 150);
	uint32_t us = cyc_delta_us(from, to);
	ASSERT_EQ(us, expected_us);
}

TEST(test_cyc_delta_zero)
{
	ASSERT_EQ(cyc_delta_us(5000, 5000), 0);
}

TEST(test_cyc_delta_one_ms)
{
	/* 150000 cycles = 1000 µs = 1 ms */
	ASSERT_EQ(cyc_delta_us(0, 150000), 1000);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: map_val()
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_map_midpoint)
{
	ASSERT_EQ(map_val(5, 0, 10, 0, 100), 50);
}

TEST(test_map_min_boundary)
{
	ASSERT_EQ(map_val(0, 0, 10, 0, 100), 0);
}

TEST(test_map_max_boundary)
{
	ASSERT_EQ(map_val(10, 0, 10, 0, 100), 100);
}

TEST(test_map_reverse_output)
{
	/* Output range is inverted */
	ASSERT_EQ(map_val(5, 0, 10, 100, 0), 50);
}

TEST(test_map_negative_input)
{
	/* Input: [-1000, 1000] → output: [0, 180] */
	ASSERT_EQ(map_val(0, -1000, 1000, 0, 180), 90);
}

TEST(test_map_servo_realistic)
{
	/* Typical servo mapping: steer [-1000..1000] → [30..150] degrees */
	ASSERT_EQ(map_val(-1000, -1000, 1000, 30, 150), 30);
	ASSERT_EQ(map_val(0, -1000, 1000, 30, 150), 90);
	ASSERT_EQ(map_val(1000, -1000, 1000, 30, 150), 150);
}

TEST(test_map_esc_forward)
{
	/* ESC mapping: speed [0..1000] → [1500..1800] µs */
	ASSERT_EQ(map_val(0, 0, 1000, 1500, 1800), 1500);
	ASSERT_EQ(map_val(500, 0, 1000, 1500, 1800), 1650);
	ASSERT_EQ(map_val(1000, 0, 1000, 1500, 1800), 1800);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: parse_set_pair()
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_parse_valid_int)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("FOD=800"));
	ASSERT_EQ(cfg.front_obstacle_dist, 800);
}

TEST(test_parse_valid_float)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("KP=1.5"));
	ASSERT_FLOAT_EQ(cfg.pid_kp, 1.5f, 0.001f);
}

TEST(test_parse_valid_bool)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("RCW=1"));
	ASSERT_TRUE(cfg.race_cw);
	ASSERT_TRUE(parse_set_pair("RCW=0"));
	ASSERT_FALSE(cfg.race_cw);
}

TEST(test_parse_no_equals)
{
	ASSERT_FALSE(parse_set_pair("FOD800"));
}

TEST(test_parse_empty_key)
{
	ASSERT_FALSE(parse_set_pair("=800"));
}

TEST(test_parse_key_too_long)
{
	ASSERT_FALSE(parse_set_pair("TOOLONGKEY=800"));
}

TEST(test_parse_unknown_key)
{
	ASSERT_FALSE(parse_set_pair("ZZZ=123"));
}

TEST(test_parse_clamp_lms_zero)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("LMS=0"));
	ASSERT_EQ(cfg.loop_ms, 10);  /* clamped to minimum 10 */
}

TEST(test_parse_clamp_lms_negative)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("LMS=-5"));
	ASSERT_EQ(cfg.loop_ms, 10);
}

TEST(test_parse_clamp_enh_zero)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("ENH=0"));
	ASSERT_EQ(cfg.encoder_holes, 1);  /* clamped to minimum 1 */
}

TEST(test_parse_clamp_fod_too_low)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("FOD=1"));
	ASSERT_EQ(cfg.front_obstacle_dist, 10);  /* clamped */
}

TEST(test_parse_clamp_fod_too_high)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("FOD=9999"));
	ASSERT_EQ(cfg.front_obstacle_dist, MAX_SENSOR_RANGE);
}

TEST(test_parse_clamp_esc_range)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("MSP=500"));
	ASSERT_EQ(cfg.min_speed, 1000);  /* clamped to ESC min */
	ASSERT_TRUE(parse_set_pair("XSP=3000"));
	ASSERT_EQ(cfg.max_speed, 2000);  /* clamped to ESC max */
}

TEST(test_parse_clamp_servo_range)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("MNP=999"));
	ASSERT_EQ(cfg.min_point, 180);  /* clamped */
	ASSERT_TRUE(parse_set_pair("XNP=-10"));
	ASSERT_EQ(cfg.max_point, 0);    /* clamped */
}

TEST(test_parse_normal_values)
{
	memset(&cfg, 0, sizeof(cfg));
	ASSERT_TRUE(parse_set_pair("LMS=40"));
	ASSERT_EQ(cfg.loop_ms, 40);
	ASSERT_TRUE(parse_set_pair("ENH=62"));
	ASSERT_EQ(cfg.encoder_holes, 62);
	ASSERT_TRUE(parse_set_pair("WDM=0.065"));
	ASSERT_FLOAT_EQ(cfg.wheel_diam_m, 0.065f, 0.001f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: speed_formula (taho_get_speed core math)
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_speed_stationary)
{
	/* elapsed > 500ms → wheel stopped */
	ASSERT_FLOAT_EQ(speed_formula(0.065f, 62, 600000), 0.0f, 0.001f);
}

TEST(test_speed_zero_interval)
{
	ASSERT_FLOAT_EQ(speed_formula(0.065f, 62, 0), 0.0f, 0.001f);
}

TEST(test_speed_zero_encoder_holes)
{
	ASSERT_FLOAT_EQ(speed_formula(0.065f, 0, 1000), 0.0f, 0.001f);
}

TEST(test_speed_negative_encoder_holes)
{
	ASSERT_FLOAT_EQ(speed_formula(0.065f, -1, 1000), 0.0f, 0.001f);
}

TEST(test_speed_known_value)
{
	/* wheel_diam=0.065m, encoder_holes=62, elapsed=3300µs
	 * speed = π×0.065 / (62 × 0.0033) = 0.2042 / 0.2046 ≈ 0.998 m/s */
	float spd = speed_formula(0.065f, 62, 3300);
	ASSERT_FLOAT_EQ(spd, 0.998f, 0.01f);
}

TEST(test_speed_fast)
{
	/* 3 m/s → elapsed ≈ 1100 µs */
	/* speed = π×0.065 / (62 × 0.0011) = 0.2042 / 0.0682 ≈ 2.994 */
	float spd = speed_formula(0.065f, 62, 1100);
	ASSERT_FLOAT_EQ(spd, 2.994f, 0.01f);
}

TEST(test_speed_very_slow)
{
	/* elapsed = 490000 µs (just under 500ms cutoff) */
	float spd = speed_formula(0.065f, 62, 490000);
	ASSERT_TRUE(spd > 0.0f);  /* should still give a reading */
	ASSERT_TRUE(spd < 0.01f); /* but very slow */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Main
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
	printf("=== umbreon_zephyr unit tests ===\n\n");

	printf("[cyc_delta_us]\n");
	RUN_TEST(test_cyc_delta_normal);
	RUN_TEST(test_cyc_delta_larger);
	RUN_TEST(test_cyc_delta_wraparound);
	RUN_TEST(test_cyc_delta_wraparound_large);
	RUN_TEST(test_cyc_delta_zero);
	RUN_TEST(test_cyc_delta_one_ms);

	printf("\n[map_val]\n");
	RUN_TEST(test_map_midpoint);
	RUN_TEST(test_map_min_boundary);
	RUN_TEST(test_map_max_boundary);
	RUN_TEST(test_map_reverse_output);
	RUN_TEST(test_map_negative_input);
	RUN_TEST(test_map_servo_realistic);
	RUN_TEST(test_map_esc_forward);

	printf("\n[parse_set_pair]\n");
	RUN_TEST(test_parse_valid_int);
	RUN_TEST(test_parse_valid_float);
	RUN_TEST(test_parse_valid_bool);
	RUN_TEST(test_parse_no_equals);
	RUN_TEST(test_parse_empty_key);
	RUN_TEST(test_parse_key_too_long);
	RUN_TEST(test_parse_unknown_key);
	RUN_TEST(test_parse_clamp_lms_zero);
	RUN_TEST(test_parse_clamp_lms_negative);
	RUN_TEST(test_parse_clamp_enh_zero);
	RUN_TEST(test_parse_clamp_fod_too_low);
	RUN_TEST(test_parse_clamp_fod_too_high);
	RUN_TEST(test_parse_clamp_esc_range);
	RUN_TEST(test_parse_clamp_servo_range);
	RUN_TEST(test_parse_normal_values);

	printf("\n[speed_formula]\n");
	RUN_TEST(test_speed_stationary);
	RUN_TEST(test_speed_zero_interval);
	RUN_TEST(test_speed_zero_encoder_holes);
	RUN_TEST(test_speed_negative_encoder_holes);
	RUN_TEST(test_speed_known_value);
	RUN_TEST(test_speed_fast);
	RUN_TEST(test_speed_very_slow);

	TEST_SUMMARY();
}
