/*
 * test_parse_set.c — parse_set_pair() validation
 */

#include "test_common.h"

static void reset_cfg(void *arg)
{
	ARG_UNUSED(arg);
	memset(&cfg, 0, sizeof(cfg));
}

ZTEST_SUITE(parse_set, NULL, NULL, reset_cfg, NULL, NULL);

/* ── Valid inputs ─────────────────────────────────────────────────────────── */

ZTEST(parse_set, test_valid_int)
{
	zassert_true(parse_set_pair("FOD=800"));
	zassert_equal(cfg.front_obstacle_dist, 800);
}

ZTEST(parse_set, test_valid_float)
{
	zassert_true(parse_set_pair("KP=1.5"));
	zassert_true(fabsf(cfg.pid_kp - 1.5f) < 0.001f);
}

ZTEST(parse_set, test_valid_bool_true)
{
	zassert_true(parse_set_pair("RCW=1"));
	zassert_true(cfg.race_cw);
}

ZTEST(parse_set, test_valid_bool_false)
{
	cfg.race_cw = true;
	zassert_true(parse_set_pair("RCW=0"));
	zassert_false(cfg.race_cw);
}

ZTEST(parse_set, test_normal_values)
{
	zassert_true(parse_set_pair("LMS=40"));
	zassert_equal(cfg.loop_ms, 40);

	zassert_true(parse_set_pair("ENH=62"));
	zassert_equal(cfg.encoder_holes, 62);

	zassert_true(parse_set_pair("WDM=0.065"));
	zassert_true(fabsf(cfg.wheel_diam_m - 0.065f) < 0.001f);
}

/* ── Invalid inputs ───────────────────────────────────────────────────────── */

ZTEST(parse_set, test_no_equals)
{
	zassert_false(parse_set_pair("FOD800"));
}

ZTEST(parse_set, test_empty_key)
{
	zassert_false(parse_set_pair("=800"));
}

ZTEST(parse_set, test_key_too_long)
{
	zassert_false(parse_set_pair("TOOLONGKEY=800"));
}

ZTEST(parse_set, test_unknown_key)
{
	zassert_false(parse_set_pair("ZZZ=123"));
}

/* ── CLAMP validation ─────────────────────────────────────────────────────── */

ZTEST(parse_set, test_lms_clamped_to_10)
{
	zassert_true(parse_set_pair("LMS=0"));
	zassert_equal(cfg.loop_ms, 10);
}

ZTEST(parse_set, test_lms_negative_clamped)
{
	zassert_true(parse_set_pair("LMS=-5"));
	zassert_equal(cfg.loop_ms, 10);
}

ZTEST(parse_set, test_enh_clamped_to_1)
{
	zassert_true(parse_set_pair("ENH=0"));
	zassert_equal(cfg.encoder_holes, 1);
}

ZTEST(parse_set, test_fod_clamped_low)
{
	zassert_true(parse_set_pair("FOD=1"));
	zassert_equal(cfg.front_obstacle_dist, 10);
}

ZTEST(parse_set, test_fod_clamped_high)
{
	zassert_true(parse_set_pair("FOD=9999"));
	zassert_equal(cfg.front_obstacle_dist, MAX_SENSOR_RANGE);
}

ZTEST(parse_set, test_esc_clamped)
{
	zassert_true(parse_set_pair("MSP=500"));
	zassert_equal(cfg.min_speed, 1000);

	zassert_true(parse_set_pair("XSP=3000"));
	zassert_equal(cfg.max_speed, 2000);
}

ZTEST(parse_set, test_servo_clamped)
{
	zassert_true(parse_set_pair("MNP=999"));
	zassert_equal(cfg.min_point, 180);

	zassert_true(parse_set_pair("XNP=-10"));
	zassert_equal(cfg.max_point, 0);
}
