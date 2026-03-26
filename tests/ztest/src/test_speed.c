/*
 * test_speed.c — taho_get_speed() core math (speed_formula)
 */

#include "test_common.h"

#define WHEEL_DIAM 0.065f
#define ENC_HOLES  62

ZTEST_SUITE(speed, NULL, NULL, NULL, NULL, NULL);

ZTEST(speed, test_stationary)
{
	/* elapsed > 500 ms → wheel stopped */
	zassert_true(speed_formula(WHEEL_DIAM, ENC_HOLES, 600000) == 0.0f);
}

ZTEST(speed, test_zero_interval)
{
	zassert_true(speed_formula(WHEEL_DIAM, ENC_HOLES, 0) == 0.0f);
}

ZTEST(speed, test_zero_encoder_holes)
{
	zassert_true(speed_formula(WHEEL_DIAM, 0, 1000) == 0.0f);
}

ZTEST(speed, test_negative_encoder_holes)
{
	zassert_true(speed_formula(WHEEL_DIAM, -1, 1000) == 0.0f);
}

ZTEST(speed, test_one_ms)
{
	/* ~1 m/s: elapsed ≈ 3300 µs
	 * speed = π×0.065 / (62 × 0.0033) ≈ 0.998 m/s */
	float spd = speed_formula(WHEEL_DIAM, ENC_HOLES, 3300);

	zassert_true(fabsf(spd - 0.998f) < 0.01f,
		     "expected ~0.998, got %f", (double)spd);
}

ZTEST(speed, test_three_ms)
{
	/* ~3 m/s: elapsed ≈ 1100 µs */
	float spd = speed_formula(WHEEL_DIAM, ENC_HOLES, 1100);

	zassert_true(fabsf(spd - 2.994f) < 0.02f,
		     "expected ~2.994, got %f", (double)spd);
}

ZTEST(speed, test_just_under_cutoff)
{
	/* 490 ms — just under 500 ms cutoff, should still read */
	float spd = speed_formula(WHEEL_DIAM, ENC_HOLES, 490000);

	zassert_true(spd > 0.0f, "should report nonzero speed");
	zassert_true(spd < 0.01f, "should be very slow");
}
