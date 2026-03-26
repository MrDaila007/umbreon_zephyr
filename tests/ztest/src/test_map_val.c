/*
 * test_map_val.c — map_val() linear interpolation
 */

#include "test_common.h"

ZTEST_SUITE(map_val_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(map_val_suite, test_midpoint)
{
	zassert_equal(map_val(5, 0, 10, 0, 100), 50);
}

ZTEST(map_val_suite, test_min_boundary)
{
	zassert_equal(map_val(0, 0, 10, 0, 100), 0);
}

ZTEST(map_val_suite, test_max_boundary)
{
	zassert_equal(map_val(10, 0, 10, 0, 100), 100);
}

ZTEST(map_val_suite, test_reverse_output)
{
	zassert_equal(map_val(5, 0, 10, 100, 0), 50);
}

ZTEST(map_val_suite, test_negative_input)
{
	zassert_equal(map_val(0, -1000, 1000, 0, 180), 90);
}

ZTEST(map_val_suite, test_servo_realistic)
{
	zassert_equal(map_val(-1000, -1000, 1000, 30, 150), 30);
	zassert_equal(map_val(0,    -1000, 1000, 30, 150), 90);
	zassert_equal(map_val(1000, -1000, 1000, 30, 150), 150);
}

ZTEST(map_val_suite, test_esc_forward)
{
	zassert_equal(map_val(0,    0, 1000, 1500, 1800), 1500);
	zassert_equal(map_val(500,  0, 1000, 1500, 1800), 1650);
	zassert_equal(map_val(1000, 0, 1000, 1500, 1800), 1800);
}
