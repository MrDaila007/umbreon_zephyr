/*
 * test_cyc_delta.c — cyc_delta_us() wraparound math
 *
 * Tests use k_cyc_to_us_floor64() for expected values so they work on
 * any platform (native_sim = 1 MHz, RP2350 = 150 MHz).
 */

#include "test_common.h"

/* Helper: how many cycles make N microseconds on this platform */
#define US_TO_CYC(us) ((uint32_t)((uint64_t)(us) * \
	CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000ULL))

ZTEST_SUITE(cyc_delta, NULL, NULL, NULL, NULL, NULL);

ZTEST(cyc_delta, test_normal)
{
	uint32_t cyc = US_TO_CYC(1);
	zassert_equal(cyc_delta_us(0, cyc), 1, "1 µs delta");
}

ZTEST(cyc_delta, test_larger)
{
	uint32_t cyc = US_TO_CYC(100);
	zassert_equal(cyc_delta_us(1000, 1000 + cyc), 100, "100 µs delta");
}

ZTEST(cyc_delta, test_one_ms)
{
	uint32_t cyc = US_TO_CYC(1000);
	zassert_equal(cyc_delta_us(0, cyc), 1000, "1 ms delta");
}

ZTEST(cyc_delta, test_zero)
{
	zassert_equal(cyc_delta_us(5000, 5000), 0, "zero delta");
}

ZTEST(cyc_delta, test_wraparound_small)
{
	/* Cycle counter wraps: from near-max to small value */
	uint32_t from = 0xFFFFFFF0U;
	uint32_t to   = from + US_TO_CYC(10);  /* 10 µs past wrap */
	zassert_equal(cyc_delta_us(from, to), 10, "wraparound 10 µs");
}

ZTEST(cyc_delta, test_wraparound_large)
{
	/* Wrap with ~5 ms past wrap point */
	uint32_t from = 0xFFFF0000U;
	uint32_t to   = from + US_TO_CYC(5000);
	zassert_equal(cyc_delta_us(from, to), 5000, "wraparound 5 ms");
}
