/*
 * zephyr_stubs.h — Minimal Zephyr API stubs for host unit tests
 *
 * Provides just enough types and functions so that production code
 * (included via #include) compiles with the host GCC.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Zephyr-like macros ──────────────────────────────────────────────────── */
#ifndef CLAMP
#define CLAMP(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/* ─── Atomics (plain integers for single-threaded tests) ──────────────────── */
typedef int32_t atomic_t;
typedef int32_t atomic_val_t;

#define ATOMIC_INIT(v) (v)

static inline atomic_val_t atomic_get(const atomic_t *target)
{
	return *target;
}

static inline void atomic_set(atomic_t *target, atomic_val_t value)
{
	*target = value;
}

static inline atomic_val_t atomic_inc(atomic_t *target)
{
	return (*target)++;
}

/* ─── Cycle / time conversion stubs ───────────────────────────────────────── */

/* Configurable "CPU frequency" for tests — default 150 MHz (RP2350) */
#ifndef TEST_CPU_FREQ_HZ
#define TEST_CPU_FREQ_HZ 150000000U
#endif

static inline int64_t k_cyc_to_us_floor64(uint64_t cycles)
{
	return (int64_t)(cycles / (TEST_CPU_FREQ_HZ / 1000000U));
}

/* Mockable: tests set this before calling code under test */
static uint32_t _mock_cycle_counter = 0;

static inline uint32_t k_cycle_get_32(void)
{
	return _mock_cycle_counter;
}

static int64_t _mock_uptime_ticks = 0;

static inline int64_t k_uptime_ticks(void)
{
	return _mock_uptime_ticks;
}

static inline int64_t k_ticks_to_us_floor64(int64_t ticks)
{
	return ticks * 1000; /* 1000 ticks/sec → 1 tick = 1000 µs */
}

/* ─── Sensor constants from settings.h ────────────────────────────────────── */
#define SENSOR_COUNT     6
#define MAX_SENSOR_RANGE 2000
#define NEUTRAL_SPEED    1500
