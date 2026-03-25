/*
 * tachometer.c — Optical encoder speed measurement
 *
 * Ported from luna_car.h: taho_interrupt(), get_speed()
 * Uses GPIO interrupt on GP13 (RISING edge) with atomic variables.
 */

#include "tachometer.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(tachometer, LOG_LEVEL_INF);

/* ─── GPIO ────────────────────────────────────────────────────────────────── */
static const struct gpio_dt_spec tach_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(tach_pin), gpios);
static struct gpio_callback tach_cb_data;

/* ─── Shared state (ISR writes, main reads — all atomic) ──────────────────── */
static atomic_t taho_count    = ATOMIC_INIT(0);
static atomic_t taho_last_us  = ATOMIC_INIT(0);
static atomic_t taho_interval = ATOMIC_INIT(0);

/* ─── Hardware timer for microseconds ─────────────────────────────────────── */
static inline uint32_t get_us(void)
{
	return (uint32_t)(k_cyc_to_us_floor64(k_cycle_get_32()));
}

/* ─── ISR ─────────────────────────────────────────────────────────────────── */
/* Port of taho_interrupt() from luna_car.h:71-78 */
static void tach_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	uint32_t now = get_us();
	uint32_t last = (uint32_t)atomic_get(&taho_last_us);
	uint32_t delta = now - last;

	/* Debounce: 200µs (supports up to ~5 m/s with 62-hole encoder) */
	if (delta < 200) {
		return;
	}

	atomic_inc(&taho_count);
	atomic_set(&taho_interval, (atomic_val_t)delta);
	atomic_set(&taho_last_us, (atomic_val_t)now);
}

/* ─── Init ────────────────────────────────────────────────────────────────── */

void taho_init(void)
{
	if (!gpio_is_ready_dt(&tach_gpio)) {
		LOG_ERR("Tachometer GPIO not ready");
		return;
	}

	gpio_pin_configure_dt(&tach_gpio, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&tach_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&tach_cb_data, tach_isr, BIT(tach_gpio.pin));
	gpio_add_callback(tach_gpio.port, &tach_cb_data);

	LOG_INF("Tachometer init (GP13, RISING edge)");
}

/* ─── Speed calculation ───────────────────────────────────────────────────── */
/* Port of get_speed() from luna_car.h:81-87 */

float taho_get_speed(void)
{
	uint32_t now = get_us();
	uint32_t last = (uint32_t)atomic_get(&taho_last_us);
	uint32_t iv = (uint32_t)atomic_get(&taho_interval);

	uint32_t elapsed = now - last;
	if (elapsed < iv) {
		elapsed = iv;
	}

	if (iv == 0 || elapsed > 500000 || cfg.encoder_holes <= 0) {
		return 0.0f;
	}

	return (3.14159265f * cfg.wheel_diam_m) /
	       ((float)cfg.encoder_holes * ((float)elapsed / 1e6f));
}

uint32_t taho_get_count(void)
{
	return (uint32_t)atomic_get(&taho_count);
}

uint32_t taho_time_since_last_us(void)
{
	uint32_t now = get_us();
	uint32_t last = (uint32_t)atomic_get(&taho_last_us);
	return now - last;
}

void taho_reset(void)
{
	atomic_set(&taho_count, 0);
	atomic_set(&taho_last_us, (atomic_val_t)get_us());
	atomic_set(&taho_interval, 0);
}
