/*
 * encoder.c — Rotary encoder driver (GP22=CLK, GP12=DT, GP19=SW)
 *
 * GPIO interrupts for rotation and button presses.
 * Display thread calls encoder_poll() each tick to get events.
 */

#include "encoder.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(encoder, LOG_LEVEL_INF);

/* ─── GPIO specs from devicetree ─────────────────────────────────────────── */
static const struct gpio_dt_spec enc_clk = GPIO_DT_SPEC_GET(DT_NODELABEL(enc_a), gpios);
static const struct gpio_dt_spec enc_dt  = GPIO_DT_SPEC_GET(DT_NODELABEL(enc_b), gpios);
static const struct gpio_dt_spec enc_sw  = GPIO_DT_SPEC_GET(DT_NODELABEL(enc_sw), gpios);

static struct gpio_callback clk_cb_data;
static struct gpio_callback sw_cb_data;

/* ─── Rotation state ─────────────────────────────────────────────────────── */
static atomic_t rotation_counter;    /* net rotation steps */
static uint32_t last_clk_us;        /* debounce timestamp */

/* Quadrature state: track previous CLK for step4 decoding */
static volatile uint8_t enc_state;   /* 2-bit: (old_CLK << 1) | old_DT */
static volatile int8_t enc_accum;    /* accumulate 4 transitions per detent */

/* ─── Button state ───────────────────────────────────────────────────────── */
static atomic_t press_time_ms;       /* timestamp of last press (0 = none) */
static atomic_t prev_press_ms;       /* timestamp of press before last */
static uint32_t last_sw_us;         /* debounce */

/* ─── Rotation ISR ───────────────────────────────────────────────────────── */

/* Standard quadrature decode lookup: oldstate -> newstate -> direction
 * Only valid transitions produce ±1, invalid ones produce 0 */
static const int8_t quad_table[16] = {
	 0, -1,  1,  0,
	 1,  0,  0, -1,
	-1,  0,  0,  1,
	 0,  1, -1,  0,
};

static void clk_isr(const struct device *dev, struct gpio_callback *cb,
		     uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Debounce: ignore edges within 1ms */
	uint32_t now = k_cycle_get_32();
	uint32_t elapsed_us = k_cyc_to_us_floor32(now - last_clk_us);
	if (elapsed_us < 1000) {
		return;
	}
	last_clk_us = now;

	/* Read current state */
	int clk_val = gpio_pin_get_dt(&enc_clk);
	int dt_val  = gpio_pin_get_dt(&enc_dt);
	uint8_t new_state = ((uint8_t)clk_val << 1) | (uint8_t)dt_val;

	/* Lookup direction from state transition */
	int8_t dir = quad_table[(enc_state << 2) | new_state];
	enc_state = new_state;

	if (dir == 0) {
		return;
	}

	/* Accumulate — 4 transitions per detent (EB_STEP4) */
	enc_accum += dir;
	if (enc_accum >= 4) {
		atomic_inc(&rotation_counter);
		enc_accum = 0;
	} else if (enc_accum <= -4) {
		atomic_dec(&rotation_counter);
		enc_accum = 0;
	}
}

/* ─── Button ISR ─────────────────────────────────────────────────────────── */

static void sw_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	uint32_t now = k_cycle_get_32();
	uint32_t elapsed_us = k_cyc_to_us_floor32(now - last_sw_us);
	if (elapsed_us < 50000) {  /* 50ms debounce */
		return;
	}
	last_sw_us = now;

	/* Only on press (active = logical 1 for active-low with pull-up) */
	if (!gpio_pin_get_dt(&enc_sw)) {
		return;
	}

	int64_t now_ms = k_uptime_get();
	atomic_set(&prev_press_ms, atomic_get(&press_time_ms));
	atomic_set(&press_time_ms, (atomic_val_t)now_ms);
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void encoder_init(void)
{
	/* CLK */
	if (!gpio_is_ready_dt(&enc_clk)) {
		LOG_ERR("Encoder CLK GPIO not ready");
		return;
	}
	gpio_pin_configure_dt(&enc_clk, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&enc_clk, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&clk_cb_data, clk_isr, BIT(enc_clk.pin));
	gpio_add_callback(enc_clk.port, &clk_cb_data);

	/* DT — input only, read in CLK ISR */
	if (!gpio_is_ready_dt(&enc_dt)) {
		LOG_ERR("Encoder DT GPIO not ready");
		return;
	}
	gpio_pin_configure_dt(&enc_dt, GPIO_INPUT);

	/* Button */
	if (!gpio_is_ready_dt(&enc_sw)) {
		LOG_ERR("Encoder SW GPIO not ready");
		return;
	}
	gpio_pin_configure_dt(&enc_sw, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&enc_sw, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&sw_cb_data, sw_isr, BIT(enc_sw.pin));
	gpio_add_callback(enc_sw.port, &sw_cb_data);

	/* Read initial quadrature state */
	enc_state = ((uint8_t)gpio_pin_get_dt(&enc_clk) << 1) |
		    (uint8_t)gpio_pin_get_dt(&enc_dt);

	LOG_INF("Encoder init OK (CLK=GP22, DT=GP12, SW=GP19)");
}

uint8_t encoder_poll(int *rotation)
{
	uint8_t events = ENC_EVT_NONE;

	/* Rotation */
	atomic_val_t rot = atomic_set(&rotation_counter, 0);
	*rotation = (int)rot;
	if (rot > 0) {
		events |= ENC_EVT_CW;
	} else if (rot < 0) {
		events |= ENC_EVT_CCW;
	}
	if (rot > 3 || rot < -3) {
		events |= ENC_EVT_FAST;
	}

	/* Button events — classify from timestamps */
	int64_t now = k_uptime_get();
	int64_t pt = (int64_t)atomic_get(&press_time_ms);
	int64_t pp = (int64_t)atomic_get(&prev_press_ms);

	if (pt > 0) {
		int64_t since = now - pt;

		/* Hold: button pressed > 800ms ago and still held */
		if (since > 800 && gpio_pin_get_dt(&enc_sw)) {
			events |= ENC_EVT_HOLD;
			atomic_set(&press_time_ms, 0);
			atomic_set(&prev_press_ms, 0);
		}
		/* Double-click: two presses within 400ms */
		else if (pp > 0 && (pt - pp) < 400 && (pt - pp) > 30) {
			events |= ENC_EVT_DOUBLE;
			atomic_set(&press_time_ms, 0);
			atomic_set(&prev_press_ms, 0);
		}
		/* Single click: 400ms elapsed since last press, no second press */
		else if (since > 400 && since < 800) {
			events |= ENC_EVT_CLICK;
			atomic_set(&press_time_ms, 0);
			atomic_set(&prev_press_ms, 0);
		}
	}

	return events;
}
