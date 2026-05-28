/*
 * encoder.c — Rotary encoder driver (GP22=CLK, GP12=DT, GP19=SW)
 *
 * 1 kHz polling for rotation and button state.
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

#define ENCODER_STACK_SIZE 1024
#define ENCODER_PRIORITY   5
#define ENCODER_POLL_MS    1
#define ENCODER_REVERSE    false   /* Set to true to reverse rotation direction */	

#define ENC_BTN_DEBOUNCE_MS 50
#define ENC_BTN_CLICK_MS    500
#define ENC_BTN_HOLD_MS     600

static K_THREAD_STACK_DEFINE(encoder_stack, ENCODER_STACK_SIZE);
static struct k_thread encoder_thread_data;

/* ─── Rotation state ─────────────────────────────────────────────────────── */
static atomic_t rotation_counter;    /* net rotation steps */

/* Quadrature state: track previous CLK/DT for step4 decoding */
static volatile uint8_t enc_state;   /* 2-bit: (old_CLK << 1) | old_DT */
static volatile int8_t enc_accum;    /* EncButton-style epos accumulator */
static atomic_t transition_counter;
static atomic_t invalid_counter;
static atomic_t same_counter;
static atomic_t synth_counter;
static atomic_t state_counter[4];

/* ─── Button state ───────────────────────────────────────────────────────── */
static atomic_t button_events;
static bool btn_raw;
static bool btn_state;
static bool btn_debouncing;
static bool btn_hold_sent;
static int64_t btn_debounce_ms;
static int64_t btn_press_ms;
static int64_t btn_last_click_ms;
static bool encoder_ready;

/* ─── Rotation polling ───────────────────────────────────────────────────── */

static uint8_t read_ab_state(void)
{
	int clk_val = gpio_pin_get_dt(&enc_clk);
	int dt_val  = gpio_pin_get_dt(&enc_dt);
	return ((uint8_t)clk_val << 1) | (uint8_t)dt_val;
}

static void process_ab_state(uint8_t new_state)
{
	uint8_t old_state = enc_state;
	if (new_state == old_state) {
		return;
	}

	bool p0 = (old_state >> 1) & 1;
	bool p1 = old_state & 1;
	bool e0 = (new_state >> 1) & 1;
	bool e1 = new_state & 1;

	/*
	 * Port of GyverLibs EncButton VirtEncoder::pollEnc(), configured as
	 * EB_STEP4_LOW. For pull-up encoders this emits one step at idle 11.
	 */
	if (!(p0 ^ p1 ^ e0 ^ e1)) {
		atomic_inc(&invalid_counter);
		return;
	}

	(p1 ^ e0) ? ++enc_accum : --enc_accum;
	enc_state = new_state;
	atomic_inc(&transition_counter);
	atomic_inc(&state_counter[new_state & 0x03]);

	if (!enc_accum) {
		return;
	}

	/* EB_STEP4_LOW: skip 01, 10, 00; emit only when both lines are high. */
	if (!(e0 && e1)) {
		return;
	}

	int8_t step = ((enc_accum > 0) ^ ENCODER_REVERSE) ? -1 : 1;
	atomic_add(&rotation_counter, step);
	if (step > 0) {
		atomic_inc(&synth_counter);
	} else {
		atomic_inc(&same_counter);
	}
	enc_accum = 0;
}

static void process_button_state(bool raw_pressed)
{
	int64_t now = k_uptime_get();

	if (raw_pressed != btn_raw) {
		btn_raw = raw_pressed;
		btn_debouncing = true;
		btn_debounce_ms = now;
	}

	if (btn_debouncing) {
		if ((now - btn_debounce_ms) < ENC_BTN_DEBOUNCE_MS) {
			return;
		}
		btn_debouncing = false;
		if (btn_state == btn_raw) {
			return;
		}

		btn_state = btn_raw;
		if (btn_state) {
			btn_press_ms = now;
			btn_hold_sent = false;
		} else {
			if (!btn_hold_sent) {
				if (btn_last_click_ms > 0 &&
				    (now - btn_last_click_ms) <= ENC_BTN_CLICK_MS) {
					atomic_or(&button_events, ENC_EVT_DOUBLE);
					btn_last_click_ms = 0;
				} else {
					atomic_or(&button_events, ENC_EVT_CLICK);
					btn_last_click_ms = now;
				}
			} else {
				btn_last_click_ms = 0;
			}
			btn_press_ms = 0;
		}
	}

	if (btn_state && !btn_hold_sent &&
	    (now - btn_press_ms) >= ENC_BTN_HOLD_MS) {
		atomic_or(&button_events, ENC_EVT_HOLD);
		btn_hold_sent = true;
		btn_last_click_ms = 0;
	}
}

static void encoder_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (encoder_ready) {
			process_ab_state(read_ab_state());
			process_button_state(gpio_pin_get_dt(&enc_sw));
		}
		k_msleep(ENCODER_POLL_MS);
	}
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void encoder_init(void)
{
	encoder_ready = false;

	/* CLK */
	if (!gpio_is_ready_dt(&enc_clk)) {
		LOG_ERR("Encoder CLK GPIO not ready");
		return;
	}
	gpio_pin_configure_dt(&enc_clk, GPIO_INPUT);

	/* DT */
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

	/* Read initial quadrature state */
	enc_state = read_ab_state();
	enc_accum = 0;
	atomic_set(&rotation_counter, 0);
	atomic_set(&button_events, 0);
	atomic_set(&transition_counter, 0);
	atomic_set(&invalid_counter, 0);
	atomic_set(&same_counter, 0);
	atomic_set(&synth_counter, 0);
	for (int i = 0; i < 4; i++) {
		atomic_set(&state_counter[i], 0);
	}
	btn_raw = gpio_pin_get_dt(&enc_sw);
	btn_state = btn_raw;
	btn_debouncing = false;
	btn_hold_sent = false;
	btn_debounce_ms = k_uptime_get();
	btn_press_ms = btn_state ? btn_debounce_ms : 0;
	btn_last_click_ms = 0;

	encoder_ready = true;
	k_thread_create(&encoder_thread_data, encoder_stack,
			K_THREAD_STACK_SIZEOF(encoder_stack),
			encoder_thread, NULL, NULL, NULL,
			ENCODER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&encoder_thread_data, "encoder");
	LOG_INF("Encoder init OK (CLK=GP22, DT=GP12, SW=GP19)");
}

bool encoder_get_debug(int *clk, int *dt, int *sw, int *pending_rotation,
		       int *transition_count, int *invalid_count, int *same_count,
		       int *synth_count, int *accum, int state_counts[4])
{
	if (!encoder_ready) {
		return false;
	}
	if (clk) {
		*clk = gpio_pin_get_dt(&enc_clk);
	}
	if (dt) {
		*dt = gpio_pin_get_dt(&enc_dt);
	}
	if (sw) {
		*sw = gpio_pin_get_dt(&enc_sw);
	}
	if (pending_rotation) {
		*pending_rotation = (int)atomic_get(&rotation_counter);
	}
	if (transition_count) {
		*transition_count = (int)atomic_get(&transition_counter);
	}
	if (invalid_count) {
		*invalid_count = (int)atomic_get(&invalid_counter);
	}
	if (same_count) {
		*same_count = (int)atomic_get(&same_counter);
	}
	if (synth_count) {
		*synth_count = (int)atomic_get(&synth_counter);
	}
	if (accum) {
		*accum = enc_accum;
	}
	if (state_counts) {
		for (int i = 0; i < 4; i++) {
			state_counts[i] = (int)atomic_get(&state_counter[i]);
		}
	}
	return true;
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

	events |= (uint8_t)atomic_set(&button_events, 0);

	return events;
}
