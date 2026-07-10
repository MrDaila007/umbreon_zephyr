/*
 * buzzer.c — passive piezo buzzer on GP18
 */

#include "buzzer.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(buzzer, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_APP_BUZZER)

static const struct gpio_dt_spec buzzer_gpio =
	GPIO_DT_SPEC_GET(DT_NODELABEL(buzzer_pin), gpios);

struct buzzer_cmd {
	uint16_t freq_hz;
	uint16_t duration_ms;
};

K_MSGQ_DEFINE(buzzer_queue, sizeof(struct buzzer_cmd), 24, 4);
static K_THREAD_STACK_DEFINE(buzzer_stack, CONFIG_APP_BUZZER_STACK_SIZE);
static struct k_thread buzzer_thread_data;
static bool buzzer_ready;

static void buzzer_set(bool on)
{
	if (buzzer_ready) {
		gpio_pin_set_dt(&buzzer_gpio, on ? 1 : 0);
	}
}

static void tone_blocking(uint16_t freq_hz, uint16_t duration_ms)
{
	if (!buzzer_ready || duration_ms == 0U) {
		return;
	}

	if (freq_hz == 0U) {
		buzzer_set(false);
		k_msleep(duration_ms);
		return;
	}

	uint32_t half_period_us = 1000000U / ((uint32_t)freq_hz * 2U);
	if (half_period_us < 50U) {
		half_period_us = 50U;
	}

	int64_t end_ms = k_uptime_get() + duration_ms;
	while (k_uptime_get() < end_ms) {
		buzzer_set(true);
		k_busy_wait(half_period_us);
		buzzer_set(false);
		k_busy_wait(half_period_us);
	}
	buzzer_set(false);
}

static void buzzer_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct buzzer_cmd cmd;
	while (1) {
		k_msgq_get(&buzzer_queue, &cmd, K_FOREVER);
		tone_blocking(cmd.freq_hz, cmd.duration_ms);
	}
}

void buzzer_init(void)
{
	buzzer_ready = false;

	if (!gpio_is_ready_dt(&buzzer_gpio)) {
		LOG_WRN("Buzzer GPIO not ready");
		return;
	}

	int rc = gpio_pin_configure_dt(&buzzer_gpio, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		LOG_WRN("Buzzer GPIO configure failed: %d", rc);
		return;
	}

	buzzer_ready = true;
	k_thread_create(&buzzer_thread_data, buzzer_stack,
			K_THREAD_STACK_SIZEOF(buzzer_stack),
			buzzer_thread, NULL, NULL, NULL,
			CONFIG_APP_BUZZER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&buzzer_thread_data, "buzzer");
	LOG_INF("Buzzer init OK (GP18)");
}

void buzzer_beep(unsigned int freq_hz, unsigned int duration_ms)
{
	if (!buzzer_ready) {
		return;
	}

	struct buzzer_cmd cmd = {
		.freq_hz = (uint16_t)MIN(freq_hz, UINT16_MAX),
		.duration_ms = (uint16_t)MIN(duration_ms, UINT16_MAX),
	};
	(void)k_msgq_put(&buzzer_queue, &cmd, K_NO_WAIT);
}

static void note(unsigned int freq_hz, unsigned int duration_ms)
{
	buzzer_beep(freq_hz, duration_ms);
	buzzer_beep(0, 25);
}

void buzzer_play(enum buzzer_event event)
{
	switch (event) {
	case BUZZER_BOOT_READY:
		note(1047, 55); /* C6 */
		note(1319, 55); /* E6 */
		note(1568, 75); /* G6 */
		break;
	case BUZZER_RUN_START:
		note(784, 55);  /* G5 */
		note(1047, 70); /* C6 */
		break;
	case BUZZER_STOP:
		note(1319, 45); /* E6 */
		note(1047, 75); /* C6 */
		break;
	case BUZZER_CAL_DONE:
		note(1047, 45); /* C6 */
		note(1319, 45); /* E6 */
		note(1568, 45); /* G6 */
		note(2093, 90); /* C7 */
		break;
	case BUZZER_ERROR:
		for (int i = 0; i < 2; i++) {
			note(392, 130); /* G4 */
			buzzer_beep(0, 90);
		}
		break;
	default:
		break;
	}
}

#else

void buzzer_init(void) { }
void buzzer_beep(unsigned int freq_hz, unsigned int duration_ms)
{
	ARG_UNUSED(freq_hz);
	ARG_UNUSED(duration_ms);
}
void buzzer_play(enum buzzer_event event)
{
	ARG_UNUSED(event);
}

#endif
