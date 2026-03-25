/*
 * battery.c — Battery voltage monitoring via ADC
 *
 * ADC ch0 (GP26), 12-bit, resistor divider R1=18k / R2=10k (multiplier 2.8)
 * Runs in its own low-priority thread, reading every 500ms.
 *
 * Port of Car::bat_update() from luna_car.h:417-431
 */

#include "battery.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

/* ─── ADC config ──────────────────────────────────────────────────────────── */
static const struct device *adc_dev;

#define ADC_CHANNEL    0
#define ADC_RESOLUTION 12
#define ADC_GAIN       ADC_GAIN_1
#define ADC_REFERENCE  ADC_REF_INTERNAL  /* 3.3V on RP2350 */

static const struct adc_channel_cfg adc_ch_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL,
};

#define BAT_EMA 0.05f

/* ─── State ───────────────────────────────────────────────────────────────── */
static float bat_voltage;

/* ─── Thread config ───────────────────────────────────────────────────────── */
#define BAT_STACK_SIZE 1024
#define BAT_PRIORITY   10
static K_THREAD_STACK_DEFINE(bat_stack, BAT_STACK_SIZE);
static struct k_thread bat_thread_data;

/* ─── Battery thread ──────────────────────────────────────────────────────── */

static void battery_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int16_t sample_buf;
	struct adc_sequence seq = {
		.channels = BIT(ADC_CHANNEL),
		.buffer = &sample_buf,
		.buffer_size = sizeof(sample_buf),
		.resolution = ADC_RESOLUTION,
	};

	while (1) {
		k_msleep(500);

		if (!cfg.bat_enabled || !adc_dev) {
			continue;
		}

		int rc = adc_read(adc_dev, &seq);
		if (rc != 0) {
			continue;
		}

		float v_adc = (float)sample_buf * (3.3f / 4095.0f);
		float v_bat = v_adc * cfg.bat_multiplier;

		/* EMA filter */
		if (bat_voltage < 0.1f) {
			bat_voltage = v_bat;  /* seed on first reading */
		} else {
			bat_voltage = BAT_EMA * v_bat + (1.0f - BAT_EMA) * bat_voltage;
		}
	}
}

/* ─── Init ────────────────────────────────────────────────────────────────── */

void battery_init(void)
{
	adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
	if (!device_is_ready(adc_dev)) {
		LOG_WRN("ADC not ready — battery monitoring disabled");
		adc_dev = NULL;
		return;
	}

	int rc = adc_channel_setup(adc_dev, &adc_ch_cfg);
	if (rc) {
		LOG_ERR("ADC channel setup failed: %d", rc);
		adc_dev = NULL;
		return;
	}

	k_thread_create(&bat_thread_data, bat_stack,
			K_THREAD_STACK_SIZEOF(bat_stack),
			battery_thread, NULL, NULL, NULL,
			BAT_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&bat_thread_data, "battery");

	LOG_INF("Battery ADC init (GP26, ch0)");
}

float battery_get_voltage(void)
{
	return bat_voltage;
}
