/*
 * Umbreon Zephyr — Main entry point
 *
 * Initializes all subsystems and starts threads.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "settings.h"
#include "car.h"
#include "tachometer.h"
#include "sensors.h"
#include "imu.h"
#include "wifi_cmd.h"
#include "control.h"
#include "battery.h"
#include "tests.h"
#include "track_learn.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define FW_VERSION "2.0.0"

/* ─── Watchdog ──────────────────────────────────────────────────────────────── */
#include <zephyr/drivers/watchdog.h>

static const struct device *wdt_dev;
static int wdt_channel_id;

static void wdt_init(void)
{
	wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
	if (!device_is_ready(wdt_dev)) {
		LOG_WRN("Watchdog not available");
		wdt_dev = NULL;
		return;
	}

	struct wdt_timeout_cfg wdt_cfg = {
		.window.min = 0,
		.window.max = 8000,  /* 8 second timeout */
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};

	wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
	if (wdt_channel_id < 0) {
		LOG_ERR("Watchdog install failed: %d", wdt_channel_id);
		wdt_dev = NULL;
		return;
	}

	int err = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (err) {
		LOG_ERR("Watchdog setup failed: %d", err);
		wdt_dev = NULL;
	}
}

void wdt_feed_kick(void)
{
	if (wdt_dev) {
		wdt_feed(wdt_dev, wdt_channel_id);
	}
}

/* ─── LED blink for startup indication ──────────────────────────────────────── */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void blink_led(int count, int ms)
{
	if (!gpio_is_ready_dt(&led)) {
		return;
	}
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	for (int i = 0; i < count; i++) {
		gpio_pin_set_dt(&led, 1);
		k_msleep(ms);
		gpio_pin_set_dt(&led, 0);
		k_msleep(ms);
	}
}

/* ─── Main ──────────────────────────────────────────────────────────────────── */
int main(void)
{
	printk("\n");
	printk("==============================\n");
	printk("  Umbreon Zephyr v%s\n", FW_VERSION);
	printk("==============================\n");

	/* Load saved settings (falls back to compile-time defaults) */
	settings_init();
	settings_load();

	/* Start watchdog early to protect init sequence */
	wdt_init();

	/* Initialize hardware subsystems */
	car_init();
	taho_init();
	sensors_init();      /* ~500 ms I2C probing */
	imu_init();
	wdt_feed_kick();
	imu_calibrate();     /* ~1 sec */
	wdt_feed_kick();
	battery_init();
	wifi_cmd_init();
	track_learn_init();

	/* Send boot status via WiFi */
	k_msleep(200); /* Let ESP boot */
	wifi_cmd_printf("$BOOT:SNS=%d,FW=%s\n",
			sensors_online_count(), FW_VERSION);

	/* ESC calibration on first boot */
	wdt_feed_kick();
	if (!cfg.calibrated) {
		car_run_calibration(); /* feeds wdt internally */
	} else {
		/* Allow ESC to arm and sensors to start */
		k_msleep(3700);
	}

	/* Start control thread */
	control_init();

	/* Signal startup complete */
	blink_led(3, 100);

	wifi_cmd_printf("$BOOT:READY,UP=%lld\n", k_uptime_get());

	/* Main thread has nothing else to do — sleep forever.
	 * All work happens in dedicated threads. */
	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
