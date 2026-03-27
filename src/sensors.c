/*
 * sensors.c — 6× VL53L0X ToF sensor array
 *
 * Uses Zephyr's built-in st,vl53l0x driver for init (XSHUT sequencing,
 * I2C address assignment, calibration), then switches to continuous
 * back-to-back mode via vl53l0x_fast for non-blocking reads (~1 ms/sensor
 * instead of ~33 ms/sensor in single-shot mode).
 *
 * Sensor order: [Hard-Right, Front-Right, Right, Left, Front-Left, Hard-Left]
 */

#include "sensors.h"
#include "settings.h"
#include "vl53l0x_fast.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

#define VL53L0X_MAX_RAW  8190  /* sensor overflow / out-of-range indicator */

/* ─── Device handles (stock Zephyr driver, used for init only) ────────────── */
static const struct device *vl53_devs[SENSOR_COUNT];
static bool vl53_valid[SENSOR_COUNT];
static int distances[SENSOR_COUNT]; /* cm×10 */
static int online_count;

/* ─── Fast continuous-mode state ─────────────────────────────────────────── */
static struct vl53l0x_fast fast[SENSOR_COUNT];
static bool fast_mode;
static const uint16_t vl53_addrs[SENSOR_COUNT] = {
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35
};

extern void wdt_feed_kick(void);

/* ─── Sensor nodelabel → device mapping ───────────────────────────────────── */
#define VL53_DEV(idx, label) \
	vl53_devs[idx] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(label))

/* ─── Init ────────────────────────────────────────────────────────────────── */

void sensors_init(void)
{
	/* Try I2C bus recovery before initializing sensors */
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (device_is_ready(i2c1)) {
		int rc = i2c_recover_bus(i2c1);
		if (rc == 0) {
			LOG_INF("I2C1 bus recovery OK");
		} else if (rc == -ENOSYS) {
			LOG_DBG("I2C1 bus recovery not supported");
		} else {
			LOG_WRN("I2C1 bus recovery failed: %d", rc);
		}
	}

	/* Get device handles — order matches sensor_config.h indices */
	VL53_DEV(0, vl53l0x_0);  /* IDX_HARD_RIGHT  — XSHUT GP6  */
	VL53_DEV(1, vl53l0x_1);  /* IDX_FRONT_RIGHT — XSHUT GP7  */
	VL53_DEV(2, vl53l0x_2);  /* IDX_RIGHT       — XSHUT GP8  */
	VL53_DEV(3, vl53l0x_3);  /* IDX_LEFT        — XSHUT GP9  */
	VL53_DEV(4, vl53l0x_4);  /* IDX_FRONT_LEFT  — XSHUT GP14 */
	VL53_DEV(5, vl53l0x_5);  /* IDX_HARD_LEFT   — XSHUT GP15 */

	online_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		distances[i] = 9999;
		if (vl53_devs[i] && device_is_ready(vl53_devs[i])) {
			vl53_valid[i] = true;
			online_count++;
		} else {
			vl53_valid[i] = false;
			LOG_WRN("VL53L0X[%d] not ready", i);
		}
	}

	LOG_INF("VL53L0X: %d/%d online", online_count, SENSOR_COUNT);

	/*
	 * Phase 2: trigger stock driver's lazy init (calibration)
	 * then switch each sensor to continuous back-to-back mode.
	 */
	int fast_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (!vl53_valid[i]) {
			continue;
		}

		/* One blocking single-shot fetch triggers vl53l0x_start()
		 * inside the stock driver: XSHUT release, address reconfig,
		 * DataInit, StaticInit, calibration (~50 ms per sensor). */
		int rc = sensor_sample_fetch(vl53_devs[i]);
		wdt_feed_kick();
		if (rc != 0) {
			LOG_WRN("VL53L0X[%d] calibration fetch failed: %d", i, rc);
			vl53_valid[i] = false;
			online_count--;
			continue;
		}

		/* Read StopVariable and FractionEnable from sensor */
		rc = vl53l0x_fast_init(&fast[i], i2c1, vl53_addrs[i]);
		if (rc != 0) {
			LOG_WRN("VL53L0X[%d] fast init failed: %d", i, rc);
			continue;  /* sensor stays on stock driver fallback */
		}

		/* Start continuous back-to-back mode */
		rc = vl53l0x_fast_start(&fast[i]);
		if (rc != 0) {
			LOG_WRN("VL53L0X[%d] continuous start failed: %d", i, rc);
			continue;
		}

		fast_count++;
	}

	fast_mode = (fast_count > 0);
	LOG_INF("VL53L0X: %d/%d continuous mode", fast_count, online_count);
}

/* ─── Poll ────────────────────────────────────────────────────────────────── */
/* Port of Car::read_sensors() from luna_car.h:323-347 */

static void store_mm(int i, int mm)
{
	if (mm >= VL53L0X_MAX_RAW || mm <= 0) {
		distances[i] = 9999;
	} else {
		distances[i] = (mm < MAX_SENSOR_RANGE) ? mm : MAX_SENSOR_RANGE;
	}
}

static void poll_one(int i)
{
	if (!vl53_valid[i]) {
		distances[i] = 9999;
		return;
	}

	/* Fast path: non-blocking read from continuous mode (~1 ms) */
	if (fast_mode && fast[i].continuous) {
		int mm = vl53l0x_fast_read(&fast[i]);
		if (mm < 0) {
			return; /* -EAGAIN or -EIO: keep previous value */
		}
		store_mm(i, mm);
		return;
	}

	/* Fallback: stock driver blocking read (~33 ms) */
	int rc = sensor_sample_fetch(vl53_devs[i]);
	if (rc != 0) {
		return;
	}

	struct sensor_value val;
	rc = sensor_channel_get(vl53_devs[i], SENSOR_CHAN_DISTANCE, &val);
	if (rc != 0) {
		return;
	}

	int mm = val.val1 * 1000 + val.val2 / 1000;
	store_mm(i, mm);
}

int *sensors_poll(void)
{
	for (int i = 0; i < SENSOR_COUNT; i++) {
		poll_one(i);
	}
	return distances;
}

int *sensors_poll_mask(uint8_t mask)
{
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (mask & BIT(i)) {
			poll_one(i);
		}
	}
	return distances;
}

int sensors_online_count(void)
{
	return online_count;
}

const int *sensors_get_distances(void)
{
	return distances;
}
