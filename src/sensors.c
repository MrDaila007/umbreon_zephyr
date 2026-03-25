/*
 * sensors.c — 6× VL53L0X ToF sensor array
 *
 * Uses Zephyr's built-in st,vl53l0x driver with xshut-gpios.
 * The driver handles XSHUT sequencing and I2C address assignment automatically.
 *
 * Sensor order: [Hard-Right, Front-Right, Right, Left, Front-Left, Hard-Left]
 */

#include "sensors.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

#define VL53L0X_MAX_RAW  8190  /* sensor overflow / out-of-range indicator */

/* ─── Device handles ──────────────────────────────────────────────────────── */
static const struct device *vl53_devs[SENSOR_COUNT];
static bool vl53_valid[SENSOR_COUNT];
static int distances[SENSOR_COUNT]; /* cm×10 */
static int online_count;

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
}

/* ─── Poll ────────────────────────────────────────────────────────────────── */
/* Port of Car::read_sensors() from luna_car.h:323-347 */

int *sensors_poll(void)
{
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (!vl53_valid[i]) {
			distances[i] = 9999;
			continue;
		}

		int rc = sensor_sample_fetch(vl53_devs[i]);
		if (rc != 0) {
			/* Keep previous value on transient error */
			continue;
		}

		struct sensor_value val;
		rc = sensor_channel_get(vl53_devs[i], SENSOR_CHAN_DISTANCE, &val);
		if (rc != 0) {
			continue;
		}

		/* Zephyr VL53L0X driver returns distance in meters:
		 * val.val1 = integer meters, val.val2 = fractional (in micro, i.e. 1e-6)
		 * Convert to mm: val1*1000 + val2/1000 */
		int mm = val.val1 * 1000 + val.val2 / 1000;
		if (mm >= VL53L0X_MAX_RAW || mm <= 0) {
			distances[i] = 9999;
		} else {
			/* mm == cm×10 (identity conversion), cap at MAX_SENSOR_RANGE */
			distances[i] = (mm < MAX_SENSOR_RANGE) ? mm : MAX_SENSOR_RANGE;
		}
	}

	return distances;
}

int sensors_online_count(void)
{
	return online_count;
}
