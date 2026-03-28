/*
 * imu.c — MPU-6050 gyroscope Z-axis for heading determination
 *
 * Ported from luna_car.h: imu_init(), imu_calibrate(), imu_update()
 * Uses Zephyr's built-in invensense,mpu6050 driver.
 */

#include "imu.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

extern void wdt_feed_kick(void);

/* ─── Constants ───────────────────────────────────────────────────────────── */
#define IMU_EMA_ALPHA  0.3f   /* EMA smoothing (low = smooth, high = responsive) */
#define IMU_DEADZONE   0.4f   /* ignore rates below this (°/s) — kills drift */
#define CAL_SAMPLES    200
#define CAL_DELAY_MS   5

/* ─── State ───────────────────────────────────────────────────────────────── */
static const struct device *mpu_dev;
static bool mpu_ok;
static float gyro_bias;
static float yaw_rate;
static float heading;
static int64_t prev_us;

/* ─── Init ────────────────────────────────────────────────────────────────── */

void imu_init(void)
{
	mpu_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(mpu6050));
	if (!mpu_dev || !device_is_ready(mpu_dev)) {
		LOG_WRN("MPU-6050 not ready");
		mpu_ok = false;
		return;
	}

	/* Set gyro full-scale to ±500°/s */
	struct sensor_value fs = { .val1 = 500, .val2 = 0 };
	int rc = sensor_attr_set(mpu_dev, SENSOR_CHAN_GYRO_Z,
				 SENSOR_ATTR_FULL_SCALE, &fs);
	if (rc) {
		LOG_WRN("MPU-6050 gyro FS set failed: %d (using driver default)", rc);
	}

	mpu_ok = true;
	prev_us = k_ticks_to_us_floor64(k_uptime_ticks());
	LOG_INF("MPU-6050 init OK");
}

/* ─── Calibrate ───────────────────────────────────────────────────────────── */
/* Port of Car::imu_calibrate() from luna_car.h:466-494 */

void imu_calibrate(void)
{
	if (!mpu_ok) {
		return;
	}

	LOG_INF("IMU: calibrating gyro bias (~1s, keep still)...");

	float sum = 0.0f;
	int count = 0;

	for (int i = 0; i < CAL_SAMPLES; i++) {
		if ((i % 40) == 0) {
			wdt_feed_kick();
		}
		int rc = sensor_sample_fetch(mpu_dev);
		if (rc != 0) {
			k_msleep(CAL_DELAY_MS);
			continue;
		}

		struct sensor_value val;
		rc = sensor_channel_get(mpu_dev, SENSOR_CHAN_GYRO_Z, &val);
		if (rc != 0) {
			k_msleep(CAL_DELAY_MS);
			continue;
		}

		/* Zephyr returns gyro in rad/s as sensor_value.
		 * Convert to °/s: val_rad * (180/π) */
		float rad_s = sensor_value_to_float(&val);
		float deg_s = rad_s * (180.0f / (float)M_PI);
		sum += deg_s;
		count++;

		k_msleep(CAL_DELAY_MS);
	}

	if (count > 0) {
		gyro_bias = sum / count;
	}

	LOG_INF("IMU: bias = %.3f deg/s (%d samples)", (double)gyro_bias, count);
}

/* ─── Update ──────────────────────────────────────────────────────────────── */
/* Port of Car::imu_update() from luna_car.h:499-525 */

void imu_update(void)
{
	if (!mpu_ok) {
		return;
	}

	int rc = sensor_sample_fetch(mpu_dev);
	if (rc != 0) {
		return;
	}

	struct sensor_value val;
	rc = sensor_channel_get(mpu_dev, SENSOR_CHAN_GYRO_Z, &val);
	if (rc != 0) {
		return;
	}

	/* Convert rad/s → °/s, subtract bias */
	float raw_rate = sensor_value_to_float(&val) * (180.0f / (float)M_PI) - gyro_bias;

	if (cfg.imu_rotate) {
		raw_rate = -raw_rate;
	}

	/* Dead zone */
	if (fabsf(raw_rate) < IMU_DEADZONE) {
		raw_rate = 0.0f;
	}

	/* EMA low-pass filter */
	yaw_rate = IMU_EMA_ALPHA * raw_rate + (1.0f - IMU_EMA_ALPHA) * yaw_rate;

	/* Integrate heading */
	int64_t now = k_ticks_to_us_floor64(k_uptime_ticks());
	float dt = (now - prev_us) / 1e6f;
	prev_us = now;

	if (dt > 0.0f && dt < 0.5f) {
		heading += yaw_rate * dt;
		if (heading > 360.0f || heading < -360.0f) {
			heading = fmodf(heading, 360.0f);
		}
	}
}

/* ─── Accessors ───────────────────────────────────────────────────────────── */

void imu_reset_heading(void)
{
	heading = 0.0f;
}

bool imu_is_ok(void)
{
	return mpu_ok;
}

float imu_get_yaw_rate(void)
{
	return yaw_rate;
}

float imu_get_heading(void)
{
	return heading;
}
