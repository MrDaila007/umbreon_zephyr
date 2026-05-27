/*
 * sensors.c — 6× VL53L0X ToF sensor array
 *
 * Uses the enhanced VL53L0X driver (out-of-tree module) which supports
 * continuous back-to-back measurement mode via standard Zephyr sensor API.
 * Non-blocking reads (~1 ms/sensor instead of ~33 ms in single-shot).
 *
 * Sensor order: [Hard-Right, Front-Right, Right, Left, Front-Left, Hard-Left]
 */

#include "sensors.h"
#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
#include <vl53l0x_enhanced.h>
#endif

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

#define VL53L0X_MAX_RAW  8190  /* sensor overflow / out-of-range indicator */
#define SENSOR_STALE_MS  1500
#define SENSOR_RESTART_COOLDOWN_MS 3000
#define SENSOR_OOR_STALE_MS 3000
#define SENSOR_OOR_MIN_BAD 3
#define SENSOR_OOR_MIN_GOOD 2
#define SENSOR_OOR_GOOD_MAX 1200
#define SENSOR_I2C_SCAN_INTERVAL_MS 1000
#define VL53L0X_WHO_AM_I_REG 0xC0
#define VL53L0X_DEFAULT_ADDR 0x29
#define VL53L0X_EXPECTED_MASK 0x3f

static int distances[SENSOR_COUNT]; /* cm×10 */
static int online_count;
static volatile bool recovery_requested;
static int64_t last_i2c_scan_ms;
static const uint8_t vl53_i2c_addrs[SENSOR_COUNT] = {
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
};

#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
/* ─── Device handles ──────────────────────────────────────────────────────── */
static const struct device *vl53_devs[SENSOR_COUNT];
static bool vl53_valid[SENSOR_COUNT];
static int64_t vl53_last_ok_ms[SENSOR_COUNT];
static int64_t vl53_last_restart_ms[SENSOR_COUNT];
static int64_t vl53_oor_since_ms[SENSOR_COUNT];
static uint16_t vl53_restarts[SENSOR_COUNT];
static uint8_t vl53_error_count[SENSOR_COUNT];

extern void wdt_feed_kick(void);

/* ─── Sensor nodelabel → device mapping ───────────────────────────────────── */
#define VL53_DEV(idx, label) \
	vl53_devs[idx] = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(label))

static void recalc_online_count(void)
{
	online_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (vl53_valid[i]) {
			online_count++;
		}
	}
}

static int configure_continuous(int i)
{
	struct sensor_value val = { .val1 = VL53L0X_PROFILE_HIGH_SPEED };
	int rc = sensor_attr_set(vl53_devs[i], SENSOR_CHAN_DISTANCE,
				 (enum sensor_attribute)SENSOR_ATTR_VL53L0X_PROFILE,
				 &val);
	if (rc != 0) {
		LOG_WRN("VL53L0X[%d] profile set failed: %d", i, rc);
	}

	val.val1 = VL53L0X_MODE_CONTINUOUS;
	rc = sensor_attr_set(vl53_devs[i], SENSOR_CHAN_DISTANCE,
			     (enum sensor_attribute)SENSOR_ATTR_VL53L0X_MODE,
			     &val);
	if (rc != 0) {
		LOG_WRN("VL53L0X[%d] continuous start failed: %d", i, rc);
		return rc;
	}

	return 0;
}

static bool restart_one(int i, const char *reason, bool force)
{
	int64_t now = k_uptime_get();
	if (!vl53_devs[i] || !device_is_ready(vl53_devs[i])) {
		return false;
	}
	if (!force && now - vl53_last_restart_ms[i] < SENSOR_RESTART_COOLDOWN_MS) {
		return false;
	}

	vl53_last_restart_ms[i] = now;
	LOG_WRN("VL53L0X[%d] restarting (%s)", i, reason);

	struct sensor_value val = { 0 };
	int rc = sensor_attr_set(vl53_devs[i], SENSOR_CHAN_DISTANCE,
				 (enum sensor_attribute)SENSOR_ATTR_VL53L0X_RESTART,
				 &val);
	wdt_feed_kick();
	if (rc == 0) {
		rc = configure_continuous(i);
	}

	if (rc == 0) {
		vl53_valid[i] = true;
		vl53_error_count[i] = 0;
		vl53_last_ok_ms[i] = k_uptime_get();
		vl53_restarts[i]++;
		recalc_online_count();
		LOG_WRN("VL53L0X[%d] restart OK (count=%u)", i, vl53_restarts[i]);
		return true;
	}

	vl53_valid[i] = false;
	vl53_error_count[i]++;
	recalc_online_count();
	LOG_ERR("VL53L0X[%d] restart failed: %d", i, rc);
	return false;
}
#endif

/* ─── Init ────────────────────────────────────────────────────────────────── */

void sensors_init(void)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
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
		vl53_last_ok_ms[i] = k_uptime_get();
		vl53_last_restart_ms[i] = 0;
		vl53_oor_since_ms[i] = 0;
		vl53_restarts[i] = 0;
		vl53_error_count[i] = 0;
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
	 * Phase 2: trigger lazy init (calibration) via first blocking fetch,
	 * then configure high-speed profile and continuous mode.
	 */
	int cont_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (!vl53_valid[i]) {
			continue;
		}

		/* First fetch triggers lazy init: XSHUT release, address reconfig,
		 * DataInit, StaticInit, calibration (~50 ms per sensor). */
		int rc = sensor_sample_fetch(vl53_devs[i]);
		wdt_feed_kick();
		if (rc != 0) {
			LOG_WRN("VL53L0X[%d] init fetch failed: %d", i, rc);
			vl53_valid[i] = false;
			online_count--;
			continue;
		}

		rc = configure_continuous(i);
		if (rc != 0) {
			vl53_valid[i] = false;
			online_count--;
		} else {
			vl53_last_ok_ms[i] = k_uptime_get();
			cont_count++;
		}
	}

	LOG_INF("VL53L0X: %d/%d continuous mode", cont_count, online_count);
#else
	online_count = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		distances[i] = 9999;
	}
	LOG_WRN("VL53 disabled by devicetree overlay (HIL no-sensors mode)");
#endif
}

/* ─── Poll ────────────────────────────────────────────────────────────────── */

#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
static void store_mm(int i, int mm)
{
	if (mm >= VL53L0X_MAX_RAW || mm <= 0) {
		distances[i] = 9999;
		if (vl53_oor_since_ms[i] == 0) {
			vl53_oor_since_ms[i] = k_uptime_get();
		}
	} else {
		distances[i] = (mm < MAX_SENSOR_RANGE) ? mm : MAX_SENSOR_RANGE;
		vl53_oor_since_ms[i] = 0;
	}
}

static uint8_t i2c_scan_mask_raw(void)
{
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	uint8_t mask = 0;
	uint8_t reg = VL53L0X_WHO_AM_I_REG;
	uint8_t id[2];

	if (!device_is_ready(i2c1)) {
		return 0;
	}

	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (i2c_write_read(i2c1, vl53_i2c_addrs[i], &reg, 1,
				   id, sizeof(id)) == 0) {
			mask |= BIT(i);
		}
	}

	if (i2c_write_read(i2c1, VL53L0X_DEFAULT_ADDR, &reg, 1,
			   id, sizeof(id)) == 0) {
		mask |= BIT(6);
	}

	return mask;
}

static void check_i2c_health(void)
{
	int64_t now = k_uptime_get();

	if (now - last_i2c_scan_ms < SENSOR_I2C_SCAN_INTERVAL_MS) {
		return;
	}
	last_i2c_scan_ms = now;

	uint8_t mask = i2c_scan_mask_raw();
	if ((mask & VL53L0X_EXPECTED_MASK) != VL53L0X_EXPECTED_MASK ||
	    (mask & BIT(6)) != 0) {
		LOG_WRN("VL53L0X I2C recovery requested: mask=0x%02x", mask);
		recovery_requested = true;
	}
}

static void check_semantic_health(void)
{
	int64_t now = k_uptime_get();
	int bad = 0;
	int good = 0;

	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (distances[i] == 9999 && vl53_oor_since_ms[i] > 0 &&
		    now - vl53_oor_since_ms[i] > SENSOR_OOR_STALE_MS) {
			bad++;
		} else if (distances[i] > 0 && distances[i] < SENSOR_OOR_GOOD_MAX) {
			good++;
		}
	}

	if (bad >= SENSOR_OOR_MIN_BAD && good >= SENSOR_OOR_MIN_GOOD) {
		LOG_WRN("VL53L0X semantic recovery requested: bad_oor=%d good=%d",
			bad, good);
		recovery_requested = true;
	}
}

static void poll_one(int i)
{
	if (!vl53_valid[i]) {
		distances[i] = 9999;
		recovery_requested = true;
		return;
	}

	int rc = sensor_sample_fetch(vl53_devs[i]);
	if (rc != 0) {
		int64_t age_ms = k_uptime_get() - vl53_last_ok_ms[i];
		if (rc != -EAGAIN && vl53_error_count[i] < UINT8_MAX) {
			vl53_error_count[i]++;
		}
		if (age_ms > SENSOR_STALE_MS || vl53_error_count[i] >= 8) {
			recovery_requested = true;
		}
		return; /* -EAGAIN (no new data) or error: keep previous value */
	}

	struct sensor_value val;
	rc = sensor_channel_get(vl53_devs[i], SENSOR_CHAN_DISTANCE, &val);
	if (rc != 0) {
		if (vl53_error_count[i] < UINT8_MAX) {
			vl53_error_count[i]++;
		}
		return;
	}

	vl53_last_ok_ms[i] = k_uptime_get();
	vl53_error_count[i] = 0;

	/* Convert meters.microns to mm (== cm×10) */
	int mm = val.val1 * 1000 + val.val2 / 1000;
	store_mm(i, mm);
}
#endif

int *sensors_poll(void)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	for (int i = 0; i < SENSOR_COUNT; i++) {
		poll_one(i);
	}
	check_semantic_health();
	check_i2c_health();
#endif
	return distances;
}

int *sensors_poll_mask(uint8_t mask)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (mask & BIT(i)) {
			poll_one(i);
		}
	}
#else
	ARG_UNUSED(mask);
#endif
	return distances;
}

int sensors_online_count(void)
{
	return online_count;
}

bool sensors_recovery_needed(void)
{
	return recovery_requested;
}

bool sensors_recover_all(void)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	recovery_requested = false;

	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (device_is_ready(i2c1)) {
		(void)i2c_recover_bus(i2c1);
	}

	for (int attempt = 0; attempt < 3; attempt++) {
		bool ok = true;

		for (int i = 0; i < SENSOR_COUNT; i++) {
			distances[i] = 9999;
			vl53_oor_since_ms[i] = 0;
			if (!restart_one(i, "array_recovery", true)) {
				ok = false;
			}
			wdt_feed_kick();
		}

		for (int pass = 0; pass < 3; pass++) {
			k_msleep(40);
			for (int i = 0; i < SENSOR_COUNT; i++) {
				poll_one(i);
			}
			wdt_feed_kick();
		}

		recalc_online_count();
		if (ok && online_count == SENSOR_COUNT) {
			return true;
		}

		wdt_feed_kick();
		k_msleep(100);
	}

	recalc_online_count();
	return false;
#else
	recovery_requested = false;
	return false;
#endif
}

uint32_t sensors_restart_count(void)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	uint32_t total = 0;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		total += vl53_restarts[i];
	}
	return total;
#else
	return 0;
#endif
}

uint8_t sensors_i2c_scan_mask(void)
{
#if CONFIG_DT_HAS_ST_VL53L0X_ENABLED
	return i2c_scan_mask_raw();
#else
	return 0;
#endif
}

int sensors_i2c_scan_count(void)
{
	return POPCOUNT(sensors_i2c_scan_mask());
}

const int *sensors_get_distances(void)
{
	return distances;
}
