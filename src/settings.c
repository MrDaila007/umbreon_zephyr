/*
 * settings.c — Persistent configuration via Zephyr NVS
 *
 * Stores 34 runtime-configurable parameters to flash.
 * NVS key 1 = CarSettings blob + checksum.
 */

#include "settings.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(settings, LOG_LEVEL_INF);

/* ─── NVS setup ───────────────────────────────────────────────────────────── */
#define NVS_PARTITION      storage_partition
#define NVS_PARTITION_ID   FIXED_PARTITION_ID(NVS_PARTITION)

#define NVS_KEY_SETTINGS   1
#define NVS_KEY_TRACK_HDR  2
#define NVS_KEY_TRACK_DATA 3

#define SETTINGS_MAGIC     0x554D4252  /* "UMBR" */
#define SETTINGS_VERSION   8

static struct nvs_fs nvs;
static bool nvs_ready;

/* ─── NVS storage structure (packed for flash) ────────────────────────────── */
struct __attribute__((packed)) nvs_settings {
	uint32_t magic;
	uint8_t  version;
	int16_t  front_obstacle_dist;
	int16_t  side_open_dist;
	int16_t  all_close_dist;
	int16_t  close_front_dist;
	float    pid_kp;
	float    pid_ki;
	float    pid_kd;
	int16_t  min_speed;
	int16_t  max_speed;
	int16_t  min_bspeed;
	int8_t   min_point;
	int8_t   max_point;
	int8_t   neutral_point;
	int8_t   encoder_holes;
	float    wheel_diam_m;
	int8_t   loop_ms;
	float    spd_clear;
	float    spd_blocked;
	float    spd_slew;
	float    kick_pct;
	int16_t  kick_ms;
	float    coe_clear;
	float    coe_blocked;
	float    wrong_dir_deg;
	uint8_t  race_cw;
	int8_t   stuck_thresh;
	uint8_t  imu_rotate;
	uint8_t  servo_reverse;
	uint8_t  calibrated;
	uint8_t  bat_enabled;
	float    bat_multiplier;
	float    bat_low;
	uint8_t  checksum;
};

/* ─── Global configuration ────────────────────────────────────────────────── */
struct car_settings cfg;

/* ─── Defaults ────────────────────────────────────────────────────────────── */
static void set_defaults(void)
{
	cfg.front_obstacle_dist = DEFAULT_FOD;
	cfg.side_open_dist      = DEFAULT_SOD;
	cfg.all_close_dist      = DEFAULT_ACD;
	cfg.close_front_dist    = DEFAULT_CFD;
	/* PID from $TEST:pidtune logs (2026-03-26): runs 2–3 where best=1 had real τ;
	 * PI rows averaged (66.21/264.86 and 66.64/222.13); KD from run-3 IMC (4.16).
	 * Skipped run-1 best=2 (τ floor 0.05 s → unrealistic KI). Re-tune after hardware change. */
	cfg.pid_kp   = 66.4f;
	cfg.pid_ki   = 243.5f;
	cfg.pid_kd   = 4.16f;
	cfg.min_speed   = 1540;
	cfg.max_speed   = 1600;
	cfg.min_bspeed  = 1460;
	cfg.min_point     = 60;
	cfg.max_point     = 120;
	cfg.neutral_point = 90;
	cfg.encoder_holes = 62;
	cfg.wheel_diam_m  = 0.060f;
	cfg.loop_ms       = 40;
	/* Cruise targets (m/s): safe bench/track defaults, not race pace */
	cfg.spd_clear     = 0.48f;
	cfg.spd_blocked   = 0.32f;
	cfg.spd_slew      = 0.85f;
	cfg.kick_pct      = 18.0f;
	cfg.kick_ms       = 300;
	cfg.coe_clear     = 0.28f;
	cfg.coe_blocked   = 0.65f;
	cfg.wrong_dir_deg = 120.0f;
	cfg.race_cw       = true;
	cfg.stuck_thresh  = 25;
	cfg.imu_rotate    = true;
	cfg.servo_reverse = false;
	cfg.calibrated    = false;
	cfg.bat_enabled    = false;
	cfg.bat_multiplier = 2.8f;
	cfg.bat_low        = 6.0f;
}

/* ─── Checksum ────────────────────────────────────────────────────────────── */
static uint8_t compute_checksum(const struct nvs_settings *s)
{
	uint8_t sum = 0;
	const uint8_t *p = (const uint8_t *)s;
	size_t len = sizeof(*s) - 1;  /* exclude checksum byte */
	for (size_t i = 0; i < len; i++) {
		sum += p[i];
	}
	return sum;
}

/* ─── Pack cfg → nvs_settings ─────────────────────────────────────────────── */
static void populate_nvs(struct nvs_settings *s)
{
	s->magic   = SETTINGS_MAGIC;
	s->version = SETTINGS_VERSION;
	s->front_obstacle_dist = (int16_t)cfg.front_obstacle_dist;
	s->side_open_dist      = (int16_t)cfg.side_open_dist;
	s->all_close_dist      = (int16_t)cfg.all_close_dist;
	s->close_front_dist    = (int16_t)cfg.close_front_dist;
	s->pid_kp       = cfg.pid_kp;
	s->pid_ki       = cfg.pid_ki;
	s->pid_kd       = cfg.pid_kd;
	s->min_speed    = (int16_t)cfg.min_speed;
	s->max_speed    = (int16_t)cfg.max_speed;
	s->min_bspeed   = (int16_t)cfg.min_bspeed;
	s->min_point    = (int8_t)cfg.min_point;
	s->max_point    = (int8_t)cfg.max_point;
	s->neutral_point = (int8_t)cfg.neutral_point;
	s->encoder_holes = (int8_t)cfg.encoder_holes;
	s->wheel_diam_m  = cfg.wheel_diam_m;
	s->loop_ms       = (int8_t)cfg.loop_ms;
	s->spd_clear     = cfg.spd_clear;
	s->spd_blocked   = cfg.spd_blocked;
	s->spd_slew      = cfg.spd_slew;
	s->kick_pct      = cfg.kick_pct;
	s->kick_ms       = (int16_t)cfg.kick_ms;
	s->coe_clear     = cfg.coe_clear;
	s->coe_blocked   = cfg.coe_blocked;
	s->wrong_dir_deg = cfg.wrong_dir_deg;
	s->race_cw       = cfg.race_cw ? 1 : 0;
	s->stuck_thresh  = (int8_t)cfg.stuck_thresh;
	s->imu_rotate    = cfg.imu_rotate ? 1 : 0;
	s->servo_reverse = cfg.servo_reverse ? 1 : 0;
	s->calibrated    = cfg.calibrated ? 1 : 0;
	s->bat_enabled   = cfg.bat_enabled ? 1 : 0;
	s->bat_multiplier = cfg.bat_multiplier;
	s->bat_low       = cfg.bat_low;
	s->checksum      = compute_checksum(s);
}

/* ─── Unpack nvs_settings → cfg ───────────────────────────────────────────── */
static void apply_nvs(const struct nvs_settings *s)
{
	cfg.front_obstacle_dist = s->front_obstacle_dist;
	cfg.side_open_dist      = s->side_open_dist;
	cfg.all_close_dist      = s->all_close_dist;
	cfg.close_front_dist    = s->close_front_dist;
	cfg.pid_kp       = s->pid_kp;
	cfg.pid_ki       = s->pid_ki;
	cfg.pid_kd       = s->pid_kd;
	cfg.min_speed    = s->min_speed;
	cfg.max_speed    = s->max_speed;
	cfg.min_bspeed   = s->min_bspeed;
	cfg.min_point    = s->min_point;
	cfg.max_point    = s->max_point;
	cfg.neutral_point = s->neutral_point;
	cfg.encoder_holes = s->encoder_holes;
	cfg.wheel_diam_m  = s->wheel_diam_m;
	cfg.loop_ms       = s->loop_ms;
	cfg.spd_clear     = s->spd_clear;
	cfg.spd_blocked   = s->spd_blocked;
	cfg.spd_slew      = s->spd_slew;
	cfg.kick_pct      = s->kick_pct;
	cfg.kick_ms       = s->kick_ms;
	if (cfg.kick_ms < 0) {
		cfg.kick_ms = 0;
	}
	if (cfg.kick_ms > 5000) {
		cfg.kick_ms = 5000;
	}
	if (cfg.kick_pct < 0.f) {
		cfg.kick_pct = 0.f;
	}
	if (cfg.kick_pct > 80.f) {
		cfg.kick_pct = 80.f;
	}
	cfg.coe_clear     = s->coe_clear;
	cfg.coe_blocked   = s->coe_blocked;
	cfg.wrong_dir_deg = s->wrong_dir_deg;
	cfg.race_cw       = s->race_cw != 0;
	cfg.stuck_thresh  = s->stuck_thresh;
	cfg.imu_rotate    = s->imu_rotate != 0;
	cfg.servo_reverse = s->servo_reverse != 0;
	cfg.calibrated    = s->calibrated != 0;
	cfg.bat_enabled   = s->bat_enabled != 0;
	cfg.bat_multiplier = s->bat_multiplier;
	cfg.bat_low       = s->bat_low;
}

/* ─── Public API ──────────────────────────────────────────────────────────── */

void settings_init(void)
{
	set_defaults();

	const struct flash_area *fa;
	int rc = flash_area_open(NVS_PARTITION_ID, &fa);
	if (rc) {
		LOG_ERR("Flash area open failed: %d", rc);
		return;
	}

	nvs.flash_device = fa->fa_dev;
	nvs.offset = fa->fa_off;
	nvs.sector_size = 4096;
	nvs.sector_count = fa->fa_size / nvs.sector_size;
	flash_area_close(fa);

	rc = nvs_mount(&nvs);
	if (rc) {
		LOG_ERR("NVS mount failed: %d", rc);
		return;
	}

	nvs_ready = true;
	LOG_INF("NVS ready (%u sectors)", nvs.sector_count);
}

bool settings_load(void)
{
	if (!nvs_ready) {
		return false;
	}

	struct nvs_settings s;
	ssize_t len = nvs_read(&nvs, NVS_KEY_SETTINGS, &s, sizeof(s));
	if (len != sizeof(s)) {
		LOG_INF("No saved settings (len=%zd)", len);
		return false;
	}

	if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION) {
		LOG_WRN("Settings magic/version mismatch");
		return false;
	}

	if (compute_checksum(&s) != s.checksum) {
		LOG_WRN("Settings checksum mismatch");
		return false;
	}

	apply_nvs(&s);
	LOG_INF("Settings loaded from NVS");
	return true;
}

bool settings_save(void)
{
	if (!nvs_ready) {
		return false;
	}

	struct nvs_settings s;
	populate_nvs(&s);

	ssize_t len = nvs_write(&nvs, NVS_KEY_SETTINGS, &s, sizeof(s));
	if (len < 0) {
		LOG_ERR("NVS write failed: %zd", len);
		return false;
	}

	LOG_INF("Settings saved to NVS (%zd bytes)", len);
	return true;
}

void settings_reset(void)
{
	set_defaults();
	LOG_INF("Settings reset to defaults");
}

/* ─── NVS accessor for track_learn ────────────────────────────────────────── */
struct nvs_fs *settings_get_nvs(void)
{
	return nvs_ready ? &nvs : NULL;
}
