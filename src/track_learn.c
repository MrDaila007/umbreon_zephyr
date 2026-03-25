/*
 * track_learn.c — Track profile learning & race mode
 *
 * Ported from track_learn.h (Arduino version).
 * Records distance-indexed speed/steering profile during qualification lap.
 * Race mode uses learned profile for anticipatory speed control.
 *
 * Storage: NVS keys 2 (header) and 3 (data).
 */

#include "track_learn.h"
#include "settings.h"
#include "tachometer.h"
#include "wifi_cmd.h"

#include <zephyr/kernel.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(track_learn, LOG_LEVEL_INF);

/* ─── Constants ───────────────────────────────────────────────────────────── */
#define TRK_MAGIC           0x4B435254  /* "TRCK" */
#define TRK_MAX_POINTS      900
#define TRK_SAMPLE_DIST_CM  10

#define NVS_KEY_TRACK_HDR   2
#define NVS_KEY_TRACK_DATA  3

/* ─── Data structures ────────────────────────────────────────────────────── */
struct __attribute__((packed)) track_point {
	uint16_t dist_cm;
	int8_t   steer;       /* steering_cmd / 8 */
	uint8_t  speed_x10;   /* target speed × 10 */
};  /* 4 bytes */

struct __attribute__((packed)) track_header {
	uint32_t magic;
	uint16_t count;
	uint16_t lap_dist_cm;
	uint8_t  checksum;
	uint8_t  _pad[3];
};  /* 12 bytes */

/* ─── State ───────────────────────────────────────────────────────────────── */
static struct track_point  trk_points[TRK_MAX_POINTS];
static struct track_header trk_hdr;
static int trk_mode = TRK_MODE_IDLE;

static float trk_odo_m;
static float trk_last_sample_m;
static uint32_t trk_taho_start;

/* ─── NVS accessor ────────────────────────────────────────────────────────── */
extern struct nvs_fs *settings_get_nvs(void);

/* ─── Odometry ────────────────────────────────────────────────────────────── */

static float trk_get_odo_m(void)
{
	uint32_t counts = taho_get_count() - trk_taho_start;
	if (cfg.encoder_holes <= 0) return 0.0f;
	return (counts * 3.14159265f * cfg.wheel_diam_m) / (float)cfg.encoder_holes;
}

/* ─── Checksum ────────────────────────────────────────────────────────────── */

static uint8_t trk_checksum(void)
{
	uint8_t sum = 0;
	const uint8_t *p = (const uint8_t *)&trk_hdr;
	for (size_t i = 0; i < 8; i++) sum += p[i];
	for (uint16_t i = 0; i < trk_hdr.count; i++) {
		const uint8_t *pp = (const uint8_t *)&trk_points[i];
		for (size_t j = 0; j < sizeof(struct track_point); j++) sum += pp[j];
	}
	return sum;
}

/* ─── Learning ────────────────────────────────────────────────────────────── */

static void trk_start_learn(void)
{
	trk_mode = TRK_MODE_LEARN;
	trk_hdr.count = 0;
	trk_hdr.lap_dist_cm = 0;
	trk_taho_start = taho_get_count();
	trk_odo_m = 0.0f;
	trk_last_sample_m = 0.0f;
}

static void trk_stop_learn(void)
{
	if (trk_mode != TRK_MODE_LEARN) return;
	trk_hdr.magic = TRK_MAGIC;
	trk_hdr.lap_dist_cm = (uint16_t)(trk_get_odo_m() * 100);
	trk_mode = TRK_MODE_IDLE;
}

void track_learn_tick(int steer_cmd, float target_speed)
{
	if (trk_mode != TRK_MODE_LEARN) return;

	trk_odo_m = trk_get_odo_m();
	const float sample_m = TRK_SAMPLE_DIST_CM / 100.0f;

	if ((trk_odo_m - trk_last_sample_m) >= sample_m && trk_hdr.count < TRK_MAX_POINTS) {
		struct track_point *pt = &trk_points[trk_hdr.count];
		pt->dist_cm   = (uint16_t)(trk_odo_m * 100);
		pt->steer     = (int8_t)CLAMP(steer_cmd / 8, -125, 125);
		pt->speed_x10 = (uint8_t)CLAMP((int)(target_speed * 10), 0, 255);
		trk_hdr.count++;
		trk_last_sample_m = trk_odo_m;
	}
}

/* ─── Race mode ───────────────────────────────────────────────────────────── */

static void trk_start_race(void)
{
	if (trk_hdr.count < 2) return;
	trk_mode = TRK_MODE_RACE;
	trk_taho_start = taho_get_count();
	trk_odo_m = 0.0f;
}

float track_learn_recommend_speed(float lookahead_m)
{
	if (trk_mode != TRK_MODE_RACE || trk_hdr.count < 2) return -1.0f;

	trk_odo_m = trk_get_odo_m();
	float lap_m = trk_hdr.lap_dist_cm / 100.0f;
	if (lap_m <= 0.0f) return -1.0f;

	float query_m = fmodf(trk_odo_m + lookahead_m, lap_m);
	uint16_t query_cm = (uint16_t)(query_m * 100);

	/* Binary search */
	int lo = 0, hi = (int)trk_hdr.count - 1;
	while (lo < hi) {
		int mid = (lo + hi) / 2;
		if (trk_points[mid].dist_cm < query_cm) lo = mid + 1;
		else hi = mid;
	}

	/* Min speed in ±3 window */
	float min_spd = 99.0f;
	int start = (lo - 3 > 0) ? lo - 3 : 0;
	int end = (lo + 3 < (int)trk_hdr.count - 1) ? lo + 3 : (int)trk_hdr.count - 1;
	for (int i = start; i <= end; i++) {
		float s = trk_points[i].speed_x10 / 10.0f;
		if (s > 0.01f && s < min_spd) min_spd = s;
	}

	return (min_spd < 99.0f) ? min_spd : -1.0f;
}

/* ─── Stop / clear ────────────────────────────────────────────────────────── */

void track_learn_stop(void)
{
	trk_stop_learn();
	if (trk_mode == TRK_MODE_RACE) trk_mode = TRK_MODE_IDLE;
}

int track_learn_get_mode(void)
{
	return trk_mode;
}

/* ─── NVS persistence ─────────────────────────────────────────────────────── */

static bool trk_save(void)
{
	struct nvs_fs *nvs = settings_get_nvs();
	if (!nvs || trk_hdr.count == 0) return false;

	trk_hdr.magic = TRK_MAGIC;
	trk_hdr.checksum = trk_checksum();

	ssize_t rc = nvs_write(nvs, NVS_KEY_TRACK_HDR, &trk_hdr, sizeof(trk_hdr));
	if (rc < 0) {
		LOG_ERR("Track header NVS write failed: %zd", rc);
		return false;
	}

	size_t data_size = trk_hdr.count * sizeof(struct track_point);
	rc = nvs_write(nvs, NVS_KEY_TRACK_DATA, trk_points, data_size);
	if (rc < 0) {
		LOG_ERR("Track data NVS write failed: %zd", rc);
		return false;
	}

	LOG_INF("Track saved: %u points, %u cm", trk_hdr.count, trk_hdr.lap_dist_cm);
	return true;
}

static bool trk_load(void)
{
	struct nvs_fs *nvs = settings_get_nvs();
	if (!nvs) return false;

	struct track_header h;
	ssize_t len = nvs_read(nvs, NVS_KEY_TRACK_HDR, &h, sizeof(h));
	if (len != sizeof(h)) return false;
	if (h.magic != TRK_MAGIC || h.count == 0 || h.count > TRK_MAX_POINTS) return false;

	size_t data_size = h.count * sizeof(struct track_point);
	len = nvs_read(nvs, NVS_KEY_TRACK_DATA, trk_points, data_size);
	if (len != (ssize_t)data_size) return false;

	trk_hdr = h;
	if (trk_checksum() != trk_hdr.checksum) {
		memset(&trk_hdr, 0, sizeof(trk_hdr));
		return false;
	}

	LOG_INF("Track loaded: %u points, %u cm", trk_hdr.count, trk_hdr.lap_dist_cm);
	return true;
}

static void trk_clear(void)
{
	trk_mode = TRK_MODE_IDLE;
	memset(&trk_hdr, 0, sizeof(trk_hdr));
}

/* ─── Status output ───────────────────────────────────────────────────────── */

static void trk_send_status(void)
{
	const char *mode_str = (trk_mode == TRK_MODE_LEARN) ? "LEARN" :
			       (trk_mode == TRK_MODE_RACE)  ? "RACE"  : "IDLE";
	wifi_cmd_printf("$TRK:STS,mode=%s,pts=%u,dist=%u,max=%d",
			mode_str, trk_hdr.count, trk_hdr.lap_dist_cm, TRK_MAX_POINTS);
	if (trk_mode != TRK_MODE_IDLE) {
		wifi_cmd_printf(",odo=%d", (int)(trk_odo_m * 100));
	}
	wifi_cmd_send("\n");
}

static void trk_send_data(void)
{
	wifi_cmd_printf("$TRK:HDR,pts=%u,lap=%u\n", trk_hdr.count, trk_hdr.lap_dist_cm);
	for (uint16_t i = 0; i < trk_hdr.count; i++) {
		wifi_cmd_printf("$TRK:D,%u,%u,%d,%u\n",
				i, trk_points[i].dist_cm,
				trk_points[i].steer, trk_points[i].speed_x10);
	}
	wifi_cmd_send("$TRK:DONE\n");
}

/* ─── Command dispatcher ─────────────────────────────────────────────────── */

void track_learn_dispatch(const char *sub)
{
	if (strcmp(sub, "LEARN") == 0) {
		trk_start_learn();
		wifi_cmd_send("$ACK\n");
		trk_send_status();
	} else if (strcmp(sub, "STOP") == 0) {
		track_learn_stop();
		wifi_cmd_send("$ACK\n");
		trk_send_status();
	} else if (strcmp(sub, "SAVE") == 0) {
		if (trk_save()) {
			wifi_cmd_send("$ACK\n");
			trk_send_status();
		} else {
			wifi_cmd_send("$NAK:no_data\n");
		}
	} else if (strcmp(sub, "LOAD") == 0) {
		if (trk_load()) {
			wifi_cmd_send("$ACK\n");
			trk_send_status();
		} else {
			wifi_cmd_send("$NAK:no_track\n");
		}
	} else if (strcmp(sub, "CLR") == 0) {
		trk_clear();
		wifi_cmd_send("$ACK\n");
		trk_send_status();
	} else if (strcmp(sub, "RACE") == 0) {
		if (trk_hdr.count >= 2) {
			trk_start_race();
			wifi_cmd_send("$ACK\n");
			trk_send_status();
		} else {
			wifi_cmd_send("$NAK:no_track\n");
		}
	} else if (strcmp(sub, "STATUS") == 0) {
		trk_send_status();
	} else if (strcmp(sub, "GET") == 0) {
		trk_send_data();
	} else {
		wifi_cmd_send("$NAK:unknown_trk_cmd\n");
	}
}

/* ─── Init ────────────────────────────────────────────────────────────────── */

void track_learn_init(void)
{
	trk_load();  /* silent fail if no saved track */
}
