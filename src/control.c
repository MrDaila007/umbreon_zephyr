/*
 * control.c — Main 40ms control loop
 *
 * Ported from Umbreon_roborace.ino: work(), loop(), go_back(), stuck/wrong-dir detection
 *
 * Thread: control_thread (priority 2, stack 4096, period ~40ms)
 */

#include "control.h"
#include "settings.h"
#include "car.h"
#include "tachometer.h"
#include "sensors.h"
#include "imu.h"
#include "wifi_cmd.h"
#include "battery.h"
#include "track_learn.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

/* ─── Thread config ───────────────────────────────────────────────────────── */
#define CONTROL_STACK_SIZE 4096
#define CONTROL_PRIORITY   2
static K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
static struct k_thread control_thread_data;

/* ─── State ───────────────────────────────────────────────────────────────── */
static volatile bool car_running;
static volatile bool manual_mode;
static volatile bool drv_enabled;
static volatile int manual_steer;
static volatile float manual_speed;
static volatile int64_t last_drv_ms;

/* Stuck detection (persistent across work() calls) */
static int stuck_time;
static float turns;

/* ─── Wall follow bias ────────────────────────────────────────────────────── */
#define WALL_FOLLOW_BIAS 800

/* ─── go_back() ───────────────────────────────────────────────────────────── */
/* Port from Umbreon_roborace.ino:1060-1084 */

extern void wdt_feed_kick(void);

static void go_back(void)
{
	car_write_speed(0);
	int64_t deadline = k_uptime_get() + 2000;
	while (taho_get_speed() > 0.1f && k_uptime_get() < deadline) {
		wdt_feed_kick();
		k_msleep(10);
	}
	car_write_speed(-150);
	k_msleep(200);
	wdt_feed_kick();
	car_write_speed(0);
	k_msleep(80);
	car_write_speed(-150);
	k_msleep(700);
	wdt_feed_kick();
	car_write_speed(0);
}

static void go_back_long(void)
{
	car_write_speed(0);
	int64_t deadline = k_uptime_get() + 2000;
	while (taho_get_speed() > 0.1f && k_uptime_get() < deadline) {
		wdt_feed_kick();
		k_msleep(10);
	}
	car_write_speed(-150);
	k_msleep(1000);
	wdt_feed_kick();
	car_write_speed(0);
	k_msleep(80);
	car_write_speed(-150);
	k_msleep(1800);
	wdt_feed_kick();
	car_write_speed(0);
}

/* ─── Telemetry ───────────────────────────────────────────────────────────── */

static void send_telemetry(int *s, int steer_val, float spd_target)
{
	wifi_cmd_printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.2f,%.1f,%.1f,%.1f\n",
			k_uptime_get(),
			s[0], s[1], s[2], s[3], s[4], s[5],
			steer_val,
			(double)taho_get_speed(),
			(double)spd_target,
			(double)imu_get_yaw_rate(),
			(double)imu_get_heading());
}

/* ─── Idle telemetry (when not driving) ───────────────────────────────────── */

static void send_idle_telemetry(void)
{
	int *s = sensors_poll();
	imu_update();
	send_telemetry(s, 0, 0.0f);
}

/* ─── work() — main autonomous control ────────────────────────────────────── */
/* Port from Umbreon_roborace.ino:1087-1240 */

static void work(void)
{
	int *s = sensors_poll();
	imu_update();

	/* ── Steering ──────────────────────────────────────────────────────── */
	int diff;
	bool f_l = s[IDX_FRONT_LEFT]  < cfg.front_obstacle_dist;
	bool f_r = s[IDX_FRONT_RIGHT] < cfg.front_obstacle_dist;

	if (s[IDX_LEFT] > cfg.side_open_dist && s[IDX_RIGHT] > cfg.side_open_dist) {
		diff = WALL_FOLLOW_BIAS;
	} else {
		diff = s[IDX_RIGHT] - s[IDX_LEFT];
	}

	/* All sensors close: hard turn */
	bool all_close = true;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (s[i] >= cfg.all_close_dist) {
			all_close = false;
			break;
		}
	}
	if (all_close) {
		diff = WALL_FOLLOW_BIAS;
	}

	/* Hard-side blend */
	diff += (int)((s[IDX_HARD_RIGHT] - s[IDX_HARD_LEFT]) * 0.25f);

	/* ── Speed ─────────────────────────────────────────────────────────── */
	int how_clear = (int)f_l + (int)f_r;
	float coef, spd;

	if (how_clear == 0) {
		coef = cfg.coe_clear;
		spd = cfg.spd_clear;
	} else {
		coef = cfg.coe_blocked;
		spd = cfg.spd_blocked;
	}

	/* Track learning integration */
	if (track_learn_get_mode() == TRK_MODE_RACE) {
		float rec = track_learn_recommend_speed(0.5f);
		if (rec > 0) {
			spd = rec;
		}
	}
	track_learn_tick((int)(diff * coef), spd);

	/* ── Actuation ─────────────────────────────────────────────────────── */
	car_write_steer((int)(diff * coef));
	car_write_speed_ms(spd);
	car_pid_control();

	/* ── Telemetry ─────────────────────────────────────────────────────── */
	send_telemetry(s, (int)(diff * coef), spd);

	/* ── Stuck detection ───────────────────────────────────────────────── */
	bool c_fl = s[IDX_FRONT_LEFT]  < cfg.close_front_dist;
	bool c_fr = s[IDX_FRONT_RIGHT] < cfg.close_front_dist;
	bool low_speed = taho_get_speed() < 0.1f;

	if (c_fl || c_fr || low_speed) {
		stuck_time++;
	} else {
		stuck_time = 0;
	}

	if (stuck_time > cfg.stuck_thresh) {
		car_write_steer(0);
		go_back();
		car_write_speed_ms(2.0f);
		stuck_time = 0;
		imu_reset_heading();
	}

	/* ── Wrong-direction detection ─────────────────────────────────────── */
	turns += imu_get_yaw_rate() * (cfg.loop_ms / 1000.0f);
	if (cfg.race_cw && turns < 0.0f) {
		turns *= 0.97f;
	}
	if (!cfg.race_cw && turns > 0.0f) {
		turns *= 0.97f;
	}
	turns = CLAMP(turns, -200.0f, 200.0f);

	bool wrong_way = cfg.race_cw ? (turns > cfg.wrong_dir_deg)
				      : (turns < -cfg.wrong_dir_deg);

	if (wrong_way) {
		car_write_speed(0);
		k_msleep(100);
		car_write_steer(cfg.race_cw ? 1000 : -1000);
		k_msleep(20);
		go_back_long();
		car_write_steer(cfg.race_cw ? -700 : 700);
		car_write_speed_ms(2.0f);

		int64_t strt = k_uptime_get();
		while ((k_uptime_get() - strt) < 900) {
			sensors_poll();
			imu_update();
			car_pid_control();
			k_msleep(10);
		}
		turns = 0.0f;
		imu_reset_heading();
	}
}

/* ─── Control thread ──────────────────────────────────────────────────────── */

static void control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Control thread started (period=%d ms)", cfg.loop_ms);

	int64_t next_loop = k_uptime_get();

	while (1) {
		wdt_feed_kick();

		int64_t now = k_uptime_get();
		if (now < next_loop) {
			k_msleep(next_loop - now);
			now = k_uptime_get();
		}
		next_loop = (next_loop + cfg.loop_ms > now)
			    ? next_loop + cfg.loop_ms
			    : now + cfg.loop_ms;

		/* Manual drive active? */
		bool drv_active = drv_enabled && manual_mode &&
				  (k_uptime_get() - last_drv_ms < 500);

		if (car_running && !drv_active) {
			/* Autonomous mode */
			manual_mode = false;
			work();
		} else if (drv_active) {
			/* Manual drive mode */
			int *s = sensors_poll();
			imu_update();
			car_write_steer(manual_steer);
			car_write_speed_ms(manual_speed);
			car_pid_control();
			send_telemetry(s, manual_steer, manual_speed);
		} else {
			/* Idle — still send telemetry */
			send_idle_telemetry();
		}

		/* Low-voltage safety cutoff */
		static int64_t bat_low_since;
		if (cfg.bat_enabled && battery_get_voltage() > 0.5f &&
		    battery_get_voltage() < cfg.bat_low) {
			if (bat_low_since == 0) {
				bat_low_since = now;
			} else if (now - bat_low_since > 10000) {
				if (car_running || drv_enabled) {
					car_running = false;
					drv_enabled = false;
					manual_mode = false;
					car_write_speed(0);
					car_write_steer(0);
					wifi_cmd_send("$STS:STOP\n");
					wifi_cmd_send("$T:BAT,phase=LOW_VOLTAGE_CUTOFF\n");
				}
			}
		} else {
			bat_low_since = 0;
		}
	}
}

/* ─── Public API ──────────────────────────────────────────────────────────── */

void control_init(void)
{
	k_thread_create(&control_thread_data, control_stack,
			K_THREAD_STACK_SIZEOF(control_stack),
			control_thread, NULL, NULL, NULL,
			CONTROL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&control_thread_data, "control");

	LOG_INF("Control thread created");
}

bool control_is_running(void)
{
	return car_running;
}

void control_cmd_start(void)
{
	car_pid_reset();
	imu_reset_heading();
	stuck_time = 0;
	turns = 0.0f;
	wifi_cmd_send("$ACK\n");

	/* 5-second countdown — idle telemetry flows */
	int64_t start_at = k_uptime_get() + 5000;
	while (k_uptime_get() < start_at) {
		wdt_feed_kick();
		send_idle_telemetry();
		k_msleep(cfg.loop_ms);
	}

	car_running = true;
	wifi_cmd_send("$STS:RUN\n");
}

void control_cmd_stop(void)
{
	car_running = false;
	drv_enabled = false;
	manual_mode = false;
	manual_steer = 0;
	manual_speed = 0.0f;
	car_write_speed(0);
	car_write_steer(0);
	track_learn_stop();
	wifi_cmd_send("$ACK\n");
	wifi_cmd_send("$STS:STOP\n");
}

void control_set_manual(int steer, float speed)
{
	manual_steer = steer;
	manual_speed = speed;
	manual_mode = true;
	last_drv_ms = k_uptime_get();
}

void control_set_drv_enabled(bool enabled)
{
	drv_enabled = enabled;
	if (!enabled) {
		manual_mode = false;
		manual_steer = 0;
		manual_speed = 0.0f;
	}
}
