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
#include "buzzer.h"
#include "display.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

/* ─── Thread config ───────────────────────────────────────────────────────── */
static K_THREAD_STACK_DEFINE(control_stack, CONFIG_APP_CONTROL_STACK_SIZE);
static struct k_thread control_thread_data;

/* ─── State ───────────────────────────────────────────────────────────────── */
static volatile bool manual_mode;
static volatile bool drv_enabled;
static volatile bool monitor_mode;
static volatile int manual_steer;
static volatile float manual_speed;
static volatile int64_t last_drv_ms;

enum control_start_state {
	CTRL_START_IDLE = 0,
	CTRL_START_COUNTDOWN,
	CTRL_START_RUNNING,
};

static enum control_start_state start_state = CTRL_START_IDLE;
static bool start_cancel_requested;
/* Protects start_state, start_cancel_requested, and external writes to mnv */
static K_MUTEX_DEFINE(start_state_mutex);

/* Stuck detection (persistent across work() calls) */
static int stuck_time;
static int stall_time;
static float turns;

/* ─── RUN sub-state telemetry ────────────────────────────────────────────── */
enum run_substate {
	RUN_CLEAR      = 0,
	RUN_BLOCKED    = 1,
	RUN_STUCK_WAIT = 2,
	RUN_REVERSE    = 3,
	RUN_WRONG_DIR  = 4,
	RUN_STALL      = 5,
	RUN_SENSOR_RECOVERY = 6,
};
static volatile int run_state;
static int run_telem_div;
static int run_csv_telem_div;

#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
struct run_prof_accum {
	uint32_t count;
	uint32_t total_max_us;
	uint64_t total_us;
	uint64_t sensors_us;
	uint64_t recover_us;
	uint64_t imu_us;
	uint64_t logic_us;
	uint64_t act_us;
	uint64_t telemetry_us;
	uint64_t state_us;
	uint64_t stuck_us;
};

static struct run_prof_accum run_prof;

static inline uint32_t prof_now_cyc(void)
{
	return k_cycle_get_32();
}

static inline uint32_t prof_elapsed_us(uint32_t start_cyc, uint32_t end_cyc)
{
	return (uint32_t)k_cyc_to_us_floor64(end_cyc - start_cyc);
}

static void run_prof_add(uint32_t total_us, uint32_t sensors_us,
			 uint32_t recover_us, uint32_t imu_us,
			 uint32_t logic_us, uint32_t act_us,
			 uint32_t telemetry_us, uint32_t state_us,
			 uint32_t stuck_us)
{
	run_prof.count++;
	run_prof.total_us += total_us;
	run_prof.sensors_us += sensors_us;
	run_prof.recover_us += recover_us;
	run_prof.imu_us += imu_us;
	run_prof.logic_us += logic_us;
	run_prof.act_us += act_us;
	run_prof.telemetry_us += telemetry_us;
	run_prof.state_us += state_us;
	run_prof.stuck_us += stuck_us;
	run_prof.total_max_us = MAX(run_prof.total_max_us, total_us);

	if (run_prof.count < CONFIG_APP_RUN_PROFILING_INTERVAL) {
		return;
	}

	uint32_t n = run_prof.count;
	wifi_cmd_printf("$T:RUNPROF,n=%u,total=%u,max=%u,sens=%u,rec=%u,imu=%u,"
			"logic=%u,act=%u,tel=%u,state=%u,stuck=%u\n",
			n,
			(uint32_t)(run_prof.total_us / n),
			run_prof.total_max_us,
			(uint32_t)(run_prof.sensors_us / n),
			(uint32_t)(run_prof.recover_us / n),
			(uint32_t)(run_prof.imu_us / n),
			(uint32_t)(run_prof.logic_us / n),
			(uint32_t)(run_prof.act_us / n),
			(uint32_t)(run_prof.telemetry_us / n),
			(uint32_t)(run_prof.state_us / n),
			(uint32_t)(run_prof.stuck_us / n));
	if (IS_ENABLED(CONFIG_APP_RUN_PROFILING_CONSOLE)) {
		printk("$T:RUNPROF,n=%u,total=%u,max=%u,sens=%u,rec=%u,imu=%u,"
		       "logic=%u,act=%u,tel=%u,state=%u,stuck=%u\n",
		       n,
		       (uint32_t)(run_prof.total_us / n),
		       run_prof.total_max_us,
		       (uint32_t)(run_prof.sensors_us / n),
		       (uint32_t)(run_prof.recover_us / n),
		       (uint32_t)(run_prof.imu_us / n),
		       (uint32_t)(run_prof.logic_us / n),
		       (uint32_t)(run_prof.act_us / n),
		       (uint32_t)(run_prof.telemetry_us / n),
		       (uint32_t)(run_prof.state_us / n),
		       (uint32_t)(run_prof.stuck_us / n));
	}
	memset(&run_prof, 0, sizeof(run_prof));
}
#endif

/* ─── Sensor masks ───────────────────────────────────────────────────────── */
#define MASK_SIDES   (BIT(IDX_LEFT) | BIT(IDX_RIGHT))
#define MASK_FRONT_L (MASK_SIDES | BIT(IDX_FRONT_LEFT))
#define MASK_FRONT_R (MASK_SIDES | BIT(IDX_FRONT_RIGHT))

/* Sensor rotation removed: continuous back-to-back mode allows
 * non-blocking reads (~1 ms/sensor), so we poll all 6 every cycle. */

/* ─── Heartbeat LED ──────────────────────────────────────────────────────── */
static const struct gpio_dt_spec heartbeat_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* ─── Wall follow bias ────────────────────────────────────────────────────── */
#define WALL_FOLLOW_BIAS 800
#define STEER_CMD_LIMIT  1000

/* ─── RUN sub-state message ───────────────────────────────────────────────── */

static void send_run_state(int state, int stuck, float trn, int how_clr, int dif)
{
	wifi_cmd_printf("$RUN:%d,%d,%.1f,%d,%d\n",
			state, stuck, (double)trn, how_clr, dif);
}

static int steer_distance(int d)
{
	if (d <= 0 || d > MAX_SENSOR_RANGE) {
		return MAX_SENSOR_RANGE;
	}
	return d;
}

static int wall_follow_diff(const int *s, int side_open_dist, int all_close_dist)
{
	int left = steer_distance(s[IDX_LEFT]);
	int right = steer_distance(s[IDX_RIGHT]);
	int hard_left = steer_distance(s[IDX_HARD_LEFT]);
	int hard_right = steer_distance(s[IDX_HARD_RIGHT]);
	int diff;

	if (left > side_open_dist && right > side_open_dist) {
		diff = WALL_FOLLOW_BIAS;
	} else {
		diff = right - left;
	}

	bool all_close = true;
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (steer_distance(s[i]) >= all_close_dist) {
			all_close = false;
			break;
		}
	}
	if (all_close) {
		diff = WALL_FOLLOW_BIAS;
	}

	diff += (int)((hard_right - hard_left) * 0.25f);
	return diff;
}

static int clamp_steer_cmd(int steer)
{
	return CLAMP(steer, -STEER_CMD_LIMIT, STEER_CMD_LIMIT);
}

/* ─── Maneuver state machine ─────────────────────────────────────────────── */
/* Non-blocking replacement for go_back() and go_back_long().
 * Called every loop_ms from work() — does not block the control loop.
 *
 * ESC brake-to-reverse protocol: RC ESCs treat the first reverse signal
 * after forward motion as BRAKE.  Must send brake → neutral → reverse
 * to engage actual reverse. */

extern void wdt_feed_kick(void);

#define REVERSE_MIN_DIST  0.13f   /* metres before checking front clear */

enum mnv_phase {
	MNV_NONE = 0,
	/* stuck-escape (go_back) */
	MNV_BACK_WAIT_STOP,
	MNV_BACK_BRAKE,
	MNV_BACK_NEUTRAL,
	MNV_BACK_REVERSE,
	/* wrong-direction (go_back_long) */
	MNV_LONG_WAIT_STOP,
	MNV_LONG_BRAKE,
	MNV_LONG_NEUTRAL,
	MNV_LONG_REVERSE,
	MNV_LONG_FORWARD,
	/* wrong-direction burst (Danila-style) */
	MNV_BURST_STOP,
	MNV_BURST_PRE_STEER,
	MNV_BURST_FORWARD,
};

static volatile enum mnv_phase mnv;
static int64_t  mnv_deadline;
static int      mnv_steer;
static uint32_t mnv_start_count;
static int      mnv_alt;
static bool     mnv_burst_chain;
static float    turns_sensor;

/* ── Maneuver starters ──────────────────────────────────────────────────── */

static void maneuver_start_back(const struct car_settings *c)
{
	int *s  = sensors_poll_mask(MASK_SIDES);
	int left  = s[IDX_LEFT];
	int right = s[IDX_RIGHT];

	if (left > right + 50) {
		mnv_steer = -600;
	} else if (right > left + 50) {
		mnv_steer = 600;
	} else {
		mnv_steer = c->race_cw ? -600 : 600;
	}

	send_run_state(RUN_REVERSE, stuck_time, turns, 0, mnv_steer);
	car_write_speed(0);
	mnv_burst_chain = false;
	mnv_deadline = k_uptime_get() + c->reverse_drive_ms;
	mnv = MNV_BACK_WAIT_STOP;
	stuck_time = 0;
}

static void maneuver_start_burst(const struct car_settings *c)
{
	send_run_state(RUN_WRONG_DIR, stuck_time, turns, 0, 0);
	car_write_speed(0);
	mnv_steer = c->race_cw ? 1000 : -1000;
	car_write_steer(mnv_steer);
	mnv_burst_chain = true;
	mnv_deadline = k_uptime_get() + c->burst_stop_ms;
	mnv = MNV_BURST_STOP;
	stuck_time = 0;
}

static void maneuver_start_long(const struct car_settings *c)
{
	send_run_state(RUN_WRONG_DIR, stuck_time, turns, 0, 0);
	car_write_speed(0);
	car_write_steer(c->race_cw ? 1000 : -1000);
	mnv_burst_chain = false;
	mnv = MNV_LONG_WAIT_STOP;
}

static float control_taho_get_speed(void)
{
	if (IS_ENABLED(CONFIG_APP_CONTROL_INSTANT_TACHOMETER)) {
		return taho_get_instant_speed();
	}

	return taho_get_speed();
}

/* ── One tick of the maneuver state machine ─────────────────────────────── */

static void maneuver_tick(const struct car_settings *c)
{
	int64_t now = k_uptime_get();

	switch (mnv) {

	/* ── stuck-escape phases ──────────────────────────────────────── */

	case MNV_BACK_WAIT_STOP:
		if (control_taho_get_speed() < 0.1f || now >= mnv_deadline) {
			car_write_steer(mnv_steer);
			car_write_speed(c->reverse_brake_cmd);
			mnv_deadline = now + c->reverse_brake_ms;
			mnv = MNV_BACK_BRAKE;
		}
		break;

	case MNV_BACK_BRAKE:
		if (now >= mnv_deadline) {
			car_write_speed(0);
			mnv_deadline = now + 80;
			mnv = MNV_BACK_NEUTRAL;
		}
		break;

	case MNV_BACK_NEUTRAL:
		if (now >= mnv_deadline) {
			car_write_speed(c->reverse_drive_cmd);
			mnv_start_count = taho_get_count();
			mnv_alt = 0;
			mnv_deadline = now + c->reverse_drive_ms;
			mnv = MNV_BACK_REVERSE;
		}
		break;

	case MNV_BACK_REVERSE: {
		int *s = sensors_poll_mask(mnv_alt ? MASK_FRONT_R
						   : MASK_FRONT_L);
		mnv_alt = !mnv_alt;

		uint32_t delta = taho_get_count() - mnv_start_count;
		float dist = ((float)delta * (float)M_PI * c->wheel_diam_m)
			     / (float)c->encoder_holes;

		bool front_clear =
			s[IDX_FRONT_LEFT]  > c->front_obstacle_dist &&
			s[IDX_FRONT_RIGHT] > c->front_obstacle_dist;

		if ((front_clear && dist >= REVERSE_MIN_DIST) ||
		    now >= mnv_deadline) {
			car_write_speed(0);
			if (mnv_burst_chain) {
				mnv_burst_chain = false;
				car_write_steer(c->race_cw ? -1000 : 1000);
				car_write_speed_ms(c->burst_forward_speed);
				mnv_deadline = now + c->burst_forward_ms;
				mnv = MNV_BURST_FORWARD;
				break;
			}
			car_write_steer(mnv_steer);
			imu_reset_heading();
			send_run_state(RUN_REVERSE, 0, turns, 0, mnv_steer);
			mnv = MNV_NONE;
			break;
		}

		int left  = s[IDX_LEFT];
		int right = s[IDX_RIGHT];
		if (left > right + 100) {
			mnv_steer = -600;
		} else if (right > left + 100) {
			mnv_steer = 600;
		}
		car_write_steer(mnv_steer);
		send_run_state(RUN_REVERSE, stuck_time, turns, 0, mnv_steer);
		break;
	}

	/* ── wrong-direction phases ───────────────────────────────────── */

	case MNV_LONG_WAIT_STOP:
		if (control_taho_get_speed() < 0.1f || now >= mnv_deadline) {
			car_write_speed(c->reverse_brake_cmd);
			mnv_deadline = now + c->long_reverse_brake_ms;
			mnv = MNV_LONG_BRAKE;
		}
		break;

	case MNV_LONG_BRAKE:
		if (now >= mnv_deadline) {
			car_write_speed(0);
			mnv_deadline = now + 80;
			mnv = MNV_LONG_NEUTRAL;
		}
		break;

	case MNV_LONG_NEUTRAL:
		if (now >= mnv_deadline) {
			car_write_speed(c->reverse_drive_cmd);
			mnv_deadline = now + c->long_reverse_drive_ms;
			mnv = MNV_LONG_REVERSE;
		}
		break;

	case MNV_LONG_REVERSE:
		if (now >= mnv_deadline) {
			car_write_speed(0);
			car_write_steer(c->race_cw ? -700 : 700);
			car_write_speed_ms(MIN(c->spd_blocked, c->long_forward_speed_cap));
			mnv_deadline = now + c->long_forward_ms;
			mnv = MNV_LONG_FORWARD;
		}
		break;

	case MNV_LONG_FORWARD:
		car_pid_control();
		if (now >= mnv_deadline) {
			turns = 0.0f;
			turns_sensor = 0.0f;
			imu_reset_heading();
			send_run_state(RUN_WRONG_DIR, 0, turns, 0, 0);
			mnv = MNV_NONE;
		}
		break;

	/* ── wrong-direction burst phases ─────────────────────────────── */

	case MNV_BURST_STOP:
		if (now >= mnv_deadline) {
			car_write_steer(mnv_steer);
			mnv_deadline = now + c->burst_pre_steer_ms;
			mnv = MNV_BURST_PRE_STEER;
		}
		break;

	case MNV_BURST_PRE_STEER:
		if (now >= mnv_deadline) {
			mnv_deadline = k_uptime_get() + c->reverse_drive_ms;
			mnv = MNV_BACK_WAIT_STOP;
		}
		break;

	case MNV_BURST_FORWARD:
		car_pid_control();
		if (now >= mnv_deadline) {
			turns = 0.0f;
			turns_sensor = 0.0f;
			imu_reset_heading();
			send_run_state(RUN_WRONG_DIR, 0, turns, 0, 0);
			mnv = MNV_NONE;
		}
		break;

	default:
		mnv = MNV_NONE;
		break;
	}
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

static bool should_send_run_csv(void)
{
	if (CONFIG_APP_RUN_CSV_TELEMETRY_DIVIDER <= 0) {
		return false;
	}
	if (++run_csv_telem_div >= CONFIG_APP_RUN_CSV_TELEMETRY_DIVIDER) {
		run_csv_telem_div = 0;
		return true;
	}
	return false;
}

/* ─── Idle telemetry (when not driving) ───────────────────────────────────── */

static void send_idle_telemetry(void)
{
	int *s = sensors_poll();
	imu_update();
	send_telemetry(s, 0, 0.0f);
}

static bool recover_sensors_while_stopped(void)
{
	run_state = RUN_SENSOR_RECOVERY;
	car_write_speed(0);
	car_write_speed_ms(0.0f);
	car_pid_reset();
	car_write_steer(0);
	mnv = MNV_NONE;
	stuck_time = 0;
	stall_time = 0;

	send_run_state(RUN_SENSOR_RECOVERY, 0, turns, 0, 0);
	wifi_cmd_send("$T:SNS,phase=recovery_start\n");

	bool ok = sensors_recover_all();

	wifi_cmd_printf("$T:SNS,phase=recovery_done,ok=%d,online=%d,restarts=%u\n",
			ok ? 1 : 0,
			sensors_online_count(),
			(unsigned int)sensors_restart_count());

	int *s = sensors_poll();
	imu_update();
	send_telemetry(s, 0, 0.0f);
	return ok;
}

/* ─── work() — main autonomous control ────────────────────────────────────── */
/* Port from Umbreon_roborace.ino:1087-1240 */

static void work(const struct car_settings *c)
{
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	uint32_t prof_total_start = prof_now_cyc();
	uint32_t prof_mark;
	uint32_t prof_sensors_us = 0;
	uint32_t prof_recover_us = 0;
	uint32_t prof_imu_us = 0;
	uint32_t prof_logic_us = 0;
	uint32_t prof_act_us = 0;
	uint32_t prof_telemetry_us = 0;
	uint32_t prof_state_us = 0;
	uint32_t prof_stuck_us = 0;
#define RUN_PROF_FLUSH()							\
	do {									\
		uint32_t prof_total_end = prof_now_cyc();			\
		run_prof_add(prof_elapsed_us(prof_total_start,		\
					     prof_total_end),		\
			     prof_sensors_us, prof_recover_us,		\
			     prof_imu_us, prof_logic_us, prof_act_us,	\
			     prof_telemetry_us, prof_state_us,		\
			     prof_stuck_us);				\
	} while (0)
#else
#define RUN_PROF_FLUSH() do { } while (0)
#endif

	/* Maneuver in progress — advance state machine, skip normal logic */
	if (mnv != MNV_NONE) {
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
		prof_mark = prof_now_cyc();
#endif
		imu_update();
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
		prof_imu_us = prof_elapsed_us(prof_mark, prof_now_cyc());
		prof_mark = prof_now_cyc();
#endif
		maneuver_tick(c);
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
		prof_logic_us = prof_elapsed_us(prof_mark, prof_now_cyc());
#endif
		RUN_PROF_FLUSH();
		return;
	}

#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_mark = prof_now_cyc();
#endif
	int *s = sensors_poll();
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_sensors_us = prof_elapsed_us(prof_mark, prof_now_cyc());
	prof_mark = prof_now_cyc();
#endif
	if (sensors_recovery_needed()) {
		(void)recover_sensors_while_stopped();
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
		prof_recover_us = prof_elapsed_us(prof_mark, prof_now_cyc());
#endif
		RUN_PROF_FLUSH();
		return;
	}
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_recover_us = prof_elapsed_us(prof_mark, prof_now_cyc());
	prof_mark = prof_now_cyc();
#endif
	imu_update();
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_imu_us = prof_elapsed_us(prof_mark, prof_now_cyc());
	prof_mark = prof_now_cyc();
#endif

	/* ── Steering ──────────────────────────────────────────────────────── */
	bool f_l = s[IDX_FRONT_LEFT]  < c->front_obstacle_dist;
	bool f_r = s[IDX_FRONT_RIGHT] < c->front_obstacle_dist;
	int diff = wall_follow_diff(s, c->side_open_dist, c->all_close_dist);

	/* ── Speed ─────────────────────────────────────────────────────────── */
	int how_clear = (int)f_l + (int)f_r;
	float coef, spd;

	if (how_clear == 0) {
		coef = c->coe_clear;
		spd = c->spd_clear;
	} else {
		coef = c->coe_blocked;
		spd = c->spd_blocked;
	}

	int steer_cmd = clamp_steer_cmd((int)(diff * coef));
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_logic_us = prof_elapsed_us(prof_mark, prof_now_cyc());
	prof_mark = prof_now_cyc();
#endif

	/* ── Actuation ─────────────────────────────────────────────────────── */
	car_write_steer(steer_cmd);
	car_write_speed_ms(spd);
	car_pid_control();
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_act_us = prof_elapsed_us(prof_mark, prof_now_cyc());
	prof_mark = prof_now_cyc();
#endif

	/* ── Telemetry ─────────────────────────────────────────────────────── */
	if (should_send_run_csv()) {
		send_telemetry(s, steer_cmd, spd);
	}
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_telemetry_us = prof_elapsed_us(prof_mark, prof_now_cyc());
	prof_mark = prof_now_cyc();
#endif

	/* ── RUN sub-state telemetry ───────────────────────────────────────── */
	int cur_state;
	if (stall_time > 0 && stuck_time == 0) {
		cur_state = RUN_STALL;
	} else if (stuck_time > 0) {
		cur_state = RUN_STUCK_WAIT;
	} else if (how_clear > 0) {
		cur_state = RUN_BLOCKED;
	} else {
		cur_state = RUN_CLEAR;
	}
	run_state = cur_state;

	if (++run_telem_div >= 5) {
		run_telem_div = 0;
		send_run_state(cur_state, stuck_time, turns,
			       how_clear, steer_cmd);
	}
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_state_us = prof_elapsed_us(prof_mark, prof_now_cyc());
	prof_mark = prof_now_cyc();
#endif

	/* ── Stuck detection ───────────────────────────────────────────────── */
	bool c_fl = s[IDX_FRONT_LEFT]  < c->close_front_dist;
	bool c_fr = s[IDX_FRONT_RIGHT] < c->close_front_dist;
	bool low_speed = control_taho_get_speed() < 0.1f;
	bool blocked = c_fl || c_fr;

	/* Path 1: sensor-confirmed wall hit — fast trigger.
	 * Do not depend on tachometer speed here: encoder noise can make a
	 * physically stopped car look like it is still moving. */
	if (blocked) {
		stuck_time++;
	} else {
		stuck_time = 0;
	}
	if (stuck_time > c->stuck_thresh) {
		maneuver_start_back(c);
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
		prof_stuck_us = prof_elapsed_us(prof_mark, prof_now_cyc());
#endif
		RUN_PROF_FLUSH();
		return;
	}

	/* Path 2: motor stall (commanded but not moving) — slow trigger */
	if (c->stall_thresh > 0 && low_speed && spd > 0.05f) {
		stall_time++;
	} else {
		stall_time = 0;
	}
	if (stall_time > c->stall_thresh) {
		stall_time = 0;
		maneuver_start_back(c);
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
		prof_stuck_us = prof_elapsed_us(prof_mark, prof_now_cyc());
#endif
		RUN_PROF_FLUSH();
		return;
	}

	/* ── Wrong-direction detection ─────────────────────────────────────── */
	float speed = control_taho_get_speed();
	bool wrong_way = false;

	if (c->wrong_detect_mode == WRONG_DETECT_SENSOR) {
		int left = steer_distance(s[IDX_LEFT]);
		int right = steer_distance(s[IDX_RIGHT]);
		turns_sensor += (float)(right - left) * speed / 1000.0f;
		turns_sensor = CLAMP(turns_sensor, -10000.0f, 80.0f);
		if (c->race_cw) {
			wrong_way = turns_sensor < c->wrong_sensor_thresh;
		} else {
			wrong_way = turns_sensor > -c->wrong_sensor_thresh;
		}
	} else {
		turns += imu_get_yaw_rate() * (c->loop_ms / 1000.0f);
		if (c->race_cw && turns < 0.0f) {
			turns *= 0.97f;
		}
		if (!c->race_cw && turns > 0.0f) {
			turns *= 0.97f;
		}
		turns = CLAMP(turns, -200.0f, 200.0f);
		wrong_way = c->race_cw ? (turns > c->wrong_dir_deg)
				       : (turns < -c->wrong_dir_deg);
	}

	if (wrong_way) {
		if (c->wrong_maneuver_mode == WRONG_MANEUVER_BURST) {
			maneuver_start_burst(c);
		} else {
			maneuver_start_long(c);
		}
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
		prof_stuck_us = prof_elapsed_us(prof_mark, prof_now_cyc());
#endif
		RUN_PROF_FLUSH();
		return;
	}
#if IS_ENABLED(CONFIG_APP_RUN_PROFILING)
	prof_stuck_us = prof_elapsed_us(prof_mark, prof_now_cyc());
#endif
	RUN_PROF_FLUSH();
#undef RUN_PROF_FLUSH
}

/* ─── work_monitor() — diagnostic mode: sensors + servo, no motor ─────────── */

static void work_monitor(void)
{
	struct car_settings c;
	settings_get_copy(&c);

	int *s = sensors_poll();
	imu_update();

	/* Steering uses the same wall-follow logic as work(), but no motor. */
	int diff = wall_follow_diff(s, c.side_open_dist, c.all_close_dist);
	int steer_val = clamp_steer_cmd((int)(diff * c.coe_clear));
	car_write_steer(steer_val);

	/* No motor — no speed, no PID, no stuck/wrong-dir detection */

	send_telemetry(s, steer_val, 0.0f);
}

static enum control_start_state get_start_state(void)
{
	enum control_start_state st;
	k_mutex_lock(&start_state_mutex, K_FOREVER);
	st = start_state;
	k_mutex_unlock(&start_state_mutex);
	return st;
}

static bool is_start_cancelled(void)
{
	bool cancelled;
	k_mutex_lock(&start_state_mutex, K_FOREVER);
	cancelled = (start_state != CTRL_START_COUNTDOWN) || start_cancel_requested;
	k_mutex_unlock(&start_state_mutex);
	return cancelled;
}

/* ─── Control thread ──────────────────────────────────────────────────────── */

static void control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct car_settings c;
	settings_get_copy(&c);
	LOG_INF("Control thread started (period=%d ms)", c.loop_ms);

	int64_t next_loop = k_uptime_get();

	while (1) {
		wdt_feed_kick();
		gpio_pin_toggle_dt(&heartbeat_led);

		int64_t now = k_uptime_get();
		if (now < next_loop) {
			k_msleep(next_loop - now);
			now = k_uptime_get();
		}
		settings_get_copy(&c);
		next_loop = (next_loop + c.loop_ms > now)
			    ? next_loop + c.loop_ms
			    : now + c.loop_ms;

		/* Manual drive active? */
		bool drv_active = drv_enabled && manual_mode &&
				  (k_uptime_get() - last_drv_ms < 500);
		enum control_start_state st = get_start_state();
		bool running = (st == CTRL_START_RUNNING);
		bool countdown = (st == CTRL_START_COUNTDOWN);

		if (countdown) {
			/* Countdown owner sends idle telemetry to avoid double polling */
		} else if (sensors_recovery_needed()) {
			(void)recover_sensors_while_stopped();
		} else if (monitor_mode && !running) {
			/* Diagnostic monitor mode */
			work_monitor();
		} else if (running && !drv_active) {
			/* Autonomous mode */
			manual_mode = false;
			work(&c);
		} else if (drv_active) {
			/* Manual drive mode */
			int *s = sensors_poll();
			imu_update();
			car_write_steer(manual_steer);
			car_write_speed_ms(manual_speed);
			car_pid_control();
			if (should_send_run_csv()) {
				send_telemetry(s, manual_steer, manual_speed);
			}
		} else {
			/* Idle — still send telemetry */
			send_idle_telemetry();
		}

		/* Low-voltage safety cutoff */
		static int64_t bat_low_since;
		if (c.bat_enabled && battery_get_voltage() > 0.5f &&
		    battery_get_voltage() < c.bat_low) {
			if (bat_low_since == 0) {
				bat_low_since = now;
			} else if (now - bat_low_since > 10000) {
				if (running || countdown || drv_enabled) {
					k_mutex_lock(&start_state_mutex, K_FOREVER);
					start_cancel_requested = true;
					start_state = CTRL_START_IDLE;
					mnv = MNV_NONE;
					k_mutex_unlock(&start_state_mutex);
					display_notify_run_state(false);
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
	if (gpio_is_ready_dt(&heartbeat_led)) {
		gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
	}

	k_thread_create(&control_thread_data, control_stack,
			K_THREAD_STACK_SIZEOF(control_stack),
			control_thread, NULL, NULL, NULL,
			CONFIG_APP_CONTROL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&control_thread_data, "control");

	LOG_INF("Control thread created");
}

bool control_is_running(void)
{
	return get_start_state() == CTRL_START_RUNNING;
}

bool control_is_countdown(void)
{
	return get_start_state() == CTRL_START_COUNTDOWN;
}

bool control_request_start(void)
{
	bool accepted = false;
	k_mutex_lock(&start_state_mutex, K_FOREVER);
	if (start_state == CTRL_START_IDLE) {
		start_state = CTRL_START_COUNTDOWN;
		start_cancel_requested = false;
		accepted = true;
	}
	k_mutex_unlock(&start_state_mutex);
	return accepted;
}

void control_cancel_start_request(void)
{
	k_mutex_lock(&start_state_mutex, K_FOREVER);
	if (start_state == CTRL_START_COUNTDOWN) {
		start_cancel_requested = true;
		start_state = CTRL_START_IDLE;
	}
	k_mutex_unlock(&start_state_mutex);
}

void control_cmd_start(void)
{
	if (!control_is_countdown()) {
		return;
	}

	car_pid_reset();
	imu_reset_heading();
	stuck_time = 0;
	stall_time = 0;
	turns = 0.0f;
	k_mutex_lock(&start_state_mutex, K_FOREVER);
	mnv = MNV_NONE;
	k_mutex_unlock(&start_state_mutex);
	run_telem_div = 0;
	run_csv_telem_div = 0;
	run_state = RUN_CLEAR;

	/* 5-second countdown — idle telemetry flows */
	int64_t start_at = k_uptime_get() + 5000;
	while (k_uptime_get() < start_at) {
		if (is_start_cancelled()) {
			return;
		}
		wdt_feed_kick();
		send_idle_telemetry();
		struct car_settings c;
		settings_get_copy(&c);
		k_msleep(c.loop_ms);
	}

	bool entered_run = false;
	k_mutex_lock(&start_state_mutex, K_FOREVER);
	if (start_state == CTRL_START_COUNTDOWN && !start_cancel_requested) {
		start_state = CTRL_START_RUNNING;
		entered_run = true;
	}
	k_mutex_unlock(&start_state_mutex);

	if (!entered_run) {
		return;
	}

	display_notify_run_state(true);
	buzzer_play(BUZZER_RUN_START);
	wifi_cmd_send("$STS:RUN\n");
}

void control_cmd_monitor(void)
{
	imu_reset_heading();
	monitor_mode = true;
	wifi_cmd_send("$ACK\n");
	wifi_cmd_send("$STS:MONITOR\n");
}

bool control_is_monitor(void)
{
	return monitor_mode;
}

void control_cmd_stop(void)
{
	k_mutex_lock(&start_state_mutex, K_FOREVER);
	start_cancel_requested = true;
	start_state = CTRL_START_IDLE;
	mnv = MNV_NONE;
	k_mutex_unlock(&start_state_mutex);

	monitor_mode = false;
	display_notify_run_state(false);
	drv_enabled = false;
	manual_mode = false;
	manual_steer = 0;
	manual_speed = 0.0f;
	car_write_speed(0);
	car_write_steer(0);
	buzzer_play(BUZZER_STOP);
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
