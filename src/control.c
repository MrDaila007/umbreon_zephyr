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
#include "display.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
static volatile bool monitor_mode;
static volatile int manual_steer;
static volatile float manual_speed;
static volatile int64_t last_drv_ms;

/* Stuck detection (persistent across work() calls) */
static int stuck_time;
static float turns;

/* ─── RUN sub-state telemetry ────────────────────────────────────────────── */
enum run_substate {
	RUN_CLEAR      = 0,
	RUN_BLOCKED    = 1,
	RUN_STUCK_WAIT = 2,
	RUN_REVERSE    = 3,
	RUN_WRONG_DIR  = 4,
};
static volatile int run_state;
static int run_telem_div;

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

/* ─── RUN sub-state message ───────────────────────────────────────────────── */

static void send_run_state(int state, int stuck, float trn, int how_clr, int dif)
{
	wifi_cmd_printf("$RUN:%d,%d,%.1f,%d,%d\n",
			state, stuck, (double)trn, how_clr, dif);
}

extern void wdt_feed_kick(void);

/* ─── Recovery constants ──────────────────────────────────────────────────── */
#define REVERSE_MIN_DIST  0.13f  /* m — minimum distance before early exit  */
#define REVERSE_TIMEOUT   1500   /* ms — maximum reverse phase              */
#define REVERSE_MIN_TIME  400    /* ms — minimum time before early exit     */
#define REVERSE_SPEED     (-250) /* raw -1000…+1000 scale                   */
#define STOP_WAIT_TIMEOUT 2000   /* ms — max wait for wheels to stop        */
#define DTAP_BRAKE_MS     120    /* ms — first (brake) pulse of double-tap  */
#define DTAP_NEUTRAL_MS   80     /* ms — neutral gap in double-tap          */
#define WDIR_PRE_STOP_MS  100    /* ms — neutral before wrong-way steer     */
#define WDIR_STEER_MS     20     /* ms — hold full-lock before long reverse */
#define WDIR_REV1_MS      1000   /* ms — first reverse phase (long)         */
#define WDIR_NEUTRAL_MS   80     /* ms — gap between long reverse phases    */
#define WDIR_REV2_MS      1800   /* ms — second reverse phase (long)        */
#define WDIR_FWD_MS       900    /* ms — forward-drive after long reverse   */
#define WDIR_FWD_SPEED    2.0f   /* m/s — forward burst speed               */

/* ─── Non-blocking recovery state machine ─────────────────────────────────── */
/*
 * All recovery maneuvers (stuck → go_back, wrong-way → go_back_long) are
 * driven as a state machine inside the normal 40 ms control loop.  Each tick
 * the main loop still polls sensors, updates IMU, feeds the watchdog, toggles
 * the heartbeat LED, sends telemetry, and checks battery — nothing blocks.
 */

enum recovery_phase {
	REC_NONE = 0,

	/* ── go_back (stuck recovery) ─────────────────────────── */
	REC_STOP_WAIT,       /* decelerate, wait for speed < 0.1  */
	REC_DTAP_BRAKE,      /* double-tap: first reverse pulse    */
	REC_DTAP_NEUTRAL,    /* double-tap: neutral gap            */
	REC_REVERSE,         /* actual reverse with sensor checks  */
	REC_REVERSE_DONE,    /* cleanup, return to normal driving  */

	/* ── go_back_long (wrong-way recovery) ────────────────── */
	REC_WD_PRE_STOP,     /* brief neutral before steer lock    */
	REC_WD_STEER,        /* full steer lock                    */
	REC_WD_STOP_WAIT,    /* decelerate                         */
	REC_WD_REV1,         /* first long reverse (= brake tap)   */
	REC_WD_NEUTRAL,      /* neutral gap (= double-tap)         */
	REC_WD_REV2,         /* second long reverse (real reverse)  */
	REC_WD_FWD,          /* forward burst with PID             */
	REC_WD_DONE,         /* cleanup                            */
};

static struct {
	enum recovery_phase phase;
	int64_t             deadline;  /* phase timeout                      */
	int64_t             start;     /* phase start timestamp              */
	int                 escape_steer;
	uint32_t            enc_start; /* encoder count at reverse start     */
	float               resume_spd;/* speed to restore after go_back     */
	int                 alt;       /* sensor alternation flag            */
} rec;

static void recovery_reset(void)
{
	if (rec.phase != REC_NONE) {
		car_write_speed(0);
	}
	rec.phase = REC_NONE;
}

static int pick_escape_steer(const int *s)
{
	int l = s[IDX_LEFT];
	int r = s[IDX_RIGHT];
	if (l > r + 50)  return -600;
	if (r > l + 50)  return  600;
	return cfg.race_cw ? -600 : 600;
}

static void update_escape_steer(const int *s)
{
	int l = s[IDX_LEFT];
	int r = s[IDX_RIGHT];
	if (l > r + 100)      rec.escape_steer = -600;
	else if (r > l + 100) rec.escape_steer =  600;
}

/* Start stuck recovery (go_back equivalent) */
static void recovery_start_stuck(const int *s, float resume_spd)
{
	rec.phase        = REC_STOP_WAIT;
	rec.escape_steer = pick_escape_steer(s);
	rec.resume_spd   = resume_spd;
	rec.deadline     = k_uptime_get() + STOP_WAIT_TIMEOUT;
	rec.alt          = 0;
	car_write_speed(0);
	send_run_state(RUN_REVERSE, stuck_time, turns, 0, rec.escape_steer);
}

/* Start wrong-way recovery (go_back_long equivalent) */
static void recovery_start_wrong_way(void)
{
	rec.phase    = REC_WD_PRE_STOP;
	rec.deadline = k_uptime_get() + WDIR_PRE_STOP_MS;
	car_write_speed(0);
	send_run_state(RUN_WRONG_DIR, stuck_time, turns, 0, 0);
}

/*
 * recovery_tick() — called once per loop iteration while rec.phase != REC_NONE.
 * Sensors and IMU are already polled by the caller.
 * Returns true while recovery is in progress, false when done.
 */
static bool recovery_tick(int *s)
{
	int64_t now = k_uptime_get();

	switch (rec.phase) {

	/* ════════════════════════════════════════════════════════════════════ */
	/*  STUCK RECOVERY  (go_back equivalent)                              */
	/* ════════════════════════════════════════════════════════════════════ */

	case REC_STOP_WAIT:
		if (taho_get_speed() <= 0.1f || now >= rec.deadline) {
			rec.phase    = REC_DTAP_BRAKE;
			rec.deadline = now + DTAP_BRAKE_MS;
			car_write_steer(rec.escape_steer);
			car_write_speed(REVERSE_SPEED);
		}
		return true;

	case REC_DTAP_BRAKE:
		if (now >= rec.deadline) {
			rec.phase    = REC_DTAP_NEUTRAL;
			rec.deadline = now + DTAP_NEUTRAL_MS;
			car_write_speed(0);
		}
		return true;

	case REC_DTAP_NEUTRAL:
		if (now >= rec.deadline) {
			rec.phase     = REC_REVERSE;
			rec.start     = now;
			rec.deadline  = now + REVERSE_TIMEOUT;
			rec.enc_start = taho_get_count();
			car_write_speed(REVERSE_SPEED);
		}
		return true;

	case REC_REVERSE: {
		s = sensors_poll_mask(rec.alt ? MASK_FRONT_R : MASK_FRONT_L);
		rec.alt = !rec.alt;

		if ((now - rec.start) >= REVERSE_MIN_TIME) {
			uint32_t delta = taho_get_count() - rec.enc_start;
			float dist = ((float)delta * (float)M_PI *
				      cfg.wheel_diam_m) /
				     (float)cfg.encoder_holes;

			bool front_clear =
				s[IDX_FRONT_LEFT]  > cfg.front_obstacle_dist &&
				s[IDX_FRONT_RIGHT] > cfg.front_obstacle_dist;

			if (front_clear && dist >= REVERSE_MIN_DIST) {
				rec.phase = REC_REVERSE_DONE;
				break;
			}
		}

		update_escape_steer(s);
		car_write_steer(rec.escape_steer);
		send_run_state(RUN_REVERSE, stuck_time, turns,
			       0, rec.escape_steer);

		if (now >= rec.deadline) {
			rec.phase = REC_REVERSE_DONE;
		}
		return true;
	}

	case REC_REVERSE_DONE:
		car_write_speed(0);
		car_write_steer(rec.escape_steer);
		send_run_state(RUN_REVERSE, 0, turns, 0, rec.escape_steer);
		car_write_speed_ms(rec.resume_spd);
		stuck_time = 0;
		imu_reset_heading();
		rec.phase = REC_NONE;
		return false;

	/* ════════════════════════════════════════════════════════════════════ */
	/*  WRONG-WAY RECOVERY  (go_back_long equivalent)                     */
	/* ════════════════════════════════════════════════════════════════════ */

	case REC_WD_PRE_STOP:
		if (now >= rec.deadline) {
			rec.phase    = REC_WD_STEER;
			rec.deadline = now + WDIR_STEER_MS;
			car_write_steer(cfg.race_cw ? 1000 : -1000);
		}
		return true;

	case REC_WD_STEER:
		if (now >= rec.deadline) {
			rec.phase    = REC_WD_STOP_WAIT;
			rec.deadline = now + STOP_WAIT_TIMEOUT;
			car_write_speed(0);
		}
		return true;

	case REC_WD_STOP_WAIT:
		if (taho_get_speed() <= 0.1f || now >= rec.deadline) {
			rec.phase    = REC_WD_REV1;
			rec.deadline = now + WDIR_REV1_MS;
			car_write_speed(REVERSE_SPEED);
		}
		return true;

	case REC_WD_REV1:
		if (now >= rec.deadline) {
			rec.phase    = REC_WD_NEUTRAL;
			rec.deadline = now + WDIR_NEUTRAL_MS;
			car_write_speed(0);
		}
		return true;

	case REC_WD_NEUTRAL:
		if (now >= rec.deadline) {
			rec.phase    = REC_WD_REV2;
			rec.deadline = now + WDIR_REV2_MS;
			car_write_speed(REVERSE_SPEED);
		}
		return true;

	case REC_WD_REV2:
		if (now >= rec.deadline) {
			rec.phase    = REC_WD_FWD;
			rec.deadline = now + WDIR_FWD_MS;
			car_write_steer(cfg.race_cw ? -700 : 700);
			car_write_speed_ms(WDIR_FWD_SPEED);
		}
		return true;

	case REC_WD_FWD:
		car_pid_control();
		if (now >= rec.deadline) {
			rec.phase = REC_WD_DONE;
		}
		return true;

	case REC_WD_DONE:
		car_write_speed(0);
		turns = 0.0f;
		imu_reset_heading();
		rec.phase = REC_NONE;
		return false;

	case REC_NONE:
	default:
		return false;
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

/* ─── Idle telemetry (when not driving) ───────────────────────────────────── */

static void send_idle_telemetry(void)
{
	int *s = sensors_poll();
	imu_update();
	send_telemetry(s, 0, 0.0f);
}

/* ─── work() — main autonomous control ────────────────────────────────────── */

static void work(void)
{
	int *s = sensors_poll();
	imu_update();

	/* ── Recovery in progress? Just tick the state machine ─────────── */
	if (rec.phase != REC_NONE) {
		recovery_tick(s);
		send_telemetry(s, rec.escape_steer, 0.0f);
		return;
	}

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
		float rec_spd = track_learn_recommend_speed(0.5f);
		if (rec_spd > 0) {
			spd = rec_spd;
		}
	}
	track_learn_tick((int)(diff * coef), spd);

	/* ── Actuation ─────────────────────────────────────────────────────── */
	car_write_steer((int)(diff * coef));
	car_write_speed_ms(spd);
	car_pid_control();

	/* ── Telemetry ─────────────────────────────────────────────────────── */
	send_telemetry(s, (int)(diff * coef), spd);

	/* ── RUN sub-state telemetry ───────────────────────────────────────── */
	int cur_state;
	if (stuck_time > 0) {
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
			       how_clear, (int)(diff * coef));
	}

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
		recovery_start_stuck(s, spd);
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
		recovery_start_wrong_way();
	}
}

/* ─── work_monitor() — diagnostic mode: sensors + servo, no motor ─────────── */

static void work_monitor(void)
{
	int *s = sensors_poll();
	imu_update();

	/* ── Steering (same wall-follow logic as work()) ──────────────── */
	int diff;

	if (s[IDX_LEFT] > cfg.side_open_dist && s[IDX_RIGHT] > cfg.side_open_dist) {
		diff = WALL_FOLLOW_BIAS;
	} else {
		diff = s[IDX_RIGHT] - s[IDX_LEFT];
	}

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

	/* Use clear coefficients for steering */
	int steer_val = (int)(diff * cfg.coe_clear);
	car_write_steer(steer_val);

	/* No motor — no speed, no PID, no stuck/wrong-dir detection */

	send_telemetry(s, steer_val, 0.0f);
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
		gpio_pin_toggle_dt(&heartbeat_led);

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

		if (monitor_mode && !car_running) {
			/* Diagnostic monitor mode */
			work_monitor();
		} else if (car_running && !drv_active) {
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
	recovery_reset();
	stuck_time = 0;
	turns = 0.0f;
	run_telem_div = 0;
	run_state = RUN_CLEAR;
	wifi_cmd_send("$ACK\n");

	/* 5-second countdown — idle telemetry flows */
	int64_t start_at = k_uptime_get() + 5000;
	while (k_uptime_get() < start_at) {
		wdt_feed_kick();
		send_idle_telemetry();
		k_msleep(cfg.loop_ms);
	}

	car_running = true;
	display_notify_run_state(true);
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
	car_running = false;
	monitor_mode = false;
	display_notify_run_state(false);
	recovery_reset();
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
