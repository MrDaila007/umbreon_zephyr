/*
 * test_maneuver.c — Host tests for the maneuver state machine
 *
 * Validates the non-blocking ESC brake→neutral���reverse protocol,
 * state transitions, timing, exit conditions, and steering logic.
 *
 * Build:  make -C tests test_maneuver
 * Run:    ./tests/test_maneuver
 */

#include "test_runner.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef CLAMP
#define CLAMP(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))
#endif
#ifndef BIT
#define BIT(n) (1U << (n))
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Settings / sensor constants (from settings.h)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SENSOR_COUNT     6
#define IDX_HARD_RIGHT   0
#define IDX_FRONT_RIGHT  1
#define IDX_RIGHT        2
#define IDX_LEFT         3
#define IDX_FRONT_LEFT   4
#define IDX_HARD_LEFT    5
#define MAX_SENSOR_RANGE 2000
#define NEUTRAL_SPEED    1500

#define TRK_MODE_IDLE  0

struct car_settings {
	int front_obstacle_dist;
	int side_open_dist;
	int all_close_dist;
	int close_front_dist;
	float pid_kp, pid_ki, pid_kd;
	int min_speed, max_speed, min_bspeed;
	int min_point, max_point, neutral_point;
	int encoder_holes;
	float wheel_diam_m;
	int loop_ms;
	float spd_clear, spd_blocked, spd_slew;
	float kick_pct;
	int kick_ms;
	float coe_clear, coe_blocked;
	float wrong_dir_deg;
	bool race_cw;
	int stuck_thresh;
	int stall_thresh;
	bool imu_rotate, servo_reverse, calibrated;
	bool bat_enabled;
	float bat_multiplier, bat_low;
};

static struct car_settings cfg;

/* ═══════════════════════════════════════════════════════════════════════════
 * Mock state
 * ═══════════════════════════════════════════════════════════════════════════ */

static int64_t mock_time_ms;
static float   mock_taho_speed;
static uint32_t mock_taho_count;
static int     mock_sensors[SENSOR_COUNT];

/* ─── Call logs ──────────────────────────────────────────────────────────── */

#define LOG_MAX 128

struct speed_entry { int val; int64_t t; };
struct steer_entry { int val; int64_t t; };

static struct speed_entry speed_log[LOG_MAX];
static int speed_log_n;

static struct steer_entry steer_log[LOG_MAX];
static int steer_log_n;

static int pid_control_calls;
static int pid_reset_calls;
static int imu_reset_calls;
static float speed_ms_log[LOG_MAX];
static int speed_ms_log_n;

/* ═══════════════════════════════════════════════════════════════════════════
 * Mock implementations (called by state machine code)
 * ═══════════════════════════════════════════════════════════════════════════ */

static int64_t k_uptime_get(void) { return mock_time_ms; }
static float   taho_get_speed(void) { return mock_taho_speed; }
static uint32_t taho_get_count(void) { return mock_taho_count; }

static void car_write_speed(int s)
{
	if (speed_log_n < LOG_MAX) {
		speed_log[speed_log_n++] =
			(struct speed_entry){ .val = s, .t = mock_time_ms };
	}
}

static void car_write_steer(int s)
{
	if (steer_log_n < LOG_MAX) {
		steer_log[steer_log_n++] =
			(struct steer_entry){ .val = s, .t = mock_time_ms };
	}
}

static void car_write_speed_ms(float target)
{
	if (speed_ms_log_n < LOG_MAX)
		speed_ms_log[speed_ms_log_n++] = target;
}

static void car_pid_control(void) { pid_control_calls++; }
static void car_pid_reset(void)   { pid_reset_calls++; }
static void imu_reset_heading(void) { imu_reset_calls++; }
static float imu_get_yaw_rate(void) { return 0.0f; }
static void wdt_feed_kick(void) { (void)0; }

static int *sensors_poll_mask(uint8_t mask)
{
	(void)mask;
	return mock_sensors;
}

/* wifi stubs */
static void wifi_cmd_printf(const char *fmt, ...) { (void)fmt; }

/* ═══════════════════════════════════════════════════════════════════════════
 * State machine code — copied from control.c (maneuver section)
 * Keep in sync with src/control.c
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Sensor masks */
#define MASK_SIDES   (BIT(IDX_LEFT) | BIT(IDX_RIGHT))
#define MASK_FRONT_L (MASK_SIDES | BIT(IDX_FRONT_LEFT))
#define MASK_FRONT_R (MASK_SIDES | BIT(IDX_FRONT_RIGHT))

#define REVERSE_MIN_DIST  0.13f

/* Persistent control state (normally in control.c) */
static int   stuck_time;
static float turns;

/* RUN substates (for send_run_state) */
enum run_substate {
	RUN_CLEAR      = 0,
	RUN_BLOCKED    = 1,
	RUN_STUCK_WAIT = 2,
	RUN_REVERSE    = 3,
	RUN_WRONG_DIR  = 4,
};

static void send_run_state(int state, int stuck, float trn,
			   int how_clr, int dif)
{
	(void)state; (void)stuck; (void)trn; (void)how_clr; (void)dif;
}

/* ─── Begin copied state machine ─────────────────────────────────────────── */

enum mnv_phase {
	MNV_NONE = 0,
	MNV_BACK_WAIT_STOP,
	MNV_BACK_BRAKE,
	MNV_BACK_NEUTRAL,
	MNV_BACK_REVERSE,
	MNV_LONG_WAIT_STOP,
	MNV_LONG_BRAKE,
	MNV_LONG_NEUTRAL,
	MNV_LONG_REVERSE,
	MNV_LONG_FORWARD,
};

static enum mnv_phase mnv;
static int64_t  mnv_deadline;
static int      mnv_steer;
static uint32_t mnv_start_count;
static int      mnv_alt;

static void maneuver_start_back(void)
{
	int *s  = sensors_poll_mask(MASK_SIDES);
	int left  = s[IDX_LEFT];
	int right = s[IDX_RIGHT];

	if (left > right + 50) {
		mnv_steer = -600;
	} else if (right > left + 50) {
		mnv_steer = 600;
	} else {
		mnv_steer = cfg.race_cw ? -600 : 600;
	}

	send_run_state(RUN_REVERSE, stuck_time, turns, 0, mnv_steer);
	car_write_speed(0);
	mnv_deadline = k_uptime_get() + 2000;
	mnv = MNV_BACK_WAIT_STOP;
	stuck_time = 0;
}

static void maneuver_start_long(void)
{
	send_run_state(RUN_WRONG_DIR, stuck_time, turns, 0, 0);
	car_write_speed(0);
	car_write_steer(cfg.race_cw ? 1000 : -1000);
	mnv_deadline = k_uptime_get() + 2000;
	mnv = MNV_LONG_WAIT_STOP;
}

static void maneuver_tick(void)
{
	int64_t now = k_uptime_get();

	switch (mnv) {

	case MNV_BACK_WAIT_STOP:
		if (taho_get_speed() < 0.1f || now >= mnv_deadline) {
			car_write_steer(mnv_steer);
			car_write_speed(-150);
			mnv_deadline = now + 500;
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
			car_write_speed(-150);
			mnv_start_count = taho_get_count();
			mnv_alt = 0;
			mnv_deadline = now + 2000;
			mnv = MNV_BACK_REVERSE;
		}
		break;

	case MNV_BACK_REVERSE: {
		int *s = sensors_poll_mask(mnv_alt ? MASK_FRONT_R
						   : MASK_FRONT_L);
		mnv_alt = !mnv_alt;

		uint32_t delta = taho_get_count() - mnv_start_count;
		float dist = ((float)delta * (float)M_PI * cfg.wheel_diam_m)
			     / (float)cfg.encoder_holes;

		bool front_clear =
			s[IDX_FRONT_LEFT]  > cfg.front_obstacle_dist &&
			s[IDX_FRONT_RIGHT] > cfg.front_obstacle_dist;

		if ((front_clear && dist >= REVERSE_MIN_DIST) ||
		    now >= mnv_deadline) {
			car_write_speed(0);
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

	case MNV_LONG_WAIT_STOP:
		if (taho_get_speed() < 0.1f || now >= mnv_deadline) {
			car_write_speed(-150);
			mnv_deadline = now + 1000;
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
			car_write_speed(-150);
			mnv_deadline = now + 1800;
			mnv = MNV_LONG_REVERSE;
		}
		break;

	case MNV_LONG_REVERSE:
		if (now >= mnv_deadline) {
			car_write_speed(0);
			car_write_steer(cfg.race_cw ? -700 : 700);
			car_write_speed_ms(2.0f);
			mnv_deadline = now + 900;
			mnv = MNV_LONG_FORWARD;
		}
		break;

	case MNV_LONG_FORWARD:
		car_pid_control();
		if (now >= mnv_deadline) {
			turns = 0.0f;
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

/* ─── End copied state machine ───────────────────────────────────────────── */

/* ═══════════════════════════════════════════════════════════════════════════
 * Test helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

static void reset_all(void)
{
	mock_time_ms    = 1000;  /* arbitrary start so deadline math works */
	mock_taho_speed = 0.0f;
	mock_taho_count = 0;
	memset(mock_sensors, 0, sizeof(mock_sensors));

	speed_log_n = 0;
	steer_log_n = 0;
	speed_ms_log_n = 0;
	pid_control_calls = 0;
	pid_reset_calls   = 0;
	imu_reset_calls   = 0;

	mnv = MNV_NONE;
	mnv_deadline = 0;
	mnv_steer = 0;
	mnv_start_count = 0;
	mnv_alt = 0;
	stuck_time = 0;
	turns = 0.0f;

	/* Sensible cfg defaults */
	memset(&cfg, 0, sizeof(cfg));
	cfg.front_obstacle_dist = 800;
	cfg.close_front_dist    = 150;
	cfg.encoder_holes       = 68;
	cfg.wheel_diam_m        = 0.060f;
	cfg.loop_ms             = 40;
	cfg.race_cw             = true;
	cfg.stuck_thresh        = 25;
	cfg.stall_thresh        = 50;
	cfg.wrong_dir_deg       = 120.0f;
}

/* Advance mock time and call maneuver_tick */
static void tick_at(int64_t time_ms)
{
	mock_time_ms = time_ms;
	maneuver_tick();
}

/* Run ticks from `from` to `to` in `step` intervals */
static void run_ticks(int64_t from, int64_t to, int64_t step)
{
	for (int64_t t = from; t <= to; t += step) {
		tick_at(t);
	}
}

/* Find last speed_log entry with given value, return its time or -1 */
static int64_t last_speed_time(int val)
{
	for (int i = speed_log_n - 1; i >= 0; i--) {
		if (speed_log[i].val == val)
			return speed_log[i].t;
	}
	return -1;
}

/* Count speed_log entries with given value */
static int count_speed(int val)
{
	int n = 0;
	for (int i = 0; i < speed_log_n; i++) {
		if (speed_log[i].val == val)
			n++;
	}
	return n;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: ESC brake-to-reverse protocol (go_back)
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_back_esc_full_sequence)
{
	/* The core fix: verify brake → neutral → reverse ESC signals */
	reset_all();
	mock_taho_speed = 0.0f;  /* already stopped */

	maneuver_start_back();
	ASSERT_EQ(mnv, MNV_BACK_WAIT_STOP);

	/* Tick: speed already < 0.1 → transitions to BRAKE, sends -150 */
	tick_at(1000);
	ASSERT_EQ(mnv, MNV_BACK_BRAKE);

	int brake_idx = speed_log_n - 1;
	ASSERT_EQ(speed_log[brake_idx].val, -150);  /* 1st reverse = BRAKE */

	/* Advance 500ms → transitions to NEUTRAL, sends 0 */
	tick_at(1500);
	ASSERT_EQ(mnv, MNV_BACK_NEUTRAL);
	ASSERT_EQ(speed_log[speed_log_n - 1].val, 0);  /* neutral */

	/* Advance 80ms → transitions to REVERSE, sends -150 */
	tick_at(1580);
	ASSERT_EQ(mnv, MNV_BACK_REVERSE);
	ASSERT_EQ(speed_log[speed_log_n - 1].val, -150);  /* 2nd = real reverse */

	/* Verify the sequence: ... → -150 → 0 → -150 */
	ASSERT_TRUE(count_speed(-150) >= 2);
	ASSERT_TRUE(count_speed(0) >= 2);  /* initial stop + neutral */
}

TEST(test_back_esc_signals_distinct_times)
{
	/* Brake and reverse -150 signals must be separated by neutral */
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	tick_at(1000);   /* → BRAKE, speed=-150 */
	tick_at(1500);   /* → NEUTRAL, speed=0 */
	tick_at(1580);   /* → REVERSE, speed=-150 */

	/* Find the two -150 entries */
	int first_rev = -1, second_rev = -1;
	for (int i = 0; i < speed_log_n; i++) {
		if (speed_log[i].val == -150) {
			if (first_rev < 0)
				first_rev = i;
			else if (second_rev < 0)
				second_rev = i;
		}
	}
	ASSERT_TRUE(first_rev >= 0);
	ASSERT_TRUE(second_rev >= 0);
	ASSERT_TRUE(second_rev > first_rev);

	/* Between first and second -150, there must be a 0 */
	bool found_neutral = false;
	for (int i = first_rev + 1; i < second_rev; i++) {
		if (speed_log[i].val == 0)
			found_neutral = true;
	}
	ASSERT_TRUE(found_neutral);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Timing of each phase (go_back)
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_back_wait_stop_by_speed)
{
	/* Should transition as soon as speed drops below 0.1 */
	reset_all();
	mock_taho_speed = 0.5f;  /* still moving */

	maneuver_start_back();
	tick_at(1040);
	ASSERT_EQ(mnv, MNV_BACK_WAIT_STOP);  /* still waiting */

	mock_taho_speed = 0.05f;  /* slowed down */
	tick_at(1080);
	ASSERT_EQ(mnv, MNV_BACK_BRAKE);  /* transitioned */
}

TEST(test_back_wait_stop_timeout)
{
	/* Should transition after 2s even if speed never drops */
	reset_all();
	mock_taho_speed = 1.0f;  /* stays fast */

	maneuver_start_back();  /* time=1000, deadline=3000 */

	tick_at(2900);
	ASSERT_EQ(mnv, MNV_BACK_WAIT_STOP);  /* not yet */

	tick_at(3000);
	ASSERT_EQ(mnv, MNV_BACK_BRAKE);  /* forced by timeout */
}

TEST(test_back_brake_duration_500ms)
{
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	tick_at(1000);  /* → BRAKE */
	ASSERT_EQ(mnv, MNV_BACK_BRAKE);

	tick_at(1400);  /* 400ms — too early */
	ASSERT_EQ(mnv, MNV_BACK_BRAKE);

	tick_at(1500);  /* 500ms — transition */
	ASSERT_EQ(mnv, MNV_BACK_NEUTRAL);
}

TEST(test_back_neutral_duration_80ms)
{
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	tick_at(1000);  /* → BRAKE */
	tick_at(1500);  /* → NEUTRAL */
	ASSERT_EQ(mnv, MNV_BACK_NEUTRAL);

	tick_at(1540);  /* 40ms — too early */
	ASSERT_EQ(mnv, MNV_BACK_NEUTRAL);

	tick_at(1580);  /* 80ms — transition */
	ASSERT_EQ(mnv, MNV_BACK_REVERSE);
}

TEST(test_back_reverse_timeout_2s)
{
	/* REVERSE phase should timeout after 2s if front never clears */
	reset_all();
	mock_taho_speed = 0.0f;
	/* Front sensors blocked */
	mock_sensors[IDX_FRONT_LEFT]  = 100;
	mock_sensors[IDX_FRONT_RIGHT] = 100;

	maneuver_start_back();
	tick_at(1000);   /* → BRAKE */
	tick_at(1500);   /* → NEUTRAL */
	tick_at(1580);   /* → REVERSE, deadline = 1580 + 2000 = 3580 */

	tick_at(3500);
	ASSERT_EQ(mnv, MNV_BACK_REVERSE);  /* still going */

	tick_at(3580);
	ASSERT_EQ(mnv, MNV_NONE);  /* timed out → done */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: REVERSE exit conditions
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_back_reverse_early_exit_front_clear)
{
	/* Front clear + sufficient distance → early exit */
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	tick_at(1000);   /* → BRAKE */
	tick_at(1500);   /* → NEUTRAL */
	tick_at(1580);   /* → REVERSE, start_count = 0 */

	/* Simulate: 50 encoder ticks (more than 0.13m) */
	/* dist = 50 × π × 0.060 / 68 = 0.1385m > 0.13m */
	mock_taho_count = 50;
	mock_sensors[IDX_FRONT_LEFT]  = 900;  /* > 800 = clear */
	mock_sensors[IDX_FRONT_RIGHT] = 900;

	tick_at(1620);
	ASSERT_EQ(mnv, MNV_NONE);  /* exited early */
	ASSERT_EQ(imu_reset_calls, 1);
}

TEST(test_back_reverse_no_exit_dist_short)
{
	/* Front clear but distance too short → stays in REVERSE */
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	tick_at(1000);
	tick_at(1500);
	tick_at(1580);  /* → REVERSE */

	/* Only 10 ticks: dist = 10 × π × 0.060 / 68 = 0.0277m < 0.13m */
	mock_taho_count = 10;
	mock_sensors[IDX_FRONT_LEFT]  = 900;
	mock_sensors[IDX_FRONT_RIGHT] = 900;

	tick_at(1620);
	ASSERT_EQ(mnv, MNV_BACK_REVERSE);  /* still reversing */
}

TEST(test_back_reverse_no_exit_front_blocked)
{
	/* Enough distance but front still blocked → stays in REVERSE */
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	tick_at(1000);
	tick_at(1500);
	tick_at(1580);  /* → REVERSE */

	mock_taho_count = 50;
	mock_sensors[IDX_FRONT_LEFT]  = 100;  /* < 800 = blocked */
	mock_sensors[IDX_FRONT_RIGHT] = 900;

	tick_at(1620);
	ASSERT_EQ(mnv, MNV_BACK_REVERSE);  /* still reversing */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Escape steering direction
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_back_steer_left_when_left_open)
{
	reset_all();
	mock_sensors[IDX_LEFT]  = 500;   /* more space on left */
	mock_sensors[IDX_RIGHT] = 200;

	maneuver_start_back();
	ASSERT_EQ(mnv_steer, -600);  /* steer left */
}

TEST(test_back_steer_right_when_right_open)
{
	reset_all();
	mock_sensors[IDX_LEFT]  = 200;
	mock_sensors[IDX_RIGHT] = 500;  /* more space on right */

	maneuver_start_back();
	ASSERT_EQ(mnv_steer, 600);  /* steer right */
}

TEST(test_back_steer_default_cw)
{
	reset_all();
	cfg.race_cw = true;
	mock_sensors[IDX_LEFT]  = 300;
	mock_sensors[IDX_RIGHT] = 310;  /* nearly equal (diff < 50) */

	maneuver_start_back();
	ASSERT_EQ(mnv_steer, -600);  /* CW default → left */
}

TEST(test_back_steer_default_ccw)
{
	reset_all();
	cfg.race_cw = false;
	mock_sensors[IDX_LEFT]  = 300;
	mock_sensors[IDX_RIGHT] = 310;

	maneuver_start_back();
	ASSERT_EQ(mnv_steer, 600);  /* CCW default → right */
}

TEST(test_back_steer_updates_during_reverse)
{
	/* Steering should update based on side sensors during REVERSE */
	reset_all();
	mock_taho_speed = 0.0f;
	mock_sensors[IDX_LEFT]  = 200;
	mock_sensors[IDX_RIGHT] = 500;  /* right open → steer right (600) */

	maneuver_start_back();
	ASSERT_EQ(mnv_steer, 600);

	tick_at(1000);  /* → BRAKE */
	tick_at(1500);  /* → NEUTRAL */
	tick_at(1580);  /* → REVERSE */

	/* Swap sides: now left is more open */
	mock_sensors[IDX_LEFT]  = 600;
	mock_sensors[IDX_RIGHT] = 200;  /* diff > 100 */
	mock_sensors[IDX_FRONT_LEFT]  = 100;  /* keep front blocked */
	mock_sensors[IDX_FRONT_RIGHT] = 100;

	tick_at(1620);
	ASSERT_EQ(mnv_steer, -600);  /* updated to left */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: go_back_long full sequence
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_long_full_sequence_phases)
{
	reset_all();
	mock_taho_speed = 0.0f;
	cfg.race_cw = true;

	maneuver_start_long();
	ASSERT_EQ(mnv, MNV_LONG_WAIT_STOP);

	/* Steer set to max for CW */
	ASSERT_TRUE(steer_log_n > 0);
	ASSERT_EQ(steer_log[steer_log_n - 1].val, 1000);

	/* WAIT_STOP → BRAKE (speed already 0) */
	tick_at(1000);
	ASSERT_EQ(mnv, MNV_LONG_BRAKE);

	/* BRAKE 1000ms */
	tick_at(1500);
	ASSERT_EQ(mnv, MNV_LONG_BRAKE);  /* not yet */
	tick_at(2000);
	ASSERT_EQ(mnv, MNV_LONG_NEUTRAL);

	/* NEUTRAL 80ms */
	tick_at(2040);
	ASSERT_EQ(mnv, MNV_LONG_NEUTRAL);
	tick_at(2080);
	ASSERT_EQ(mnv, MNV_LONG_REVERSE);

	/* REVERSE 1800ms */
	tick_at(3000);
	ASSERT_EQ(mnv, MNV_LONG_REVERSE);
	tick_at(3880);
	ASSERT_EQ(mnv, MNV_LONG_FORWARD);

	/* FORWARD 900ms — should call PID */
	pid_control_calls = 0;
	tick_at(4000);
	ASSERT_TRUE(pid_control_calls > 0);
	ASSERT_EQ(mnv, MNV_LONG_FORWARD);

	tick_at(4780);
	ASSERT_EQ(mnv, MNV_NONE);  /* done */
}

TEST(test_long_esc_brake_neutral_reverse)
{
	/* Verify same ESC protocol: brake → neutral → reverse */
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_long();
	tick_at(1000);   /* ��� BRAKE, speed=-150 */
	tick_at(2000);   /* → NEUTRAL, speed=0 */
	tick_at(2080);   /* → REVERSE, speed=-150 */

	/* Should have: 0 (start) → -150 (brake) → 0 (neutral) → -150 (reverse) */
	ASSERT_TRUE(count_speed(-150) >= 2);

	/* Verify order */
	int found_seq = 0;  /* 0=looking for -150, 1=got brake looking for 0, 2=got 0 looking for -150, 3=done */
	for (int i = 0; i < speed_log_n && found_seq < 3; i++) {
		if (found_seq == 0 && speed_log[i].val == -150) found_seq = 1;
		else if (found_seq == 1 && speed_log[i].val == 0) found_seq = 2;
		else if (found_seq == 2 && speed_log[i].val == -150) found_seq = 3;
	}
	ASSERT_EQ(found_seq, 3);  /* full brake→neutral→reverse sequence found */
}

TEST(test_long_forward_steer_cw)
{
	reset_all();
	mock_taho_speed = 0.0f;
	cfg.race_cw = true;

	maneuver_start_long();
	tick_at(1000);   /* → BRAKE */
	tick_at(2000);   /* → NEUTRAL */
	tick_at(2080);   /* → REVERSE */
	tick_at(3880);   /* → FORWARD */

	/* CW race → steer -700 (turn left) during forward phase */
	ASSERT_EQ(steer_log[steer_log_n - 1].val, -700);
}

TEST(test_long_forward_steer_ccw)
{
	reset_all();
	mock_taho_speed = 0.0f;
	cfg.race_cw = false;

	maneuver_start_long();

	/* CCW → initial steer = -1000 */
	ASSERT_EQ(steer_log[steer_log_n - 1].val, -1000);

	tick_at(1000);
	tick_at(2000);
	tick_at(2080);
	tick_at(3880);   /* → FORWARD */

	/* CCW race → steer 700 (turn right) during forward */
	ASSERT_EQ(steer_log[steer_log_n - 1].val, 700);
}

TEST(test_long_forward_speed_2ms)
{
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_long();
	tick_at(1000);
	tick_at(2000);
	tick_at(2080);
	tick_at(3880);   /* → FORWARD */

	ASSERT_TRUE(speed_ms_log_n > 0);
	ASSERT_FLOAT_EQ(speed_ms_log[speed_ms_log_n - 1], 2.0f, 0.01f);
}

TEST(test_long_finish_resets_turns)
{
	reset_all();
	mock_taho_speed = 0.0f;
	turns = 150.0f;  /* was wrong-way */

	maneuver_start_long();
	tick_at(1000);
	tick_at(2000);
	tick_at(2080);
	tick_at(3880);
	tick_at(4780);  /* → NONE */

	ASSERT_FLOAT_EQ(turns, 0.0f, 0.001f);
	ASSERT_TRUE(imu_reset_calls > 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Edge cases
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(test_tick_when_none_is_noop)
{
	reset_all();
	maneuver_tick();
	/* Nothing should change */
	ASSERT_EQ(mnv, MNV_NONE);
	ASSERT_EQ(speed_log_n, 0);
	ASSERT_EQ(steer_log_n, 0);
}

TEST(test_back_stuck_time_reset)
{
	reset_all();
	stuck_time = 30;

	maneuver_start_back();
	ASSERT_EQ(stuck_time, 0);
}

TEST(test_back_completes_to_none)
{
	/* Full go_back from start to finish */
	reset_all();
	mock_taho_speed = 0.0f;
	mock_sensors[IDX_FRONT_LEFT]  = 100;
	mock_sensors[IDX_FRONT_RIGHT] = 100;

	maneuver_start_back();

	/* Run through all phases at 40ms intervals */
	/* WAIT_STOP → BRAKE (immediately) */
	tick_at(1000);
	ASSERT_EQ(mnv, MNV_BACK_BRAKE);

	/* BRAKE → NEUTRAL (500ms): 1040..1480 still BRAKE, 1500 triggers */
	run_ticks(1040, 1480, 40);
	ASSERT_EQ(mnv, MNV_BACK_BRAKE);
	tick_at(1500);
	ASSERT_EQ(mnv, MNV_BACK_NEUTRAL);

	/* NEUTRAL → REVERSE (80ms) */
	run_ticks(1540, 1580, 40);
	ASSERT_EQ(mnv, MNV_BACK_REVERSE);

	/* REVERSE → NONE (2000ms timeout since front blocked) */
	run_ticks(1620, 3580, 40);
	ASSERT_EQ(mnv, MNV_NONE);
	ASSERT_TRUE(imu_reset_calls > 0);
}

TEST(test_long_wait_stop_timeout)
{
	/* Speed stays high → timeout after 2s */
	reset_all();
	mock_taho_speed = 2.0f;

	maneuver_start_long();  /* deadline = 1000 + 2000 = 3000 */

	tick_at(2000);
	ASSERT_EQ(mnv, MNV_LONG_WAIT_STOP);

	tick_at(3000);
	ASSERT_EQ(mnv, MNV_LONG_BRAKE);
}

TEST(test_back_total_time_bounded)
{
	/* Worst case: wait_stop(2s) + brake(500ms) + neutral(80ms) + reverse(2s)
	 * = 4580ms. Verify it finishes within that. */
	reset_all();
	mock_taho_speed = 1.0f;  /* won't stop naturally */
	mock_sensors[IDX_FRONT_LEFT]  = 100;
	mock_sensors[IDX_FRONT_RIGHT] = 100;

	maneuver_start_back();  /* time=1000 */

	/* Run for 5 seconds at 40ms steps */
	for (int64_t t = 1040; t <= 6000; t += 40)
		tick_at(t);

	ASSERT_EQ(mnv, MNV_NONE);
}

TEST(test_long_total_time_bounded)
{
	/* Worst case: wait_stop(2s) + brake(1s) + neutral(80ms) + reverse(1.8s) + forward(900ms)
	 * = 5780ms. */
	reset_all();
	mock_taho_speed = 1.0f;

	maneuver_start_long();  /* time=1000 */

	for (int64_t t = 1040; t <= 7000; t += 40)
		tick_at(t);

	ASSERT_EQ(mnv, MNV_NONE);
}

TEST(test_no_blocking_sleep)
{
	/* State machine must never call k_msleep — we don't even define it.
	 * If it compiled and runs, there is no blocking sleep. This test
	 * exists as documentation of the non-blocking guarantee. */
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	for (int64_t t = 1000; t <= 5000; t += 40)
		tick_at(t);

	ASSERT_EQ(mnv, MNV_NONE);  /* finished without blocking */
}

TEST(test_long_pid_called_every_forward_tick)
{
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_long();
	tick_at(1000);   /* → BRAKE */
	tick_at(2000);   /* → NEUTRAL */
	tick_at(2080);   /* → REVERSE */
	tick_at(3880);   /* → FORWARD */

	pid_control_calls = 0;

	/* 900ms of forward at 40ms steps = ~22 ticks */
	int ticks = 0;
	for (int64_t t = 3920; t < 4780; t += 40) {
		tick_at(t);
		ticks++;
	}

	/* PID should be called every tick in FORWARD phase */
	ASSERT_EQ(pid_control_calls, ticks);
}

TEST(test_back_encoder_distance_calculation)
{
	/* Verify distance formula: dist = count × π × diam / holes */
	reset_all();
	mock_taho_speed = 0.0f;

	maneuver_start_back();
	tick_at(1000);
	tick_at(1500);
	tick_at(1580);  /* → REVERSE, start_count = 0 */

	/* Exact threshold: dist = 47 × π × 0.060 / 68 = 0.1301m ≈ 0.13m
	 * Need >= 0.13m, so 47 ticks should be right at boundary */
	mock_taho_count = 47;
	mock_sensors[IDX_FRONT_LEFT]  = 900;
	mock_sensors[IDX_FRONT_RIGHT] = 900;

	float dist = (47.0f * (float)M_PI * 0.060f) / 68.0f;

	tick_at(1620);
	if (dist >= REVERSE_MIN_DIST) {
		ASSERT_EQ(mnv, MNV_NONE);  /* should exit */
	} else {
		ASSERT_EQ(mnv, MNV_BACK_REVERSE);  /* still going */
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Main
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
	printf("=== maneuver state machine tests ===\n\n");

	printf("[ESC brake-to-reverse protocol]\n");
	RUN_TEST(test_back_esc_full_sequence);
	RUN_TEST(test_back_esc_signals_distinct_times);
	RUN_TEST(test_long_esc_brake_neutral_reverse);

	printf("\n[go_back timing]\n");
	RUN_TEST(test_back_wait_stop_by_speed);
	RUN_TEST(test_back_wait_stop_timeout);
	RUN_TEST(test_back_brake_duration_500ms);
	RUN_TEST(test_back_neutral_duration_80ms);
	RUN_TEST(test_back_reverse_timeout_2s);

	printf("\n[go_back exit conditions]\n");
	RUN_TEST(test_back_reverse_early_exit_front_clear);
	RUN_TEST(test_back_reverse_no_exit_dist_short);
	RUN_TEST(test_back_reverse_no_exit_front_blocked);

	printf("\n[escape steering]\n");
	RUN_TEST(test_back_steer_left_when_left_open);
	RUN_TEST(test_back_steer_right_when_right_open);
	RUN_TEST(test_back_steer_default_cw);
	RUN_TEST(test_back_steer_default_ccw);
	RUN_TEST(test_back_steer_updates_during_reverse);

	printf("\n[go_back_long sequence]\n");
	RUN_TEST(test_long_full_sequence_phases);
	RUN_TEST(test_long_forward_steer_cw);
	RUN_TEST(test_long_forward_steer_ccw);
	RUN_TEST(test_long_forward_speed_2ms);
	RUN_TEST(test_long_finish_resets_turns);

	printf("\n[edge cases]\n");
	RUN_TEST(test_tick_when_none_is_noop);
	RUN_TEST(test_back_stuck_time_reset);
	RUN_TEST(test_back_completes_to_none);
	RUN_TEST(test_long_wait_stop_timeout);
	RUN_TEST(test_back_total_time_bounded);
	RUN_TEST(test_long_total_time_bounded);
	RUN_TEST(test_no_blocking_sleep);
	RUN_TEST(test_long_pid_called_every_forward_tick);
	RUN_TEST(test_back_encoder_distance_calculation);

	TEST_SUMMARY();
}
