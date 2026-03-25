/*
 * tests.c — 8 diagnostic/calibration tests accessible via WiFi ($TEST:<name>)
 *
 * Ported from Umbreon_roborace.ino: wifi_test_lidar(), wifi_test_servo(), etc.
 *
 * Tests run in the caller's thread context (wifi_cmd thread).
 * k_msleep() is used instead of delay() — yields CPU to other threads.
 */

#include "tests.h"
#include "settings.h"
#include "car.h"
#include "tachometer.h"
#include "sensors.h"
#include "imu.h"
#include "wifi_cmd.h"
#include "control.h"
#include "display.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(tests, LOG_LEVEL_INF);

/* ─── Abort check ─────────────────────────────────────────────────────────── */
extern void wdt_feed_kick(void);

/* Tests cannot easily check UART for $STOP mid-flight in the threaded model.
 * We rely on test timeouts and wdt_feed_kick() to keep the watchdog happy. */

/* ─── Test: lidar (sensors) ───────────────────────────────────────────────── */

static void test_lidar(void)
{
	int64_t start = k_uptime_get();
	while ((k_uptime_get() - start) < 5000) {
		wdt_feed_kick();
		int *s = sensors_poll();
		wifi_cmd_printf("$T:LIDAR"
				",HL=%d,L=%d,FL=%d,FR=%d,R=%d,HR=%d\n",
				s[IDX_HARD_LEFT], s[IDX_LEFT], s[IDX_FRONT_LEFT],
				s[IDX_FRONT_RIGHT], s[IDX_RIGHT], s[IDX_HARD_RIGHT]);
		k_msleep(100);
	}
	wifi_cmd_send("$TDONE:lidar\n");
}

/* ─── Test: servo ─────────────────────────────────────────────────────────── */

static void test_servo(void)
{
	wifi_cmd_send("$T:SERVO,phase=left\n");
	car_write_steer(-1000);
	k_msleep(800);

	wifi_cmd_send("$T:SERVO,phase=right\n");
	car_write_steer(1000);
	k_msleep(800);

	wifi_cmd_send("$T:SERVO,phase=center\n");
	car_write_steer(0);
	k_msleep(400);

	wifi_cmd_send("$T:SERVO,phase=sweep\n");
	for (int i = -100; i <= 100; i++) {
		car_write_steer(i * 10);
		k_msleep(8);
		wdt_feed_kick();
	}
	for (int i = 100; i >= -100; i--) {
		car_write_steer(i * 10);
		k_msleep(8);
		wdt_feed_kick();
	}
	car_write_steer(0);
	wifi_cmd_send("$TDONE:servo\n");
}

/* ─── Test: taho ──────────────────────────────────────────────────────────── */

static void test_taho(void)
{
	taho_reset();
	int64_t start = k_uptime_get();
	while ((k_uptime_get() - start) < 5000) {
		wdt_feed_kick();
		uint32_t cnt = taho_get_count();
		bool stopped = taho_time_since_last_us() > 500000;
		float speed_ms = stopped ? 0.0f : taho_get_speed();

		wifi_cmd_printf("$T:TAHO,pulses=%u,speed=%.2f,state=%s\n",
				cnt, (double)speed_ms,
				stopped ? "stopped" : "spinning");
		k_msleep(150);
	}
	wifi_cmd_send("$TDONE:taho\n");
}

/* ─── Test: esc ───────────────────────────────────────────────────────────── */

static void test_esc(void)
{
	wifi_cmd_send("$T:ESC,phase=arm\n");
	car_write_speed(0);
	k_msleep(2000);

	taho_reset();

	wifi_cmd_send("$T:ESC,phase=run\n");
	car_write_esc_us(cfg.min_speed);

	int64_t esc_start = k_uptime_get();
	while ((k_uptime_get() - esc_start) < 2000) {
		wdt_feed_kick();
		uint32_t cnt = taho_get_count();
		bool stopped = taho_time_since_last_us() > 500000;
		float speed_ms = stopped ? 0.0f : taho_get_speed();
		float revs = (float)cnt / (float)cfg.encoder_holes;

		wifi_cmd_printf("$T:ESC,pulses=%u,revs=%.1f,speed=%.2f\n",
				cnt, (double)revs, (double)speed_ms);
		k_msleep(100);
	}

	car_write_speed(0);
	k_msleep(500);

	uint32_t final_cnt = taho_get_count();
	wifi_cmd_printf("$T:ESC,phase=done,total_pulses=%u,total_revs=%.1f\n",
			final_cnt,
			(double)((float)final_cnt / cfg.encoder_holes));
	wifi_cmd_send("$TDONE:esc\n");
}

/* ─── Test: speed (PID) ───────────────────────────────────────────────────── */

static void test_speed(void)
{
	float target = 1.5f;

	wifi_cmd_send("$T:SPEED,phase=arm\n");
	car_write_speed(0);
	k_msleep(2000);

	car_pid_reset();
	taho_reset();

	wifi_cmd_send("$T:SPEED,phase=run\n");
	car_write_speed_ms(target);
	int64_t prev_ms = k_uptime_get();
	int64_t start = k_uptime_get();

	while ((k_uptime_get() - start) < 10000) {
		wdt_feed_kick();
		int64_t now_ms = k_uptime_get();
		if (now_ms - prev_ms < 80) {
			k_msleep(10);
			continue;
		}
		prev_ms = now_ms;

		car_pid_control();

		wifi_cmd_printf("$T:SPEED,target=%.2f,actual=%.2f\n",
				(double)target, (double)taho_get_speed());
	}

	car_write_speed(0);
	car_write_speed_ms(0);
	wifi_cmd_send("$TDONE:speed\n");
}

/* ─── Test: autotune (relay method) ───────────────────────────────────────── */

static void test_autotune(void)
{
	const float TARGET = 1.5f;
	const int RELAY_D = 20;
	const float HYST = 0.10f;
	const int BASE_ESC = cfg.min_speed + 20;
	const int SKIP_HALF = 4;
	const int NEED_HALF = 12;
	const int64_t TIMEOUT = 40000;

	wifi_cmd_send("$T:TUNE,phase=arm\n");
	car_write_speed(0);
	k_msleep(2000);
	taho_reset();

	wifi_cmd_send("$T:TUNE,phase=relay\n");

	bool relay_high = true;
	float filtered = 0;
	float speed_peak = 0, speed_trough = 999.0f;
	uint32_t prev_cnt = 0;
	int64_t prev_ms = k_uptime_get();
	int64_t start_ms = prev_ms;
	int half_cycle = 0;

	const int MAXM = 16;
	float meas_peaks[MAXM], meas_troughs[MAXM];
	int64_t sw_times[MAXM * 2];
	int np = 0, nt = 0, nsw = 0;

	int esc_val = CLAMP(BASE_ESC + RELAY_D, NEUTRAL_SPEED, cfg.max_speed);
	car_write_esc_us(esc_val);

	while (k_uptime_get() - start_ms < TIMEOUT) {
		wdt_feed_kick();
		int64_t now = k_uptime_get();
		if (now - prev_ms < 80) {
			k_msleep(10);
			continue;
		}
		float dt = (now - prev_ms) / 1000.0f;
		prev_ms = now;

		uint32_t cnt = taho_get_count();
		uint32_t dc = cnt - prev_cnt;
		prev_cnt = cnt;
		float raw = (dc / (float)cfg.encoder_holes) *
			    ((float)M_PI * cfg.wheel_diam_m) / dt;
		filtered = 0.7f * raw + 0.3f * filtered;  /* match PID filter */
		if (taho_time_since_last_us() > 500000) {
			filtered = 0;
		}

		if (filtered > speed_peak)   speed_peak = filtered;
		if (filtered < speed_trough) speed_trough = filtered;

		float spd_err = filtered - TARGET;

		if (relay_high && spd_err > HYST) {
			relay_high = false;
			if (half_cycle >= SKIP_HALF && nt < MAXM)
				meas_troughs[nt++] = speed_trough;
			if (half_cycle >= SKIP_HALF && nsw < MAXM * 2)
				sw_times[nsw++] = now;
			half_cycle++;
			speed_peak = filtered;
			speed_trough = 999.0f;
		} else if (!relay_high && spd_err < -HYST) {
			relay_high = true;
			if (half_cycle >= SKIP_HALF && np < MAXM)
				meas_peaks[np++] = speed_peak;
			if (half_cycle >= SKIP_HALF && nsw < MAXM * 2)
				sw_times[nsw++] = now;
			half_cycle++;
			speed_trough = filtered;
			speed_peak = 0;
		}

		esc_val = relay_high ? (BASE_ESC + RELAY_D) : (BASE_ESC - RELAY_D);
		esc_val = CLAMP(esc_val, NEUTRAL_SPEED, cfg.max_speed);
		car_write_esc_us(esc_val);

		wifi_cmd_printf("$T:TUNE,speed=%.2f,relay=%d,half=%d\n",
				(double)filtered, relay_high ? 1 : 0, half_cycle);

		if (half_cycle >= SKIP_HALF + NEED_HALF) break;
	}

	car_write_speed(0);
	k_msleep(500);

	if (np < 2 || nt < 2 || nsw < 4) {
		wifi_cmd_send("$T:TUNE,phase=error,msg=not_enough_data\n");
		wifi_cmd_send("$TDONE:autotune\n");
		return;
	}

	float avg_peak = 0, avg_trough = 0;
	for (int i = 0; i < np; i++) avg_peak += meas_peaks[i];
	for (int i = 0; i < nt; i++) avg_trough += meas_troughs[i];
	avg_peak /= np;
	avg_trough /= nt;
	float amplitude = (avg_peak - avg_trough) / 2.0f;
	if (amplitude < 0.01f) {
		wifi_cmd_send("$T:TUNE,phase=error,msg=no_oscillation\n");
		wifi_cmd_send("$TDONE:autotune\n");
		return;
	}

	float sum_period = 0;
	int n_periods = 0;
	for (int i = 0; i + 2 < nsw; i++) {
		sum_period += (sw_times[i + 2] - sw_times[i]) / 1000.0f;
		n_periods++;
	}
	float Tu = (n_periods > 0) ? sum_period / n_periods : 1.0f;
	float Ku = 4.0f * RELAY_D / (3.14159f * amplitude);

	wifi_cmd_printf("$TR:TUNE,Ku=%.2f,Tu=%.3f,amp=%.3f\n",
			(double)Ku, (double)Tu, (double)amplitude);

	/* Ziegler-Nichols */
	float zn_kP = 0.6f * Ku;
	float zn_kI = zn_kP / (0.5f * Tu);
	float zn_kD = zn_kP * Tu / 8.0f;
	wifi_cmd_printf("$TR:ZN,KP=%.2f,KI=%.2f,KD=%.3f\n",
			(double)zn_kP, (double)zn_kI, (double)zn_kD);

	/* Tyreus-Luyben */
	float tl_kP = Ku / 2.2f;
	float tl_kI = tl_kP / (2.2f * Tu);
	float tl_kD = tl_kP * Tu / 6.3f;
	wifi_cmd_printf("$TR:TL,KP=%.2f,KI=%.2f,KD=%.3f\n",
			(double)tl_kP, (double)tl_kI, (double)tl_kD);

	/* PI only */
	float pi_kP = 0.45f * Ku;
	float pi_kI = pi_kP / (0.83f * Tu);
	wifi_cmd_printf("$TR:PI,KP=%.2f,KI=%.2f,KD=0.000\n",
			(double)pi_kP, (double)pi_kI);

	wifi_cmd_send("$TDONE:autotune\n");
}

/* ─── Test: pidtune (step-response FOPDT identification) ──────────────────── */
/*
 * Step-response PID tuning for low-speed operation (0.1–0.5 m/s).
 * Applies open-loop ESC steps while steering reactively to avoid walls.
 * Fits FOPDT model at each operating point; Lambda/IMC tuning produces
 * conservative coefficients that won't "kick" at low speed.
 *
 * Protocol:
 *   $T:PTUNE,phase=arm
 *   $T:PTUNE,phase=step,n=1/5,esc=1530
 *   $T:PTUNE,n=1,v=0.12,t=0.55          (real-time speed samples)
 *   $TR:PTUNE,n=1,K=0.00230,L=0.30,tau=0.85,ss=0.45
 *   ...
 *   $TR:PTUNE,best=1,K=...,L=...,tau=...  (best ≈ step nearest 0.4 m/s ss, else lowest ss)
 *   $TR:IMC,KP=...,KI=...,KD=...,lambda=...
 *   $TR:PI,KP=...,KI=...,KD=0.0000
 *   $TR:PTUNE,ff_esc=1520
 *   $TDONE:pidtune
 */

#define PTUNE_MAX_STEPS     8
#define PTUNE_MAX_SAMPLES   60   /* 3 s @ 50 ms */
#define PTUNE_SAMPLE_MS     50
#define PTUNE_STEP_MS       3000 /* drive duration per step */
#define PTUNE_SETTLE_MS     1500 /* wait after stop */
#define PTUNE_NOISE_THRESH  0.03f
#define PTUNE_MAX_SPEED     0.50f /* abort step if exceeded */

/* Reactive steering for pidtune — keeps car away from walls */
static void ptune_steer(void)
{
	int *s = sensors_poll();

	int diff;
	if (s[IDX_LEFT] > cfg.side_open_dist &&
	    s[IDX_RIGHT] > cfg.side_open_dist) {
		diff = 0;
	} else {
		diff = s[IDX_RIGHT] - s[IDX_LEFT];
	}

	diff += (int)((s[IDX_HARD_RIGHT] - s[IDX_HARD_LEFT]) * 0.25f);

	/* Front obstacle — steer hard away */
	if (s[IDX_FRONT_LEFT] < cfg.front_obstacle_dist)
		diff += cfg.front_obstacle_dist - s[IDX_FRONT_LEFT];
	if (s[IDX_FRONT_RIGHT] < cfg.front_obstacle_dist)
		diff -= cfg.front_obstacle_dist - s[IDX_FRONT_RIGHT];

	int steer = (int)(diff * cfg.coe_clear);
	car_write_steer(steer);
}

static void test_pidtune(void)
{
	const int N_STEPS = 5;

	wifi_cmd_send("$T:PTUNE,phase=arm\n");
	car_write_speed(0);
	car_write_steer(0);
	k_msleep(2000);
	taho_reset();

	/* ESC values: N steps within the low-speed region.
	 * We cap the range so steady-state stays ≤ 0.5 m/s.
	 * Use min_speed … min_speed + 60 µs (typically covers 0–0.5 m/s). */
	int esc_vals[PTUNE_MAX_STEPS];
	int esc_lo  = cfg.min_speed;
	int esc_hi  = cfg.min_speed + 60;
	if (esc_hi > cfg.max_speed) esc_hi = cfg.max_speed;
	for (int i = 0; i < N_STEPS; i++) {
		esc_vals[i] = esc_lo + (i * (esc_hi - esc_lo)) / (N_STEPS - 1);
	}

	/* Per-step FOPDT results */
	float step_K[PTUNE_MAX_STEPS];
	float step_L[PTUNE_MAX_STEPS];
	float step_tau[PTUNE_MAX_STEPS];
	float step_ss[PTUNE_MAX_STEPS];
	int   step_esc[PTUNE_MAX_STEPS];
	int   nv = 0; /* valid count */

	float samples[PTUNE_MAX_SAMPLES];

	for (int si = 0; si < N_STEPS; si++) {
		int esc = esc_vals[si];

		wifi_cmd_printf("$T:PTUNE,phase=step,n=%d/%d,esc=%d\n",
				si + 1, N_STEPS, esc);

		/* Full stop between steps */
		car_write_esc_us(NEUTRAL_SPEED);
		car_write_steer(0);
		taho_reset();
		k_msleep(PTUNE_SETTLE_MS);

		/* Apply ESC step */
		uint32_t prev_cnt = taho_get_count();
		float filtered = 0;
		int ns = 0;
		int64_t t0 = k_uptime_get();
		int64_t prev_t = t0;
		bool speed_cut = false;

		car_write_esc_us(esc);

		while ((k_uptime_get() - t0) < PTUNE_STEP_MS &&
		       ns < PTUNE_MAX_SAMPLES) {
			wdt_feed_kick();

			/* Reactive wall-avoidance steering */
			ptune_steer();

			int64_t now = k_uptime_get();
			if (now - prev_t < PTUNE_SAMPLE_MS) {
				k_msleep(5);
				continue;
			}
			float dt = (now - prev_t) / 1000.0f;
			prev_t = now;

			uint32_t cnt = taho_get_count();
			uint32_t dc = cnt - prev_cnt;
			prev_cnt = cnt;
			float raw = (dc / (float)cfg.encoder_holes) *
				    ((float)M_PI * cfg.wheel_diam_m) / dt;
			filtered = 0.7f * raw + 0.3f * filtered;
			if (taho_time_since_last_us() > 500000)
				filtered = 0;

			/* Safety: abort step if speed exceeds limit */
			if (filtered > PTUNE_MAX_SPEED) {
				speed_cut = true;
				break;
			}

			samples[ns++] = filtered;

			wifi_cmd_printf("$T:PTUNE,n=%d,v=%.3f,t=%.2f\n",
					si + 1, (double)filtered,
					(double)((now - t0) / 1000.0f));
		}

		car_write_esc_us(NEUTRAL_SPEED);
		car_write_steer(0);

		if (speed_cut) {
			wifi_cmd_printf("$T:PTUNE,step_cut,n=%d,v=%.2f\n",
					si + 1, (double)filtered);
			/* Skip this and all higher ESC steps */
			break;
		}

		if (ns < 10) {
			wifi_cmd_printf("$T:PTUNE,step_skip,n=%d,samples=%d\n",
					si + 1, ns);
			continue;
		}

		/* --- Steady-state: mean of last 25% of samples --- */
		int i0 = ns * 3 / 4;
		float ss = 0;
		for (int i = i0; i < ns; i++) ss += samples[i];
		ss /= (ns - i0);

		if (ss < PTUNE_NOISE_THRESH) {
			wifi_cmd_printf("$T:PTUNE,step_skip,n=%d,no_motion\n",
					si + 1);
			continue;
		}

		/* --- Dead time L: first sample above threshold --- */
		float L = 0;
		for (int i = 0; i < ns; i++) {
			if (samples[i] > PTUNE_NOISE_THRESH) {
				L = i * (PTUNE_SAMPLE_MS / 1000.0f);
				break;
			}
		}

		/* --- Time constant tau: L → 63.2 % of ss --- */
		float tgt63 = 0.632f * ss;
		float tau = 0.5f; /* fallback */
		for (int i = 0; i < ns; i++) {
			float t = i * (PTUNE_SAMPLE_MS / 1000.0f);
			if (t > L && samples[i] >= tgt63) {
				tau = t - L;
				if (tau < 0.05f) tau = 0.05f;
				break;
			}
		}

		/* --- Process gain K = Δspeed / ΔESC (in (m/s) / µs) --- */
		float d = (float)(esc - NEUTRAL_SPEED);
		float K = ss / d;

		step_K[nv]   = K;
		step_L[nv]   = L;
		step_tau[nv] = tau;
		step_ss[nv]  = ss;
		step_esc[nv] = esc;
		nv++;

		wifi_cmd_printf("$TR:PTUNE,n=%d,K=%.5f,L=%.3f,tau=%.3f,ss=%.3f\n",
				si + 1, (double)K, (double)L, (double)tau,
				(double)ss);
	}

	car_write_speed(0);
	car_write_steer(0);
	k_msleep(500);

	if (nv < 1) {
		wifi_cmd_send("$T:PTUNE,phase=error,msg=no_valid_steps\n");
		wifi_cmd_send("$TDONE:pidtune\n");
		return;
	}

	/* Prefer a valid step whose steady-state is in the usual cruise band
	 * (~0.3–0.5 m/s) so IMC/PI matches real driving.  Fallback: lowest ss. */
#define PTUNE_SS_TARGET   0.40f
#define PTUNE_SS_BAND_LO  0.22f
#define PTUNE_SS_BAND_HI  0.55f

	int best = 0;
	bool in_band = false;
	float best_dist = 1e9f;
	for (int i = 0; i < nv; i++) {
		float ss = step_ss[i];
		if (ss >= PTUNE_SS_BAND_LO && ss <= PTUNE_SS_BAND_HI) {
			float d = fabsf(ss - PTUNE_SS_TARGET);
			if (!in_band || d < best_dist) {
				in_band = true;
				best_dist = d;
				best = i;
			}
		}
	}
	if (!in_band) {
		for (int i = 1; i < nv; i++) {
			if (step_ss[i] < step_ss[best]) {
				best = i;
			}
		}
	}

	float K   = step_K[best];
	float L   = step_L[best];
	float tau = step_tau[best];

	wifi_cmd_printf("$TR:PTUNE,best=%d,K=%.5f,L=%.3f,tau=%.3f\n",
			best + 1, (double)K, (double)L, (double)tau);

	/* Lambda / IMC tuning — conservative, minimal overshoot */
	float lambda = fmaxf(tau, 3.0f * L);
	if (lambda < 0.3f) lambda = 0.3f;

	float kP = tau / (K * (lambda + L));
	float kI = kP / tau;
	float kD = kP * L / 2.0f;

	wifi_cmd_printf("$TR:IMC,KP=%.2f,KI=%.2f,KD=%.4f,lambda=%.3f\n",
			(double)kP, (double)kI, (double)kD, (double)lambda);

	/* PI-only variant — safest for low speed */
	float pi_kP = 0.8f * kP;
	float pi_kI = pi_kP / tau;

	wifi_cmd_printf("$TR:PI,KP=%.2f,KI=%.2f,KD=0.0000\n",
			(double)pi_kP, (double)pi_kI);

	/* Feedforward reference: lowest ESC that produced motion */
	wifi_cmd_printf("$TR:PTUNE,ff_esc=%d\n", step_esc[0]);

	wifi_cmd_send("$TDONE:pidtune\n");
}

/* ─── Test: reactive steering ─────────────────────────────────────────────── */

static void test_reactive(void)
{
	const int CLOSE_DIST = cfg.front_obstacle_dist;
	const int FAR_DIST = 3000;

	int64_t start = k_uptime_get();
	while ((k_uptime_get() - start) < 30000) {
		wdt_feed_kick();
		int *s = sensors_poll();

		int L  = s[IDX_LEFT],  FL = s[IDX_FRONT_LEFT];
		int FR = s[IDX_FRONT_RIGHT], R = s[IDX_RIGHT];
		float diff = (float)(R - L);
		if (FL < CLOSE_DIST) diff += (float)(CLOSE_DIST - FL);
		if (FR < CLOSE_DIST) diff -= (float)(CLOSE_DIST - FR);
		diff += (float)(s[IDX_HARD_RIGHT] - s[IDX_HARD_LEFT]) * 0.3f;

		float steer_f = CLAMP(diff / (float)FAR_DIST, -1.0f, 1.0f);
		int steer_val = (int)(steer_f * 1000.0f);
		car_write_steer(steer_val);

		wifi_cmd_printf("$T:REACT,L=%d,FL=%d,FR=%d,R=%d,steer=%d\n",
				L, FL, FR, R, steer_val);
		k_msleep(50);
	}

	car_write_steer(0);
	wifi_cmd_send("$TDONE:reactive\n");
}

/* ─── Test: calibration ───────────────────────────────────────────────────── */

static void test_cal(void)
{
	cfg.calibrated = false;
	car_run_calibration();
}

/* ─── Dispatcher ──────────────────────────────────────────────────────────── */

void tests_run_by_name(const char *name)
{
	/* Auto-stop car before running any test */
	if (control_is_running()) {
		control_cmd_stop();
	}

	display_notify_test_state(true);

	if      (strcmp(name, "lidar")    == 0) test_lidar();
	else if (strcmp(name, "servo")    == 0) test_servo();
	else if (strcmp(name, "taho")     == 0) test_taho();
	else if (strcmp(name, "esc")      == 0) test_esc();
	else if (strcmp(name, "speed")    == 0) test_speed();
	else if (strcmp(name, "autotune") == 0) test_autotune();
	else if (strcmp(name, "pidtune")  == 0) test_pidtune();
	else if (strcmp(name, "reactive") == 0) test_reactive();
	else if (strcmp(name, "cal")      == 0) test_cal();
	else {
		wifi_cmd_printf("$NAK:unknown_test:%s\n", name);
	}

	display_notify_test_state(false);
}
