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
		filtered = 0.5f * raw + 0.5f * filtered;
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
	else if (strcmp(name, "reactive") == 0) test_reactive();
	else if (strcmp(name, "cal")      == 0) test_cal();
	else {
		wifi_cmd_printf("$NAK:unknown_test:%s\n", name);
	}

	display_notify_test_state(false);
}
