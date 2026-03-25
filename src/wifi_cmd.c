/*
 * wifi_cmd.c — UART1 WiFi command protocol + debug console
 *
 * Ported from Umbreon_roborace.ino: process_commands(), dispatch_command()
 *
 * Architecture:
 *   - UART1 (GP4/GP5) IRQ callback fills a ring buffer
 *   - Dedicated thread wakes on '\n', parses command, dispatches
 *   - Commands that affect control loop are sent via k_msgq
 *   - UART0 (GP16/GP17) is free for debug console / LOG output
 *
 * Debug console commands ($LOG, $SNS, $IMU, $PID, $SYS, $DIAG, $HELP)
 */

#include "wifi_cmd.h"
#include "settings.h"
#include "car.h"
#include "tachometer.h"
#include "sensors.h"
#include "imu.h"
#include "battery.h"
#include "control.h"
#include "tests.h"
#include "track_learn.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

LOG_MODULE_REGISTER(wifi_cmd, LOG_LEVEL_INF);

/* ─── UART device ─────────────────────────────────────────────────────────── */
static const struct device *uart_dev;

/* ─── Ring buffer for UART RX ─────────────────────────────────────────────── */
#define RX_BUF_SIZE 512
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

/* Semaphore: ISR posts when '\n' received */
static K_SEM_DEFINE(rx_line_sem, 0, 1);

/* ─── Command line buffer ─────────────────────────────────────────────────── */
#define CMD_BUF_SIZE 512
static char cmd_buf[CMD_BUF_SIZE];
static int cmd_len;

/* ─── TX output buffer ────────────────────────────────────────────────────── */
static K_MUTEX_DEFINE(tx_mutex);

/* ─── Debug log flag ──────────────────────────────────────────────────────── */
static volatile bool log_on;

/* ─── Thread ──────────────────────────────────────────────────────────────── */
#define WIFI_STACK_SIZE 2048
#define WIFI_PRIORITY   5
static K_THREAD_STACK_DEFINE(wifi_stack, WIFI_STACK_SIZE);
static struct k_thread wifi_thread_data;

/* ─── UART TX helpers ─────────────────────────────────────────────────────── */

void wifi_cmd_send(const char *str)
{
	if (!uart_dev) {
		return;
	}
	k_mutex_lock(&tx_mutex, K_FOREVER);
	for (const char *p = str; *p; p++) {
		uart_poll_out(uart_dev, *p);
	}
	k_mutex_unlock(&tx_mutex);
}

void wifi_cmd_printf(const char *fmt, ...)
{
	char buf[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	wifi_cmd_send(buf);
}

void wifi_log(const char *fmt, ...)
{
	if (!log_on || !uart_dev) {
		return;
	}
	char buf[256];
	int n = snprintf(buf, sizeof(buf), "$L:");
	va_list args;
	va_start(args, fmt);
	n += vsnprintf(buf + n, sizeof(buf) - n, fmt, args);
	va_end(args);
	/* Ensure newline */
	if (n > 0 && n < (int)sizeof(buf) - 1 && buf[n - 1] != '\n') {
		buf[n++] = '\n';
		buf[n] = '\0';
	}
	wifi_cmd_send(buf);
}

bool wifi_log_enabled(void)
{
	return log_on;
}

/* ─── Ring buffer helpers ─────────────────────────────────────────────────── */

static inline bool rb_empty(void)
{
	return rx_head == rx_tail;
}

static inline bool rb_get(uint8_t *c)
{
	if (rb_empty()) {
		return false;
	}
	*c = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
	return true;
}

/* ─── UART ISR ────────────────────────────────────────────────────────────── */

static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		uint8_t c;
		int len = uart_fifo_read(dev, &c, 1);
		if (len != 1) {
			continue;
		}

		uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
		if (next != rx_tail) {
			rx_buf[rx_head] = c;
			rx_head = next;
		}

		if (c == '\n') {
			k_sem_give(&rx_line_sem);
		}
	}
}

/* ─── SET command parser ──────────────────────────────────────────────────── */

static bool parse_set_pair(const char *pair)
{
	const char *eq = strchr(pair, '=');
	if (!eq) {
		return false;
	}

	char key[8];
	int klen = eq - pair;
	if (klen <= 0 || klen >= (int)sizeof(key)) {
		return false;
	}
	memcpy(key, pair, klen);
	key[klen] = '\0';
	const char *val = eq + 1;

	if      (strcmp(key, "FOD")  == 0) cfg.front_obstacle_dist = atoi(val);
	else if (strcmp(key, "SOD")  == 0) cfg.side_open_dist      = atoi(val);
	else if (strcmp(key, "ACD")  == 0) cfg.all_close_dist      = atoi(val);
	else if (strcmp(key, "CFD")  == 0) cfg.close_front_dist    = atoi(val);
	else if (strcmp(key, "KP")   == 0) cfg.pid_kp              = strtof(val, NULL);
	else if (strcmp(key, "KI")   == 0) cfg.pid_ki              = strtof(val, NULL);
	else if (strcmp(key, "KD")   == 0) cfg.pid_kd              = strtof(val, NULL);
	else if (strcmp(key, "MSP")  == 0) cfg.min_speed           = atoi(val);
	else if (strcmp(key, "XSP")  == 0) cfg.max_speed           = atoi(val);
	else if (strcmp(key, "BSP")  == 0) cfg.min_bspeed          = atoi(val);
	else if (strcmp(key, "MNP")  == 0) cfg.min_point           = atoi(val);
	else if (strcmp(key, "XNP")  == 0) cfg.max_point           = atoi(val);
	else if (strcmp(key, "NTP")  == 0) cfg.neutral_point       = atoi(val);
	else if (strcmp(key, "ENH")  == 0) cfg.encoder_holes       = atoi(val);
	else if (strcmp(key, "WDM")  == 0) cfg.wheel_diam_m        = strtof(val, NULL);
	else if (strcmp(key, "LMS")  == 0) cfg.loop_ms             = atoi(val);
	else if (strcmp(key, "SPD1") == 0) cfg.spd_clear           = strtof(val, NULL);
	else if (strcmp(key, "SPD2") == 0) cfg.spd_blocked         = strtof(val, NULL);
	else if (strcmp(key, "COE1") == 0) cfg.coe_clear           = strtof(val, NULL);
	else if (strcmp(key, "COE2") == 0) cfg.coe_blocked         = strtof(val, NULL);
	else if (strcmp(key, "WDD")  == 0) cfg.wrong_dir_deg       = strtof(val, NULL);
	else if (strcmp(key, "RCW")  == 0) cfg.race_cw             = atoi(val) != 0;
	else if (strcmp(key, "STK")  == 0) cfg.stuck_thresh        = atoi(val);
	else if (strcmp(key, "IMR")  == 0) cfg.imu_rotate          = atoi(val) != 0;
	else if (strcmp(key, "SVR")  == 0) cfg.servo_reverse       = atoi(val) != 0;
	else if (strcmp(key, "CAL")  == 0) cfg.calibrated          = atoi(val) != 0;
	else if (strcmp(key, "BEN")  == 0) cfg.bat_enabled         = atoi(val) != 0;
	else if (strcmp(key, "BML")  == 0) cfg.bat_multiplier      = strtof(val, NULL);
	else if (strcmp(key, "BLV")  == 0) cfg.bat_low             = strtof(val, NULL);
	else return false;

	return true;
}

/* ─── GET response ────────────────────────────────────────────────────────── */

static void cmd_get(void)
{
	wifi_cmd_printf(
		"$CFG:FOD=%d,SOD=%d,ACD=%d,CFD=%d"
		",KP=%.4f,KI=%.4f,KD=%.4f"
		",MSP=%d,XSP=%d,BSP=%d"
		",MNP=%d,XNP=%d,NTP=%d"
		",ENH=%d,WDM=%.4f,LMS=%d"
		",SPD1=%.1f,SPD2=%.1f,COE1=%.2f,COE2=%.2f"
		",WDD=%.1f,RCW=%d,STK=%d"
		",IMR=%d,SVR=%d,CAL=%d"
		",BEN=%d,BML=%.4f,BLV=%.1f"
		",IMU=1,DBG=1,SNS=%d,SMX=%d,FWV=2.0.0\n",
		cfg.front_obstacle_dist, cfg.side_open_dist,
		cfg.all_close_dist, cfg.close_front_dist,
		(double)cfg.pid_kp, (double)cfg.pid_ki, (double)cfg.pid_kd,
		cfg.min_speed, cfg.max_speed, cfg.min_bspeed,
		cfg.min_point, cfg.max_point, cfg.neutral_point,
		cfg.encoder_holes, (double)cfg.wheel_diam_m, cfg.loop_ms,
		(double)cfg.spd_clear, (double)cfg.spd_blocked,
		(double)cfg.coe_clear, (double)cfg.coe_blocked,
		(double)cfg.wrong_dir_deg, cfg.race_cw ? 1 : 0, cfg.stuck_thresh,
		cfg.imu_rotate ? 1 : 0, cfg.servo_reverse ? 1 : 0,
		cfg.calibrated ? 1 : 0,
		cfg.bat_enabled ? 1 : 0, (double)cfg.bat_multiplier,
		(double)cfg.bat_low,
		SENSOR_COUNT, MAX_SENSOR_RANGE);
}

/* ─── SET command ─────────────────────────────────────────────────────────── */

static void cmd_set(const char *args)
{
	char buf[CMD_BUF_SIZE];
	strncpy(buf, args, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	char *token = strtok(buf, ",");
	while (token) {
		if (!parse_set_pair(token)) {
			wifi_cmd_printf("$NAK:%s\n", token);
			return;
		}
		token = strtok(NULL, ",");
	}
	wifi_cmd_send("$ACK\n");
}

/* ─── DRV command ─────────────────────────────────────────────────────────── */

static void cmd_drv(const char *args)
{
	char buf[32];
	strncpy(buf, args, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	char *comma = strchr(buf, ',');
	if (!comma) {
		return;
	}
	*comma = '\0';
	int steer = atoi(buf);
	float speed = strtof(comma + 1, NULL);
	control_set_manual(steer, speed);
}

/* ─── Debug console commands ──────────────────────────────────────────────── */

static void cmd_diag(void)
{
	wifi_cmd_printf(
		"$DIAG:SNS=%d,IMU=%d,UP=%lld,BAT=%.2f"
		",RUN=%d,DRV=%d,TAHO=%u,SPD=%.2f\n",
		sensors_online_count(),
		imu_is_ok() ? 1 : 0,
		k_uptime_get(),
		(double)battery_get_voltage(),
		control_is_running() ? 1 : 0,
		0, /* drv_enabled is static in control.c */
		taho_get_count(),
		(double)taho_get_speed());
}

static void cmd_sns(void)
{
	int *s = sensors_poll();
	wifi_cmd_printf("$SNS:%d,%d,%d,%d,%d,%d,online=%d\n",
			s[0], s[1], s[2], s[3], s[4], s[5],
			sensors_online_count());
}

static void cmd_imu(void)
{
	imu_update();
	wifi_cmd_printf("$IMU:ok=%d,yaw=%.2f,hdg=%.1f\n",
			imu_is_ok() ? 1 : 0,
			(double)imu_get_yaw_rate(),
			(double)imu_get_heading());
}

static void cmd_pid(void)
{
	wifi_cmd_printf("$PID:KP=%.4f,KI=%.4f,KD=%.4f"
			",SPD=%.2f,TAHO=%u,TSPD=%.2f\n",
			(double)cfg.pid_kp, (double)cfg.pid_ki,
			(double)cfg.pid_kd,
			(double)taho_get_speed(),
			taho_get_count(),
			(double)taho_get_speed());
}

static void cmd_sys(void)
{
	wifi_cmd_printf("$SYS:UP=%lld,BAT=%.2f"
			",MNP=%d,XNP=%d,NTP=%d"
			",LOOP=%d,SNS=%d\n",
			k_uptime_get(),
			(double)battery_get_voltage(),
			cfg.min_point, cfg.max_point, cfg.neutral_point,
			cfg.loop_ms,
			sensors_online_count());
}

static void cmd_help(void)
{
	wifi_cmd_send(
		"$HELP:Commands:\n"
		"$L: $PING $GET $SET:<k>=<v>,... $SAVE $LOAD $RST\n"
		"$L: $START $STOP $STATUS $BAT\n"
		"$L: $DRV:<steer>,<speed> $DRVEN $DRVOFF\n"
		"$L: $SRV:<angle> $ESC:<us>\n"
		"$L: $TEST:<name> (lidar,servo,taho,esc,speed,autotune,reactive,cal)\n"
		"$L: $TRK:<cmd> (START,STOP,RACE,STATUS,CLEAR)\n"
		"$L: --- Debug ---\n"
		"$L: $DIAG $SNS $IMU $PID $SYS $HELP\n"
		"$L: $LOG:ON $LOG:OFF (toggle debug log forwarding)\n"
	);
}

/* ─── Command dispatcher ─────────────────────────────────────────────────── */

static void dispatch_command(const char *line)
{
	if (strcmp(line, "$PING") == 0) {
		wifi_cmd_send("$PONG\n");
	} else if (strcmp(line, "$GET") == 0) {
		cmd_get();
	} else if (strncmp(line, "$SET:", 5) == 0) {
		cmd_set(line + 5);
	} else if (strcmp(line, "$SAVE") == 0) {
		settings_save();
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$LOAD") == 0) {
		if (settings_load()) {
			wifi_cmd_send("$ACK\n");
		} else {
			wifi_cmd_send("$NAK:no_saved_config\n");
		}
	} else if (strcmp(line, "$RST") == 0) {
		settings_reset();
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$START") == 0) {
		control_cmd_start();
	} else if (strcmp(line, "$STOP") == 0) {
		control_cmd_stop();
	} else if (strcmp(line, "$STATUS") == 0) {
		wifi_cmd_printf("$STS:%s\n", control_is_running() ? "RUN" : "STOP");
	} else if (strcmp(line, "$BAT") == 0) {
		wifi_cmd_printf("$BAT:%.2f\n", (double)battery_get_voltage());
	} else if (strncmp(line, "$TEST:", 6) == 0) {
		tests_run_by_name(line + 6);
	} else if (strncmp(line, "$DRV:", 5) == 0) {
		cmd_drv(line + 5);
	} else if (strncmp(line, "$SRV:", 5) == 0) {
		int angle = CLAMP(atoi(line + 5), 0, 180);
		car_write_servo_raw(angle);
	} else if (strncmp(line, "$ESC:", 5) == 0) {
		int val = CLAMP(atoi(line + 5), 1000, 2000);
		car_write_esc_us(val);
	} else if (strcmp(line, "$DRVEN") == 0) {
		control_set_drv_enabled(true);
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$DRVOFF") == 0) {
		control_set_drv_enabled(false);
		car_write_steer(0);
		car_write_speed(0);
		wifi_cmd_send("$ACK\n");
	} else if (strncmp(line, "$TRK:", 5) == 0) {
		track_learn_dispatch(line + 5);
	/* ── Debug console ─────────────────────────────────────── */
	} else if (strcmp(line, "$DIAG") == 0) {
		cmd_diag();
	} else if (strcmp(line, "$SNS") == 0) {
		cmd_sns();
	} else if (strcmp(line, "$IMU") == 0) {
		cmd_imu();
	} else if (strcmp(line, "$PID") == 0) {
		cmd_pid();
	} else if (strcmp(line, "$SYS") == 0) {
		cmd_sys();
	} else if (strcmp(line, "$HELP") == 0) {
		cmd_help();
	} else if (strcmp(line, "$LOG:ON") == 0) {
		log_on = true;
		wifi_cmd_send("$ACK\n");
	} else if (strcmp(line, "$LOG:OFF") == 0) {
		log_on = false;
		wifi_cmd_send("$ACK\n");
	}
	/* Unknown commands silently ignored */
}

/* ─── Thread entry point ──────────────────────────────────────────────────── */

static void wifi_cmd_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("WiFi command thread started");

	/* Send CSV header */
	wifi_cmd_printf("#ms,s0,s1,s2,s3,s4,s5,steer,speed,target,yaw,heading\n");
	/* Query ESP status */
	wifi_cmd_send("#WIFISTATUS\n");

	while (1) {
		/* Block until a full line is available */
		k_sem_take(&rx_line_sem, K_FOREVER);

		/* Drain ring buffer into cmd_buf */
		uint8_t c;
		while (rb_get(&c)) {
			if (c == '\n' || c == '\r') {
				if (cmd_len > 0) {
					cmd_buf[cmd_len] = '\0';
					if (cmd_buf[0] == '$') {
						dispatch_command(cmd_buf);
					}
					/* # ESP status lines — just log, no parsing needed */
					cmd_len = 0;
				}
			} else if (cmd_len < CMD_BUF_SIZE - 1) {
				cmd_buf[cmd_len++] = (char)c;
			}
		}
	}
}

/* ─── Init ────────────────────────────────────────────────────────────────── */

void wifi_cmd_init(void)
{
	uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART1 not ready");
		return;
	}

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	k_thread_create(&wifi_thread_data, wifi_stack,
			K_THREAD_STACK_SIZEOF(wifi_stack),
			wifi_cmd_thread, NULL, NULL, NULL,
			WIFI_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&wifi_thread_data, "wifi_cmd");

	LOG_INF("WiFi CMD init (UART1 GP4/GP5, 115200)");
}
