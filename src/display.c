/*
 * display.c — SSD1306 OLED 128x64 menu system with rotary encoder
 *
 * Ported from Umbreon_roborace/menu.h to Zephyr RTOS.
 *
 * Thread: display_thread (priority 8, stack 2048, ~8 FPS)
 * Display is OFF by default, wakes on encoder double-click,
 * sleeps on 10s inactivity or when car enters RUN/test state.
 */

#include "display.h"
#include "encoder.h"
#include "settings.h"
#include "sensors.h"
#include "imu.h"
#include "tachometer.h"
#include "battery.h"
#include "control.h"
#include "wifi_cmd.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(display, LOG_LEVEL_INF);

/* ─── Thread config ──────────────────────────────────────────────────────── */
#define DISPLAY_STACK_SIZE   2048
#define DISPLAY_PRIORITY     8
#define DISPLAY_REFRESH_MS   120
#define INACTIVITY_MS        10000

static K_THREAD_STACK_DEFINE(display_stack, DISPLAY_STACK_SIZE);
static struct k_thread display_thread_data;

/* ─── I2C0 bus mutex (shared with IMU) ───────────────────────────────────── */
K_MUTEX_DEFINE(i2c0_mutex);

/* ─── Menu command queue ─────────────────────────────────────────────────── */
K_MSGQ_DEFINE(menu_cmd_q, sizeof(uint8_t), 4, 4);

/* ─── Wake event ─────────────────────────────────────────────────────────── */
static K_EVENT_DEFINE(display_event);
#define EVT_WAKE BIT(0)

/* ─── State ──────────────────────────────────────────────────────────────── */
static const struct device *oled_dev;
static volatile bool car_is_running;
static volatile bool test_is_active;

/* ─── Screen state machine ───────────────────────────────────────────────── */
enum screen {
	SCR_DASHBOARD,
	SCR_MAIN_MENU,
	SCR_SETTINGS_GROUPS,
	SCR_SETTINGS_LIST,
	SCR_SETTINGS_EDIT,
	SCR_TESTS,
	SCR_TEST_RUNNING,
	SCR_ACTIONS,
	SCR_CONFIRM,
	SCR_INFO,
};

static enum screen cur_scr;
static enum screen prev_scr;
static int sel;
static int scroll;
static int64_t last_activity;

/* ─── Font metrics (Zephyr CFB default 6x8) ──────────────────────────────── */
#define FONT_W  6
#define FONT_H  8
#define SCR_W   128
#define SCR_H   64

/* ─── Sub-screen state ───────────────────────────────────────────────────── */
static int grp_sel;
static int param_idx;
static float edit_val;
static int test_idx;
static int action_idx;
static bool confirm_yes;
static int info_scroll;

/* ─── Parameter descriptor ───────────────────────────────────────────────── */
enum param_type { PT_INT, PT_FLOAT, PT_BOOL };

struct param_desc {
	const char *key;
	const char *label;
	enum param_type type;
	float step;
	float step_fast;
	float lo, hi;
	size_t offset;  /* offsetof(struct car_settings, field) */
	float scale;
};

#define P_OFF(field) offsetof(struct car_settings, field)

static const struct param_desc params[] = {
	/* Obstacles (cm×10) */
	{"FOD", "Front Obs",  PT_INT,   10,  100,   50, 9000, P_OFF(front_obstacle_dist), 1},
	{"SOD", "Side Open",  PT_INT,   10,  100,   50, 9000, P_OFF(side_open_dist),      1},
	{"ACD", "All Close",  PT_INT,   10,  100,   50, 9000, P_OFF(all_close_dist),      1},
	{"CFD", "Close Frt",  PT_INT,   10,  100,   50, 9000, P_OFF(close_front_dist),    1},
	/* PID */
	{"KP",  "PID Kp",     PT_FLOAT, 0.1f, 1.0f, 0, 200, P_OFF(pid_kp),              1},
	{"KI",  "PID Ki",     PT_FLOAT, 0.1f, 1.0f, 0, 200, P_OFF(pid_ki),              1},
	{"KD",  "PID Kd",     PT_FLOAT, 0.1f, 1.0f, 0, 200, P_OFF(pid_kd),              1},
	/* Speed / ESC (µs) */
	{"MSP", "Min Spd",    PT_INT,   5,   20,  1500, 2000, P_OFF(min_speed),           1},
	{"XSP", "Max Spd",    PT_INT,   5,   20,  1500, 2000, P_OFF(max_speed),           1},
	{"BSP", "Min Bck",    PT_INT,   5,   20,  1000, 1500, P_OFF(min_bspeed),          1},
	/* Steering (degrees) */
	{"MNP", "Srv Min",    PT_INT,   1,    5,    0,  90,   P_OFF(min_point),           1},
	{"XNP", "Srv Max",    PT_INT,   1,    5,   90, 180,   P_OFF(max_point),           1},
	{"NTP", "Srv Neu",    PT_INT,   1,    5,    0, 180,   P_OFF(neutral_point),       1},
	/* Tachometer */
	{"ENH", "Enc Holes",  PT_INT,   1,   10,    1, 200,   P_OFF(encoder_holes),       1},
	{"WDM", "Wheel mm",   PT_FLOAT, 1,   10,   10, 500,   P_OFF(wheel_diam_m),     1000},
	/* Control */
	{"LMS", "Loop ms",    PT_INT,   5,   10,   10, 200,   P_OFF(loop_ms),             1},
	{"SPD1","Spd Clr",    PT_FLOAT, 0.1f, 0.5f, 0, 5.0f, P_OFF(spd_clear),          1},
	{"SPD2","Spd Blk",    PT_FLOAT, 0.1f, 0.5f, 0, 5.0f, P_OFF(spd_blocked),        1},
	{"SLW", "Spd Slew",   PT_FLOAT, 0.05f,0.2f, 0, 20.0f,P_OFF(spd_slew),           1},
	{"COE1","Coe Clr",    PT_FLOAT, 0.05f,0.1f, 0, 2.0f, P_OFF(coe_clear),          1},
	{"COE2","Coe Blk",    PT_FLOAT, 0.05f,0.1f, 0, 2.0f, P_OFF(coe_blocked),        1},
	/* Navigation */
	{"WDD", "Wrong Dir",  PT_FLOAT, 5.0f, 20.0f, 10, 360, P_OFF(wrong_dir_deg),     1},
	{"RCW", "Race CW",    PT_BOOL,  1,    1,     0, 1,    P_OFF(race_cw),            1},
	{"STK", "Stuck Thr",  PT_INT,   1,    5,     1, 100,  P_OFF(stuck_thresh),        1},
	/* Hardware */
	{"IMR", "IMU Rot",    PT_BOOL,  1,    1,     0, 1,    P_OFF(imu_rotate),          1},
	{"SVR", "Srv Rev",    PT_BOOL,  1,    1,     0, 1,    P_OFF(servo_reverse),       1},
	{"BEN", "Bat En",     PT_BOOL,  1,    1,     0, 1,    P_OFF(bat_enabled),         1},
	{"BML", "Bat Mult",   PT_FLOAT, 0.1f, 0.5f,1.0f,10.0f, P_OFF(bat_multiplier),   1},
	{"BLV", "Bat Low V",  PT_FLOAT, 0.1f, 0.5f,3.0f,12.0f, P_OFF(bat_low),          1},
};
#define PARAM_COUNT ((int)(sizeof(params) / sizeof(params[0])))

/* ─── Parameter groups ───────────────────────────────────────────────────── */
struct param_group {
	const char *name;
	uint8_t start;
	uint8_t count;
};

static const struct param_group groups[] = {
	{"Obstacles",   0,  4},
	{"PID",         4,  3},
	{"Speed/ESC",   7,  3},
	{"Steering",   10,  3},
	{"Tachometer", 13,  2},
	{"Control",    15,  6},
	{"Navigation", 21,  3},
	{"Hardware",   24,  5},
};
#define GROUP_COUNT ((int)(sizeof(groups) / sizeof(groups[0])))

/* ─── Tests ──────────────────────────────────────────────────────────────── */
struct test_item {
	const char *name;
	const char *label;
	bool motor;
};

static const struct test_item tests[] = {
	{"lidar",    "Lidar Scan",   false},
	{"servo",    "Servo Sweep",  false},
	{"taho",     "Tachometer",   false},
	{"esc",      "ESC Test",     true},
	{"speed",    "Speed Hold",   true},
	{"autotune", "PID Autotune", true},
	{"reactive", "Reactive",     false},
	{"cal",      "ESC Calibr.",  true},
};
#define TEST_COUNT ((int)(sizeof(tests) / sizeof(tests[0])))

/* ─── Actions ────────────────────────────────────────────────────────────── */
enum action_id { ACT_START, ACT_STOP, ACT_SAVE, ACT_LOAD, ACT_RESET };

struct action_item {
	const char *label;
	enum action_id id;
	bool confirm;
};

static const struct action_item actions[] = {
	{"Start Car",      ACT_START, true},
	{"Stop Car",       ACT_STOP,  false},
	{"Save NVS",       ACT_SAVE,  true},
	{"Load NVS",       ACT_LOAD,  true},
	{"Reset Defaults", ACT_RESET, true},
};
#define ACTION_COUNT ((int)(sizeof(actions) / sizeof(actions[0])))

/* ─── Main menu ──────────────────────────────────────────────────────────── */
static const char *main_items[] = {"Settings", "Tests", "Actions", "Info"};
#define MAIN_REAL  4
#define MAIN_COUNT (MAIN_REAL + 1)  /* +1 for "< Back" */

/* ─── Sensor labels ──────────────────────────────────────────────────────── */
static const char *sns_short[] = {"HR", "FR", "R ", "L ", "FL", "HL"};

/* ─── List layout ────────────────────────────────────────────────────────── */
#define LIST_VISIBLE 6
#define INFO_VISIBLE 6

/* ─── Parameter helpers ──────────────────────────────────────────────────── */

static float param_get(const struct param_desc *p)
{
	void *ptr = (char *)&cfg + p->offset;
	float raw;
	switch (p->type) {
	case PT_INT:   raw = (float)(*(int *)ptr);  break;
	case PT_FLOAT: raw = *(float *)ptr;         break;
	case PT_BOOL:  return *(bool *)ptr ? 1.0f : 0.0f;
	default:       return 0;
	}
	return raw * p->scale;
}

static void param_set(const struct param_desc *p, float v)
{
	if (v < p->lo) v = p->lo;
	if (v > p->hi) v = p->hi;
	float raw = v / p->scale;
	void *ptr = (char *)&cfg + p->offset;
	switch (p->type) {
	case PT_INT:   *(int *)ptr   = (int)raw;   break;
	case PT_FLOAT: *(float *)ptr = raw;         break;
	case PT_BOOL:  *(bool *)ptr  = (v > 0.5f); break;
	}
}

static int param_fmt(char *buf, int sz, const struct param_desc *p, float v)
{
	switch (p->type) {
	case PT_INT:   return snprintf(buf, sz, "%d", (int)v);
	case PT_FLOAT:
		if (p->scale != 1.0f) return snprintf(buf, sz, "%d", (int)(v + 0.5f));
		return snprintf(buf, sz, "%.2f", (double)v);
	case PT_BOOL:  return snprintf(buf, sz, "%s", v > 0.5f ? "ON" : "OFF");
	}
	return 0;
}

static void clamp_scroll(int *s, int *sc, int total, int visible)
{
	if (*s < 0) *s = 0;
	if (*s >= total) *s = total - 1;
	if (*s < *sc) *sc = *s;
	if (*s >= *sc + visible) *sc = *s - visible + 1;
}

/* ─── CFB text helper ────────────────────────────────────────────────────── */
/* cfb_print returns the text at a given row, but we want pixel-level positioning.
 * cfb_draw_text() takes (dev, str, x, y) */

static void draw_text(int x, int y, const char *str)
{
	cfb_draw_text(oled_dev, str, x, y);
}

/* ─── Screen drawing functions ───────────────────────────────────────────── */

static void draw_dashboard(void)
{
	char buf[22];

	/* Line 0: status */
	draw_text(0, 0, control_is_running() ? "RUN" : "STOP");

	/* Battery */
	if (cfg.bat_enabled) {
		snprintf(buf, sizeof(buf), " %.1fV", (double)battery_get_voltage());
		draw_text(24, 0, buf);
	}

	/* Speed */
	snprintf(buf, sizeof(buf), "%.2fm/s", (double)taho_get_speed());
	int sw = (int)strlen(buf) * FONT_W;
	draw_text(SCR_W - sw, 0, buf);

	/* Separator */
	cfb_draw_line(oled_dev, &(struct cfb_position){0, 10},
		      &(struct cfb_position){SCR_W - 1, 10});

	/* Sensor bars */
	const int *s = sensors_get_distances();
	const int bar_y = 14;
	const int bar_spacing = 7;
	const int bar_max_w = 76;
	const int val_x = 102;

	for (int i = 0; i < SENSOR_COUNT && i < 6; i++) {
		int y = bar_y + i * bar_spacing;
		draw_text(0, y, sns_short[i]);

		int val = (s[i] == 9999) ? 0 : s[i];
		int bw = val * bar_max_w / MAX_SENSOR_RANGE;
		if (bw > bar_max_w) bw = bar_max_w;

		/* Bar outline */
		cfb_draw_rect(oled_dev, &(struct cfb_position){20, y},
			      &(struct cfb_position){20 + bar_max_w + 1, y + 4});
		/* Bar fill */
		if (bw > 0) {
			cfb_draw_rect(oled_dev, &(struct cfb_position){21, y + 1},
				      &(struct cfb_position){21 + bw - 1, y + 3});
		}

		/* Value text */
		if (s[i] == 9999) {
			draw_text(val_x, y, "--");
		} else {
			snprintf(buf, sizeof(buf), "%d", s[i] / 10);
			draw_text(val_x, y, buf);
		}
	}

	/* IMU heading at bottom */
	snprintf(buf, sizeof(buf), "Yaw:%5.1f Hdg:%5.1f",
		 (double)imu_get_yaw_rate(), (double)imu_get_heading());
	draw_text(0, 56, buf);
}

/* Generic list renderer */
typedef void (*item_fn_t)(int idx, char *buf, int sz);

static void draw_list(const char *title, int count, item_fn_t fn,
		      int sel_idx, int scroll_off)
{
	/* Title bar (inverted) */
	cfb_invert_area(oled_dev, 0, 0, SCR_W, 10);
	draw_text(2, 1, title);

	/* Items */
	for (int i = 0; i < LIST_VISIBLE && (scroll_off + i) < count; i++) {
		int idx = scroll_off + i;
		int y = 12 + i * 9;

		char buf[22];
		fn(idx, buf, sizeof(buf));

		if (idx == sel_idx) {
			cfb_invert_area(oled_dev, 0, y - 1, SCR_W, 9);
		}

		draw_text(2, y, buf);
	}

	/* Scroll indicator */
	if (count > LIST_VISIBLE) {
		int bar_h = (SCR_H * LIST_VISIBLE) / count;
		if (bar_h < 4) bar_h = 4;
		int max_scroll = count - LIST_VISIBLE;
		int bar_y = (max_scroll > 0) ? (scroll_off * (SCR_H - bar_h) / max_scroll) : 0;
		cfb_draw_rect(oled_dev, &(struct cfb_position){SCR_W - 2, bar_y},
			      &(struct cfb_position){SCR_W - 1, bar_y + bar_h - 1});
	}
}

/* Item renderers for each list */
static void main_item_fn(int idx, char *buf, int sz)
{
	if (idx == 0) { snprintf(buf, sz, "< Back"); return; }
	snprintf(buf, sz, "%s", main_items[idx - 1]);
}

static void group_item_fn(int idx, char *buf, int sz)
{
	if (idx == 0) { snprintf(buf, sz, "< Back"); return; }
	snprintf(buf, sz, "%s (%d)", groups[idx - 1].name, groups[idx - 1].count);
}

static void param_item_fn(int idx, char *buf, int sz)
{
	if (idx == 0) { snprintf(buf, sz, "< Back"); return; }
	int pi = groups[grp_sel].start + (idx - 1);
	const struct param_desc *p = &params[pi];
	char vbuf[10];
	param_fmt(vbuf, sizeof(vbuf), p, param_get(p));
	snprintf(buf, sz, "%-8s %s", p->label, vbuf);
}

static void test_item_fn(int idx, char *buf, int sz)
{
	if (idx == 0) { snprintf(buf, sz, "< Back"); return; }
	snprintf(buf, sz, "%s%s", tests[idx - 1].label,
		 tests[idx - 1].motor ? " [!]" : "");
}

static void action_item_fn(int idx, char *buf, int sz)
{
	if (idx == 0) { snprintf(buf, sz, "< Back"); return; }
	snprintf(buf, sz, "%s", actions[idx - 1].label);
}

/* Edit screen */
static void draw_edit(void)
{
	const struct param_desc *p = &params[param_idx];

	/* Title bar */
	cfb_invert_area(oled_dev, 0, 0, SCR_W, 10);
	draw_text(2, 1, p->label);

	/* Key */
	char kbuf[16];
	snprintf(kbuf, sizeof(kbuf), "Key: %s", p->key);
	draw_text(2, 14, kbuf);

	/* Value (centered) */
	char vbuf[12];
	param_fmt(vbuf, sizeof(vbuf), p, edit_val);
	int tw = (int)strlen(vbuf) * FONT_W;
	draw_text((SCR_W - tw) / 2, 28, vbuf);

	/* Range + Step */
	char rbuf[22];
	if (p->type == PT_BOOL) {
		snprintf(rbuf, sizeof(rbuf), "Turn to toggle");
	} else if (p->type == PT_INT) {
		snprintf(rbuf, sizeof(rbuf), "%d..%d step %d",
			 (int)p->lo, (int)p->hi, (int)p->step);
	} else {
		snprintf(rbuf, sizeof(rbuf), "%.1f..%.1f step %.2f",
			 (double)p->lo, (double)p->hi, (double)p->step);
	}
	draw_text(2, 48, rbuf);

	draw_text(2, 56, "Click=OK Hold=Cancel");
}

/* Confirm dialog */
static const char *confirm_msg = "";

static void draw_confirm(void)
{
	draw_text(2, 8, confirm_msg);
	draw_text(2, 24, "Are you sure?");

	int y = 42;
	if (confirm_yes) {
		cfb_invert_area(oled_dev, 10, y - 2, 44, 14);
	}
	draw_text(16, y, "[ Yes ]");

	if (!confirm_yes) {
		cfb_invert_area(oled_dev, 70, y - 2, 44, 14);
	}
	draw_text(76, y, "[ No ]");
}

/* Test running screen */
static void draw_test_running(void)
{
	draw_text(2, 10, "Running...");

	char tbuf[22];
	snprintf(tbuf, sizeof(tbuf), "Test: %s", tests[test_idx].label);
	draw_text(2, 28, tbuf);

	/* Animated dots */
	static uint8_t dots;
	dots = (dots + 1) % 4;
	char dbuf[5] = "";
	for (int i = 0; i < dots; i++) dbuf[i] = '.';
	dbuf[dots] = '\0';
	draw_text(2, 42, dbuf);

	draw_text(2, 56, "Click = Abort");
}

/* Info screen */
static void draw_info(void)
{
	cfb_invert_area(oled_dev, 0, 0, SCR_W, 10);
	draw_text(2, 1, "Info");

	char lines[14][22];
	int count = 0;

	snprintf(lines[count++], 22, "FW: v2.0.0");
	snprintf(lines[count++], 22, "Core: M33 (Zephyr)");
	snprintf(lines[count++], 22, "Sensors: %d (VL53)", sensors_online_count());
	snprintf(lines[count++], 22, "IMU: %s", imu_is_ok() ? "OK" : "FAIL");
	if (cfg.bat_enabled) {
		snprintf(lines[count++], 22, "Bat: %.1fV", (double)battery_get_voltage());
	} else {
		snprintf(lines[count++], 22, "Bat: Disabled");
	}
	snprintf(lines[count++], 22, "Loop: %dms Enc: %d", cfg.loop_ms, cfg.encoder_holes);

	int64_t secs = k_uptime_get() / 1000;
	snprintf(lines[count++], 22, "Up: %lldm %llds", secs / 60, secs % 60);

	/* Clamp scroll */
	if (info_scroll > count - INFO_VISIBLE) info_scroll = count - INFO_VISIBLE;
	if (info_scroll < 0) info_scroll = 0;

	for (int i = 0; i < INFO_VISIBLE && (info_scroll + i) < count; i++) {
		draw_text(2, 12 + i * 9, lines[info_scroll + i]);
	}

	if (count > INFO_VISIBLE) {
		int bar_h = SCR_H * INFO_VISIBLE / count;
		if (bar_h < 4) bar_h = 4;
		int max_sc = count - INFO_VISIBLE;
		int bar_y = (max_sc > 0) ? (info_scroll * (SCR_H - bar_h) / max_sc) : 0;
		cfb_draw_rect(oled_dev, &(struct cfb_position){SCR_W - 2, bar_y},
			      &(struct cfb_position){SCR_W - 1, bar_y + bar_h - 1});
	}
}

/* ─── Navigation ─────────────────────────────────────────────────────────── */

static void go_screen(enum screen scr)
{
	prev_scr = cur_scr;
	cur_scr = scr;
	sel = (scr == SCR_MAIN_MENU || scr == SCR_SETTINGS_GROUPS ||
	       scr == SCR_SETTINGS_LIST || scr == SCR_TESTS ||
	       scr == SCR_ACTIONS) ? 1 : 0;
	scroll = 0;
}

static void go_back(void)
{
	switch (cur_scr) {
	case SCR_MAIN_MENU:       cur_scr = SCR_DASHBOARD;       break;
	case SCR_SETTINGS_GROUPS: cur_scr = SCR_MAIN_MENU;       break;
	case SCR_SETTINGS_LIST:   cur_scr = SCR_SETTINGS_GROUPS; break;
	case SCR_SETTINGS_EDIT:   cur_scr = SCR_SETTINGS_LIST;   break;
	case SCR_TESTS:           cur_scr = SCR_MAIN_MENU;       break;
	case SCR_ACTIONS:         cur_scr = SCR_MAIN_MENU;       break;
	case SCR_CONFIRM:         cur_scr = prev_scr;            break;
	case SCR_INFO:            cur_scr = SCR_MAIN_MENU;       break;
	case SCR_TEST_RUNNING:    break;
	default:                  cur_scr = SCR_DASHBOARD;       break;
	}
	sel = 1;
	scroll = 0;
}

/* ─── Command dispatch ───────────────────────────────────────────────────── */

static void exec_action(enum action_id id)
{
	uint8_t cmd;
	switch (id) {
	case ACT_START: cmd = MCMD_START; break;
	case ACT_STOP:  cmd = MCMD_STOP;  break;
	case ACT_SAVE:  cmd = MCMD_SAVE;  break;
	case ACT_LOAD:  cmd = MCMD_LOAD;  break;
	case ACT_RESET: cmd = MCMD_RESET; break;
	default: return;
	}
	k_msgq_put(&menu_cmd_q, &cmd, K_NO_WAIT);
}

static void run_test(int idx)
{
	uint8_t cmd = MCMD_TEST_BASE + (uint8_t)idx;
	k_msgq_put(&menu_cmd_q, &cmd, K_NO_WAIT);
	test_idx = idx;
	test_is_active = true;
	go_screen(SCR_TEST_RUNNING);
}

/* ─── Input handling ─────────────────────────────────────────────────────── */

static void handle_input(void)
{
	int rot = 0;
	uint8_t events = encoder_poll(&rot);

	bool click = events & ENC_EVT_CLICK;
	bool held  = events & ENC_EVT_HOLD;
	bool fast  = events & ENC_EVT_FAST;
	int dir = rot;

	if (events || dir) {
		last_activity = k_uptime_get();
	}

	/* Long-press → dashboard */
	if (held) {
		if (cur_scr == SCR_SETTINGS_EDIT) {
			cur_scr = SCR_SETTINGS_LIST;
		} else {
			cur_scr = SCR_DASHBOARD;
		}
		sel = 0;
		scroll = 0;
		return;
	}

	switch (cur_scr) {
	case SCR_DASHBOARD:
		if (click) go_screen(SCR_MAIN_MENU);
		break;

	case SCR_MAIN_MENU:
		sel += dir;
		clamp_scroll(&sel, &scroll, MAIN_COUNT, LIST_VISIBLE);
		if (click) {
			if (sel == 0) { go_back(); }
			else switch (sel) {
			case 1: go_screen(SCR_SETTINGS_GROUPS); break;
			case 2: go_screen(SCR_TESTS);           break;
			case 3: go_screen(SCR_ACTIONS);          break;
			case 4: go_screen(SCR_INFO);             break;
			}
		}
		break;

	case SCR_SETTINGS_GROUPS: {
		int cnt = GROUP_COUNT + 1;
		sel += dir;
		clamp_scroll(&sel, &scroll, cnt, LIST_VISIBLE);
		if (click) {
			if (sel == 0) { go_back(); }
			else { grp_sel = sel - 1; go_screen(SCR_SETTINGS_LIST); }
		}
		break;
	}

	case SCR_SETTINGS_LIST: {
		int cnt = groups[grp_sel].count + 1;
		sel += dir;
		clamp_scroll(&sel, &scroll, cnt, LIST_VISIBLE);
		if (click) {
			if (sel == 0) { go_back(); }
			else {
				param_idx = groups[grp_sel].start + (sel - 1);
				edit_val = param_get(&params[param_idx]);
				go_screen(SCR_SETTINGS_EDIT);
			}
		}
		break;
	}

	case SCR_SETTINGS_EDIT: {
		const struct param_desc *p = &params[param_idx];
		if (dir != 0) {
			if (p->type == PT_BOOL) {
				edit_val = (edit_val > 0.5f) ? 0.0f : 1.0f;
			} else {
				float s = fast ? p->step_fast : p->step;
				edit_val += dir * s;
				if (edit_val < p->lo) edit_val = p->lo;
				if (edit_val > p->hi) edit_val = p->hi;
			}
		}
		if (click) {
			param_set(p, edit_val);
			cur_scr = SCR_SETTINGS_LIST;
			sel = (param_idx - groups[grp_sel].start) + 1;
			scroll = 0;
			clamp_scroll(&sel, &scroll, groups[grp_sel].count + 1, LIST_VISIBLE);
		}
		break;
	}

	case SCR_TESTS: {
		int tcnt = TEST_COUNT + 1;
		sel += dir;
		clamp_scroll(&sel, &scroll, tcnt, LIST_VISIBLE);
		if (click) {
			if (sel == 0) { go_back(); }
			else {
				int ti = sel - 1;
				if (tests[ti].motor) {
					test_idx = ti;
					confirm_msg = tests[ti].label;
					confirm_yes = false;
					prev_scr = SCR_TESTS;
					go_screen(SCR_CONFIRM);
				} else {
					run_test(ti);
				}
			}
		}
		break;
	}

	case SCR_TEST_RUNNING:
		if (!test_is_active) {
			cur_scr = SCR_TESTS;
			sel = test_idx + 1;
			scroll = 0;
		}
		if (click) {
			uint8_t cmd = MCMD_STOP;
			k_msgq_put(&menu_cmd_q, &cmd, K_NO_WAIT);
			cur_scr = SCR_TESTS;
		}
		break;

	case SCR_ACTIONS: {
		int acnt = ACTION_COUNT + 1;
		sel += dir;
		clamp_scroll(&sel, &scroll, acnt, LIST_VISIBLE);
		if (click) {
			if (sel == 0) { go_back(); }
			else {
				int ai = sel - 1;
				action_idx = ai;
				if (actions[ai].confirm) {
					confirm_msg = actions[ai].label;
					confirm_yes = false;
					prev_scr = SCR_ACTIONS;
					go_screen(SCR_CONFIRM);
				} else {
					exec_action(actions[ai].id);
					go_screen(SCR_DASHBOARD);
				}
			}
		}
		break;
	}

	case SCR_CONFIRM:
		if (dir != 0) confirm_yes = !confirm_yes;
		if (click) {
			if (confirm_yes) {
				if (prev_scr == SCR_TESTS) {
					run_test(test_idx);
				} else if (prev_scr == SCR_ACTIONS) {
					exec_action(actions[action_idx].id);
					go_screen(SCR_DASHBOARD);
				}
			} else {
				go_back();
			}
		}
		break;

	case SCR_INFO:
		if (dir != 0) info_scroll += dir;
		if (click) { info_scroll = 0; go_back(); }
		break;
	}
}

/* ─── Render current screen ──────────────────────────────────────────────── */

static void draw_current_screen(void)
{
	switch (cur_scr) {
	case SCR_DASHBOARD:
		draw_dashboard();
		break;
	case SCR_MAIN_MENU:
		draw_list("Menu", MAIN_COUNT, main_item_fn, sel, scroll);
		break;
	case SCR_SETTINGS_GROUPS:
		draw_list("Settings", GROUP_COUNT + 1, group_item_fn, sel, scroll);
		break;
	case SCR_SETTINGS_LIST:
		draw_list(groups[grp_sel].name, groups[grp_sel].count + 1,
			  param_item_fn, sel, scroll);
		break;
	case SCR_SETTINGS_EDIT:
		draw_edit();
		break;
	case SCR_TESTS:
		draw_list("Tests", TEST_COUNT + 1, test_item_fn, sel, scroll);
		break;
	case SCR_TEST_RUNNING:
		draw_test_running();
		break;
	case SCR_ACTIONS:
		draw_list("Actions", ACTION_COUNT + 1, action_item_fn, sel, scroll);
		break;
	case SCR_CONFIRM:
		draw_confirm();
		break;
	case SCR_INFO:
		draw_info();
		break;
	}
}

/* ─── Display thread ─────────────────────────────────────────────────────── */

static void display_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* I2C0 raw diagnostics */
	const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (device_is_ready(i2c0)) {
		/* Try raw I2C command to SSD1306: display off */
		uint8_t cmd_off[] = {0x00, 0xAE};
		int rc = i2c_write(i2c0, cmd_off, sizeof(cmd_off), 0x3C);
		wifi_cmd_printf("$L:I2C0 raw write to 0x3C: rc=%d\n", rc);

		/* Try display on */
		uint8_t cmd_on[] = {0x00, 0xAF};
		rc = i2c_write(i2c0, cmd_on, sizeof(cmd_on), 0x3C);
		wifi_cmd_printf("$L:I2C0 display-on cmd: rc=%d\n", rc);
	} else {
		wifi_cmd_printf("$L:I2C0 bus not ready!\n");
	}

	oled_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
	wifi_cmd_printf("$L:OLED device_is_ready=%d\n", device_is_ready(oled_dev));
	if (!device_is_ready(oled_dev)) {
		LOG_ERR("SSD1306 not ready");
		wifi_cmd_printf("$L:OLED not ready! Trying manual init...\n");

		/* Try full SSD1306 init sequence manually */
		uint8_t init_cmds[] = {
			0x00,  /* Co=0, D/C#=0 (command stream) */
			0xAE,  /* display off */
			0xD5, 0x80,  /* clock divide ratio */
			0xA8, 0x3F,  /* multiplex ratio = 63 */
			0xD3, 0x00,  /* display offset = 0 */
			0x40,        /* start line = 0 */
			0x8D, 0x14,  /* charge pump ON */
			0x20, 0x00,  /* horizontal addressing mode */
			0xA1,        /* segment remap */
			0xC8,        /* COM output scan direction */
			0xDA, 0x12,  /* COM pins config: alternative */
			0x81, 0xCF,  /* contrast */
			0xD9, 0x22,  /* precharge period */
			0xDB, 0x40,  /* VCOM deselect level */
			0xA4,        /* display from RAM */
			0xA6,        /* normal display */
			0xAF,        /* display ON */
		};
		int rc2 = i2c_write(i2c0, init_cmds, sizeof(init_cmds), 0x3C);
		wifi_cmd_printf("$L:Manual init: rc=%d\n", rc2);
		if (rc2 == 0) {
			wifi_cmd_printf("$L:Display should be ON now (manual)\n");
		}
		return;
	}

	k_mutex_lock(&i2c0_mutex, K_FOREVER);
	int cfb_rc = cfb_framebuffer_init(oled_dev);
	if (cfb_rc) {
		LOG_ERR("CFB init failed: %d", cfb_rc);
		wifi_cmd_printf("$L:CFB init fail %d\n", cfb_rc);
		k_mutex_unlock(&i2c0_mutex);
		return;
	}
	display_blanking_on(oled_dev);  /* start blanked (OFF) */
	k_mutex_unlock(&i2c0_mutex);

	cfb_framebuffer_set_font(oled_dev, 0);  /* default 6x8 */
	LOG_INF("OLED display ready (128x64, off by default)");
	wifi_cmd_printf("$L:OLED ready, waiting for double-click\n");

	while (1) {
		/* ── SLEEP: wait for wake event ─────────────────────────── */
		k_event_wait(&display_event, EVT_WAKE, true, K_FOREVER);

		/* Don't wake if car is running or test active */
		if (car_is_running || test_is_active) {
			wifi_cmd_printf("$L:OLED wake blocked (run=%d test=%d)\n",
					car_is_running, test_is_active);
			continue;
		}

		wifi_cmd_printf("$L:OLED waking up!\n");

		/* Wake: turn on display */
		k_mutex_lock(&i2c0_mutex, K_FOREVER);
		display_blanking_off(oled_dev);
		k_mutex_unlock(&i2c0_mutex);

		cur_scr = SCR_DASHBOARD;
		sel = 0;
		scroll = 0;
		last_activity = k_uptime_get();

		/* ── ACTIVE LOOP ────────────────────────────────────────── */
		while (1) {
			/* Check shutdown conditions */
			if (car_is_running || test_is_active) {
				break;
			}
			if ((k_uptime_get() - last_activity) > INACTIVITY_MS) {
				break;
			}

			handle_input();

			/* Render framebuffer (no I2C) */
			cfb_framebuffer_clear(oled_dev, false);
			draw_current_screen();

			/* Transfer to display (I2C) */
			k_mutex_lock(&i2c0_mutex, K_FOREVER);
			cfb_framebuffer_finalize(oled_dev);
			k_mutex_unlock(&i2c0_mutex);

			k_msleep(DISPLAY_REFRESH_MS);
		}

		/* ── Go to sleep: blank display ─────────────────────────── */
		k_mutex_lock(&i2c0_mutex, K_FOREVER);
		cfb_framebuffer_clear(oled_dev, true);
		display_blanking_on(oled_dev);
		k_mutex_unlock(&i2c0_mutex);
	}
}

/* ─── Encoder double-click watcher (runs in encoder ISR context via poll) ── */
/* The display thread is blocked on k_event_wait when sleeping.
 * We need a way to detect double-click while sleeping.
 * Use a k_work to poll encoder from system workqueue. */

static struct k_work_delayable enc_watch_work;

static void enc_watch_handler(struct k_work *work)
{
	int rot;
	uint8_t ev = encoder_poll(&rot);

	if (ev) {
		wifi_cmd_printf("$L:ENC ev=0x%02x rot=%d\n", ev, rot);
	}

	if (ev & ENC_EVT_DOUBLE) {
		wifi_cmd_printf("$L:ENC double-click! waking display\n");
		k_event_post(&display_event, EVT_WAKE);
	}

	k_work_reschedule(&enc_watch_work, K_MSEC(50));
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void display_init(void)
{
	/* Display functionality disabled — only verify hardware is reachable.
	 * Uncomment thread creation when ready to enable full menu. */
	LOG_INF("Display module disabled (hardware check only)");

#if 0  /* Enable when display is verified working */
	k_thread_create(&display_thread_data, display_stack,
			K_THREAD_STACK_SIZEOF(display_stack),
			display_thread, NULL, NULL, NULL,
			DISPLAY_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&display_thread_data, "display");

	/* Start encoder watcher for double-click wake detection */
	k_work_init_delayable(&enc_watch_work, enc_watch_handler);
	k_work_reschedule(&enc_watch_work, K_MSEC(50));
#endif
}

void display_notify_run_state(bool running)
{
	car_is_running = running;
}

void display_notify_test_state(bool active)
{
	test_is_active = active;
}
