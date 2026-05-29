/*
 * display.c — SSD1306 OLED 128×64 menu system (u8g2 backend)
 *
 * Thread: display_thread (priority 8, stack 4096, ~8 FPS)
 * Display is OFF by default; wakes on encoder double-click,
 * sleeps after 10s inactivity or when car enters RUN/test state.
 *
 * Navigation state and all data tables live here. Screen rendering
 * is delegated to modules in src/screens/.
 */

#include "display.h"
#include "display_hal.h"
#include "display_internal.h"
#include "encoder.h"
#include "settings.h"
#include "sensors.h"
#include "imu.h"
#include "tachometer.h"
#include "battery.h"
#include "control.h"
#include "wifi_cmd.h"

#include "screens/screen_dashboard.h"
#include "screens/screen_menu.h"
#include "screens/screen_settings.h"
#include "screens/screen_edit.h"
#include "screens/screen_tests.h"
#include "screens/screen_actions.h"
#include "screens/screen_confirm.h"
#include "screens/screen_info.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(display, LOG_LEVEL_INF);

/* ─── Thread config ──────────────────────────────────────────────────────── */
#define DISPLAY_STACK_SIZE   4096
#define DISPLAY_PRIORITY     8
#define DISPLAY_REFRESH_MS   120  /* dashboard: ~8 FPS, sensor bars don't need more */
#define DISPLAY_MENU_MS       50  /* menus: fast encoder response */

static K_THREAD_STACK_DEFINE(display_stack, DISPLAY_STACK_SIZE);
static struct k_thread display_thread_data;

/* ─── I2C0 bus mutex (shared with imu.c) ─────────────────────────────────── */
K_MUTEX_DEFINE(i2c0_mutex);

/* ─── Menu command queue (read by wifi_cmd.c) ────────────────────────────── */
K_MSGQ_DEFINE(menu_cmd_q, sizeof(uint8_t), 4, 4);

/* ─── Run/test state (volatile — written from other threads) ─────────────── */
static volatile bool car_is_running;
static volatile bool test_is_active;

/* ─── Navigation state ───────────────────────────────────────────────────── */
static struct ui_state st;

/* ─── Parameter descriptor table ────────────────────────────────────────── */
#define P_OFF(field) offsetof(struct car_settings, field)

const struct param_desc params[PARAM_COUNT] = {
	/* Obstacles (cm×10) */
	{"FOD", "Front Obs",  PT_INT,   10,  100,   50, 9000, P_OFF(front_obstacle_dist), 1},
	{"SOD", "Side Open",  PT_INT,   10,  100,   50, 9000, P_OFF(side_open_dist),      1},
	{"ACD", "All Close",  PT_INT,   10,  100,   50, 9000, P_OFF(all_close_dist),      1},
	{"CFD", "Close Frt",  PT_INT,   10,  100,   50, 9000, P_OFF(close_front_dist),    1},
	/* PID */
	{"KP",  "PID Kp",     PT_FLOAT, 0.1f, 1.0f, 0, 5000,  P_OFF(pid_kp),             1},
	{"KI",  "PID Ki",     PT_FLOAT, 0.5f, 5.0f, 0, 10000, P_OFF(pid_ki),             1},
	{"KD",  "PID Kd",     PT_FLOAT, 0.05f,0.5f, 0, 2000,  P_OFF(pid_kd),             1},
	/* Speed / ESC (µs) */
	{"MSP", "Min Spd",    PT_INT,   5,   20,  1000, 2000, P_OFF(min_speed),           1},
	{"XSP", "Max Spd",    PT_INT,   5,   20,  1000, 2000, P_OFF(max_speed),           1},
	{"BSP", "Min Bck",    PT_INT,   5,   20,  1000, 2000, P_OFF(min_bspeed),          1},
	{"KOP", "Kick %",     PT_FLOAT, 0.5f, 2.0f, 0, 80.0f, P_OFF(kick_pct),            1},
	{"KOM", "Kick ms",    PT_INT,   10,  50,    0, 5000,  P_OFF(kick_ms),             1},
	{"CKU", "Crn Kick",   PT_INT,   1,   5,     0, 120,   P_OFF(corner_kick_us),      1},
	/* Steering (degrees) */
	{"MNP", "Srv Min",    PT_INT,   1,    5,    0, 180,   P_OFF(min_point),           1},
	{"XNP", "Srv Max",    PT_INT,   1,    5,    0, 180,   P_OFF(max_point),           1},
	{"NTP", "Srv Neu",    PT_INT,   1,    5,    0, 180,   P_OFF(neutral_point),       1},
	/* Tachometer */
	{"ENH", "Enc Holes",  PT_INT,   1,   10,    1, 2000,  P_OFF(encoder_holes),       1},
	{"WDM", "Wheel mm",   PT_FLOAT, 1,   10,   10, 1000,  P_OFF(wheel_diam_m),     1000},
	{"TGF", "Tach Flt",   PT_INT,   1,   10,    1, 500,   P_OFF(tach_glitch_filter_us), 1},
	/* Control */
	{"LMS", "Loop ms",    PT_INT,   5,   10,   10, 1000,  P_OFF(loop_ms),             1},
	{"SPD1","Spd Clr",    PT_FLOAT, 0.1f, 0.5f, 0, 5.0f, P_OFF(spd_clear),           1},
	{"SPD2","Spd Blk",    PT_FLOAT, 0.1f, 0.5f, 0, 5.0f, P_OFF(spd_blocked),         1},
	{"SLW", "Spd Slew",   PT_FLOAT, 0.05f,0.2f, 0, 20.0f,P_OFF(spd_slew),            1},
	{"COE1","Coe Clr",    PT_FLOAT, 0.05f,0.1f, 0, 5.0f, P_OFF(coe_clear),           1},
	{"COE2","Coe Blk",    PT_FLOAT, 0.05f,0.1f, 0, 5.0f, P_OFF(coe_blocked),         1},
	/* Navigation */
	{"WDD", "Wrong Dir",  PT_FLOAT, 5.0f, 20.0f, 1, 360,  P_OFF(wrong_dir_deg),      1},
	{"RCW", "Race CW",    PT_BOOL,  1,    1,     0, 1,    P_OFF(race_cw),             1},
	{"STK", "Stuck Thr",  PT_INT,   1,    5,     0, 1000, P_OFF(stuck_thresh),        1},
	{"STL", "Stall Thr",  PT_INT,   1,    5,     0, 1000, P_OFF(stall_thresh),        1},
	/* Maneuvers */
	{"RBC", "Rev Brk",    PT_INT,   5,   20, -1000, 0,    P_OFF(reverse_brake_cmd),   1},
	{"RDC", "Rev Drv",    PT_INT,   5,   20, -1000, 0,    P_OFF(reverse_drive_cmd),   1},
	{"RBM", "Brk ms",     PT_INT,   10, 100,     0, 5000, P_OFF(reverse_brake_ms),    1},
	{"RDM", "Drv ms",     PT_INT,   10, 100,     0, 5000, P_OFF(reverse_drive_ms),    1},
	{"LBM", "LBrk ms",    PT_INT,   10, 100,     0, 5000, P_OFF(long_reverse_brake_ms), 1},
	{"LDM", "LDrv ms",    PT_INT,   10, 100,     0, 5000, P_OFF(long_reverse_drive_ms), 1},
	{"LFS", "Long Fwd",   PT_FLOAT, 0.01f,0.05f, 0, 2.0f, P_OFF(long_forward_speed_cap), 1},
	{"LFM", "LFwd ms",    PT_INT,   10, 100,     0, 5000, P_OFF(long_forward_ms),     1},
	/* Hardware */
	{"IMR", "IMU Rot",    PT_BOOL,  1,    1,     0, 1,    P_OFF(imu_rotate),          1},
	{"SVR", "Srv Rev",    PT_BOOL,  1,    1,     0, 1,    P_OFF(servo_reverse),       1},
	{"CAL", "Calibr.",    PT_BOOL,  1,    1,     0, 1,    P_OFF(calibrated),          1},
	{"BEN", "Bat En",     PT_BOOL,  1,    1,     0, 1,    P_OFF(bat_enabled),         1},
	{"BML", "Bat Mult",   PT_FLOAT, 0.1f, 0.5f,0.1f,20.0f, P_OFF(bat_multiplier),    1},
	{"BLV", "Bat Low V",  PT_FLOAT, 0.1f, 0.5f,0.1f,20.0f, P_OFF(bat_low),           1},
};

const struct param_group groups[GROUP_COUNT] = {
	{"Obstacles",   0,  4},
	{"PID",         4,  3},
	{"Speed/ESC",   7,  6},
	{"Steering",   13,  3},
	{"Tachometer", 16,  3},
	{"Control",    19,  6},
	{"Navigation", 25,  4},
	{"Maneuver",   29,  8},
	{"Hardware",   37,  6},
};

const struct test_item tests[TEST_COUNT] = {
	{"lidar",    "Lidar Scan",   false},
	{"servo",    "Servo Sweep",  false},
	{"taho",     "Tachometer",   false},
	{"esc",      "ESC Test",     true},
	{"speed",    "Speed Hold",   true},
	{"autotune", "PID Autotune", true},
	{"reactive", "Reactive",     false},
	{"cal",      "ESC Calibr.",  true},
};

const struct action_item actions[ACTION_COUNT] = {
	{"Start Car",      ACT_START, true},
	{"Stop Car",       ACT_STOP,  false},
	{"Save NVS",       ACT_SAVE,  true},
	{"Load NVS",       ACT_LOAD,  true},
	{"Reset Defaults", ACT_RESET, true},
};

const char *main_items[MAIN_REAL] = {"Settings", "Tests", "Actions", "Info"};

/* ─── Parameter helpers ──────────────────────────────────────────────────── */

float param_get(const struct param_desc *p)
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

int param_fmt(char *buf, int sz, const struct param_desc *p, float v)
{
	switch (p->type) {
	case PT_INT:
		return snprintf(buf, sz, "%d", (int)v);
	case PT_FLOAT:
		if (p->scale != 1.0f) return snprintf(buf, sz, "%d", (int)(v + 0.5f));
		return snprintf(buf, sz, "%.2f", (double)v);
	case PT_BOOL:
		return snprintf(buf, sz, "%s", v > 0.5f ? "ON" : "OFF");
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

/* ─── Navigation helpers ─────────────────────────────────────────────────── */

static const char *screen_name(enum screen scr)
{
	switch (scr) {
	case SCR_DASHBOARD:       return "dashboard";
	case SCR_MAIN_MENU:       return "menu";
	case SCR_SETTINGS_GROUPS: return "settings_groups";
	case SCR_SETTINGS_LIST:   return "settings_list";
	case SCR_SETTINGS_EDIT:   return "settings_edit";
	case SCR_TESTS:           return "tests";
	case SCR_TEST_RUNNING:    return "test_running";
	case SCR_ACTIONS:         return "actions";
	case SCR_CONFIRM:         return "confirm";
	case SCR_INFO:            return "info";
	default:                  return "unknown";
	}
}

static void go_screen(enum screen scr)
{
	enum screen prev = st.cur_scr;
	st.prev_scr = st.cur_scr;
	st.cur_scr  = scr;
	st.sel = (scr == SCR_MAIN_MENU || scr == SCR_SETTINGS_GROUPS ||
		  scr == SCR_SETTINGS_LIST || scr == SCR_TESTS ||
		  scr == SCR_ACTIONS) ? 1 : 0;
	st.scroll = 0;
	wifi_log("DSP:screen %s->%s", screen_name(prev), screen_name(scr));
}

static void go_back(void)
{
	enum screen prev = st.cur_scr;
	switch (st.cur_scr) {
	case SCR_MAIN_MENU:       st.cur_scr = SCR_DASHBOARD;       break;
	case SCR_SETTINGS_GROUPS: st.cur_scr = SCR_MAIN_MENU;       break;
	case SCR_SETTINGS_LIST:   st.cur_scr = SCR_SETTINGS_GROUPS; break;
	case SCR_SETTINGS_EDIT:   st.cur_scr = SCR_SETTINGS_LIST;   break;
	case SCR_TESTS:           st.cur_scr = SCR_MAIN_MENU;       break;
	case SCR_ACTIONS:         st.cur_scr = SCR_MAIN_MENU;       break;
	case SCR_CONFIRM:         st.cur_scr = st.prev_scr;         break;
	case SCR_INFO:            st.cur_scr = SCR_MAIN_MENU;       break;
	case SCR_TEST_RUNNING:                                       break;
	default:                  st.cur_scr = SCR_DASHBOARD;       break;
	}
	st.sel = 1;
	st.scroll = 0;
	wifi_log("DSP:back %s->%s", screen_name(prev), screen_name(st.cur_scr));
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
	st.test_idx   = idx;
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

	if (events || rot != 0) {
		wifi_log("ENC:rot=%d,ev=%02x,screen=%s,sel=%d",
			 rot, events, screen_name(st.cur_scr), st.sel);
	}

	if (held) {
		enum screen prev = st.cur_scr;
		if (st.cur_scr == SCR_SETTINGS_EDIT) {
			st.cur_scr = SCR_SETTINGS_LIST;
		} else {
			st.cur_scr = SCR_DASHBOARD;
		}
		st.sel = 0;
		st.scroll = 0;
		wifi_log("DSP:hold %s->%s", screen_name(prev), screen_name(st.cur_scr));
		return;
	}

	switch (st.cur_scr) {
	case SCR_DASHBOARD:
#if !IS_ENABLED(CONFIG_DISPLAY_DASHBOARD_ONLY)
		if (click) go_screen(SCR_MAIN_MENU);
#endif
		break;

	case SCR_MAIN_MENU:
		st.sel += dir;
		clamp_scroll(&st.sel, &st.scroll, MAIN_REAL + 1, LIST_VISIBLE);
		if (click) {
			if (st.sel == 0) { go_back(); }
			else switch (st.sel) {
			case 1: go_screen(SCR_SETTINGS_GROUPS); break;
			case 2: go_screen(SCR_TESTS);           break;
			case 3: go_screen(SCR_ACTIONS);          break;
			case 4: go_screen(SCR_INFO);             break;
			}
		}
		break;

	case SCR_SETTINGS_GROUPS:
		st.sel += dir;
		clamp_scroll(&st.sel, &st.scroll, GROUP_COUNT + 1, LIST_VISIBLE);
		if (click) {
			if (st.sel == 0) { go_back(); }
			else { st.grp_sel = st.sel - 1; go_screen(SCR_SETTINGS_LIST); }
		}
		break;

	case SCR_SETTINGS_LIST:
		st.sel += dir;
		clamp_scroll(&st.sel, &st.scroll, groups[st.grp_sel].count + 1, LIST_VISIBLE);
		if (click) {
			if (st.sel == 0) { go_back(); }
			else {
				st.param_idx = groups[st.grp_sel].start + (st.sel - 1);
				st.edit_val = param_get(&params[st.param_idx]);
				go_screen(SCR_SETTINGS_EDIT);
			}
		}
		break;

	case SCR_SETTINGS_EDIT: {
		const struct param_desc *p = &params[st.param_idx];
		if (dir != 0) {
			if (p->type == PT_BOOL) {
				st.edit_val = (st.edit_val > 0.5f) ? 0.0f : 1.0f;
			} else {
				float s = fast ? p->step_fast : p->step;
				st.edit_val += dir * s;
				if (st.edit_val < p->lo) st.edit_val = p->lo;
				if (st.edit_val > p->hi) st.edit_val = p->hi;
			}
		}
		if (click) {
			param_set(p, st.edit_val);
			st.cur_scr = SCR_SETTINGS_LIST;
			st.sel = (st.param_idx - groups[st.grp_sel].start) + 1;
			st.scroll = 0;
			clamp_scroll(&st.sel, &st.scroll,
				     groups[st.grp_sel].count + 1, LIST_VISIBLE);
		}
		break;
	}

	case SCR_TESTS:
		st.sel += dir;
		clamp_scroll(&st.sel, &st.scroll, TEST_COUNT + 1, LIST_VISIBLE);
		if (click) {
			if (st.sel == 0) { go_back(); }
			else {
				int ti = st.sel - 1;
				if (tests[ti].motor) {
					st.test_idx   = ti;
					st.confirm_msg = tests[ti].label;
					st.confirm_yes = false;
					st.prev_scr    = SCR_TESTS;
					go_screen(SCR_CONFIRM);
				} else {
					run_test(ti);
				}
			}
		}
		break;

	case SCR_TEST_RUNNING:
		if (!test_is_active) {
			st.cur_scr = SCR_TESTS;
			st.sel = st.test_idx + 1;
			st.scroll = 0;
		}
		if (click) {
			uint8_t cmd = MCMD_STOP;
			k_msgq_put(&menu_cmd_q, &cmd, K_NO_WAIT);
			st.cur_scr = SCR_TESTS;
		}
		break;

	case SCR_ACTIONS:
		st.sel += dir;
		clamp_scroll(&st.sel, &st.scroll, ACTION_COUNT + 1, LIST_VISIBLE);
		if (click) {
			if (st.sel == 0) { go_back(); }
			else {
				int ai = st.sel - 1;
				st.action_idx = ai;
				if (actions[ai].confirm) {
					st.confirm_msg = actions[ai].label;
					st.confirm_yes = false;
					st.prev_scr    = SCR_ACTIONS;
					go_screen(SCR_CONFIRM);
				} else {
					exec_action(actions[ai].id);
					go_screen(SCR_DASHBOARD);
				}
			}
		}
		break;

	case SCR_CONFIRM:
		if (dir != 0) st.confirm_yes = !st.confirm_yes;
		if (click) {
			if (st.confirm_yes) {
				if (st.prev_scr == SCR_TESTS) {
					run_test(st.test_idx);
				} else if (st.prev_scr == SCR_ACTIONS) {
					exec_action(actions[st.action_idx].id);
					go_screen(SCR_DASHBOARD);
				}
			} else {
				go_back();
			}
		}
		break;

	case SCR_INFO:
		if (dir != 0) st.info_scroll += dir;
		if (click) { st.info_scroll = 0; go_back(); }
		break;
	}
}

/* ─── Render dispatcher ──────────────────────────────────────────────────── */

static void draw_current_screen(void)
{
	/* Each screen module clears nothing — u8g2_ClearBuffer() was called before.
	 * Each module calls u8g2_SendBuffer() at the end. */
	switch (st.cur_scr) {
	case SCR_DASHBOARD:        screen_dashboard_draw();             break;
	case SCR_MAIN_MENU:        screen_menu_draw(&st);               break;
	case SCR_SETTINGS_GROUPS:  screen_settings_groups_draw(&st);    break;
	case SCR_SETTINGS_LIST:    screen_settings_list_draw(&st);      break;
	case SCR_SETTINGS_EDIT:    screen_edit_draw(&st);               break;
	case SCR_TESTS:            screen_tests_draw(&st);              break;
	case SCR_TEST_RUNNING:     screen_tests_running_draw(&st);      break;
	case SCR_ACTIONS:          screen_actions_draw(&st);            break;
	case SCR_CONFIRM:          screen_confirm_draw(&st);            break;
	case SCR_INFO:             screen_info_draw(&st);               break;
	}
}

/* ─── Display thread ─────────────────────────────────────────────────────── */

static void display_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_msleep(500); /* let drivers settle */

	bool display_ok = (display_hal_init() == 0);
	if (!display_ok) {
		LOG_ERR("display HAL init failed — encoder polling only");
	}

	if (display_ok) {
		u8g2_SetPowerSave(&u8g2, 0); /* display always on at boot */
	}
	st.cur_scr = SCR_DASHBOARD;
	bool disp_on = display_ok;

	LOG_INF("Display thread ready display=%d", display_ok ? 1 : 0);
	wifi_log("DSP:ready,display=%d", display_ok ? 1 : 0);

	while (1) {
		bool running = car_is_running || test_is_active;
		display_ok = display_hal_is_present();

		if (running && disp_on && display_ok) {
			/* Car started — blank and sleep display */
			u8g2_ClearBuffer(&u8g2);
			u8g2_SendBuffer(&u8g2);
			u8g2_SetPowerSave(&u8g2, 1);
			disp_on = false;
			wifi_log("DSP:sleep running=%d,test=%d",
				 car_is_running ? 1 : 0, test_is_active ? 1 : 0);
		} else if (!running && !disp_on && display_ok) {
			/* Car stopped — wake display, back to dashboard */
			u8g2_SetPowerSave(&u8g2, 0);
			st.cur_scr = SCR_DASHBOARD;
			st.sel     = 0;
			st.scroll  = 0;
			disp_on = true;
			wifi_log("DSP:wake");
		}

		if (!running) {
			handle_input();
			if (display_ok) {
				u8g2_ClearBuffer(&u8g2);
				draw_current_screen(); /* calls u8g2_SendBuffer() */
			}

			int ms = (st.cur_scr == SCR_DASHBOARD)
				? DISPLAY_REFRESH_MS : DISPLAY_MENU_MS;
			k_msleep(ms);
		} else {
			int ignored_rot;
			(void)encoder_poll(&ignored_rot);
			k_msleep(100); /* low-rate poll while car runs */
		}
	}
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

void display_init(void)
{
	k_thread_create(&display_thread_data, display_stack,
			K_THREAD_STACK_SIZEOF(display_stack),
			display_thread_fn, NULL, NULL, NULL,
			DISPLAY_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&display_thread_data, "display");
}

void display_notify_run_state(bool running)
{
	car_is_running = running;
}

void display_notify_test_state(bool active)
{
	test_is_active = active;
}
