#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ─── Screen IDs ─────────────────────────────────────────────────────────── */
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

/* ─── Navigation state (owned by display.c, read-only for screen modules) ── */
struct ui_state {
	enum screen cur_scr;
	enum screen prev_scr;
	int sel;
	int scroll;
	int grp_sel;
	int param_idx;
	float edit_val;
	int test_idx;
	int action_idx;
	bool confirm_yes;
	const char *confirm_msg;
	int info_scroll;
};

/* ─── Display dimensions ─────────────────────────────────────────────────── */
#define SCR_W  128
#define SCR_H   64

/* ─── Parameter descriptor ───────────────────────────────────────────────── */
enum param_type { PT_INT, PT_FLOAT, PT_BOOL };

struct param_desc {
	const char *key;
	const char *label;
	enum param_type type;
	float step;
	float step_fast;
	float lo, hi;
	size_t offset;
	float scale;
};

/* ─── Parameter group ────────────────────────────────────────────────────── */
struct param_group {
	const char *name;
	uint8_t start;
	uint8_t count;
};

/* ─── Test item ──────────────────────────────────────────────────────────── */
struct test_item {
	const char *name;
	const char *label;
	bool motor;
};

/* ─── Action item ────────────────────────────────────────────────────────── */
enum action_id { ACT_START, ACT_STOP, ACT_SAVE, ACT_LOAD, ACT_RESET };

struct action_item {
	const char *label;
	enum action_id id;
	bool confirm;
};

/* ─── Counts (must match definitions in display.c) ──────────────────────── */
#define PARAM_COUNT   43
#define GROUP_COUNT    9
#define TEST_COUNT     8
#define ACTION_COUNT   5
#define MAIN_REAL      4
#define LIST_VISIBLE   5

/* ─── Data tables (defined in display.c) ────────────────────────────────── */
extern const struct param_desc  params[PARAM_COUNT];
extern const struct param_group groups[GROUP_COUNT];
extern const struct test_item   tests[TEST_COUNT];
extern const struct action_item actions[ACTION_COUNT];
extern const char *main_items[MAIN_REAL];

/* ─── Parameter helpers (defined in display.c) ───────────────────────────── */
float param_get(const struct param_desc *p);
int   param_fmt(char *buf, int sz, const struct param_desc *p, float v);
