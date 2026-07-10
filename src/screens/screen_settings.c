/*
 * screen_settings.c — Settings groups list and parameter list screens
 */

#include "screen_settings.h"
#include "../display_hal.h"
#include "../display_internal.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>

/* ─── Shared list renderer ───────────────────────────────────────────────── */

typedef void (*item_fn_t)(int idx, char *buf, int sz, const struct ui_state *st);

static void draw_list(const char *title, int count, item_fn_t fn,
		      int sel_idx, int scroll_off, const struct ui_state *st)
{
	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	int tw = u8g2_GetStrWidth(&u8g2, title);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, (SCR_W - tw) / 2, 11, title);
	u8g2_SetDrawColor(&u8g2, 1);

	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

	/* Visible items */
	for (int i = 0; i < LIST_VISIBLE && (scroll_off + i) < count; i++) {
		int idx = scroll_off + i;
		int y = 15 + i * 9;
		char buf[24];
		fn(idx, buf, sizeof(buf), st);

		if (idx == sel_idx) {
			u8g2_DrawBox(&u8g2, 0, y - 1, SCR_W - 4, 10);
			u8g2_SetDrawColor(&u8g2, 0);
			u8g2_DrawStr(&u8g2, 3, y + 7, buf);
			u8g2_SetDrawColor(&u8g2, 1);
		} else {
			u8g2_DrawStr(&u8g2, 3, y + 7, buf);
		}
	}

	/* Scroll bar */
	if (count > LIST_VISIBLE) {
		int bar_h = (SCR_H * LIST_VISIBLE) / count;
		if (bar_h < 4) bar_h = 4;
		int max_sc = count - LIST_VISIBLE;
		int bar_y = (max_sc > 0) ? (scroll_off * (SCR_H - bar_h) / max_sc) : 0;
		u8g2_DrawBox(&u8g2, SCR_W - 3, bar_y, 3, bar_h);
	}
}

/* ─── Item formatters ────────────────────────────────────────────────────── */

static void groups_item_fn(int idx, char *buf, int sz, const struct ui_state *st)
{
	ARG_UNUSED(st);
	if (idx == 0) {
		snprintf(buf, sz, "< Back");
		return;
	}
	snprintf(buf, sz, "%s (%d)", groups[idx - 1].name, groups[idx - 1].count);
}

static void params_item_fn(int idx, char *buf, int sz, const struct ui_state *st)
{
	if (idx == 0) {
		snprintf(buf, sz, "< Back");
		return;
	}
	int pi = groups[st->grp_sel].start + (idx - 1);
	const struct param_desc *p = &params[pi];
	char vbuf[10];
	param_fmt(vbuf, sizeof(vbuf), p, param_get(p));
	snprintf(buf, sz, "%-8s %s", p->label, vbuf);
}

/* ─── Public entry points ────────────────────────────────────────────────── */

void screen_settings_groups_draw(const struct ui_state *st)
{
	draw_list("Settings", GROUP_COUNT + 1, groups_item_fn,
		  st->sel, st->scroll, st);
	u8g2_SendBuffer(&u8g2);
}

void screen_settings_list_draw(const struct ui_state *st)
{
	const struct param_group *g = &groups[st->grp_sel];
	draw_list(g->name, g->count + 1, params_item_fn,
		  st->sel, st->scroll, st);
	u8g2_SendBuffer(&u8g2);
}
