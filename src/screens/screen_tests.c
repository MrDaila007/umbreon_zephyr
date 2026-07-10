/*
 * screen_tests.c — Tests list and test running screens
 */

#include "screen_tests.h"
#include "../display_hal.h"
#include "../display_internal.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>

void screen_tests_draw(const struct ui_state *st)
{
	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, 30, 11, "Tests");
	u8g2_SetDrawColor(&u8g2, 1);

	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

	int count = TEST_COUNT + 1; /* +1 for Back */
	for (int i = 0; i < LIST_VISIBLE && (st->scroll + i) < count; i++) {
		int idx = st->scroll + i;
		int y = 15 + i * 9;
		char buf[24];

		if (idx == 0) {
			snprintf(buf, sizeof(buf), "< Back");
		} else {
			const struct test_item *t = &tests[idx - 1];
			snprintf(buf, sizeof(buf), "%s%s",
				 t->label, t->motor ? " [!]" : "");
		}

		if (idx == st->sel) {
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
		int bar_y = (max_sc > 0) ? (st->scroll * (SCR_H - bar_h) / max_sc) : 0;
		u8g2_DrawBox(&u8g2, SCR_W - 3, bar_y, 3, bar_h);
	}

	u8g2_SendBuffer(&u8g2);
}

void screen_tests_running_draw(const struct ui_state *st)
{
	const struct test_item *t = &tests[st->test_idx];

	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, 16, 11, "Running...");
	u8g2_SetDrawColor(&u8g2, 1);

	/* Test name */
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	char tbuf[24];
	snprintf(tbuf, sizeof(tbuf), "Test: %s", t->label);
	u8g2_DrawStr(&u8g2, 3, 30, tbuf);

	/* Animated dots */
	static uint8_t dot_frame;
	dot_frame = (dot_frame + 1) % 4;
	char dots[5] = "    ";
	for (int i = 0; i < dot_frame; i++) dots[i] = '.';
	dots[dot_frame > 0 ? dot_frame : 0] = '\0';
	u8g2_DrawStr(&u8g2, 3, 44, dots);

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	u8g2_DrawStr(&u8g2, 3, 62, "Click = Abort");

	u8g2_SendBuffer(&u8g2);
}
