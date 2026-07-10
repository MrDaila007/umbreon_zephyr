/*
 * screen_actions.c — Actions list screen (Start, Stop, Save, Load, Reset)
 */

#include "screen_actions.h"
#include "../display_hal.h"
#include "../display_internal.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>

void screen_actions_draw(const struct ui_state *st)
{
	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, 26, 11, "Actions");
	u8g2_SetDrawColor(&u8g2, 1);

	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

	int count = ACTION_COUNT + 1; /* +1 for Back */
	for (int i = 0; i < LIST_VISIBLE && (st->scroll + i) < count; i++) {
		int idx = st->scroll + i;
		int y = 15 + i * 9;
		char buf[20];

		if (idx == 0) {
			snprintf(buf, sizeof(buf), "< Back");
		} else {
			const struct action_item *a = &actions[idx - 1];
			snprintf(buf, sizeof(buf), "%s%s",
				 a->label, a->confirm ? " ?" : "");
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

	u8g2_SendBuffer(&u8g2);
}
