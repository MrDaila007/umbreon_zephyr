/*
 * screen_edit.c — Parameter edit screen
 *
 * Shows the parameter label (title bar), key, large current value,
 * range/step info, and help text.
 */

#include "screen_edit.h"
#include "../display_hal.h"
#include "../display_internal.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>

void screen_edit_draw(const struct ui_state *st)
{
	const struct param_desc *p = &params[st->param_idx];

	/* Title bar (inverted) */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	int tw = u8g2_GetStrWidth(&u8g2, p->label);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, (SCR_W - tw) / 2, 11, p->label);
	u8g2_SetDrawColor(&u8g2, 1);

	/* Key */
	char kbuf[16];
	snprintf(kbuf, sizeof(kbuf), "Key: %s", p->key);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 3, 25, kbuf);

	/* Current value (large, centered) */
	char vbuf[14];
	param_fmt(vbuf, sizeof(vbuf), p, st->edit_val);
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	int vw = u8g2_GetStrWidth(&u8g2, vbuf);
	u8g2_DrawStr(&u8g2, (SCR_W - vw) / 2, 42, vbuf);

	/* Range + step */
	char rbuf[24];
	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	if (p->type == PT_BOOL) {
		snprintf(rbuf, sizeof(rbuf), "Turn to toggle");
	} else if (p->type == PT_INT) {
		snprintf(rbuf, sizeof(rbuf), "%d..%d step %d",
			 (int)p->lo, (int)p->hi, (int)p->step);
	} else {
		snprintf(rbuf, sizeof(rbuf), "%.1f..%.1f s%.2f",
			 (double)p->lo, (double)p->hi, (double)p->step);
	}
	int rw = u8g2_GetStrWidth(&u8g2, rbuf);
	u8g2_DrawStr(&u8g2, (SCR_W - rw) / 2, 53, rbuf);

	/* Help */
	const char *help = "Click=OK  Hold=Cancel";
	int hw = u8g2_GetStrWidth(&u8g2, help);
	u8g2_DrawStr(&u8g2, (SCR_W - hw) / 2, 62, help);

	u8g2_SendBuffer(&u8g2);
}
