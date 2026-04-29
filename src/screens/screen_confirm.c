/*
 * screen_confirm.c — Yes/No confirmation dialog
 */

#include "screen_confirm.h"
#include "../display_hal.h"

#include <u8g2.h>
#include <string.h>

void screen_confirm_draw(const struct ui_state *st)
{
	/* Message */
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	int mw = u8g2_GetStrWidth(&u8g2, st->confirm_msg);
	u8g2_DrawStr(&u8g2, (SCR_W - mw) / 2, 16, st->confirm_msg);

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	u8g2_DrawStr(&u8g2, 34, 28, "Are you sure?");

	/* Yes button */
	u8g2_DrawFrame(&u8g2, 10, 38, 46, 14);
	if (st->confirm_yes) {
		u8g2_DrawBox(&u8g2, 10, 38, 46, 14);
		u8g2_SetDrawColor(&u8g2, 0);
	}
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 18, 49, "[ Yes ]");
	u8g2_SetDrawColor(&u8g2, 1);

	/* No button */
	u8g2_DrawFrame(&u8g2, 72, 38, 46, 14);
	if (!st->confirm_yes) {
		u8g2_DrawBox(&u8g2, 72, 38, 46, 14);
		u8g2_SetDrawColor(&u8g2, 0);
	}
	u8g2_DrawStr(&u8g2, 80, 49, "[ No  ]");
	u8g2_SetDrawColor(&u8g2, 1);

	u8g2_SendBuffer(&u8g2);
}
