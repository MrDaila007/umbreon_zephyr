/*
 * screen_calibrating.c — IMU calibration progress screen
 *
 * Phases (cal_phase field in ui_state):
 *   CAL_COUNTDOWN — 5s countdown, "Keep still…"
 *   CAL_RUNNING   — blocking cal call in display.c, show "Calibrating…"
 *   CAL_DONE      — show result for 2s, then go_back()
 */

#include "screen_calibrating.h"
#include "../display_hal.h"
#include "../display_internal.h"

#include <u8g2.h>
#include <zephyr/kernel.h>
#include <stdio.h>

void screen_calibrating_draw(const struct ui_state *st)
{
	const char *title = (st->cal_type == 0) ? "Gyro Cal" : "Accel Cal";

	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	int tw = (int)u8g2_GetStrWidth(&u8g2, title);
	u8g2_DrawStr(&u8g2, (SCR_W - tw) / 2, 11, title);
	u8g2_SetDrawColor(&u8g2, 1);

	switch (st->cal_phase) {
	case CAL_COUNTDOWN: {
		int64_t elapsed   = k_uptime_get() - st->cal_phase_start_ms;
		int remaining     = (int)((5000 - elapsed) / 1000) + 1;
		if (remaining < 1) remaining = 1;
		if (remaining > 5) remaining = 5;

		u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
		u8g2_DrawStr(&u8g2, 20, 30, "Keep still...");

		char nbuf[4];
		snprintf(nbuf, sizeof(nbuf), "%d", remaining);
		u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
		int nw = (int)u8g2_GetStrWidth(&u8g2, nbuf);
		u8g2_DrawStr(&u8g2, (SCR_W - nw) / 2, 52, nbuf);

		u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
		u8g2_DrawStr(&u8g2, 22, 62, "Click = Cancel");
		break;
	}
	case CAL_RUNNING:
		u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
		u8g2_DrawStr(&u8g2, 12, 38, "Calibrating...");
		break;

	case CAL_DONE: {
		u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
		u8g2_DrawStr(&u8g2, 40, 32, "Done!");

		u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
		int rw = (int)u8g2_GetStrWidth(&u8g2, st->cal_result);
		u8g2_DrawStr(&u8g2, (SCR_W - rw) / 2, 48, st->cal_result);
		break;
	}
	}

	u8g2_SendBuffer(&u8g2);
}
