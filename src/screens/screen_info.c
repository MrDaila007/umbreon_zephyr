/*
 * screen_info.c — Scrollable system information screen
 */

#include "screen_info.h"
#include "../display_hal.h"
#include "../battery.h"
#include "../sensors.h"
#include "../imu.h"
#include "../settings.h"

#include <u8g2.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>

#define INFO_MAX_LINES 12
#define INFO_VISIBLE    5

void screen_info_draw(const struct ui_state *st)
{
	char lines[INFO_MAX_LINES][22];
	int count = 0;

	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, 42, 11, "Info");
	u8g2_SetDrawColor(&u8g2, 1);

	/* Build info lines */
	snprintf(lines[count++], 22, "FW: v2.0.0 (u8g2)");
	snprintf(lines[count++], 22, "Core: M33 Zephyr v4.4");
	snprintf(lines[count++], 22, "Sensors: %d/6 online", sensors_online_count());
	snprintf(lines[count++], 22, "IMU: %s", imu_is_ok() ? "OK" : "FAIL");
	if (cfg.bat_enabled) {
		snprintf(lines[count++], 22, "Bat: %.2fV (%d%%)",
			 (double)battery_get_voltage(),
			 (int)((battery_get_voltage() - 6.6f) / 1.8f * 100));
	} else {
		snprintf(lines[count++], 22, "Bat: disabled");
	}
	snprintf(lines[count++], 22, "Loop: %dms", cfg.loop_ms);
	snprintf(lines[count++], 22, "Enc holes: %d", cfg.encoder_holes);

	int64_t secs = k_uptime_get() / 1000;
	snprintf(lines[count++], 22, "Uptime: %dm %ds",
		 (int)(secs / 60), (int)(secs % 60));

	/* Clamp scroll */
	int scroll = st->info_scroll;
	int max_sc = count - INFO_VISIBLE;
	if (scroll > max_sc) scroll = max_sc;
	if (scroll < 0)      scroll = 0;

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	for (int i = 0; i < INFO_VISIBLE && (scroll + i) < count; i++) {
		u8g2_DrawStr(&u8g2, 3, 16 + i * 9, lines[scroll + i]);
	}

	/* Scroll bar */
	if (count > INFO_VISIBLE) {
		int bar_h = (SCR_H * INFO_VISIBLE) / count;
		if (bar_h < 4) bar_h = 4;
		int bar_y = (max_sc > 0) ? (scroll * (SCR_H - bar_h) / max_sc) : 0;
		u8g2_DrawBox(&u8g2, SCR_W - 3, bar_y, 3, bar_h);
	}

	u8g2_SendBuffer(&u8g2);
}
