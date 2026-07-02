/*
 * screen_boot.c — OLED boot splash (logo + version + status)
 */

#include "screen_boot.h"
#include "../display_hal.h"
#include "../display_internal.h"
#include "../assets/umbreon_logo.h"
#include "../version.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>

void screen_boot_draw(const char *status)
{
	const char *line = (status != NULL && status[0] != '\0') ? status : "Starting...";

	int lx = (SCR_W - (int)UMBREON_LOGO_W) / 2;
	int ly = 4;
	u8g2_DrawXBMP(&u8g2, lx, ly, UMBREON_LOGO_W, UMBREON_LOGO_H, umbreon_logo_bits);

	char ver[32];
	snprintf(ver, sizeof(ver), "v%s", FW_VERSION_FULL);
	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	int vw = u8g2_GetStrWidth(&u8g2, ver);
	int ver_y = ly + UMBREON_LOGO_H + 6;
	u8g2_DrawStr(&u8g2, (SCR_W - vw) / 2, ver_y, ver);

	int sw = u8g2_GetStrWidth(&u8g2, line);
	if (sw > SCR_W - 4) {
		sw = SCR_W - 4;
	}
	u8g2_DrawStr(&u8g2, (SCR_W - sw) / 2, SCR_H - 4, line);

	u8g2_SendBuffer(&u8g2);
}
