/*
 * screen_wifi.c — WiFi status screen (mode, SSID, IP, RSSI)
 */

#include "screen_wifi.h"
#include "../display_hal.h"
#include "../wifi_cmd.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>

#define WIFI_MAX_LINES  6
#define WIFI_VISIBLE    5

void screen_wifi_draw(const struct ui_state *st)
{
	char lines[WIFI_MAX_LINES][22];
	int count = 0;

	/* Title bar */
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, 46, 11, "WiFi");
	u8g2_SetDrawColor(&u8g2, 1);

	snprintf(lines[count++], 22, "Mode: %s", wifi_status_is_ap() ? "AP" : "STA");

	const char *ssid = wifi_status_get_ssid();
	if (ssid[0]) {
		char s[15];
		strncpy(s, ssid, 14);
		s[14] = '\0';
		if (strlen(ssid) > 14) {
			s[12] = '.'; s[13] = '.';
		}
		snprintf(lines[count++], 22, "SSID: %s", s);
	} else {
		snprintf(lines[count++], 22, "SSID: ...");
	}

	const char *ip = wifi_status_get_ip();
	snprintf(lines[count++], 22, "IP: %s", ip[0] ? ip : "...");

	if (!wifi_status_is_ap()) {
		snprintf(lines[count++], 22, "RSSI: %d dBm", wifi_status_get_rssi());
	}

	snprintf(lines[count++], 22, "St: %s",
		 wifi_status_is_ready() ? "ready" : "connecting");

	int scroll = st->wifi_scroll;
	int max_sc = count - WIFI_VISIBLE;
	if (scroll > max_sc) scroll = max_sc;
	if (scroll < 0)      scroll = 0;

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	for (int i = 0; i < WIFI_VISIBLE && (scroll + i) < count; i++) {
		u8g2_DrawStr(&u8g2, 3, 22 + i * 9, lines[scroll + i]);
	}

	if (count > WIFI_VISIBLE) {
		int bar_h = (SCR_H * WIFI_VISIBLE) / count;
		if (bar_h < 4) bar_h = 4;
		int bar_y = (max_sc > 0) ? (scroll * (SCR_H - bar_h) / max_sc) : 0;
		u8g2_DrawBox(&u8g2, SCR_W - 3, bar_y, 3, bar_h);
	}

	u8g2_SendBuffer(&u8g2);
}
