/*
 * screen_wifi.c — WiFi screen: status + WiFi join QR + web UI QR
 *
 * Pages (encoder rotation):
 *   0 — mode, SSID, IP, RSSI/pass, status
 *   1 — QR for WiFi join (WIFI:… payload, AP mode with password)
 *   2 — QR for http://<ip>/ web UI
 */

#include "screen_wifi.h"
#include "../display_hal.h"
#include "../display_internal.h"
#include "../wifi_cmd.h"
#include "../qrcodegen.h"

#include <u8g2.h>
#include <stdio.h>
#include <string.h>

#define WIFI_PAGE_COUNT  3
#define QR_MAX_VERSION   10
#define QR_AREA_Y        16
#define QR_AREA_H        48
#define QR_AREA_W        SCR_W

static uint8_t qr_temp[qrcodegen_BUFFER_LEN_FOR_VERSION(QR_MAX_VERSION)];
static uint8_t qr_code[qrcodegen_BUFFER_LEN_FOR_VERSION(QR_MAX_VERSION)];

static void draw_title(int page)
{
	char title[16];

	snprintf(title, sizeof(title), "WiFi %d/%d", page + 1, WIFI_PAGE_COUNT);
	u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
	u8g2_DrawBox(&u8g2, 0, 0, SCR_W, 14);
	u8g2_SetDrawColor(&u8g2, 0);
	u8g2_DrawStr(&u8g2, 28, 11, title);
	u8g2_SetDrawColor(&u8g2, 1);
}

static void draw_page_dots(int page)
{
	for (int i = 0; i < WIFI_PAGE_COUNT; i++) {
		int x = SCR_W / 2 - (WIFI_PAGE_COUNT * 4) / 2 + i * 8;
		if (i == page) {
			u8g2_DrawDisc(&u8g2, x, SCR_H - 2, 2, U8G2_DRAW_ALL);
		} else {
			u8g2_DrawCircle(&u8g2, x, SCR_H - 2, 1, U8G2_DRAW_ALL);
		}
	}
}

static void wifi_qr_escape(const char *in, char *out, size_t out_sz)
{
	size_t j = 0;

	for (; *in && j + 2 < out_sz; in++) {
		if (*in == '\\' || *in == ';' || *in == ',' || *in == ':') {
			out[j++] = '\\';
		}
		out[j++] = *in;
	}
	out[j] = '\0';
}

static bool build_wifi_payload(char *buf, size_t sz)
{
	const char *ssid = wifi_status_get_ssid();

	if (!ssid[0]) {
		return false;
	}

	char essid[40];
	char pass[70];

	wifi_qr_escape(ssid, essid, sizeof(essid));

	if (wifi_status_is_ap()) {
		const char *ap_pass = wifi_status_get_ap_pass();

		if (!ap_pass[0]) {
			return false;
		}
		wifi_qr_escape(ap_pass, pass, sizeof(pass));
		snprintf(buf, sz, "WIFI:T:WPA;S:%s;P:%s;;", essid, pass);
		return true;
	}

	/* STA: SSID only — phone cannot auto-join without stored password */
	snprintf(buf, sz, "WIFI:T:nopass;S:%s;;", essid);
	return true;
}

static bool build_web_url(char *buf, size_t sz)
{
	const char *ip = wifi_status_get_ip();

	if (!ip[0]) {
		return false;
	}
	snprintf(buf, sz, "http://%s/", ip);
	return true;
}

static void draw_qr_payload(const char *payload)
{
	if (!qrcodegen_encodeText(payload, qr_temp, qr_code,
				  qrcodegen_Ecc_LOW,
				  qrcodegen_VERSION_MIN, QR_MAX_VERSION,
				  qrcodegen_Mask_AUTO, true)) {
		u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
		u8g2_DrawStr(&u8g2, 8, 36, "QR encode fail");
		return;
	}

	int modules = qrcodegen_getSize(qr_code);
	int max_px = QR_AREA_H < QR_AREA_W ? QR_AREA_H : QR_AREA_W;
	int scale = max_px / (modules + 2);

	if (scale < 1) {
		scale = 1;
	}

	int qr_px = modules * scale;
	int x0 = (SCR_W - qr_px) / 2;
	int y0 = QR_AREA_Y + (QR_AREA_H - qr_px) / 2;

	for (int y = 0; y < modules; y++) {
		for (int x = 0; x < modules; x++) {
			if (qrcodegen_getModule(qr_code, x, y)) {
				u8g2_DrawBox(&u8g2,
					     x0 + x * scale,
					     y0 + y * scale,
					     scale, scale);
			}
		}
	}
}

static void draw_status_page(void)
{
	char line[22];
	int y = 22;

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);

	snprintf(line, sizeof(line), "Mode: %s",
		 wifi_status_is_ap() ? "AP" : "STA");
	u8g2_DrawStr(&u8g2, 3, y, line);
	y += 9;

	const char *ssid = wifi_status_get_ssid();

	if (ssid[0]) {
		char s[15];

		strncpy(s, ssid, 14);
		s[14] = '\0';
		if (strlen(ssid) > 14) {
			s[12] = '.';
			s[13] = '.';
		}
		snprintf(line, sizeof(line), "SSID: %s", s);
	} else {
		snprintf(line, sizeof(line), "SSID: ...");
	}
	u8g2_DrawStr(&u8g2, 3, y, line);
	y += 9;

	const char *ip = wifi_status_get_ip();

	snprintf(line, sizeof(line), "IP: %s", ip[0] ? ip : "...");
	u8g2_DrawStr(&u8g2, 3, y, line);
	y += 9;

	if (wifi_status_is_ap()) {
		const char *pass = wifi_status_get_ap_pass();

		if (pass[0]) {
			char p[15];

			strncpy(p, pass, 14);
			p[14] = '\0';
			if (strlen(pass) > 14) {
				p[12] = '.';
				p[13] = '.';
			}
			snprintf(line, sizeof(line), "Pass: %s", p);
		} else {
			snprintf(line, sizeof(line), "Pass: ...");
		}
	} else {
		snprintf(line, sizeof(line), "RSSI: %d dBm", wifi_status_get_rssi());
	}
	u8g2_DrawStr(&u8g2, 3, y, line);
	y += 9;

	snprintf(line, sizeof(line), "St: %s",
		 wifi_status_is_ready() ? "ready" : "connecting");
	u8g2_DrawStr(&u8g2, 3, y, line);
}

static void draw_wifi_qr_page(void)
{
	char payload[160];

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	u8g2_DrawStr(&u8g2, 3, 22, "Scan to join WiFi");

	if (!build_wifi_payload(payload, sizeof(payload))) {
		u8g2_DrawStr(&u8g2, 8, 36, "Waiting for AP...");
		return;
	}

	draw_qr_payload(payload);
}

static void draw_web_qr_page(void)
{
	char url[32];

	u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	u8g2_DrawStr(&u8g2, 3, 22, "Scan for web UI");

	if (!build_web_url(url, sizeof(url))) {
		u8g2_DrawStr(&u8g2, 8, 36, "Waiting for IP...");
		return;
	}

	draw_qr_payload(url);

	/* Show URL under QR if space allows */
	if (strlen(url) <= 20) {
		u8g2_DrawStr(&u8g2, 3, SCR_H - 10, url);
	}
}

void screen_wifi_draw(const struct ui_state *st)
{
	int page = st->wifi_scroll;

	if (page < 0) {
		page = 0;
	}
	if (page >= WIFI_PAGE_COUNT) {
		page = WIFI_PAGE_COUNT - 1;
	}

	draw_title(page);

	switch (page) {
	case 0:
		draw_status_page();
		break;
	case 1:
		draw_wifi_qr_page();
		break;
	default:
		draw_web_qr_page();
		break;
	}

	draw_page_dots(page);
	u8g2_SendBuffer(&u8g2);
}
