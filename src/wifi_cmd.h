#pragma once

#include <stdbool.h>

/* ─── UART command protocol ──────────────────────────────────────────────── */

void wifi_cmd_init(void);

/* Send a string over UART1 and debug UART0 (telemetry/responses) */
void wifi_cmd_send(const char *str);

/* Printf-style send over UART1 and debug UART0 */
void wifi_cmd_printf(const char *fmt, ...);

/* Log a message via WiFi with $L: prefix (thread-safe).
 * Use for debug output that should be visible over WiFi.
 * Respects log_enabled flag ($LOG:ON / $LOG:OFF). */
void wifi_log(const char *fmt, ...);

/* Check if WiFi logging is enabled */
bool wifi_log_enabled(void);

/* WiFi connection state — updated from ESP #WIFISTATUS replies */
bool        wifi_status_is_ready(void);
bool        wifi_status_is_ap(void);
int         wifi_status_get_rssi(void);
const char *wifi_status_get_ssid(void);
const char *wifi_status_get_ip(void);
const char *wifi_status_get_ap_pass(void);
