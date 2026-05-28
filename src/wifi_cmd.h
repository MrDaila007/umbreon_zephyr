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
const char *wifi_status_get_ssid(void); /* decrypted SSID, empty until first poll */
const char *wifi_status_get_ip(void);   /* IP address string, empty until first poll */

/* Send encrypted WiFi credentials to ESP (ssid\tpassword XOR+hex).
 * Retransmits every 10 s (up to CFG_MAX_RETRIES=5) until $WIFICFG:ACK.
 * Must be called from wifi_cmd thread context (e.g. from dispatch_command). */
void wifi_cfg_set(const char *ssid, const char *password);
