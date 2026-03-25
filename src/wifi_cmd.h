#pragma once

#include <stdbool.h>

/* ─── UART0 WiFi command protocol ────────────────────────────────────────── */

void wifi_cmd_init(void);

/* Send a string over UART0 (used by other modules for telemetry/responses) */
void wifi_cmd_send(const char *str);

/* Printf-style send over UART0 */
void wifi_cmd_printf(const char *fmt, ...);

/* Log a message via WiFi with $L: prefix (thread-safe).
 * Use for debug output that should be visible over WiFi.
 * Respects log_enabled flag ($LOG:ON / $LOG:OFF). */
void wifi_log(const char *fmt, ...);

/* Check if WiFi logging is enabled */
bool wifi_log_enabled(void);
