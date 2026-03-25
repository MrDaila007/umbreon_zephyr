#pragma once

#include <stdbool.h>

/* ─── Main control loop ──────────────────────────────────────────────────── */

void control_init(void);

/* State queries */
bool control_is_running(void);

/* Commands from wifi_cmd thread */
void control_cmd_start(void);
void control_cmd_stop(void);

/* Manual drive ($DRV command) */
void control_set_manual(int steer, float speed);
void control_set_drv_enabled(bool enabled);
