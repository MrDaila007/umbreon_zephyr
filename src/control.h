#pragma once

#include <stdbool.h>

/* ─── Main control loop ──────────────────────────────────────────────────── */

void control_init(void);

/* State queries */
bool control_is_running(void);
bool control_is_monitor(void);
bool control_is_countdown(void);

/* Start/stop state machine hooks for command layer */
bool control_request_start(void);
void control_cancel_start_request(void);

/* Commands from wifi_cmd thread */
void control_cmd_start(void);
void control_cmd_stop(void);
void control_cmd_monitor(void);

/* Manual drive ($DRV command) */
void control_set_manual(int steer, float speed);
void control_set_drv_enabled(bool enabled);
