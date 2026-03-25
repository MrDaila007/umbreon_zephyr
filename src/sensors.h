#pragma once

#include <stdbool.h>

/* ─── VL53L0X ToF sensor array (6 sensors on I2C1) ───────────────────────── */

void sensors_init(void);

/* Poll all sensors — updates internal distance array.
 * Returns pointer to static int[6] of distances in cm×10. */
int *sensors_poll(void);

/* Get number of successfully initialized sensors */
int sensors_online_count(void);

/* Get pointer to last-polled distance array (6 values, cm×10).
 * Updated by sensors_poll() in control thread. Read-only. */
const int *sensors_get_distances(void);
