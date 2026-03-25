#pragma once

#include <stdbool.h>

/* ─── VL53L0X ToF sensor array (6 sensors on I2C1) ───────────────────────── */

void sensors_init(void);

/* Poll all sensors — updates internal distance array.
 * Returns pointer to static int[6] of distances in cm×10. */
int *sensors_poll(void);

/* Get number of successfully initialized sensors */
int sensors_online_count(void);
