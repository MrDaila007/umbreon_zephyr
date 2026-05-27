#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ─── VL53L0X ToF sensor array (6 sensors on I2C1) ───────────────────────── */

void sensors_init(void);

/* Poll all sensors — updates internal distance array.
 * Returns pointer to static int[6] of distances in cm×10. */
int *sensors_poll(void);

/* Poll only sensors selected by bitmask (BIT(IDX_LEFT) | BIT(IDX_RIGHT) etc.).
 * Unselected sensors keep their previous values.
 * Returns same pointer as sensors_poll(). */
int *sensors_poll_mask(uint8_t mask);

/* Get number of successfully initialized sensors */
int sensors_online_count(void);

/* True when sensor polling detected stale/error state requiring a safe stop
 * and full ToF array recovery from the control thread. */
bool sensors_recovery_needed(void);

/* Restart and verify the full ToF array. Intended to be called while stopped. */
bool sensors_recover_all(void);

/* Total automatic VL53L0X restarts since boot. */
uint32_t sensors_restart_count(void);

/* Probe expected VL53L0X I2C addresses.
 * Bit0..5 map to 0x30..0x35; bit6 maps to default 0x29. */
uint8_t sensors_i2c_scan_mask(void);
int sensors_i2c_scan_count(void);

/* Get pointer to last-polled distance array (6 values, cm×10).
 * Updated by sensors_poll() in control thread. Read-only. */
const int *sensors_get_distances(void);
