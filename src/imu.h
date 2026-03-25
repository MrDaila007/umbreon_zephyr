#pragma once

#include <stdbool.h>

/* ─── IMU: MPU-6050 gyro Z axis ──────────────────────────────────────────── */

void imu_init(void);

/* Sample gyro bias while stationary (~1s, 200 samples).
 * Call after init, before driving. */
void imu_calibrate(void);

/* Read gyro Z, apply bias/deadzone/EMA, integrate heading.
 * Call every control tick (~40ms). */
void imu_update(void);

/* Reset accumulated heading to zero */
void imu_reset_heading(void);

/* ─── Accessors ───────────────────────────────────────────────────────────── */
bool imu_is_ok(void);
float imu_get_yaw_rate(void);   /* filtered yaw rate (°/s) */
float imu_get_heading(void);    /* accumulated heading change (°) */
