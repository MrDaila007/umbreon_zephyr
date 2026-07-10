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

/* Calibrate accelerometer: average CAL_SAMPLES reads of all axes, store bias */
void  imu_calibrate_accel(void);

/* Expose calibration results for display */
float imu_get_gyro_bias(void);
void  imu_get_accel_bias(float *x, float *y, float *z);

/* ─── Accessors ───────────────────────────────────────────────────────────── */
bool imu_is_ok(void);
float imu_get_yaw_rate(void);   /* filtered yaw rate (°/s) */
float imu_get_heading(void);    /* accumulated heading change (°) */
