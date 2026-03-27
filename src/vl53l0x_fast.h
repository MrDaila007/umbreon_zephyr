#pragma once

/*
 * vl53l0x_fast — lightweight continuous-mode driver for VL53L0X
 *
 * Uses direct I2C register access to run sensors in back-to-back
 * continuous mode.  Designed to work alongside the stock Zephyr
 * st,vl53l0x driver which handles initialisation and calibration.
 */

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

/* ─── VL53L0X register addresses ─────────────────────────────────────────── */
#define VL53_REG_SYSRANGE_START              0x00
#define VL53_REG_SYSTEM_RANGE_CONFIG         0x09
#define VL53_REG_SYSTEM_INTERRUPT_CONFIG     0x0A
#define VL53_REG_SYSTEM_INTERRUPT_CLEAR      0x0B
#define VL53_REG_RESULT_INTERRUPT_STATUS     0x13
#define VL53_REG_RESULT_RANGE_STATUS         0x14

/* SYSRANGE_START mode values */
#define VL53_MODE_SINGLESHOT    0x00
#define VL53_MODE_BACKTOBACK    0x02
#define VL53_MODE_TIMED         0x04

/* Interrupt config */
#define VL53_INT_NEW_SAMPLE_READY  0x04

/* ─── Per-sensor state ───────────────────────────────────────────────────── */
struct vl53l0x_fast {
	const struct device *i2c;
	uint16_t addr;
	uint8_t  stop_variable;  /* opaque value from reg 0x91 after ST HAL init */
	bool     frac_enable;    /* reg 0x09 bit 0: range is 11.2 fixed-point */
	bool     continuous;     /* true once back-to-back mode is running */
};

/*
 * Read StopVariable and FractionEnable from sensor.
 * Call AFTER the stock Zephyr driver has completed init
 * (i.e. after one successful sensor_sample_fetch).
 */
int vl53l0x_fast_init(struct vl53l0x_fast *s,
		      const struct device *i2c, uint16_t addr);

/*
 * Switch sensor to continuous back-to-back mode.
 * Sensor will measure non-stop; results are read with vl53l0x_fast_read().
 */
int vl53l0x_fast_start(struct vl53l0x_fast *s);

/*
 * Non-blocking read of the latest range measurement.
 * Returns range in mm (>= 0), or:
 *   -EAGAIN  no new measurement ready yet
 *   -EIO     I2C communication error
 */
int vl53l0x_fast_read(struct vl53l0x_fast *s);

/*
 * Stop continuous mode, return sensor to idle.
 */
int vl53l0x_fast_stop(struct vl53l0x_fast *s);
