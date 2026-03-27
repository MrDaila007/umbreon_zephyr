/*
 * vl53l0x_fast.c — continuous back-to-back mode for VL53L0X
 *
 * Register sequences derived from ST VL53L0X API (vl53l0x_api.c):
 *   - VL53L0X_DataInit        (StopVariable read)
 *   - VL53L0X_StartMeasurement (continuous ranging start)
 *   - VL53L0X_GetRangingMeasurementData (result read)
 *   - VL53L0X_ClearInterruptMask
 *   - VL53L0X_StopMeasurement
 */

#include "vl53l0x_fast.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(vl53l0x_fast, LOG_LEVEL_INF);

/* ─── I2C helpers ────────────────────────────────────────────────────────── */

static inline int wr(const struct device *i2c, uint16_t addr,
		     uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(i2c, addr, reg, val);
}

static inline int rd(const struct device *i2c, uint16_t addr,
		     uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte(i2c, addr, reg, val);
}

/* ─── Init ───────────────────────────────────────────────────────────────── */

int vl53l0x_fast_init(struct vl53l0x_fast *s,
		      const struct device *i2c, uint16_t addr)
{
	int rc;
	uint8_t val;

	s->i2c = i2c;
	s->addr = addr;
	s->continuous = false;

	/*
	 * Read StopVariable — same page-access sequence as
	 * VL53L0X_DataInit (vl53l0x_api.c:429-433)
	 */
	rc  = wr(i2c, addr, 0x80, 0x01);
	rc |= wr(i2c, addr, 0xFF, 0x01);
	rc |= wr(i2c, addr, 0x00, 0x00);
	rc |= rd(i2c, addr, 0x91, &val);
	rc |= wr(i2c, addr, 0x00, 0x01);
	rc |= wr(i2c, addr, 0xFF, 0x00);
	rc |= wr(i2c, addr, 0x80, 0x00);
	if (rc) {
		LOG_ERR("addr 0x%02x: StopVariable read failed", addr);
		return -EIO;
	}
	s->stop_variable = val;

	/* Read FractionEnable (reg 0x09 bit 0) */
	rc = rd(i2c, addr, VL53_REG_SYSTEM_RANGE_CONFIG, &val);
	if (rc) {
		LOG_ERR("addr 0x%02x: FractionEnable read failed", addr);
		return -EIO;
	}
	s->frac_enable = (val & 0x01);

	/* Configure interrupt: NEW_SAMPLE_READY on GPIO1 */
	rc = wr(i2c, addr, VL53_REG_SYSTEM_INTERRUPT_CONFIG, VL53_INT_NEW_SAMPLE_READY);
	if (rc) {
		LOG_ERR("addr 0x%02x: interrupt config failed", addr);
		return -EIO;
	}

	LOG_DBG("addr 0x%02x: stop_var=0x%02x frac=%d",
		addr, s->stop_variable, s->frac_enable);
	return 0;
}

/* ─── Start continuous ───────────────────────────────────────────────────── */

int vl53l0x_fast_start(struct vl53l0x_fast *s)
{
	int rc;

	/*
	 * Preamble + start — matches VL53L0X_StartMeasurement
	 * for VL53L0X_DEVICEMODE_CONTINUOUS_RANGING (vl53l0x_api.c:2218-2258)
	 */
	rc  = wr(s->i2c, s->addr, 0x80, 0x01);
	rc |= wr(s->i2c, s->addr, 0xFF, 0x01);
	rc |= wr(s->i2c, s->addr, 0x00, 0x00);
	rc |= wr(s->i2c, s->addr, 0x91, s->stop_variable);
	rc |= wr(s->i2c, s->addr, 0x00, 0x01);
	rc |= wr(s->i2c, s->addr, 0xFF, 0x00);
	rc |= wr(s->i2c, s->addr, 0x80, 0x00);

	/* Start back-to-back continuous ranging */
	rc |= wr(s->i2c, s->addr, VL53_REG_SYSRANGE_START, VL53_MODE_BACKTOBACK);
	if (rc) {
		LOG_ERR("addr 0x%02x: continuous start failed", s->addr);
		return -EIO;
	}

	s->continuous = true;
	LOG_INF("addr 0x%02x: continuous mode started", s->addr);
	return 0;
}

/* ─── Non-blocking read ──────────────────────────────────────────────────── */

int vl53l0x_fast_read(struct vl53l0x_fast *s)
{
	int rc;
	uint8_t status;

	/* 1. Check interrupt status — is new data ready? */
	rc = rd(s->i2c, s->addr, VL53_REG_RESULT_INTERRUPT_STATUS, &status);
	if (rc) {
		return -EIO;
	}
	if ((status & 0x07) == 0) {
		return -EAGAIN;
	}

	/* 2. Burst-read 12-byte result block from 0x14
	 *    (matches VL53L0X_ReadMulti in GetRangingMeasurementData) */
	uint8_t buf[12];
	rc = i2c_burst_read(s->i2c, s->addr,
			    VL53_REG_RESULT_RANGE_STATUS, buf, sizeof(buf));
	if (rc) {
		return -EIO;
	}

	/* 3. Extract range — buf[10]=MSB, buf[11]=LSB
	 *    (VL53L0X_MAKEUINT16(lsb=buf[11], msb=buf[10])) */
	uint16_t raw = ((uint16_t)buf[10] << 8) | buf[11];
	int mm = s->frac_enable ? (int)(raw >> 2) : (int)raw;

	/* 4. Clear interrupt (VL53L0X_ClearInterruptMask, vl53l0x_api.c:2869-2896)
	 *    Retry up to 3 times if interrupt doesn't clear */
	for (int retry = 0; retry < 3; retry++) {
		wr(s->i2c, s->addr, VL53_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
		wr(s->i2c, s->addr, VL53_REG_SYSTEM_INTERRUPT_CLEAR, 0x00);

		uint8_t check;
		rc = rd(s->i2c, s->addr, VL53_REG_RESULT_INTERRUPT_STATUS, &check);
		if (rc == 0 && (check & 0x07) == 0) {
			break;
		}
	}

	return mm;
}

/* ─── Stop continuous ────────────────────────────────────────────────────── */

int vl53l0x_fast_stop(struct vl53l0x_fast *s)
{
	int rc;

	/*
	 * VL53L0X_StopMeasurement (vl53l0x_api.c:2289-2314)
	 */
	rc  = wr(s->i2c, s->addr, VL53_REG_SYSRANGE_START, VL53_MODE_SINGLESHOT);
	rc |= wr(s->i2c, s->addr, 0xFF, 0x01);
	rc |= wr(s->i2c, s->addr, 0x00, 0x00);
	rc |= wr(s->i2c, s->addr, 0x91, 0x00);
	rc |= wr(s->i2c, s->addr, 0x00, 0x01);
	rc |= wr(s->i2c, s->addr, 0xFF, 0x00);
	if (rc) {
		LOG_ERR("addr 0x%02x: stop failed", s->addr);
		return -EIO;
	}

	s->continuous = false;
	LOG_INF("addr 0x%02x: continuous mode stopped", s->addr);
	return 0;
}
