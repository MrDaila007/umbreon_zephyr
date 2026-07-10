/*
 * display_hal.c — u8g2 HAL callbacks for Zephyr I2C + delay
 *
 * u8x8_byte_zephyr_hw_i2c():
 *   Accumulates bytes sent by u8g2 into i2c_buf[], then dispatches a
 *   single i2c_write() at END_TRANSFER. Holds i2c0_mutex from
 *   START_TRANSFER to END_TRANSFER to share I2C0 safely with the IMU.
 *
 * u8x8_gpio_and_delay_zephyr():
 *   Maps u8g2 delay requests to k_msleep()/k_busy_wait(). GPIO_RESET
 *   is a no-op because the RST pin is not wired on this board.
 *
 * Buffer size rationale:
 *   SSD1306 full-buffer write = 1 control byte (0x40) + 1024 data bytes.
 *   u8g2 may prepend additional command bytes, so 1026 covers all cases.
 *   Static allocation avoids heap fragmentation.
 */

#include "display_hal.h"
#include "display.h"        /* extern struct k_mutex i2c0_mutex */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(display_hal, LOG_LEVEL_INF);

/* ─── u8g2 instance ──────────────────────────────────────────────────────── */
u8g2_t u8g2;

/* ─── I2C device ─────────────────────────────────────────────────────────── */
static const struct device *i2c_dev;

/* ─── I2C byte accumulation buffer ──────────────────────────────────────── */
#define I2C_BUF_SIZE 1026
static uint8_t  i2c_buf[I2C_BUF_SIZE];
static int      i2c_buf_len;
static bool     transfer_active;
static bool     display_present;
static uint32_t display_i2c_errors;

#define DISPLAY_I2C_ADDR       0x3c
#define DISPLAY_MAX_I2C_ERRORS 3

static int display_probe(void)
{
	uint8_t control = 0x00;
	return i2c_write(i2c_dev, &control, sizeof(control), DISPLAY_I2C_ADDR);
}

/* ─── u8x8_byte_zephyr_hw_i2c ───────────────────────────────────────────── */

uint8_t u8x8_byte_zephyr_hw_i2c(u8x8_t *u8x8, uint8_t msg,
                                  uint8_t arg_int, void *arg_ptr)
{
	switch (msg) {

	case U8X8_MSG_BYTE_INIT:
		transfer_active = false;
		i2c_buf_len = 0;
		break;

	case U8X8_MSG_BYTE_SET_DC:
		/* I2C has no D/C line — no-op */
		break;

	case U8X8_MSG_BYTE_START_TRANSFER:
		/*
		 * Acquire I2C bus mutex before any bus activity.
		 * IMU (priority 2) holds this mutex for ~2 ms per sample_fetch.
		 * Display thread (priority 8) accepts up to 50 ms of contention.
		 * On timeout, skip this frame rather than crash.
		 */
		if (k_mutex_lock(&i2c0_mutex, K_MSEC(50)) != 0) {
			LOG_WRN("i2c0_mutex timeout — display frame dropped");
			transfer_active = false;
			return 0;
		}
		i2c_buf_len = 0;
		transfer_active = true;
		break;

	case U8X8_MSG_BYTE_SEND: {
		if (!transfer_active) {
			return 0;
		}
		const uint8_t *data = (const uint8_t *)arg_ptr;
		int remaining = I2C_BUF_SIZE - i2c_buf_len;

		if (arg_int > remaining) {
			LOG_ERR("I2C buffer overflow: need %d have %d — truncating",
				arg_int, remaining);
			arg_int = (uint8_t)remaining;
		}
		memcpy(i2c_buf + i2c_buf_len, data, arg_int);
		i2c_buf_len += arg_int;
		break;
	}

	case U8X8_MSG_BYTE_END_TRANSFER: {
		if (!transfer_active) {
			return 0;
		}
		if (i2c_buf_len > 0) {
			if (!display_present) {
				i2c_buf_len = 0;
				transfer_active = false;
				k_mutex_unlock(&i2c0_mutex);
				return 0;
			}
			/*
			 * u8x8 stores the 8-bit write address (7-bit addr << 1).
			 * SSD1306 default: u8x8->i2c_address = 0x78 → 7-bit = 0x3C.
			 * Zephyr i2c_write() takes the 7-bit address directly.
			 */
			uint8_t addr = u8x8_GetI2CAddress(u8x8) >> 1;
			int rc = i2c_write(i2c_dev, i2c_buf,
					   (uint32_t)i2c_buf_len, addr);
			if (rc != 0) {
				display_i2c_errors++;
				LOG_ERR("i2c_write addr=0x%02x len=%d rc=%d",
					addr, i2c_buf_len, rc);
				if (display_i2c_errors >= DISPLAY_MAX_I2C_ERRORS) {
					display_present = false;
					LOG_ERR("SSD1306 disabled after %u I2C errors",
						display_i2c_errors);
				}
			}
			i2c_buf_len = 0;
		}
		transfer_active = false;
		k_mutex_unlock(&i2c0_mutex);
		break;
	}

	default:
		return 0;
	}
	return 1;
}

/* ─── u8x8_gpio_and_delay_zephyr ────────────────────────────────────────── */

uint8_t u8x8_gpio_and_delay_zephyr(u8x8_t *u8x8, uint8_t msg,
                                    uint8_t arg_int, void *arg_ptr)
{
	ARG_UNUSED(u8x8);
	ARG_UNUSED(arg_ptr);

	switch (msg) {

	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		/* I2C pins are configured by DT overlay (SDA=GP0, SCL=GP1). */
		break;

	case U8X8_MSG_DELAY_MILLI:
		k_msleep(arg_int);
		break;

	case U8X8_MSG_DELAY_I2C:
		/*
		 * One I2C unit at 400 kHz ≈ 1.25 µs.
		 * Use 2 µs (conservative) to satisfy SSD1306 timing constraints.
		 */
		k_busy_wait(2U);
		break;

	case U8X8_MSG_GPIO_RESET:
		/* RST pin not wired on this board — SSD1306 init uses software reset */
		break;

	default:
		return 0;
	}
	return 1;
}

/* ─── display_hal_init ───────────────────────────────────────────────────── */

int display_hal_init(void)
{
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C0 not ready — check GP0/GP1 wiring and DT overlay");
		return -ENODEV;
	}

	display_i2c_errors = 0;
	display_present = false;
	int rc = display_probe();
	if (rc != 0) {
		display_i2c_errors++;
		LOG_ERR("SSD1306 not detected at I2C0 addr=0x%02x rc=%d",
			DISPLAY_I2C_ADDR, rc);
		return -ENODEV;
	}
	display_present = true;

	/*
	 * Full-buffer mode (_f suffix): u8g2 allocates a 1024-byte framebuffer
	 * inside u8g2_t. All drawing goes to RAM first; u8g2_SendBuffer() does
	 * one I2C transfer per frame (~10 ms at 400 kHz).
	 */
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		u8x8_byte_zephyr_hw_i2c,
		u8x8_gpio_and_delay_zephyr
	);

	/* Runs the SSD1306 initialisation command sequence via I2C. */
	u8g2_InitDisplay(&u8g2);

	/* Start with display off — display.c powers it on at wake. */
	u8g2_SetPowerSave(&u8g2, 1);

	LOG_INF("u8g2 HAL init OK (SSD1306 128x64, I2C0 addr=0x3C)");
	return 0;
}

bool display_hal_is_present(void)
{
	return display_present;
}

uint32_t display_hal_error_count(void)
{
	return display_i2c_errors;
}
