/*
 * ssd1306_oled.c — Custom SSD1306 I2C display driver for Zephyr
 *
 * Init sequence ported from Adafruit SSD1306 Arduino library.
 * Implements standard Zephyr display_driver_api for CFB compatibility.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT solomon_ssd1306fb

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ssd1306_oled_regs.h"

LOG_MODULE_REGISTER(ssd1306_oled, CONFIG_DISPLAY_LOG_LEVEL);

/* ─── Config & data ──────────────────────────────────────────────────────── */

struct ssd1306_oled_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec reset;
	uint16_t width;
	uint16_t height;
	uint8_t segment_offset;
	uint8_t page_offset;
	uint8_t display_offset;
	uint8_t multiplex_ratio;
	uint8_t prechargep;
	bool segment_remap;
	bool com_invdir;
	bool com_sequential;
	bool color_inversion;
	int ready_time_ms;
};

struct ssd1306_oled_data {
	enum display_pixel_format pf;
};

/* ─── I2C helpers ────────────────────────────────────────────────────────── */

static int ssd1306_cmd(const struct device *dev,
		       const uint8_t *cmds, size_t len)
{
	const struct ssd1306_oled_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->i2c, SSD1306_CTRL_CMD, cmds, len);
}

static int ssd1306_data(const struct device *dev,
			const uint8_t *buf, size_t len)
{
	const struct ssd1306_oled_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->i2c, SSD1306_CTRL_DATA, buf, len);
}

static int ssd1306_cmd1(const struct device *dev, uint8_t cmd)
{
	return ssd1306_cmd(dev, &cmd, 1);
}

/* ─── Display API: blanking ──────────────────────────────────────────────── */

static int ssd1306_oled_blanking_on(const struct device *dev)
{
	LOG_DBG("blanking ON (display off)");
	return ssd1306_cmd1(dev, SSD1306_DISPLAY_OFF);
}

static int ssd1306_oled_blanking_off(const struct device *dev)
{
	LOG_DBG("blanking OFF (display on)");
	return ssd1306_cmd1(dev, SSD1306_DISPLAY_ON);
}

/* ─── Display API: write ─────────────────────────────────────────────────── */

static int ssd1306_oled_write(const struct device *dev,
			      const uint16_t x, const uint16_t y,
			      const struct display_buffer_descriptor *desc,
			      const void *buf)
{
	const struct ssd1306_oled_config *cfg = dev->config;

	if (desc->buf_size == 0 || buf == NULL) {
		return 0;
	}

	/* CFB uses MONO_VTILED: y and height are in 8-pixel pages */
	uint8_t page_start = y / 8;
	uint8_t page_end = page_start + desc->height / 8 - 1;
	uint8_t col_start = x + cfg->segment_offset;
	uint8_t col_end = col_start + desc->width - 1;

	/* Set column address range */
	uint8_t col_cmd[] = {SSD1306_SET_COL_ADDR, col_start, col_end};
	int rc = ssd1306_cmd(dev, col_cmd, sizeof(col_cmd));
	if (rc) {
		return rc;
	}

	/* Set page address range */
	uint8_t page_cmd[] = {SSD1306_SET_PAGE_ADDR,
			      page_start + cfg->page_offset,
			      page_end + cfg->page_offset};
	rc = ssd1306_cmd(dev, page_cmd, sizeof(page_cmd));
	if (rc) {
		return rc;
	}

	/* Write pixel data */
	return ssd1306_data(dev, buf, desc->buf_size);
}

/* ─── Display API: contrast ──────────────────────────────────────────────── */

static int ssd1306_oled_set_contrast(const struct device *dev,
				     const uint8_t contrast)
{
	uint8_t cmd[] = {SSD1306_SET_CONTRAST, contrast};

	LOG_DBG("contrast = %d", contrast);
	return ssd1306_cmd(dev, cmd, sizeof(cmd));
}

/* ─── Display API: capabilities ──────────────────────────────────────────── */

static void ssd1306_oled_get_capabilities(const struct device *dev,
					  struct display_capabilities *caps)
{
	const struct ssd1306_oled_config *cfg = dev->config;

	memset(caps, 0, sizeof(*caps));
	caps->x_resolution = cfg->width;
	caps->y_resolution = cfg->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO10 | PIXEL_FORMAT_MONO01;
	caps->current_pixel_format = ((struct ssd1306_oled_data *)dev->data)->pf;
	caps->screen_info = SCREEN_INFO_MONO_VTILED;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

/* ─── Display API: pixel format ──────────────────────────────────────────── */

static int ssd1306_oled_set_pixel_format(const struct device *dev,
					 const enum display_pixel_format pf)
{
	struct ssd1306_oled_data *data = dev->data;
	uint8_t cmd;

	if (pf == data->pf) {
		return 0;
	}

	if (pf == PIXEL_FORMAT_MONO10) {
		cmd = SSD1306_INVERT_DISPLAY;
	} else if (pf == PIXEL_FORMAT_MONO01) {
		cmd = SSD1306_NORMAL_DISPLAY;
	} else {
		return -ENOTSUP;
	}

	int rc = ssd1306_cmd1(dev, cmd);
	if (rc == 0) {
		data->pf = pf;
	}
	return rc;
}

/* ─── Init ───────────────────────────────────────────────────────────────── */

static int ssd1306_oled_hw_init(const struct device *dev)
{
	const struct ssd1306_oled_config *cfg = dev->config;
	int rc;

	/*
	 * Adafruit SSD1306 init sequence — proven on millions of devices.
	 * Adapted to use DT-configurable parameters where appropriate.
	 */
	uint8_t init_cmds[] = {
		/* Display off during configuration */
		SSD1306_DISPLAY_OFF,

		/* Clock: default oscillator, divide ratio 1 */
		SSD1306_SET_CLOCK_DIV, 0x80,

		/* Multiplex ratio from DT (height - 1) */
		SSD1306_SET_MUX_RATIO, cfg->multiplex_ratio,

		/* Display offset from DT */
		SSD1306_SET_DISPLAY_OFFSET, cfg->display_offset,

		/* Start line 0 */
		SSD1306_SET_START_LINE | 0x00,

		/* Charge pump: ON (internal VCC — required for most modules) */
		SSD1306_SET_CHARGE_PUMP, SSD1306_CHARGE_PUMP_ON,

		/* Horizontal addressing mode (needed for sequential buffer write) */
		SSD1306_SET_MEM_ADDR_MODE, SSD1306_ADDR_MODE_HORIZ,

		/* Segment remap from DT */
		cfg->segment_remap ? SSD1306_SEG_REMAP_FLIP
				   : SSD1306_SEG_REMAP_NORMAL,

		/* COM scan direction from DT */
		cfg->com_invdir ? SSD1306_COM_SCAN_FLIP
				: SSD1306_COM_SCAN_NORMAL,

		/* COM pins configuration from DT */
		SSD1306_SET_COM_PINS,
		cfg->com_sequential ? SSD1306_COM_PINS_SEQ
				    : SSD1306_COM_PINS_ALT,

		/* Contrast */
		SSD1306_SET_CONTRAST,
		(uint8_t)CONFIG_SSD1306_OLED_DEFAULT_CONTRAST,

		/* Precharge period: 0xF1 for internal VCC (Adafruit default) */
		SSD1306_SET_PRECHARGE, 0xF1,

		/* VCOM deselect level: 0x40 (Adafruit/GyverOLED standard) */
		SSD1306_SET_VCOM_DESEL, 0x40,

		/* Output follows RAM content */
		SSD1306_DISPLAY_RAM,

		/* Normal display (not inverted) */
		cfg->color_inversion ? SSD1306_INVERT_DISPLAY
				     : SSD1306_NORMAL_DISPLAY,

		/* Deactivate scroll (prevent artifacts) */
		SSD1306_DEACTIVATE_SCROLL,

		/* Display ON */
		SSD1306_DISPLAY_ON,
	};

	rc = ssd1306_cmd(dev, init_cmds, sizeof(init_cmds));
	if (rc) {
		LOG_ERR("[%s] init commands failed: %d", dev->name, rc);
		return -EIO;
	}

	LOG_INF("[%s] initialized %dx%d on %s addr 0x%02x",
		dev->name, cfg->width, cfg->height,
		cfg->i2c.bus->name, cfg->i2c.addr);
	return 0;
}

static int ssd1306_oled_init(const struct device *dev)
{
	const struct ssd1306_oled_config *cfg = dev->config;
	struct ssd1306_oled_data *data = dev->data;

	data->pf = cfg->color_inversion ? PIXEL_FORMAT_MONO10
					: PIXEL_FORMAT_MONO01;

	/* Wait for power-up */
	k_sleep(K_TIMEOUT_ABS_MS(cfg->ready_time_ms));

	/* Check I2C bus ready */
	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("[%s] I2C bus %s not ready!", dev->name,
			cfg->i2c.bus->name);
		return -ENODEV;
	}

	/* Reset if pin connected */
	if (cfg->reset.port) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("[%s] reset GPIO not ready", dev->name);
			return -ENODEV;
		}
		gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_ACTIVE);
		k_msleep(1);
		gpio_pin_set_dt(&cfg->reset, 0);
		k_msleep(1);
	}

	return ssd1306_oled_hw_init(dev);
}

/* ─── Driver API ─────────────────────────────────────────────────────────── */

static DEVICE_API(display, ssd1306_oled_api) = {
	.blanking_on = ssd1306_oled_blanking_on,
	.blanking_off = ssd1306_oled_blanking_off,
	.write = ssd1306_oled_write,
	.set_contrast = ssd1306_oled_set_contrast,
	.get_capabilities = ssd1306_oled_get_capabilities,
	.set_pixel_format = ssd1306_oled_set_pixel_format,
};

/* ─── Device instantiation ───────────────────────────────────────────────── */

#define SSD1306_OLED_INIT(inst)                                               \
	static struct ssd1306_oled_config ssd1306_oled_cfg_##inst = {         \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                           \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {}),    \
		.width = DT_INST_PROP(inst, width),                           \
		.height = DT_INST_PROP(inst, height),                         \
		.segment_offset = DT_INST_PROP(inst, segment_offset),         \
		.page_offset = DT_INST_PROP(inst, page_offset),               \
		.display_offset = DT_INST_PROP(inst, display_offset),         \
		.multiplex_ratio = DT_INST_PROP(inst, multiplex_ratio),       \
		.prechargep = DT_INST_PROP(inst, prechargep),                 \
		.segment_remap = DT_INST_PROP(inst, segment_remap),           \
		.com_invdir = DT_INST_PROP(inst, com_invdir),                 \
		.com_sequential = DT_INST_PROP_OR(inst, com_sequential,       \
						  false),                     \
		.color_inversion = DT_INST_PROP_OR(inst, inversion_on,        \
						   false),                    \
		.ready_time_ms = DT_INST_PROP_OR(inst, ready_time_ms, 10),    \
	};                                                                    \
	static struct ssd1306_oled_data ssd1306_oled_data_##inst;             \
	DEVICE_DT_INST_DEFINE(inst, ssd1306_oled_init, NULL,                  \
			      &ssd1306_oled_data_##inst,                      \
			      &ssd1306_oled_cfg_##inst,                       \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,      \
			      &ssd1306_oled_api);

DT_INST_FOREACH_STATUS_OKAY(SSD1306_OLED_INIT)
