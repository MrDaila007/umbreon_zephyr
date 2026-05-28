#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <u8g2.h>

/* Global u8g2 instance. Initialized by display_hal_init(). */
extern u8g2_t u8g2;

/* Initialize u8g2 with Zephyr I2C HAL and run SSD1306 init sequence.
 * Must be called from the display thread after the I2C bus is ready.
 * Returns 0 on success, negative errno on failure. */
int display_hal_init(void);

bool display_hal_is_present(void);
uint32_t display_hal_error_count(void);

/* HAL callbacks — exposed for unit testing only. */
uint8_t u8x8_byte_zephyr_hw_i2c(u8x8_t *u8x8, uint8_t msg,
                                  uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_zephyr(u8x8_t *u8x8, uint8_t msg,
                                    uint8_t arg_int, void *arg_ptr);
