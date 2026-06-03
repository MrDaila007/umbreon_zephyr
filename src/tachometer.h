#pragma once

#include <stdint.h>

/* ─── Tachometer: Hall sensor on GP13, 2 magnets/rev (RISING edge interrupt) ── */

void taho_init(void);

/* Get current filtered speed in m/s */
float taho_get_speed(void);

/* Get current speed in m/s from the last pulse interval, like old get_speed() */
float taho_get_instant_speed(void);

/* Get raw pulse count (atomic) */
uint32_t taho_get_count(void);

/* Get µs since last pulse */
uint32_t taho_time_since_last_us(void);

/* Reset counters (for tests) */
void taho_reset(void);

/* Runtime update of ISR glitch filter threshold (µs) */
void taho_set_glitch_filter_us(uint32_t us);
