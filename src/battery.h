#pragma once

/* ─── Battery voltage monitor (ADC on GP26) ──────────────────────────────── */

void battery_init(void);

/* Get filtered battery voltage (V). Updated by the battery thread. */
float battery_get_voltage(void);

/* Latest raw ADC-derived voltage and minimum since boot. */
float battery_get_raw_voltage(void);
float battery_get_min_voltage(void);
