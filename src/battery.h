#pragma once

/* ─── Battery voltage monitor (ADC on GP26) ──────────────────────────────── */

void battery_init(void);

/* Get filtered battery voltage (V). Updated every 500ms by battery thread. */
float battery_get_voltage(void);
