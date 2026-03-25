#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ─── Track profile learning & race mode ─────────────────────────────────── */

/* Track modes */
#define TRK_MODE_IDLE  0
#define TRK_MODE_LEARN 1
#define TRK_MODE_RACE  2

void track_learn_init(void);

/* Call from work() every control tick */
void track_learn_tick(int steer_cmd, float target_speed);

/* Returns recommended speed, or -1.0 if no recommendation */
float track_learn_recommend_speed(float lookahead_m);

/* Stop learning or racing */
void track_learn_stop(void);

/* Current mode */
int track_learn_get_mode(void);

/* WiFi command dispatcher for $TRK:* commands */
void track_learn_dispatch(const char *sub);
