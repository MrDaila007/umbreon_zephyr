#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Encoder event flags (bitmask) */
#define ENC_EVT_NONE    0
#define ENC_EVT_CW      BIT(0)
#define ENC_EVT_CCW     BIT(1)
#define ENC_EVT_CLICK   BIT(2)
#define ENC_EVT_DOUBLE  BIT(3)
#define ENC_EVT_HOLD    BIT(4)
#define ENC_EVT_FAST    BIT(5)

void encoder_init(void);

/* Poll accumulated encoder events (call from display thread tick).
 * Returns event bitmask, *rotation = net steps (+ = CW, - = CCW). */
uint8_t encoder_poll(int *rotation);
