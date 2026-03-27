#pragma once

#include <stdbool.h>
#include <zephyr/kernel.h>

/* Initialize display hardware and create display thread.
 * Display starts OFF (blanked). */
void display_init(void);

/* Notify display that car run state changed. Thread-safe. */
void display_notify_run_state(bool running);

/* Notify display that a test started/ended. Thread-safe. */
void display_notify_test_state(bool active);

/* Menu command queue — checked by wifi_cmd thread */
extern struct k_msgq menu_cmd_q;

/* Menu command IDs */
enum menu_cmd {
	MCMD_START = 1,
	MCMD_STOP,
	MCMD_SAVE,
	MCMD_LOAD,
	MCMD_RESET,
	MCMD_TEST_BASE = 0x10, /* + test index 0-7 */
};
