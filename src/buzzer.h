#pragma once

enum buzzer_event {
	BUZZER_BOOT_READY,
	BUZZER_RUN_START,
	BUZZER_STOP,
	BUZZER_CAL_DONE,
	BUZZER_ERROR,
};

void buzzer_init(void);
void buzzer_beep(unsigned int freq_hz, unsigned int duration_ms);
void buzzer_play(enum buzzer_event event);
