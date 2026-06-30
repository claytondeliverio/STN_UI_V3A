#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t buzzer_init(void);
void buzzer_beep(uint32_t duration_ms);
void buzzer_error_pattern_set(bool enabled);
void buzzer_update(uint32_t now_ms);

#endif