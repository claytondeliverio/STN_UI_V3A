#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    BTN_UP = 0,
    BTN_LIGHT,
    BTN_POWER_MODE,
    BTN_MENU_SAVE,
    BTN_DOWN,
    BUTTON_COUNT
} button_id_t;

esp_err_t buttons_init(void);
void buttons_update(uint32_t now_ms);
void buttons_set_hold_times(uint32_t menu_save_hold_ms, uint32_t power_mode_hold_ms);
bool buttons_is_pressed(button_id_t id);
bool buttons_consume_short(button_id_t id);
bool buttons_consume_long(button_id_t id);
bool buttons_consume_step(button_id_t id);

#endif