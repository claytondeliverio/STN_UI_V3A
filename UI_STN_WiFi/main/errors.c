#include "errors.h"

#include <stdio.h>
#include "display.h"
#include "buzzer.h"

#define SLAVE_ERROR_TOGGLE_MS 1000U

static uint8_t s_error_code;
static bool s_visible;
static bool s_from_slave;
static uint8_t s_slave_id;
static bool s_show_slave;
static uint32_t s_last_toggle_ms;

void errors_init(void)
{
    s_error_code = 0;
    s_visible = false;
    s_from_slave = false;
    s_slave_id = 0;
    s_show_slave = true;
    s_last_toggle_ms = 0;
}

void errors_set(uint8_t error_code, bool visible)
{
    if (error_code == 0) return;
    s_error_code = error_code;
    s_from_slave = false;
    s_slave_id = 0;
    if (visible) {
        s_visible = true;
        display_show_error(s_error_code);
        buzzer_error_pattern_set(true);
    }
}

void errors_set_slave(uint8_t slave_id, uint8_t error_code, bool visible)
{
    if (error_code == 0) return;
    s_error_code = error_code;
    s_from_slave = true;
    s_slave_id = slave_id;
    s_show_slave = true;
    if (visible) {
        s_visible = true;
        display_show_slave_error(s_slave_id, s_error_code);
        buzzer_error_pattern_set(true);
    }
}

void errors_clear_visible(void)
{
    s_visible = false;
    s_error_code = 0;
    s_from_slave = false;
    s_slave_id = 0;
    buzzer_error_pattern_set(false);
}

bool errors_has_stored(void)
{
    return s_error_code != 0;
}

bool errors_is_visible(void)
{
    return s_visible;
}

uint8_t errors_code(void)
{
    return s_error_code;
}

void errors_update(uint32_t now_ms)
{
    if (!s_visible || s_error_code == 0) return;
    if (!s_from_slave) {
        display_show_error(s_error_code);
        return;
    }

    if ((uint32_t)(now_ms - s_last_toggle_ms) >= SLAVE_ERROR_TOGGLE_MS) {
        s_last_toggle_ms = now_ms;
        s_show_slave = !s_show_slave;
    }

    if (s_show_slave) {
        char text[8];
        snprintf(text, sizeof(text), "S%u", (unsigned)s_slave_id);
        display_show_text(text);
    } else {
        display_show_error(s_error_code);
    }
}