#ifndef UI_STATE_H
#define UI_STATE_H

#include <stdint.h>
#include "pc_comm.h"

typedef enum {
    UI_MAIN_STARTUP = 0,
    UI_MAIN_OFF,
    UI_MAIN_POWER_ON_PENDING,
    UI_MAIN_ON,
    UI_MAIN_ERROR
} ui_main_state_t;

typedef enum {
    UI_MODE_SESSION = 0,
    UI_MODE_STANDBY = 1
} ui_operation_mode_t;

void ui_state_init(void);
void ui_state_update(uint32_t now_ms);
void ui_state_on_pc_event(const pc_event_t *event, void *ctx);

#endif