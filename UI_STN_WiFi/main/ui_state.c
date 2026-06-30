#include "ui_state.h"
#include "buttons.h"
#include "buzzer.h"
#include "display.h"
#include "errors.h"
#include "hardware.h"
#include "settings.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#define VERSION_REQUEST_INTERVAL_MS 1000U
#define VERSION_REPLY_TIMEOUT_MS 10000U
#define VERSION_DISPLAY_TIME_MS 1000U
#define POWER_ON_RETRY_MS 1000U
#define MENU_AUTOSAVE_MS 5000U
#define MENU_SAVE_HOLD_MS 2000U
#define MENU_FORCE_DRAIN_HOLD_MS 5000U
#define POWER_OFF_HOLD_MS 2000U
#define SERVICE_MODE_ENTRY_HOLD_MS 3000U
#define SERVICE_MODE_SAVE_HOLD_MS 3000U
#define SERVICE_STATUS_REQUEST_INTERVAL_MS 1000U
#define TARGET_TEMP_MIN_C 30U
#define SCENT_MIN 1U
#define SCENT_MAX 20U
#define SESSION_TIME_MIN_MIN 5U
#define SESSION_TIME_STEP_MIN 5U
#define SESSION_TIME_24H_MIN 1440U
#define INVALID_TEMP (-1000)
#define MAX_SLAVES 7

typedef enum {
    START_REQUEST_PC_VERSION = 0,
    START_SHOW_PC_VERSION,
    START_SHOW_UI_VERSION
} startup_step_t;

typedef enum {
    UI_MENU_FAN = 0,
    UI_MENU_TARGET_TEMP,
    UI_MENU_SCENT_PUMP,
    UI_MENU_SESSION_TIME
} ui_menu_feature_t;

static ui_main_state_t s_main_state;
static ui_operation_mode_t s_mode;
static startup_step_t s_startup_step;
static uint32_t s_state_started_ms;
static uint32_t s_last_version_request_ms;
static uint32_t s_last_power_on_ms;
static uint8_t s_pc_version_major;
static uint8_t s_pc_version_minor;
static bool s_pc_version_received;
static bool s_menu_active;
static bool s_menu_dirty;
static ui_menu_feature_t s_menu_feature;
static uint32_t s_last_menu_key_ms;
static bool s_cabin_light_on;
static bool s_error_light_applied;
static int s_actual_temp_c = INVALID_TEMP;
static bool s_service_active;
static uint32_t s_service_combo_started_ms;
static uint8_t s_service_slave = 1;
static uint8_t s_service_status[MAX_SLAVES + 1];
static uint8_t s_service_error[MAX_SLAVES + 1];
static uint8_t s_service_desired = 1;
static uint32_t s_last_service_request_ms;
static uint32_t s_service_menu_hold_started_ms;
static bool s_service_menu_saved;

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void send_current_settings(void)
{
    ui_settings_t *s = settings_get();
    uint16_t standby = s->standbyDisabled ? 0 : s->standbyTimeMin;
    pc_comm_send_ui_settings(s->targetTemperatureC, s->sessionTimeMin, standby);
}

static void enter_off(void)
{
    s_main_state = UI_MAIN_OFF;
    s_mode = UI_MODE_SESSION;
    s_menu_active = false;
    s_menu_dirty = false;
    s_service_active = false;
    display_show_off();
    buzzer_beep(100);
}

static void enter_error(uint8_t code)
{
    s_main_state = UI_MAIN_ERROR;
    s_menu_active = false;
    s_menu_dirty = false;
    errors_set(code, true);
    if (!s_error_light_applied) {
        s_error_light_applied = true;
        s_cabin_light_on = true;
        pc_comm_send_cabin_light(settings_get()->cabinLightPercent);
    }
}

static void power_off_to_off(void)
{
    pc_comm_send_power_off();
    errors_clear_visible();
    s_error_light_applied = false;
    enter_off();
}

static void show_normal_on_display(void)
{
    if (s_mode == UI_MODE_STANDBY) {
        display_show_time_minutes(settings_get()->standbyTimeMin);
    } else {
        display_show_temperature(s_actual_temp_c);
    }
}

static void show_menu_display(void)
{
    ui_settings_t *s = settings_get();
    switch (s_menu_feature) {
    case UI_MENU_FAN:
        display_show_text(s->fanOn ? "ON" : "OFF");
        break;
    case UI_MENU_TARGET_TEMP:
        display_show_text("TEMP");
        break;
    case UI_MENU_SCENT_PUMP:
        display_show_text("SCENT");
        break;
    case UI_MENU_SESSION_TIME:
        display_show_text("TIME");
        break;
    }
}

static void save_menu_settings(void)
{
    ui_settings_t *s = settings_get();
    send_current_settings();
    pc_comm_send_scent_pump(s->scentPumpSetpoint);
    pc_comm_send_fan(s->fanOn);
    settings_save();
    s_menu_active = false;
    s_menu_dirty = false;
    s_menu_feature = UI_MENU_FAN;
    buzzer_beep(300);
    show_normal_on_display();
}

static void mark_menu_changed(uint32_t now_ms)
{
    s_menu_dirty = true;
    s_last_menu_key_ms = now_ms;
}

static void cycle_menu(uint32_t now_ms)
{
    if (!s_menu_active) {
        s_menu_active = true;
        s_menu_feature = UI_MENU_FAN;
    } else {
        switch (s_menu_feature) {
        case UI_MENU_FAN: s_menu_feature = UI_MENU_TARGET_TEMP; break;
        case UI_MENU_TARGET_TEMP: s_menu_feature = UI_MENU_SCENT_PUMP; break;
        case UI_MENU_SCENT_PUMP: s_menu_feature = UI_MENU_SESSION_TIME; break;
        case UI_MENU_SESSION_TIME: s_menu_feature = UI_MENU_FAN; break;
        }
    }
    s_last_menu_key_ms = now_ms;
    show_menu_display();
}

static void adjust_menu(bool increase, uint32_t now_ms)
{
    ui_settings_t *s = settings_get();
    bool changed = false;

    switch (s_menu_feature) {
    case UI_MENU_FAN:
        if (s->fanOn != increase) {
            s->fanOn = increase;
            pc_comm_send_fan(s->fanOn);
            changed = true;
        }
        display_show_text(s->fanOn ? "ON" : "OFF");
        break;
    case UI_MENU_TARGET_TEMP:
        if (increase && s->targetTemperatureC < s->maxTargetTemperatureC) {
            s->targetTemperatureC++;
            changed = true;
            display_show_temperature(s->targetTemperatureC);
        } else if (!increase && s->targetTemperatureC > TARGET_TEMP_MIN_C) {
            s->targetTemperatureC--;
            changed = true;
            display_show_temperature(s->targetTemperatureC);
        } else {
            buzzer_beep(75);
        }
        break;
    case UI_MENU_SCENT_PUMP:
        if (increase && s->scentPumpSetpoint < SCENT_MAX) {
            s->scentPumpSetpoint++;
            changed = true;
        } else if (!increase && s->scentPumpSetpoint > SCENT_MIN) {
            s->scentPumpSetpoint--;
            changed = true;
        } else {
            buzzer_beep(75);
        }
        if (changed) {
            char text[8];
            snprintf(text, sizeof(text), "N%u", (unsigned)s->scentPumpSetpoint);
            display_show_text(text);
        }
        break;
    case UI_MENU_SESSION_TIME: {
        uint16_t max = s->maxSessionTimeMin;
        if (max == 0 || max > SESSION_TIME_24H_MIN) max = SESSION_TIME_24H_MIN;
        if (increase && s->sessionTimeMin < max) {
            s->sessionTimeMin += SESSION_TIME_STEP_MIN;
            if (s->sessionTimeMin > max) s->sessionTimeMin = max;
            changed = true;
        } else if (!increase && s->sessionTimeMin > SESSION_TIME_MIN_MIN) {
            s->sessionTimeMin -= SESSION_TIME_STEP_MIN;
            if (s->sessionTimeMin < SESSION_TIME_MIN_MIN) s->sessionTimeMin = SESSION_TIME_MIN_MIN;
            changed = true;
        } else {
            buzzer_beep(75);
        }
        if (changed) display_show_time_minutes(s->sessionTimeMin);
        break;
    }
    }

    if (changed) mark_menu_changed(now_ms);
}

static void toggle_cabin_light(void)
{
    ui_settings_t *s = settings_get();
    s_cabin_light_on = !s_cabin_light_on;
    pc_comm_send_cabin_light(s_cabin_light_on ? s->cabinLightPercent : 0);
    buzzer_beep(100);
}

static void enter_service(uint32_t now_ms)
{
    s_service_active = true;
    s_service_slave = 1;
    s_service_desired = 1;
    s_last_service_request_ms = now_ms - SERVICE_STATUS_REQUEST_INTERVAL_MS;
    s_service_menu_hold_started_ms = 0;
    s_service_menu_saved = false;
    display_show_text("SM");
    buzzer_beep(100);
}

static void update_service(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_service_request_ms) >= SERVICE_STATUS_REQUEST_INTERVAL_MS) {
        s_last_service_request_ms = now_ms;
        pc_comm_send_slave_status_request(s_service_slave);
    }

    if (buttons_consume_step(BTN_DOWN) && s_service_slave > 1) {
        s_service_slave--;
        display_show_service_status(s_service_slave, s_service_status[s_service_slave], s_service_error[s_service_slave]);
    }
    if (buttons_consume_step(BTN_UP) && s_service_slave < MAX_SLAVES) {
        s_service_slave++;
        display_show_service_status(s_service_slave, s_service_status[s_service_slave], s_service_error[s_service_slave]);
    }
    if (buttons_consume_short(BTN_MENU_SAVE)) {
        s_service_desired = (s_service_desired == 1) ? 3 : 1;
        s_service_status[s_service_slave] = s_service_desired;
        display_show_service_status(s_service_slave, s_service_desired, 0);
    }
    if (buttons_is_pressed(BTN_MENU_SAVE)) {
        if (s_service_menu_hold_started_ms == 0) s_service_menu_hold_started_ms = now_ms;
        if (!s_service_menu_saved && (uint32_t)(now_ms - s_service_menu_hold_started_ms) >= SERVICE_MODE_SAVE_HOLD_MS) {
            s_service_menu_saved = true;
            pc_comm_send_slave_service_set(s_service_slave, s_service_desired);
            buzzer_beep(300);
        }
    } else {
        s_service_menu_hold_started_ms = 0;
        s_service_menu_saved = false;
    }
    if (buttons_consume_short(BTN_POWER_MODE)) {
        s_service_active = false;
        display_show_off();
        buzzer_beep(100);
    }
}

static void handle_off(uint32_t now_ms)
{
    buttons_set_hold_times(MENU_SAVE_HOLD_MS, POWER_OFF_HOLD_MS);

    if (buttons_is_pressed(BTN_MENU_SAVE) && buttons_is_pressed(BTN_LIGHT)) {
        if (s_service_combo_started_ms == 0) s_service_combo_started_ms = now_ms;
        if ((uint32_t)(now_ms - s_service_combo_started_ms) >= SERVICE_MODE_ENTRY_HOLD_MS) {
            enter_service(now_ms);
            s_service_combo_started_ms = 0;
            return;
        }
    } else {
        s_service_combo_started_ms = 0;
    }

    if (s_service_active) {
        update_service(now_ms);
        return;
    }

    if (buttons_consume_short(BTN_LIGHT)) toggle_cabin_light();

    if (buttons_consume_short(BTN_POWER_MODE)) {
        if (errors_has_stored()) {
            enter_error(errors_code());
            return;
        }
        send_current_settings();
        pc_comm_send_power_on();
        s_main_state = UI_MAIN_POWER_ON_PENDING;
        s_last_power_on_ms = now_ms;
        display_show_text("WAIT");
    }
}

static void handle_pending(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_power_on_ms) >= POWER_ON_RETRY_MS) {
        pc_comm_send_power_on();
        s_last_power_on_ms = now_ms;
    }
    if (buttons_consume_long(BTN_POWER_MODE)) power_off_to_off();
}

static void handle_on(uint32_t now_ms)
{
    buttons_set_hold_times(s_mode == UI_MODE_STANDBY ? MENU_FORCE_DRAIN_HOLD_MS : MENU_SAVE_HOLD_MS, POWER_OFF_HOLD_MS);

    if (buttons_consume_short(BTN_LIGHT)) toggle_cabin_light();

    if (buttons_consume_long(BTN_POWER_MODE)) {
        power_off_to_off();
        return;
    }

    if (buttons_consume_short(BTN_POWER_MODE)) {
        s_mode = (s_mode == UI_MODE_SESSION) ? UI_MODE_STANDBY : UI_MODE_SESSION;
        pc_comm_send_operation_mode((uint8_t)s_mode);
        s_menu_active = false;
        buzzer_beep(100);
        show_normal_on_display();
        return;
    }

    if (buttons_consume_long(BTN_MENU_SAVE)) {
        if (s_mode == UI_MODE_STANDBY) {
            if (s_menu_dirty) save_menu_settings();
            pc_comm_send_drain_fill_start();
            display_show_text("DRN");
            buzzer_beep(300);
        } else if (s_menu_active) {
            save_menu_settings();
        }
        return;
    }

    if (buttons_consume_short(BTN_MENU_SAVE)) {
        cycle_menu(now_ms);
        return;
    }

    if (s_menu_active) {
        if (buttons_consume_step(BTN_DOWN)) adjust_menu(false, now_ms);
        if (buttons_consume_step(BTN_UP)) adjust_menu(true, now_ms);
        if (s_menu_dirty && (uint32_t)(now_ms - s_last_menu_key_ms) >= MENU_AUTOSAVE_MS) save_menu_settings();
    } else if (s_mode == UI_MODE_STANDBY) {
        ui_settings_t *s = settings_get();
        bool changed = false;
        if (buttons_consume_step(BTN_DOWN) && s->standbyTimeMin > SESSION_TIME_MIN_MIN) {
            s->standbyTimeMin -= SESSION_TIME_STEP_MIN;
            changed = true;
        }
        if (buttons_consume_step(BTN_UP) && s->standbyTimeMin < s->maxStandbyTimeMin) {
            s->standbyTimeMin += SESSION_TIME_STEP_MIN;
            changed = true;
        }
        if (changed) display_show_time_minutes(s->standbyTimeMin);
    }
}

static void handle_error(void)
{
    buttons_set_hold_times(MENU_SAVE_HOLD_MS, POWER_OFF_HOLD_MS);
    if (buttons_consume_short(BTN_LIGHT)) toggle_cabin_light();
    if (buttons_consume_long(BTN_POWER_MODE)) {
        power_off_to_off();
    }
}

void ui_state_init(void)
{
    errors_init();
    pc_comm_set_event_handler(ui_state_on_pc_event, NULL);
    s_main_state = UI_MAIN_STARTUP;
    s_mode = UI_MODE_SESSION;
    s_startup_step = START_REQUEST_PC_VERSION;
    s_state_started_ms = hardware_millis();
    s_last_version_request_ms = s_state_started_ms - VERSION_REQUEST_INTERVAL_MS;
    s_menu_feature = UI_MENU_FAN;
    display_show_text("BOOT");
}

void ui_state_update(uint32_t now_ms)
{
    errors_update(now_ms);

    switch (s_main_state) {
    case UI_MAIN_STARTUP:
        if (s_startup_step == START_REQUEST_PC_VERSION) {
            if ((uint32_t)(now_ms - s_last_version_request_ms) >= VERSION_REQUEST_INTERVAL_MS) {
                pc_comm_send_version_request();
                s_last_version_request_ms = now_ms;
            }
            if ((uint32_t)(now_ms - s_state_started_ms) >= VERSION_REPLY_TIMEOUT_MS && !s_pc_version_received) {
                enter_error(11);
            }
        } else if (s_startup_step == START_SHOW_PC_VERSION) {
            if ((uint32_t)(now_ms - s_state_started_ms) >= VERSION_DISPLAY_TIME_MS) {
                s_startup_step = START_SHOW_UI_VERSION;
                s_state_started_ms = now_ms;
                display_show_version(UI_VERSION_MAJOR, UI_VERSION_MINOR);
            }
        } else if (s_startup_step == START_SHOW_UI_VERSION) {
            if ((uint32_t)(now_ms - s_state_started_ms) >= VERSION_DISPLAY_TIME_MS) {
                pc_comm_send_dip_request();
                if (errors_code() == 13) enter_error(13);
                else enter_off();
            }
        }
        break;
    case UI_MAIN_OFF:
        handle_off(now_ms);
        break;
    case UI_MAIN_POWER_ON_PENDING:
        handle_pending(now_ms);
        break;
    case UI_MAIN_ON:
        handle_on(now_ms);
        break;
    case UI_MAIN_ERROR:
        handle_error();
        break;
    }
}

void ui_state_on_pc_event(const pc_event_t *event, void *ctx)
{
    (void)ctx;
    if (event == NULL) return;

    switch (event->type) {
    case PC_EVENT_COMM_TIMEOUT:
        enter_error(11);
        break;
    case PC_EVENT_VERSION_REPLY:
        if (event->len >= 2 && s_main_state == UI_MAIN_STARTUP) {
            s_pc_version_major = event->data[0];
            s_pc_version_minor = event->data[1];
            s_pc_version_received = true;
            s_startup_step = START_SHOW_PC_VERSION;
            s_state_started_ms = hardware_millis();
            display_show_version(s_pc_version_major, s_pc_version_minor);
        }
        break;
    case PC_EVENT_POWER_ON_ACK:
        if (event->len >= 1 && event->data[0] != 0 && s_main_state == UI_MAIN_POWER_ON_PENDING) {
            s_main_state = UI_MAIN_ON;
            s_mode = UI_MODE_SESSION;
            s_menu_active = false;
            display_show_temperature(settings_get()->targetTemperatureC);
            buzzer_beep(100);
        }
        break;
    case PC_EVENT_ERROR_REPORT:
        if (event->len >= 3 && event->data[1] == 1) {
            errors_set_slave(event->data[2], event->data[0], s_main_state != UI_MAIN_STARTUP);
            if (s_main_state != UI_MAIN_STARTUP) s_main_state = UI_MAIN_ERROR;
        } else if (event->len >= 1 && event->data[0] != 0) {
            errors_set(event->data[0], s_main_state != UI_MAIN_STARTUP && s_main_state != UI_MAIN_OFF);
            if (s_main_state == UI_MAIN_ON || s_main_state == UI_MAIN_POWER_ON_PENDING) enter_error(event->data[0]);
        }
        break;
    case PC_EVENT_HEARTBEAT_ACK:
        if (event->len >= 17) {
            uint8_t pc_mode = event->data[1];
            s_actual_temp_c = (int8_t)event->data[15];
            if (s_main_state == UI_MAIN_ON && !s_menu_active) {
                s_mode = pc_mode == 1 ? UI_MODE_STANDBY : UI_MODE_SESSION;
                show_normal_on_display();
            }
        } else if (event->len >= 10) {
            uint8_t pc_mode = event->data[1];
            (void)read_u32_le(&event->data[2]);
            (void)read_u32_le(&event->data[6]);
            if (s_main_state == UI_MAIN_ON) s_mode = pc_mode == 1 ? UI_MODE_STANDBY : UI_MODE_SESSION;
        }
        break;
    case PC_EVENT_DIP_REPLY:
        settings_apply_dip_reply(event->data, event->len);
        break;
    case PC_EVENT_SLAVE_STATUS_REPLY:
        if (event->len >= 3 && event->data[0] >= 1 && event->data[0] <= MAX_SLAVES) {
            s_service_status[event->data[0]] = event->data[1];
            s_service_error[event->data[0]] = event->data[2];
            if (s_service_active && event->data[0] == s_service_slave) {
                display_show_service_status(s_service_slave, s_service_status[s_service_slave], s_service_error[s_service_slave]);
            }
        }
        break;
    case PC_EVENT_CABIN_LIGHT_STATUS:
        if (event->len >= 2) s_cabin_light_on = event->data[1] != 0;
        break;
    case PC_EVENT_FAN_STATUS:
        if (event->len >= 1) settings_get()->fanOn = event->data[0] != 0;
        break;
    case PC_EVENT_SCENT_PUMP_STATUS:
        if (event->len >= 1) settings_get()->scentPumpSetpoint = event->data[0];
        break;
    default:
        break;
    }
}