#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icsc.h"

static const char *TAG = "ui_v3a";

/*
 * Porting source of truth:
 * - STN_UI_Arduino_Code.ino defines the stable UI behavior/state machine.
 * - STN_PC_Arduino_Code.ino defines the stable Power Controller ICSC protocol.
 *
 * Keep command IDs, payload lengths, payload byte order, retry behavior, and
 * error handling compatible with those two files unless the Power Controller
 * firmware is changed intentionally.
 */

#define UI_ADDRESS 1
#define POWER_CONTROLLER_ADDRESS 2
#define ICSC_BAUD 57600UL

/* UI-V3A PCB: U0TXD/U0RXD are routed; TXEN2 is ESP32-C5 IO8. */
#define ICSC_UART_NUM UART_NUM_0
#define ICSC_TX_PIN GPIO_NUM_NC
#define ICSC_RX_PIN GPIO_NUM_NC
#define PIN_TXEN GPIO_NUM_8

#define CMD_HEARTBEAT 'H'
#define CMD_HEARTBEAT_ACK 'h'
#define CMD_VERSION_REQUEST 'V'
#define CMD_VERSION_REPLY 'v'
#define CMD_POWER_ON 'P'
#define CMD_POWER_ON_ACK 'p'
#define CMD_POWER_OFF 'X'
#define CMD_CABIN_LIGHT_SETPOINT 'C'
#define CMD_CABIN_LIGHT_STATUS 'c'
#define CMD_ERROR_REPORT 'E'
#define CMD_DIP_REQUEST 'D'
#define CMD_DIP_REPLY 'd'
#define CMD_UI_SETTINGS 'U'
#define CMD_UI_SETTINGS_ACK 'u'
#define CMD_SET_OPERATION_MODE 'M'
#define CMD_DRAIN_FILL_START 'A'
#define CMD_DRAIN_FILL_STOP 'a'
#define CMD_SCENT_PUMP_SETPOINT 'N'
#define CMD_SCENT_PUMP_STATUS 'n'
#define CMD_FAN_CONTROL 'F'
#define CMD_FAN_STATUS 'f'

#define UI_VERSION_MAJOR 1
#define UI_VERSION_MINOR 8

#define HEARTBEAT_INTERVAL_MS 1000
#define HEARTBEAT_ACK_TIMEOUT_MS 10000
#define VERSION_REQUEST_INTERVAL_MS 1000
#define VERSION_REPLY_TIMEOUT_MS 5000
#define VERSION_DISPLAY_TIME_MS 1000
#define DIP_REQUEST_INTERVAL_MS 2000
#define POWER_COMMAND_REPEAT_MS 1000

#define TARGET_TEMP_MIN_C 30
#define TARGET_TEMP_DEFAULT_C 40
#define SESSION_TIME_STEP_MIN 5
#define SESSION_TIME_MIN_MIN 5
#define SESSION_TIME_24H_MIN 1440U
#define SESSION_TIME_UNLIMITED 0xFFFFU
#define SCENT_PUMP_MIN_SETPOINT 1
#define SCENT_PUMP_MAX_SETPOINT 20
#define CABIN_LIGHT_DEFAULT_PERCENT 100

#define BUTTON_SCAN_MS 10
#define BUTTON_DEBOUNCE_MS 25
#define BUTTON_LONG_PRESS_MS 2000
#define BUTTON_REPEAT_START_MS 600
#define BUTTON_REPEAT_INTERVAL_MS 150
#define DISPLAY_DIGITS 5
#define DISPLAY_REFRESH_US 500
#define SEG_ACTIVE_LOW 1
#define DIGIT_ACTIVE_HIGH 0
#define TEMP_BLINK_DURATION_MS 5000
#define TEMP_BLINK_INTERVAL_MS 500
#define ERROR_BLINK_INTERVAL_MS 500
#define SETTINGS_TIMEOUT_MS 5000
#define S4_FORCE_DRAIN_HOLD_MS 5000
#define COUNTDOWN_UNLIMITED_SECONDS 0xFFFFFFFFUL

/* UI-V3A backup C-F hardware mapping. */
#define PIN_SEG_A GPIO_NUM_7
#define PIN_SEG_B GPIO_NUM_6
#define PIN_SEG_C GPIO_NUM_1
#define PIN_SEG_D GPIO_NUM_0
#define PIN_SEG_E GPIO_NUM_3
#define PIN_SEG_F GPIO_NUM_2
#define PIN_SEG_G GPIO_NUM_9
#define PIN_SEG_DP GPIO_NUM_10
#define PIN_COL1 GPIO_NUM_5
#define PIN_COL2 GPIO_NUM_4
#define PIN_COL3 GPIO_NUM_26
#define PIN_COL4 GPIO_NUM_25
#define PIN_COL5 GPIO_NUM_24
#define PIN_BUZZER GPIO_NUM_14
#define PIN_SW3 GPIO_NUM_13
#define PIN_BLUE_LED GPIO_NUM_23
#define PIN_RESET_BTN GPIO_NUM_28

/* UI-V3A physical buttons mapped to STN-UI functions:
 * UI-V3A S1 = Increase  -> STN S7
 * UI-V3A S2 = Menu      -> STN S4
 * UI-V3A S3 = Light     -> STN S3
 * UI-V3A S4 = Power     -> STN S6
 * UI-V3A S5 = Decrease  -> STN S5
 */
#define PIN_BTN_UI_S1_INCREASE PIN_SEG_D /* GPIO0 */
#define PIN_BTN_UI_S2_MENU     PIN_SEG_B /* GPIO6 */
#define PIN_BTN_UI_S3_LIGHT    PIN_SW3   /* GPIO13 */
#define PIN_BTN_UI_S4_POWER    PIN_SEG_C /* GPIO1 */
#define PIN_BTN_UI_S5_DECREASE PIN_SEG_A /* GPIO7 */

#define BIT_A (1U << 0)
#define BIT_B (1U << 1)
#define BIT_C (1U << 2)
#define BIT_D (1U << 3)
#define BIT_E (1U << 4)
#define BIT_F (1U << 5)
#define BIT_G (1U << 6)
#define BIT_DP (1U << 7)

#define LED_SCENT BIT_A
#define LED_FAN BIT_B
#define LED_TEMP BIT_C
#define LED_CABIN_LIGHT BIT_D
#define LED_ON BIT_E
#define LED_SESSION BIT_F
#define LED_STANDBY BIT_G

typedef enum { BTN_LIGHT = 0, BTN_MENU, BTN_DECREASE, BTN_POWER, BTN_INCREASE, BTN_COUNT } button_id_t;
typedef enum { START_REQUEST_PC_VERSION = 0, START_SHOW_PC_VERSION, START_SHOW_UI_VERSION, START_OFF, START_FAIL } startup_state_t;
typedef enum { MAIN_STARTUP = 0, MAIN_OFF, MAIN_POWER_ON_PENDING, MAIN_ON } main_state_t;
typedef enum { MODE_SESSION = 0, MODE_STANDBY = 1 } operation_mode_t;
typedef enum { NORMAL_TEMP = 0, NORMAL_SESSION_TIME } normal_display_t;
typedef enum { EDIT_NONE = 0, EDIT_FAN, EDIT_TEMP, EDIT_SCENT, EDIT_SESSION_TIME, EDIT_CABIN_LIGHT, EDIT_STANDBY_TIME } edit_mode_t;

typedef struct {
    gpio_num_t pin;
    bool raw_pressed;
    bool stable_pressed;
    bool short_event;
    bool long_event;
    bool long_reported;
    bool hold5_reported;
    int64_t raw_changed_ms;
    int64_t pressed_ms;
    int64_t last_repeat_ms;
} button_t;

static const gpio_num_t s_segment_pins[8] = { PIN_SEG_A, PIN_SEG_B, PIN_SEG_C, PIN_SEG_D, PIN_SEG_E, PIN_SEG_F, PIN_SEG_G, PIN_SEG_DP };
static const gpio_num_t s_digit_pins[DISPLAY_DIGITS] = { PIN_COL1, PIN_COL2, PIN_COL3, PIN_COL4, PIN_COL5 };
static button_t s_buttons[BTN_COUNT] = {
    {.pin = PIN_BTN_UI_S3_LIGHT},
    {.pin = PIN_BTN_UI_S2_MENU},
    {.pin = PIN_BTN_UI_S5_DECREASE},
    {.pin = PIN_BTN_UI_S4_POWER},
    {.pin = PIN_BTN_UI_S1_INCREASE},
};

static icsc_t s_bus;
static volatile uint8_t s_display_buffer[DISPLAY_DIGITS];
static uint8_t s_indicator_mask;

static startup_state_t s_startup_state = START_REQUEST_PC_VERSION;
static main_state_t s_main_state = MAIN_STARTUP;
static operation_mode_t s_operation_mode = MODE_SESSION;
static normal_display_t s_normal_display = NORMAL_TEMP;
static edit_mode_t s_edit_mode = EDIT_NONE;

static bool s_pc_version_received;
static uint8_t s_pc_version_major;
static uint8_t s_pc_version_minor;
static bool s_heartbeat_ack_received;
static bool s_local_e11_active;
static bool s_dip_settings_received;

static int16_t s_temperature_setpoint_c = TARGET_TEMP_DEFAULT_C;
static int16_t s_max_target_temperature_c = 50;
static uint16_t s_selected_session_time_min = 30;
static uint16_t s_max_session_time_min_from_dip = 30;
static uint16_t s_selected_standby_time_min = 240;
static uint16_t s_max_standby_time_min_from_dip = 240;
static bool s_standby_disabled_by_dip;
static uint32_t s_remaining_session_sec = 30UL * 60UL;
static uint32_t s_remaining_standby_sec = 240UL * 60UL;

static bool s_cabin_light_on;
static uint8_t s_cabin_light_percent = CABIN_LIGHT_DEFAULT_PERCENT;
static bool s_fan_requested_on;
static bool s_scent_pump_requested_on;
static uint8_t s_scent_pump_setpoint = 1;
static uint8_t s_active_error_code;
static bool s_error_display_active;
static bool s_error_blink_visible = true;
static uint8_t s_pc_drain_state;
static uint32_t s_remaining_drain_sec = 10UL * 60UL;
static bool s_show_temp_blinking;
static bool s_temp_blink_visible = true;

static int64_t s_startup_state_start_ms;
static int64_t s_last_heartbeat_send_ms;
static int64_t s_last_heartbeat_ack_ms;
static int64_t s_last_version_request_ms;
static int64_t s_last_dip_request_ms;
static int64_t s_last_power_command_ms;
static int64_t s_last_local_countdown_ms;
static int64_t s_last_error_blink_ms;
static int64_t s_last_temp_blink_ms;
static int64_t s_temp_blink_start_ms;
static int64_t s_last_edit_action_ms;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000LL; }

static void set_segment_pin(gpio_num_t pin, bool on)
{
#if SEG_ACTIVE_LOW
    gpio_set_level(pin, on ? 0 : 1);
#else
    gpio_set_level(pin, on ? 1 : 0);
#endif
}

static void set_digit_pin(gpio_num_t pin, bool on)
{
#if DIGIT_ACTIVE_HIGH
    gpio_set_level(pin, on ? 1 : 0);
#else
    gpio_set_level(pin, on ? 0 : 1);
#endif
}

static void all_digits_off(void)
{
    for (int i = 0; i < DISPLAY_DIGITS; i++) set_digit_pin(s_digit_pins[i], false);
}

static void all_segments_off(void)
{
    for (int i = 0; i < 8; i++) set_segment_pin(s_segment_pins[i], false);
}

static void configure_segment_outputs(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < 8; i++) mask |= BIT64(s_segment_pins[i]);
    const gpio_config_t cfg = {.pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&cfg);
}

static void configure_button_inputs_temporarily(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < BTN_COUNT; i++) mask |= BIT64(s_buttons[i].pin);
    const gpio_config_t cfg = {.pin_bit_mask = mask, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&cfg);
}

static uint8_t glyph_for_char(char c)
{
    switch (c) {
    case '0': return BIT_A | BIT_B | BIT_C | BIT_D | BIT_E | BIT_F;
    case '1': return BIT_B | BIT_C;
    case '2': return BIT_A | BIT_B | BIT_D | BIT_E | BIT_G;
    case '3': return BIT_A | BIT_B | BIT_C | BIT_D | BIT_G;
    case '4': return BIT_B | BIT_C | BIT_F | BIT_G;
    case '5': return BIT_A | BIT_C | BIT_D | BIT_F | BIT_G;
    case '6': return BIT_A | BIT_C | BIT_D | BIT_E | BIT_F | BIT_G;
    case '7': return BIT_A | BIT_B | BIT_C;
    case '8': return BIT_A | BIT_B | BIT_C | BIT_D | BIT_E | BIT_F | BIT_G;
    case '9': return BIT_A | BIT_B | BIT_C | BIT_D | BIT_F | BIT_G;
    case 'A': return BIT_A | BIT_B | BIT_C | BIT_E | BIT_F | BIT_G;
    case 'b': return BIT_C | BIT_D | BIT_E | BIT_F | BIT_G;
    case 'C': return BIT_A | BIT_D | BIT_E | BIT_F;
    case 'd': return BIT_B | BIT_C | BIT_D | BIT_E | BIT_G;
    case 'E': return BIT_A | BIT_D | BIT_E | BIT_F | BIT_G;
    case 'F': return BIT_A | BIT_E | BIT_F | BIT_G;
    case 'H': return BIT_B | BIT_C | BIT_E | BIT_F | BIT_G;
    case 'I': return BIT_B | BIT_C;
    case 'L': return BIT_D | BIT_E | BIT_F;
    case 'O': return BIT_A | BIT_B | BIT_C | BIT_D | BIT_E | BIT_F;
    case 'P': return BIT_A | BIT_B | BIT_E | BIT_F | BIT_G;
    case 'S': return BIT_A | BIT_C | BIT_D | BIT_F | BIT_G;
    case 'Y': return BIT_B | BIT_C | BIT_D | BIT_F | BIT_G;
    case 'n': return BIT_C | BIT_E | BIT_G;
    case 'o': return BIT_C | BIT_D | BIT_E | BIT_G;
    case 'r': return BIT_E | BIT_G;
    case 't': return BIT_D | BIT_E | BIT_F | BIT_G;
    case '-': return BIT_G;
    case ' ': return 0;
    default: return 0;
    }
}

static void display_set_raw(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    s_display_buffer[0] = d0; s_display_buffer[1] = d1; s_display_buffer[2] = d2; s_display_buffer[3] = d3; s_display_buffer[4] = d4;
}

static void display_set_chars(char c0, char c1, char c2, char c3, char c4)
{
    display_set_raw(glyph_for_char(c0), glyph_for_char(c1), glyph_for_char(c2), glyph_for_char(c3), glyph_for_char(c4));
}

static void display_set_chars_dp(char c0, char c1, char c2, char c3, char c4, uint8_t dp_mask)
{
    uint8_t b[DISPLAY_DIGITS] = {glyph_for_char(c0), glyph_for_char(c1), glyph_for_char(c2), glyph_for_char(c3), glyph_for_char(c4)};
    for (int i = 0; i < DISPLAY_DIGITS; i++) if (dp_mask & (1U << i)) b[i] |= BIT_DP;
    display_set_raw(b[0], b[1], b[2], b[3], b[4]);
}

static char version_digit(uint8_t value)
{
    return (char)('0' + (value <= 9 ? value : 9));
}

static void display_pc_version(uint8_t major, uint8_t minor)
{
    display_set_chars(' ', ' ', version_digit(major), version_digit(minor), ' ');
    s_indicator_mask = 0;
}

static void display_ui_version(uint8_t major, uint8_t minor)
{
    display_set_chars(version_digit(major), version_digit(minor), ' ', ' ', ' ');
    s_indicator_mask = 0;
}

static void display_colon_only(void)
{
    display_set_raw(0, 0, BIT_DP, BIT_DP, 0);
    s_indicator_mask = 0;
}

static void display_number_2digit(int value)
{
    if (value < 0) value = 0;
    if (value > 99) value = 99;
    display_set_chars(' ', ' ', (char)('0' + value / 10), (char)('0' + value % 10), ' ');
}

static void display_error_code(uint8_t error)
{
    if (error > 99) error = 99;
    display_set_chars(' ', ' ', 'E', error >= 10 ? (char)('0' + error / 10) : ' ', (char)('0' + error % 10));
}

static void display_time_minutes(uint16_t minutes)
{
    if (minutes == SESSION_TIME_UNLIMITED || minutes > SESSION_TIME_24H_MIN) minutes = SESSION_TIME_24H_MIN;
    uint16_t hours = minutes / 60;
    uint16_t mins = minutes % 60;
    display_set_chars_dp(hours >= 10 ? (char)('0' + hours / 10) : ' ', (char)('0' + hours % 10), (char)('0' + mins / 10), (char)('0' + mins % 10), ' ', (1U << 1));
}

static void display_off(void) { display_colon_only(); }
static void display_standby(void) { display_set_chars('S', 't', 'b', 'Y', ' '); }
static void display_on_text(void) { display_set_chars(' ', 'O', 'n', ' ', ' '); }
static void display_of_text(void) { display_set_chars(' ', 'O', 'F', ' ', ' '); }

static void set_indicators_for_state(void)
{
    if (s_startup_state != START_OFF || s_main_state != MAIN_ON) {
        s_indicator_mask = 0;
        return;
    }

    uint8_t mask = LED_ON;
    mask |= s_operation_mode == MODE_STANDBY ? LED_STANDBY : LED_SESSION;
    if (s_normal_display == NORMAL_TEMP || s_edit_mode == EDIT_TEMP) mask |= LED_TEMP;
    if (s_cabin_light_on) mask |= LED_CABIN_LIGHT;
    if (s_fan_requested_on) mask |= LED_FAN;
    if (s_scent_pump_requested_on) mask |= LED_SCENT;
    s_indicator_mask = mask;
}

static void display_current_normal(void)
{
    if (s_main_state != MAIN_ON || s_active_error_code != 0 || s_edit_mode != EDIT_NONE || s_pc_drain_state != 0) return;
    if (s_operation_mode == MODE_STANDBY) display_standby();
    else if (s_normal_display == NORMAL_SESSION_TIME) display_time_minutes((uint16_t)((s_remaining_session_sec + 59UL) / 60UL));
    else display_number_2digit(s_temperature_setpoint_c);
}

static void buzzer_on(void) { gpio_set_level(PIN_BUZZER, 1); }
static void buzzer_off(void) { gpio_set_level(PIN_BUZZER, 0); }
static void beep_ms(uint32_t ms) { buzzer_on(); vTaskDelay(pdMS_TO_TICKS(ms)); buzzer_off(); }

static void refresh_one_digit(int index)
{
    uint8_t mask = index == 4 ? s_indicator_mask : s_display_buffer[index];
    all_digits_off();
    for (int s = 0; s < 8; s++) set_segment_pin(s_segment_pins[s], (mask & (1U << s)) != 0);
    set_digit_pin(s_digit_pins[index], true);
}

static void update_button_debounce(button_id_t id, bool raw_pressed)
{
    button_t *b = &s_buttons[id];
    int64_t now = now_ms();
    if (raw_pressed != b->raw_pressed) { b->raw_pressed = raw_pressed; b->raw_changed_ms = now; }
    if ((now - b->raw_changed_ms) >= BUTTON_DEBOUNCE_MS && b->stable_pressed != b->raw_pressed) {
        b->stable_pressed = b->raw_pressed;
        if (b->stable_pressed) { b->pressed_ms = now; b->last_repeat_ms = now; b->long_reported = false; b->hold5_reported = false; }
        else if (!b->long_reported) b->short_event = true;
    }
    if (b->stable_pressed && !b->long_reported && (now - b->pressed_ms) >= BUTTON_LONG_PRESS_MS) {
        b->long_reported = true; b->long_event = true;
    }
}

static void scan_buttons(void)
{
    all_digits_off(); all_segments_off(); configure_button_inputs_temporarily(); esp_rom_delay_us(80);
    for (button_id_t i = 0; i < BTN_COUNT; i++) update_button_debounce(i, gpio_get_level(s_buttons[i].pin) == 0);
    configure_segment_outputs();
}

static bool consume_short(button_id_t id) { if (s_buttons[id].short_event) { s_buttons[id].short_event = false; return true; } return false; }
static bool consume_long(button_id_t id) { if (s_buttons[id].long_event) { s_buttons[id].long_event = false; return true; } return false; }

static bool consume_hold_ms(button_id_t id, int64_t hold_ms)
{
    button_t *b = &s_buttons[id];
    if (!b->stable_pressed || b->hold5_reported) return false;
    if ((now_ms() - b->pressed_ms) < hold_ms) return false;
    b->hold5_reported = true;
    b->long_event = false;
    return true;
}

static bool consume_repeat(button_id_t id)
{
    button_t *b = &s_buttons[id];
    int64_t now = now_ms();
    if (!b->stable_pressed || (now - b->pressed_ms) < BUTTON_REPEAT_START_MS) return false;
    if ((now - b->last_repeat_ms) >= BUTTON_REPEAT_INTERVAL_MS) { b->last_repeat_ms = now; return true; }
    return false;
}

static void send_heartbeat(void) { char p[1] = {1}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_HEARTBEAT, sizeof(p), p); }
static void send_version_request(void) { char p[1] = {1}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_VERSION_REQUEST, sizeof(p), p); }
static void send_dip_request(void) { char p[1] = {1}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_DIP_REQUEST, sizeof(p), p); }

static void send_ui_settings(void)
{
    uint16_t session_to_send = s_selected_session_time_min;
    uint16_t standby_to_send = s_selected_standby_time_min;

    if (s_max_session_time_min_from_dip == SESSION_TIME_UNLIMITED && s_selected_session_time_min >= SESSION_TIME_24H_MIN) {
        session_to_send = SESSION_TIME_UNLIMITED;
    }

    if (s_standby_disabled_by_dip) {
        standby_to_send = 0;
    } else if (s_max_standby_time_min_from_dip == SESSION_TIME_UNLIMITED && s_selected_standby_time_min >= SESSION_TIME_24H_MIN) {
        standby_to_send = SESSION_TIME_UNLIMITED;
    }

    char p[5];
    p[0] = (char)s_temperature_setpoint_c;
    p[1] = (char)(session_to_send & 0xFF);
    p[2] = (char)((session_to_send >> 8) & 0xFF);
    p[3] = (char)(standby_to_send & 0xFF);
    p[4] = (char)((standby_to_send >> 8) & 0xFF);
    icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_UI_SETTINGS, sizeof(p), p);
}

static void send_power_on(void) { char p[1] = {1}; send_ui_settings(); icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_POWER_ON, sizeof(p), p); s_last_power_command_ms = now_ms(); }
static void send_power_off(void) { char p[1] = {0}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_POWER_OFF, sizeof(p), p); }
static void send_operation_mode(void) { char p[1] = {(char)s_operation_mode}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_SET_OPERATION_MODE, sizeof(p), p); }
static void send_cabin_light(void) { char p[1] = {s_cabin_light_on ? (char)s_cabin_light_percent : 0}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_CABIN_LIGHT_SETPOINT, sizeof(p), p); }
static void send_fan(void) { char p[1] = {s_fan_requested_on ? 1 : 0}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_FAN_CONTROL, sizeof(p), p); }
static void send_scent_pump(void) { char p[1] = {(char)s_scent_pump_setpoint}; icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_SCENT_PUMP_SETPOINT, sizeof(p), p); }

static void activate_error_display(uint8_t code) { s_active_error_code = code; s_error_display_active = true; s_error_blink_visible = true; s_last_error_blink_ms = now_ms(); display_error_code(code); }
static void clear_error_display(void) { s_active_error_code = 0; s_error_display_active = false; display_current_normal(); }
static void enter_off_state(void);

static uint32_t read_u32_le(const char *data, uint8_t index)
{
    return ((uint32_t)(uint8_t)data[index + 0]) |
           ((uint32_t)(uint8_t)data[index + 1] << 8) |
           ((uint32_t)(uint8_t)data[index + 2] << 16) |
           ((uint32_t)(uint8_t)data[index + 3] << 24);
}

static void on_heartbeat_from_pc(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    (void)len;
    (void)data;
    if (station != POWER_CONTROLLER_ADDRESS) return;

    /* Legacy compatibility: receiving PC heartbeat proves communication is alive.
       STN v26+ uses UI-master heartbeat, so do not auto-reply and create ping-pong traffic. */
    s_heartbeat_ack_received = true;
    s_last_heartbeat_ack_ms = now_ms();

    if (s_local_e11_active && s_active_error_code == 11 && !s_error_display_active) {
        s_local_e11_active = false;
        s_active_error_code = 0;
    }
}

static void on_heartbeat_ack(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station != POWER_CONTROLLER_ADDRESS) {
        return;
    }

    s_heartbeat_ack_received = true;
    s_last_heartbeat_ack_ms = now_ms();

    if (s_local_e11_active && s_active_error_code == 11 && !s_error_display_active) {
        s_local_e11_active = false;
        clear_error_display();
    }

    if (s_startup_state != START_OFF) {
        return;
    }

    if (len >= 10 && s_active_error_code == 0) {
        uint8_t pc_state = (uint8_t)data[0];
        uint8_t pc_mode = (uint8_t)data[1];
        s_remaining_session_sec = read_u32_le(data, 2);
        s_remaining_standby_sec = read_u32_le(data, 6);

        if (len >= 15) {
            s_pc_drain_state = (uint8_t)data[10];
            s_remaining_drain_sec = read_u32_le(data, 11);
        } else {
            s_pc_drain_state = 0;
            s_remaining_drain_sec = 0;
        }

        if (pc_state == 0 && s_main_state == MAIN_ON) {
            enter_off_state();
            return;
        }

        if (pc_state == 1 && s_main_state == MAIN_POWER_ON_PENDING) {
            s_main_state = MAIN_ON;
        }

        if (s_main_state == MAIN_ON) {
            s_operation_mode = pc_mode == 1 ? MODE_STANDBY : MODE_SESSION;
            if (s_pc_drain_state == 1 || s_pc_drain_state == 2) {
                display_set_chars(' ', ' ', 'd', 'r', ' ');
            } else {
                display_current_normal();
            }
        }
    }
}
static void on_version_reply(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station == POWER_CONTROLLER_ADDRESS && len >= 2) {
        s_pc_version_major = (uint8_t)data[0];
        s_pc_version_minor = (uint8_t)data[1];
        s_pc_version_received = true;
        if (s_startup_state == START_REQUEST_PC_VERSION) {
            s_startup_state = START_SHOW_PC_VERSION;
            s_startup_state_start_ms = now_ms();
            display_pc_version(s_pc_version_major, s_pc_version_minor);
        }
    }
}

static void on_power_on_ack(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station != POWER_CONTROLLER_ADDRESS || len < 1) return;

    if (data[0] != 0) {
        if (s_active_error_code != 0) {
            activate_error_display(s_active_error_code);
            return;
        }
        s_main_state = MAIN_ON;
        s_operation_mode = MODE_SESSION;
        s_normal_display = NORMAL_TEMP;
        s_remaining_session_sec = (uint32_t)s_selected_session_time_min * 60UL;
        s_remaining_standby_sec = (uint32_t)s_selected_standby_time_min * 60UL;
        s_show_temp_blinking = true;
        s_temp_blink_visible = true;
        s_temp_blink_start_ms = now_ms();
        s_last_temp_blink_ms = s_temp_blink_start_ms;
        set_indicators_for_state();
        display_number_2digit(s_temperature_setpoint_c);
        beep_ms(60);
    } else if (s_active_error_code != 0) {
        s_main_state = MAIN_OFF;
        activate_error_display(s_active_error_code);
    } else {
        s_main_state = MAIN_POWER_ON_PENDING;
        display_set_chars(' ', ' ', ' ', ' ', ' ');
    }
}
static void on_error_report(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station != POWER_CONTROLLER_ADDRESS || len < 1) return;

    uint8_t code = (uint8_t)data[0];
    if (code == 0) {
        clear_error_display();
        return;
    }
    if (code > 13) code = 13;

    s_active_error_code = code;
    if (s_startup_state == START_OFF &&
        (s_main_state == MAIN_ON || s_main_state == MAIN_POWER_ON_PENDING || s_error_display_active || code == 13)) {
        activate_error_display(code);
    }
}
static void on_dip_reply(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station != POWER_CONTROLLER_ADDRESS || len < 11) return;
    s_max_session_time_min_from_dip = (uint16_t)(uint8_t)data[1] | ((uint16_t)(uint8_t)data[2] << 8);
    s_max_standby_time_min_from_dip = (uint16_t)(uint8_t)data[3] | ((uint16_t)(uint8_t)data[4] << 8);
    s_standby_disabled_by_dip = data[5] != 0;
    s_max_target_temperature_c = (uint8_t)data[10];
    if (s_max_target_temperature_c < TARGET_TEMP_MIN_C) s_max_target_temperature_c = 50;
    if (s_temperature_setpoint_c > s_max_target_temperature_c) s_temperature_setpoint_c = s_max_target_temperature_c;
    if (s_selected_session_time_min > s_max_session_time_min_from_dip) s_selected_session_time_min = s_max_session_time_min_from_dip;
    if (s_selected_standby_time_min > s_max_standby_time_min_from_dip) s_selected_standby_time_min = s_max_standby_time_min_from_dip;
    s_dip_settings_received = true;
}

static void on_cabin_light_status(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station != POWER_CONTROLLER_ADDRESS || len < 4) return;

    uint8_t level = (uint8_t)data[2];
    bool actual_on = data[3] != 0;
    s_cabin_light_on = actual_on && level != 0;
    if (level != 0) s_cabin_light_percent = level;
    set_indicators_for_state();
}

static void on_fan_status(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station != POWER_CONTROLLER_ADDRESS || len < 3) return;

    bool requested = data[0] != 0;
    bool dip_enabled = data[1] != 0;
    s_fan_requested_on = requested && dip_enabled;
    set_indicators_for_state();
}

static void on_scent_status(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    if (station != POWER_CONTROLLER_ADDRESS || len < 4) return;

    uint8_t setpoint = (uint8_t)data[0];
    bool dip_enabled = data[1] != 0;
    if (setpoint > SCENT_PUMP_MAX_SETPOINT) setpoint = SCENT_PUMP_MAX_SETPOINT;
    s_scent_pump_setpoint = setpoint;
    s_scent_pump_requested_on = (setpoint > 0) && dip_enabled;
    set_indicators_for_state();
}

static void register_icsc_callbacks(void)
{
    icsc_register_command(&s_bus, CMD_HEARTBEAT, on_heartbeat_from_pc);
    icsc_register_command(&s_bus, CMD_HEARTBEAT_ACK, on_heartbeat_ack);
    icsc_register_command(&s_bus, CMD_VERSION_REPLY, on_version_reply);
    icsc_register_command(&s_bus, CMD_POWER_ON_ACK, on_power_on_ack);
    icsc_register_command(&s_bus, CMD_ERROR_REPORT, on_error_report);
    icsc_register_command(&s_bus, CMD_CABIN_LIGHT_STATUS, on_cabin_light_status);
    icsc_register_command(&s_bus, CMD_DIP_REPLY, on_dip_reply);
    icsc_register_command(&s_bus, CMD_FAN_STATUS, on_fan_status);
    icsc_register_command(&s_bus, CMD_SCENT_PUMP_STATUS, on_scent_status);
}

static void enter_off_state(void)
{
    s_main_state = MAIN_OFF; s_edit_mode = EDIT_NONE; s_pc_drain_state = 0; s_show_temp_blinking = false; s_error_display_active = false; set_indicators_for_state(); display_off();
}

static void update_startup_sequence(void)
{
    int64_t now = now_ms();
    switch (s_startup_state) {
    case START_REQUEST_PC_VERSION:
        if ((now - s_last_version_request_ms) >= VERSION_REQUEST_INTERVAL_MS) {
            s_last_version_request_ms = now;
            send_version_request();
        }
        if (s_pc_version_received) {
            s_startup_state = START_SHOW_PC_VERSION;
            s_startup_state_start_ms = now;
            display_pc_version(s_pc_version_major, s_pc_version_minor);
        } else if ((now - s_startup_state_start_ms) >= HEARTBEAT_ACK_TIMEOUT_MS) {
            s_startup_state = START_OFF;
            s_main_state = MAIN_OFF;
            s_local_e11_active = true;
            activate_error_display(11);
        }
        break;
    case START_SHOW_PC_VERSION:
        if ((now - s_startup_state_start_ms) >= VERSION_DISPLAY_TIME_MS) {
            s_startup_state = START_SHOW_UI_VERSION;
            s_startup_state_start_ms = now;
            display_ui_version(UI_VERSION_MAJOR, UI_VERSION_MINOR);
        }
        break;
    case START_SHOW_UI_VERSION:
        if ((now - s_startup_state_start_ms) >= VERSION_DISPLAY_TIME_MS) {
            s_startup_state = START_OFF;
            enter_off_state();
            send_dip_request();
            beep_ms(100);
        }
        break;
    case START_FAIL:
        display_set_chars('F', 'A', 'I', 'L', ' ');
        s_indicator_mask = 0;
        break;
    default: break;
    }
}

static void update_heartbeat(void)
{
    int64_t now = now_ms();
    if ((now - s_last_heartbeat_send_ms) >= HEARTBEAT_INTERVAL_MS) { s_last_heartbeat_send_ms = now; send_heartbeat(); }
    if (s_heartbeat_ack_received && (now - s_last_heartbeat_ack_ms) >= HEARTBEAT_ACK_TIMEOUT_MS) { s_local_e11_active = true; activate_error_display(11); }
}

static void update_dip_request(void)
{
    if (s_startup_state != START_OFF) return;
    int64_t now = now_ms();
    if (!s_dip_settings_received || (now - s_last_dip_request_ms) >= DIP_REQUEST_INTERVAL_MS) { s_last_dip_request_ms = now; send_dip_request(); }
}

static void update_local_countdown(void)
{
    int64_t now = now_ms();
    if ((now - s_last_local_countdown_ms) < 1000) return;
    uint32_t elapsed = (uint32_t)((now - s_last_local_countdown_ms) / 1000); s_last_local_countdown_ms += (int64_t)elapsed * 1000;
    if (s_main_state != MAIN_ON || s_active_error_code != 0) return;
    if (s_operation_mode == MODE_SESSION && s_remaining_session_sec != COUNTDOWN_UNLIMITED_SECONDS) s_remaining_session_sec = s_remaining_session_sec > elapsed ? s_remaining_session_sec - elapsed : 0;
    else if (s_operation_mode == MODE_STANDBY && s_remaining_standby_sec != COUNTDOWN_UNLIMITED_SECONDS) s_remaining_standby_sec = s_remaining_standby_sec > elapsed ? s_remaining_standby_sec - elapsed : 0;
    if (s_pc_drain_state != 0) s_remaining_drain_sec = s_remaining_drain_sec > elapsed ? s_remaining_drain_sec - elapsed : 0;
    else display_current_normal();
}

static void update_error_display(void)
{
    if (s_active_error_code == 0 || !s_error_display_active) return;
    int64_t now = now_ms();
    if ((now - s_last_error_blink_ms) >= ERROR_BLINK_INTERVAL_MS) { s_last_error_blink_ms = now; s_error_blink_visible = !s_error_blink_visible; }
    if (s_error_blink_visible) { display_error_code(s_active_error_code); buzzer_on(); }
    else { display_set_chars(' ', ' ', ' ', ' ', ' '); buzzer_off(); }
}

static void update_temperature_blink(void)
{
    if (!s_show_temp_blinking || s_main_state != MAIN_ON || s_error_display_active || s_edit_mode != EDIT_NONE) return;
    int64_t now = now_ms();
    if ((now - s_temp_blink_start_ms) >= TEMP_BLINK_DURATION_MS) { s_show_temp_blinking = false; s_temp_blink_visible = true; display_current_normal(); return; }
    if ((now - s_last_temp_blink_ms) >= TEMP_BLINK_INTERVAL_MS) { s_last_temp_blink_ms = now; s_temp_blink_visible = !s_temp_blink_visible; if (s_temp_blink_visible) display_number_2digit(s_temperature_setpoint_c); else display_set_chars(' ', ' ', ' ', ' ', ' '); }
}

static void update_power_on_pending(void)
{
    if (s_main_state != MAIN_POWER_ON_PENDING) return;
    display_set_chars(' ', ' ', ' ', ' ', ' ');
    if ((now_ms() - s_last_power_command_ms) >= POWER_COMMAND_REPEAT_MS) send_power_on();
}

static void finish_edit_mode(bool save)
{
    edit_mode_t old = s_edit_mode;
    s_edit_mode = EDIT_NONE;
    if (save) {
        if (old == EDIT_SCENT) { s_scent_pump_requested_on = s_scent_pump_setpoint > 0; send_scent_pump(); }
        else if (old == EDIT_CABIN_LIGHT) send_cabin_light();
        else if (old == EDIT_FAN) send_fan();
        else send_ui_settings();
        beep_ms(120);
    }
    set_indicators_for_state();
    display_current_normal();
}

static void enter_edit_mode(edit_mode_t mode)
{
    s_edit_mode = mode;
    s_last_edit_action_ms = now_ms();
    if (mode == EDIT_FAN) {
        if (s_fan_requested_on) display_on_text(); else display_of_text();
    } else if (mode == EDIT_TEMP) display_number_2digit(s_temperature_setpoint_c);
    else if (mode == EDIT_SESSION_TIME) display_time_minutes(s_selected_session_time_min);
    else if (mode == EDIT_STANDBY_TIME) display_time_minutes(s_selected_standby_time_min);
    else if (mode == EDIT_SCENT) display_number_2digit(s_scent_pump_setpoint);
    else if (mode == EDIT_CABIN_LIGHT) display_number_2digit(s_cabin_light_percent);
    set_indicators_for_state();
}

static void adjust_current_edit(int delta)
{
    if (s_edit_mode == EDIT_NONE) return;
    s_last_edit_action_ms = now_ms();
    switch (s_edit_mode) {
    case EDIT_FAN:
        s_fan_requested_on = delta > 0;
        send_fan();
        if (s_fan_requested_on) display_on_text(); else display_of_text();
        set_indicators_for_state();
        break;
    case EDIT_TEMP:
        s_temperature_setpoint_c += delta; if (s_temperature_setpoint_c < TARGET_TEMP_MIN_C) s_temperature_setpoint_c = TARGET_TEMP_MIN_C; if (s_temperature_setpoint_c > s_max_target_temperature_c) s_temperature_setpoint_c = s_max_target_temperature_c; display_number_2digit(s_temperature_setpoint_c); break;
    case EDIT_SESSION_TIME:
        if (delta > 0) s_selected_session_time_min += SESSION_TIME_STEP_MIN; else if (s_selected_session_time_min > SESSION_TIME_MIN_MIN) s_selected_session_time_min -= SESSION_TIME_STEP_MIN; if (s_selected_session_time_min < SESSION_TIME_MIN_MIN) s_selected_session_time_min = SESSION_TIME_MIN_MIN; if (s_selected_session_time_min > s_max_session_time_min_from_dip) s_selected_session_time_min = s_max_session_time_min_from_dip; display_time_minutes(s_selected_session_time_min); break;
    case EDIT_STANDBY_TIME:
        if (delta > 0) s_selected_standby_time_min += SESSION_TIME_STEP_MIN; else if (s_selected_standby_time_min > SESSION_TIME_MIN_MIN) s_selected_standby_time_min -= SESSION_TIME_STEP_MIN; if (s_selected_standby_time_min < SESSION_TIME_MIN_MIN) s_selected_standby_time_min = SESSION_TIME_MIN_MIN; if (s_selected_standby_time_min > s_max_standby_time_min_from_dip) s_selected_standby_time_min = s_max_standby_time_min_from_dip; display_time_minutes(s_selected_standby_time_min); break;
    case EDIT_SCENT:
        if (delta > 0 && s_scent_pump_setpoint < SCENT_PUMP_MAX_SETPOINT) s_scent_pump_setpoint++; else if (delta < 0 && s_scent_pump_setpoint > SCENT_PUMP_MIN_SETPOINT) s_scent_pump_setpoint--; display_number_2digit(s_scent_pump_setpoint); break;
    case EDIT_CABIN_LIGHT:
        if (delta > 0 && s_cabin_light_percent < 100) s_cabin_light_percent += 5; else if (delta < 0 && s_cabin_light_percent >= 5) s_cabin_light_percent -= 5; if (s_cabin_light_percent > 100) s_cabin_light_percent = 100; display_number_2digit(s_cabin_light_percent); break;
    default: break;
    }
}

static void update_edit_timeout(void)
{
    if (s_edit_mode != EDIT_NONE && (now_ms() - s_last_edit_action_ms) >= SETTINGS_TIMEOUT_MS) finish_edit_mode(true);
}

static edit_mode_t next_menu_edit_mode(edit_mode_t current)
{
    switch (current) {
    case EDIT_FAN: return EDIT_TEMP;
    case EDIT_TEMP: return EDIT_SCENT;
    case EDIT_SCENT: return EDIT_SESSION_TIME;
    case EDIT_SESSION_TIME: return EDIT_FAN;
    default: return EDIT_FAN;
    }
}

static void start_force_drain(void)
{
    s_pc_drain_state = 1;
    s_remaining_drain_sec = 10UL * 60UL;
    display_set_chars(' ', ' ', 'd', 'r', ' ');
    s_indicator_mask = 0;
    char p[1] = {1};
    icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_DRAIN_FILL_START, sizeof(p), p);
    beep_ms(120);
}

static void handle_buttons(void)
{
    bool down = consume_short(BTN_DECREASE) || consume_repeat(BTN_DECREASE);
    bool up = consume_short(BTN_INCREASE) || consume_repeat(BTN_INCREASE);

    if ((down || up) && s_main_state == MAIN_ON && s_active_error_code == 0) {
        if (s_edit_mode != EDIT_NONE) {
            adjust_current_edit(up ? 1 : -1);
        } else if (s_operation_mode == MODE_STANDBY && !s_standby_disabled_by_dip) {
            enter_edit_mode(EDIT_STANDBY_TIME);
            adjust_current_edit(up ? 1 : -1);
        }
    }

    if (consume_short(BTN_LIGHT) && s_startup_state == START_OFF && s_active_error_code == 0) {
        s_cabin_light_on = !s_cabin_light_on;
        if (s_cabin_light_on && s_cabin_light_percent == 0) s_cabin_light_percent = CABIN_LIGHT_DEFAULT_PERCENT;
        send_cabin_light();
        set_indicators_for_state();
        if (s_main_state == MAIN_ON) display_number_2digit(s_cabin_light_on ? s_cabin_light_percent : 0);
        beep_ms(60);
    }
    (void)consume_long(BTN_LIGHT);

    if (s_main_state == MAIN_ON && s_active_error_code == 0 && s_operation_mode == MODE_STANDBY) {
        if (consume_hold_ms(BTN_MENU, S4_FORCE_DRAIN_HOLD_MS)) {
            start_force_drain();
        }
        if (s_buttons[BTN_MENU].long_event) s_buttons[BTN_MENU].long_event = false;
        if (consume_short(BTN_MENU)) { }
    } else {
        if (consume_short(BTN_MENU) && s_main_state == MAIN_ON && s_active_error_code == 0 && s_operation_mode == MODE_SESSION) {
            enter_edit_mode(next_menu_edit_mode(s_edit_mode));
        }
        if (consume_long(BTN_MENU) && s_main_state == MAIN_ON && s_active_error_code == 0 && s_operation_mode == MODE_SESSION && s_edit_mode != EDIT_NONE) {
            finish_edit_mode(true);
        }
    }

    if (consume_short(BTN_POWER)) {
        if (s_error_display_active && s_active_error_code != 0) {
            return;
        }
        if (s_active_error_code != 0 && s_main_state == MAIN_OFF) {
            send_power_on();
            activate_error_display(s_active_error_code);
            return;
        }
        if (s_main_state == MAIN_OFF) {
            s_main_state = MAIN_POWER_ON_PENDING;
            display_set_chars(' ', ' ', ' ', ' ', ' ');
            s_indicator_mask = 0;
            send_power_on();
            return;
        }
        if (s_main_state == MAIN_ON && s_edit_mode == EDIT_NONE && s_active_error_code == 0) {
            if (s_operation_mode == MODE_STANDBY) {
                s_operation_mode = MODE_SESSION;
                s_normal_display = NORMAL_TEMP;
                display_current_normal();
            } else if (!s_standby_disabled_by_dip) {
                s_operation_mode = MODE_STANDBY;
                display_standby();
            }
            send_operation_mode();
            set_indicators_for_state();
        }
    }

    if (consume_long(BTN_POWER)) {
        if (s_main_state == MAIN_ON || s_main_state == MAIN_POWER_ON_PENDING || s_error_display_active) {
            send_power_off();
            s_active_error_code = 0;
            s_error_display_active = false;
            enter_off_state();
            beep_ms(120);
        }
    }
}

static void display_task(void *arg)
{
    (void)arg;
    int digit = 0;
    int64_t last_scan = 0;
    while (true) { refresh_one_digit(digit); digit = (digit + 1) % DISPLAY_DIGITS; int64_t now = now_ms(); if ((now - last_scan) >= BUTTON_SCAN_MS) { last_scan = now; scan_buttons(); } esp_rom_delay_us(DISPLAY_REFRESH_US); }
}

static esp_err_t gpio_init_all(void)
{
    uint64_t push_pull_mask = BIT64(PIN_BUZZER) | BIT64(PIN_TXEN) | BIT64(PIN_BLUE_LED);
    for (int i = 0; i < 8; i++) push_pull_mask |= BIT64(s_segment_pins[i]);
    const gpio_config_t push_pull_cfg = {
        .pin_bit_mask = push_pull_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&push_pull_cfg), TAG, "gpio push-pull outputs");

    uint64_t column_mask = 0;
    for (int i = 0; i < DISPLAY_DIGITS; i++) column_mask |= BIT64(s_digit_pins[i]);
    const gpio_config_t column_cfg = {
        .pin_bit_mask = column_mask,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&column_cfg), TAG, "gpio display columns");

    all_digits_off();
    all_segments_off();
    buzzer_off();
    gpio_set_level(PIN_TXEN, 0);
    gpio_set_level(PIN_BLUE_LED, 0);
    int64_t now = now_ms();
    for (int i = 0; i < BTN_COUNT; i++) s_buttons[i].raw_changed_ms = now;
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_init_all());
    ESP_ERROR_CHECK(icsc_init(&s_bus, UI_ADDRESS, ICSC_BAUD, ICSC_UART_NUM, ICSC_TX_PIN, ICSC_RX_PIN, PIN_TXEN));
    register_icsc_callbacks();
    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
    int64_t now = now_ms();
    s_startup_state = START_REQUEST_PC_VERSION; s_startup_state_start_ms = now; s_last_heartbeat_ack_ms = now; s_last_local_countdown_ms = now;
    display_set_chars(' ', ' ', ' ', ' ', ' ');
    ESP_LOGI(TAG, "STN UI-V3A ESP32-C5 firmware started");
    while (true) {
        icsc_process(&s_bus); update_heartbeat(); update_dip_request();
        if (s_startup_state != START_OFF) update_startup_sequence();
        handle_buttons(); update_power_on_pending(); update_local_countdown(); update_edit_timeout(); set_indicators_for_state();
        if (s_error_display_active && s_active_error_code != 0) update_error_display(); else { buzzer_off(); update_temperature_blink(); }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}






