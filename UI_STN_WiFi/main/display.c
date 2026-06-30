#include "display.h"
#include "hardware.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "display";

#define DISPLAY_DIGITS 5
#define DISPLAY_MAIN_DIGITS 4
#define DISPLAY_REFRESH_US 500

#define SEG_A  (1U << 0)
#define SEG_B  (1U << 1)
#define SEG_C  (1U << 2)
#define SEG_D  (1U << 3)
#define SEG_E  (1U << 4)
#define SEG_F  (1U << 5)
#define SEG_G  (1U << 6)
#define SEG_DP (1U << 7)

static const gpio_num_t s_segment_pins[8] = {
    DISPLAY_SEG_A_GPIO,
    DISPLAY_SEG_B_GPIO,
    DISPLAY_SEG_C_GPIO,
    DISPLAY_SEG_D_GPIO,
    DISPLAY_SEG_E_GPIO,
    DISPLAY_SEG_F_GPIO,
    DISPLAY_SEG_G_GPIO,
    DISPLAY_SEG_DP_GPIO,
};

static const gpio_num_t s_column_pins[DISPLAY_DIGITS] = {
    DISPLAY_COL1_GPIO,
    DISPLAY_COL2_GPIO,
    DISPLAY_COL3_GPIO,
    DISPLAY_COL4_GPIO,
    DISPLAY_LED_COL_GPIO,
};

static volatile uint8_t s_frame[DISPLAY_DIGITS];
static portMUX_TYPE s_frame_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_gpio_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_refresh_paused;
static esp_timer_handle_t s_refresh_timer;
static char s_last_text[24];

static uint8_t glyph(char c)
{
    switch (c) {
    case '0': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case '1': return SEG_B | SEG_C;
    case '2': return SEG_A | SEG_B | SEG_D | SEG_E | SEG_G;
    case '3': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_G;
    case '4': return SEG_B | SEG_C | SEG_F | SEG_G;
    case '5': return SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
    case '6': return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '7': return SEG_A | SEG_B | SEG_C;
    case '8': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '9': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G;
    case 'A': return SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;
    case 'B': return SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case 'C': return SEG_A | SEG_D | SEG_E | SEG_F;
    case 'D': return SEG_B | SEG_C | SEG_D | SEG_E | SEG_G;
    case 'E': return SEG_A | SEG_D | SEG_E | SEG_F | SEG_G;
    case 'F': return SEG_A | SEG_E | SEG_F | SEG_G;
    case 'H': return SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;
    case 'I': return SEG_B | SEG_C;
    case 'K': return SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;
    case 'L': return SEG_D | SEG_E | SEG_F;
    case 'M': return SEG_A | SEG_C | SEG_E | SEG_G;
    case 'N': return SEG_C | SEG_E | SEG_G;
    case 'O': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'P': return SEG_A | SEG_B | SEG_E | SEG_F | SEG_G;
    case 'R': return SEG_E | SEG_G;
    case 'S': return SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
    case 'T': return SEG_D | SEG_E | SEG_F | SEG_G;
    case 'U': return SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'W': return SEG_B | SEG_D | SEG_F;
    case '-': return SEG_G;
    case '_': return SEG_D;
    case ' ': return 0;
    default: return 0;
    }
}

static void segment_set(gpio_num_t pin, bool on)
{
    gpio_set_level(pin, on ? 0 : 1);
}

static void column_set(gpio_num_t pin, bool on)
{
    gpio_set_level(pin, on ? 0 : 1);
}

static void all_segments_off(void)
{
    for (int i = 0; i < 8; i++) segment_set(s_segment_pins[i], false);
}

static void all_columns_off(void)
{
    for (int i = 0; i < DISPLAY_DIGITS; i++) column_set(s_column_pins[i], false);
}


static void configure_segments_output(void)
{
    uint64_t seg_mask = 0;
    for (int i = 0; i < 8; i++) seg_mask |= (1ULL << s_segment_pins[i]);

    gpio_config_t seg_cfg = {
        .pin_bit_mask = seg_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&seg_cfg);
    all_segments_off();
}

void display_pause_for_button_scan(void)
{
    portENTER_CRITICAL(&s_gpio_mux);
    s_refresh_paused = true;
    all_columns_off();
    all_segments_off();
    portEXIT_CRITICAL(&s_gpio_mux);
}

void display_resume_after_button_scan(void)
{
    configure_segments_output();
    portENTER_CRITICAL(&s_gpio_mux);
    all_columns_off();
    all_segments_off();
    s_refresh_paused = false;
    portEXIT_CRITICAL(&s_gpio_mux);
}
static void commit_frame(const uint8_t frame[DISPLAY_DIGITS])
{
    portENTER_CRITICAL(&s_frame_mux);
    for (int i = 0; i < DISPLAY_DIGITS; i++) s_frame[i] = frame[i];
    portEXIT_CRITICAL(&s_frame_mux);
}

static void clear_frame(uint8_t frame[DISPLAY_DIGITS])
{
    for (int i = 0; i < DISPLAY_DIGITS; i++) frame[i] = 0;
}

static void frame_chars(const char chars[DISPLAY_MAIN_DIGITS])
{
    uint8_t frame[DISPLAY_DIGITS];
    clear_frame(frame);
    for (int i = 0; i < DISPLAY_MAIN_DIGITS; i++) frame[i] = glyph(chars[i]);
    commit_frame(frame);
}

static void frame_right_text(const char *text)
{
    char chars[DISPLAY_MAIN_DIGITS] = {' ', ' ', ' ', ' '};
    size_t len = text ? strlen(text) : 0;
    if (len > DISPLAY_MAIN_DIGITS) len = DISPLAY_MAIN_DIGITS;
    for (size_t i = 0; i < len; i++) chars[DISPLAY_MAIN_DIGITS - len + i] = text[i];
    frame_chars(chars);
}

static void log_change(const char *text)
{
    if (text == NULL) text = "";
    if (strncmp(s_last_text, text, sizeof(s_last_text)) == 0) return;
    snprintf(s_last_text, sizeof(s_last_text), "%s", text);
    ESP_LOGI(TAG, "%s", s_last_text);
}

static void render_known_text(const char *text)
{
    if (text == NULL) {
        frame_right_text("---");
    } else if (strcmp(text, "OFF") == 0) {
        char chars[4] = {' ', 'O', 'F', 'F'};
        frame_chars(chars);
    } else if (strcmp(text, "ON") == 0) {
        frame_right_text("ON");
    } else if (strcmp(text, "FAN") == 0) {
        frame_right_text("FAN");
    } else if (strcmp(text, "TEMP") == 0) {
        char chars[4] = {'T', 'E', 'M', 'P'};
        frame_chars(chars);
    } else if (strcmp(text, "TIME") == 0) {
        char chars[4] = {'T', 'I', 'M', 'E'};
        frame_chars(chars);
    } else if (strcmp(text, "SCENT") == 0) {
        char chars[4] = {'S', 'C', 'N', 'T'};
        frame_chars(chars);
    } else if (strcmp(text, "OK") == 0) {
        frame_right_text("OK");
    } else if (strcmp(text, "OF") == 0) {
        frame_right_text("OF");
    } else if (strcmp(text, "--") == 0) {
        frame_right_text("--");
    } else if (strcmp(text, "SM") == 0) {
        frame_right_text("SM");
    } else if (strcmp(text, "WAIT") == 0) {
        char chars[4] = {'W', 'A', 'I', 'T'};
        frame_chars(chars);
    } else if (strcmp(text, "BOOT") == 0) {
        char chars[4] = {'B', 'O', 'O', 'T'};
        frame_chars(chars);
    } else if (strcmp(text, "DRN") == 0) {
        frame_right_text("DRN");
    } else if (text[0] == 'N') {
        frame_right_text(text);
    } else if (text[0] == 'S') {
        frame_right_text(text);
    } else {
        frame_right_text("---");
    }
}

static void refresh_timer_cb(void *arg)
{
    (void)arg;
    static uint8_t column = 0;
    uint8_t mask;

    portENTER_CRITICAL(&s_gpio_mux);
    all_columns_off();

    if (s_refresh_paused) {
        portEXIT_CRITICAL(&s_gpio_mux);
        return;
    }

    all_segments_off();

    portENTER_CRITICAL(&s_frame_mux);
    mask = s_frame[column];
    portEXIT_CRITICAL(&s_frame_mux);

    for (int seg = 0; seg < 8; seg++) {
        segment_set(s_segment_pins[seg], (mask & (1U << seg)) != 0);
    }

    if (mask != 0) column_set(s_column_pins[column], true);

    column++;
    if (column >= DISPLAY_DIGITS) column = 0;
    portEXIT_CRITICAL(&s_gpio_mux);
}

esp_err_t display_init(void)
{
    uint64_t seg_mask = 0;
    uint64_t col_mask = 0;
    for (int i = 0; i < 8; i++) seg_mask |= (1ULL << s_segment_pins[i]);
    for (int i = 0; i < DISPLAY_DIGITS; i++) col_mask |= (1ULL << s_column_pins[i]);

    gpio_config_t seg_cfg = {
        .pin_bit_mask = seg_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&seg_cfg), TAG, "segment gpio config");

    gpio_config_t col_cfg = {
        .pin_bit_mask = col_mask,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&col_cfg), TAG, "column gpio config");

    all_columns_off();
    all_segments_off();
    uint8_t blank[DISPLAY_DIGITS] = {0};
    commit_frame(blank);
    s_last_text[0] = '\0';

    if (s_refresh_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = refresh_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "display_mux",
            .skip_unhandled_events = true,
        };
        ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_refresh_timer), TAG, "create refresh timer");
        ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_refresh_timer, DISPLAY_REFRESH_US), TAG, "start refresh timer");
    }

    ESP_LOGI(TAG, "7-segment display initialized");
    return ESP_OK;
}

void display_show_text(const char *text)
{
    render_known_text(text);
    log_change(text);
}

void display_show_off(void)
{
    display_show_text("OFF");
}

void display_show_error(uint8_t error_code)
{
    char text[8];
    if (error_code == 0) {
        display_show_off();
        return;
    }
    snprintf(text, sizeof(text), "E%u", (unsigned)error_code);
    frame_right_text(text);
    log_change(text);
}

void display_show_slave_error(uint8_t slave_id, uint8_t error_code)
{
    uint8_t frame[DISPLAY_DIGITS];
    clear_frame(frame);
    frame[0] = glyph('S');
    frame[1] = glyph((char)('0' + (slave_id % 10U)));
    frame[2] = glyph('E');
    frame[3] = glyph((char)('0' + (error_code % 10U)));
    commit_frame(frame);

    char text[12];
    snprintf(text, sizeof(text), "S%u E%u", (unsigned)slave_id, (unsigned)error_code);
    log_change(text);
}

void display_show_version(uint8_t major, uint8_t minor)
{
    uint8_t frame[DISPLAY_DIGITS];
    clear_frame(frame);
    frame[1] = glyph((char)('0' + (major <= 9 ? major : 9))) | SEG_DP;
    frame[2] = glyph((char)('0' + (minor <= 9 ? minor : 9)));
    commit_frame(frame);

    char text[8];
    snprintf(text, sizeof(text), "%u.%u", (unsigned)major, (unsigned)minor);
    log_change(text);
}

void display_show_temperature(int value)
{
    char text[8];
    if (value < -100) {
        display_show_text("--");
        return;
    }
    if (value < 0) value = 0;
    if (value > 999) value = 999;
    snprintf(text, sizeof(text), "%d", value);
    frame_right_text(text);
    log_change(text);
}

void display_show_time_minutes(uint16_t minutes)
{
    uint8_t frame[DISPLAY_DIGITS];
    clear_frame(frame);

    uint16_t hours = minutes / 60U;
    uint16_t mins = minutes % 60U;
    if (hours > 9) hours = 9;

    frame[0] = glyph((char)('0' + hours));
    frame[1] = SEG_DP;
    frame[2] = glyph((char)('0' + (mins / 10U)));
    frame[3] = glyph((char)('0' + (mins % 10U)));
    commit_frame(frame);

    char text[12];
    snprintf(text, sizeof(text), "%u:%02u", (unsigned)hours, (unsigned)mins);
    log_change(text);
}

void display_show_service_status(uint8_t slave_id, uint8_t status, uint8_t error_code)
{
    if (status == 2) {
        display_show_slave_error(slave_id, error_code);
        return;
    }

    uint8_t frame[DISPLAY_DIGITS];
    clear_frame(frame);
    frame[0] = glyph('S');
    frame[1] = glyph((char)('0' + (slave_id % 10U)));
    if (status == 1) {
        frame[2] = glyph('O');
        frame[3] = glyph('K');
    } else if (status == 3) {
        frame[2] = glyph('O');
        frame[3] = glyph('F');
    } else {
        frame[2] = glyph('-');
        frame[3] = glyph('-');
    }
    commit_frame(frame);

    char text[16];
    const char *status_text = status == 1 ? "OK" : (status == 3 ? "OF" : "--");
    snprintf(text, sizeof(text), "S%u %s", (unsigned)slave_id, status_text);
    log_change(text);
}
