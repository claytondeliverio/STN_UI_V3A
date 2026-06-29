#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icsc.h"

static const char *TAG = "ui_v3a_heartbeat";

#define UI_ADDRESS 1
#define POWER_CONTROLLER_ADDRESS 2
#define ICSC_BAUD 57600UL

#define CMD_HEARTBEAT 'H'
#define CMD_HEARTBEAT_ACK 'h'

#define HEARTBEAT_INTERVAL_MS 1000UL
#define HEARTBEAT_ACK_TIMEOUT_MS 10000UL

#define DISPLAY_DIGITS 5
#define DISPLAY_REFRESH_US 500

/* UI-V3A C-F hardware mapping. */
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
#define PIN_BLUE_LED GPIO_NUM_23
#define PIN_TXEN GPIO_NUM_8

/* UI-V3A routes ESP32-C5 U0TXD/U0RXD to RS485. */
#define ICSC_UART_NUM UART_NUM_0
#define ICSC_TX_PIN GPIO_NUM_NC
#define ICSC_RX_PIN GPIO_NUM_NC

#define BIT_A  (1U << 0)
#define BIT_B  (1U << 1)
#define BIT_C  (1U << 2)
#define BIT_D  (1U << 3)
#define BIT_E  (1U << 4)
#define BIT_F  (1U << 5)
#define BIT_G  (1U << 6)
#define BIT_DP (1U << 7)

static const gpio_num_t s_segment_pins[8] = {
    PIN_SEG_A, PIN_SEG_B, PIN_SEG_C, PIN_SEG_D,
    PIN_SEG_E, PIN_SEG_F, PIN_SEG_G, PIN_SEG_DP,
};

static const gpio_num_t s_column_pins[DISPLAY_DIGITS] = {
    PIN_COL1, PIN_COL2, PIN_COL3, PIN_COL4, PIN_COL5,
};

static icsc_t s_bus;
static volatile uint8_t s_display_buffer[DISPLAY_DIGITS];

static uint32_t s_last_heartbeat_send_ms;
static uint32_t s_last_heartbeat_ack_ms;
static uint32_t s_watchdog_start_ms;
static bool s_heartbeat_ack_received;
static bool s_local_e11_active;
static bool s_showing_e11;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
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
    case 'E': return BIT_A | BIT_D | BIT_E | BIT_F | BIT_G;
    case ' ': return 0;
    default: return 0;
    }
}

static void set_segment(gpio_num_t pin, bool on)
{
    gpio_set_level(pin, on ? 0 : 1); /* segments are active-low */
}

static void set_column(gpio_num_t pin, bool on)
{
    gpio_set_level(pin, on ? 0 : 1); /* columns are active-low/open-drain */
}

static void all_segments_off(void)
{
    for (int i = 0; i < 8; i++) set_segment(s_segment_pins[i], false);
}

static void all_columns_off(void)
{
    for (int i = 0; i < DISPLAY_DIGITS; i++) set_column(s_column_pins[i], false);
}

static void display_set_raw(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    s_display_buffer[0] = d0;
    s_display_buffer[1] = d1;
    s_display_buffer[2] = d2;
    s_display_buffer[3] = d3;
    s_display_buffer[4] = d4;
}

static void show_colon(void)
{
    display_set_raw(0, 0, BIT_DP, BIT_DP, 0);
    s_showing_e11 = false;
}

static void show_e11(void)
{
    display_set_raw(0, glyph_for_char('E'), glyph_for_char('1'), glyph_for_char('1'), 0);
    s_showing_e11 = true;
}

static void refresh_one_column(int column)
{
    uint8_t mask = s_display_buffer[column];

    all_columns_off();
    all_segments_off();

    for (int seg = 0; seg < 8; seg++) {
        set_segment(s_segment_pins[seg], (mask & (1U << seg)) != 0);
    }

    if (mask != 0) {
        set_column(s_column_pins[column], true);
    }
}

static void display_refresh_timer_cb(void *arg)
{
    (void)arg;
    static int column = 0;

    refresh_one_column(column);
    column = (column + 1) % DISPLAY_DIGITS;
}

static esp_err_t start_display_refresh_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = display_refresh_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "display_refresh",
        .skip_unhandled_events = true,
    };

    esp_timer_handle_t timer;
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &timer), TAG, "create display timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(timer, DISPLAY_REFRESH_US), TAG, "start display timer");
    return ESP_OK;
}

static void send_heartbeat_to_power_controller(void)
{
    char payload[1] = {1};
    icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_HEARTBEAT, sizeof(payload), payload);
    s_last_heartbeat_send_ms = now_ms();
}

static void send_heartbeat_ack_to_power_controller(void)
{
    char payload[1] = {1};
    icsc_send(&s_bus, POWER_CONTROLLER_ADDRESS, CMD_HEARTBEAT_ACK, sizeof(payload), payload);
}

static void mark_communication_alive(void)
{
    s_heartbeat_ack_received = true;
    s_last_heartbeat_ack_ms = now_ms();

    if (s_local_e11_active) {
        s_local_e11_active = false;
        show_colon();
    }
}

static void on_heartbeat_from_power_controller(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    (void)len;
    (void)data;

    if (station != POWER_CONTROLLER_ADDRESS) return;

    /* Legacy compatibility: old PC builds may send H. Seeing it proves the bus is alive.
       Replying with h is safe here because this test firmware has no other traffic. */
    mark_communication_alive();
    send_heartbeat_ack_to_power_controller();
}

static void on_heartbeat_ack(uint8_t station, char command, uint8_t len, char *data)
{
    (void)command;
    (void)len;
    (void)data;

    if (station != POWER_CONTROLLER_ADDRESS) return;

    mark_communication_alive();
}

static void register_icsc_callbacks(void)
{
    icsc_register_command(&s_bus, CMD_HEARTBEAT, on_heartbeat_from_power_controller);
    icsc_register_command(&s_bus, CMD_HEARTBEAT_ACK, on_heartbeat_ack);
}

static void update_heartbeat(void)
{
    uint32_t now = now_ms();
    if ((uint32_t)(now - s_last_heartbeat_send_ms) >= HEARTBEAT_INTERVAL_MS) {
        send_heartbeat_to_power_controller();
    }
}

static void trigger_local_e11_communication_error(void)
{
    if (s_local_e11_active && s_showing_e11) return;

    s_local_e11_active = true;
    show_e11();
    ESP_LOGW(TAG, "E11: no heartbeat ACK from Power Controller");
}

static void update_ui_communication_watchdog(void)
{
    uint32_t now = now_ms();
    bool communication_lost = false;

    if (!s_heartbeat_ack_received) {
        if ((uint32_t)(now - s_watchdog_start_ms) >= HEARTBEAT_ACK_TIMEOUT_MS) communication_lost = true;
    } else if ((uint32_t)(now - s_last_heartbeat_ack_ms) >= HEARTBEAT_ACK_TIMEOUT_MS) {
        communication_lost = true;
    }

    if (communication_lost) {
        trigger_local_e11_communication_error();
    }
}

static esp_err_t gpio_init_all(void)
{
    uint64_t output_mask = BIT64(PIN_BUZZER) | BIT64(PIN_BLUE_LED) | BIT64(PIN_TXEN);
    for (int i = 0; i < 8; i++) output_mask |= BIT64(s_segment_pins[i]);

    const gpio_config_t output_cfg = {
        .pin_bit_mask = output_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_cfg), TAG, "outputs");

    uint64_t column_mask = 0;
    for (int i = 0; i < DISPLAY_DIGITS; i++) column_mask |= BIT64(s_column_pins[i]);

    const gpio_config_t column_cfg = {
        .pin_bit_mask = column_mask,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&column_cfg), TAG, "columns");

    all_columns_off();
    all_segments_off();
    gpio_set_level(PIN_BUZZER, 0);
    gpio_set_level(PIN_BLUE_LED, 0);
    gpio_set_level(PIN_TXEN, 0);
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_init_all());
    ESP_ERROR_CHECK(start_display_refresh_timer());

    /* Boot self-test: prove the E11 glyph exists on the seven-segment display. */
    show_e11();
    vTaskDelay(pdMS_TO_TICKS(2000));
    show_colon();

    ESP_ERROR_CHECK(icsc_init(&s_bus, UI_ADDRESS, ICSC_BAUD, ICSC_UART_NUM, ICSC_TX_PIN, ICSC_RX_PIN, PIN_TXEN));
    register_icsc_callbacks();

    uint32_t now = now_ms();
    s_watchdog_start_ms = now;
    s_last_heartbeat_send_ms = now - HEARTBEAT_INTERVAL_MS;
    s_last_heartbeat_ack_ms = now;

    ESP_LOGI(TAG, "UI-V3A heartbeat test started");

    while (true) {
        icsc_process(&s_bus);
        update_heartbeat();
        update_ui_communication_watchdog();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}