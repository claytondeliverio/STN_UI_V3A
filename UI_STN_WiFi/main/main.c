#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ui_stn_wifi";

/*
 * ATmega328P -> ESP32-C5-WROOM-1 porting scaffold.
 * The original sketch uses Arduino core APIs such as pinMode(), digitalWrite(),
 * digitalRead(), millis(), and EEPROM. This file provides an ESP-IDF equivalent
 * foundation so the UI firmware can be migrated without changing the original
 * control logic structure.
 */

/* UART / RS485 */
#define UART_PORT UART_NUM_1
#define UART_BAUD_RATE 57600
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_18
#define RS485_TXEN_PIN GPIO_NUM_12

/* UI button / LED / display pins mapped to ESP32-C5 GPIOs */
#define S1_ON_OFF_PIN GPIO_NUM_4
#define S2_CABIN_LIGHT_PIN GPIO_NUM_5
#define S4_STANDBY_PIN GPIO_NUM_6
#define S3_ADC_PIN GPIO_NUM_7
#define S5_DOWN_PIN GPIO_NUM_8
#define S6_TEMP_TIME_PIN GPIO_NUM_9
#define S7_UP_PIN GPIO_NUM_10
#define BUZZER_PIN GPIO_NUM_11

#define SEG_A_PIN GPIO_NUM_13
#define SEG_B_PIN GPIO_NUM_14
#define SEG_C_PIN GPIO_NUM_15
#define SEG_D_PIN GPIO_NUM_16
#define SEG_E_PIN GPIO_NUM_19
#define SEG_F_PIN GPIO_NUM_20
#define SEG_G_PIN GPIO_NUM_21
#define SEG_DP_PIN GPIO_NUM_22

#define COL1_PIN GPIO_NUM_23
#define COL2_PIN GPIO_NUM_24
#define COL3_PIN GPIO_NUM_25
#define COL4_PIN GPIO_NUM_26
#define COL5_PIN GPIO_NUM_27

/* EEPROM/NVS compatibility layer */
#define EEPROM_NAMESPACE "ui_settings"
#define EEPROM_KEY "settings"
#define EEPROM_SIZE 32

static uint8_t eeprom_image[EEPROM_SIZE];
static bool eeprom_loaded = false;

static uint32_t millis_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void pin_mode(gpio_num_t pin, gpio_mode_t mode)
{
    ESP_ERROR_CHECK(gpio_set_direction(pin, mode));
}

static void digital_write(gpio_num_t pin, uint32_t level)
{
    ESP_ERROR_CHECK(gpio_set_level(pin, level));
}

static int digital_read(gpio_num_t pin)
{
    return gpio_get_level(pin);
}

static void init_nvs_storage(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t handle;
    err = nvs_open(EEPROM_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        size_t required_size = EEPROM_SIZE;
        err = nvs_get_blob(handle, EEPROM_KEY, eeprom_image, &required_size);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            memset(eeprom_image, 0x00, sizeof(eeprom_image));
            err = nvs_set_blob(handle, EEPROM_KEY, eeprom_image, EEPROM_SIZE);
            if (err == ESP_OK) {
                err = nvs_commit(handle);
            }
        }
        nvs_close(handle);
    }
    ESP_ERROR_CHECK(err);
    eeprom_loaded = true;
}

static uint8_t eeprom_read_byte(uint16_t addr)
{
    if (!eeprom_loaded) {
        return 0;
    }
    if (addr >= EEPROM_SIZE) {
        return 0;
    }
    return eeprom_image[addr];
}

static void eeprom_write_byte(uint16_t addr, uint8_t value)
{
    if (!eeprom_loaded) {
        return;
    }
    if (addr >= EEPROM_SIZE) {
        return;
    }
    eeprom_image[addr] = value;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(EEPROM_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(handle, EEPROM_KEY, eeprom_image, EEPROM_SIZE);
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "EEPROM/NVS write failed: %s", esp_err_to_name(err));
    }
}

static void configure_uart_rs485(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    pin_mode(RS485_TXEN_PIN, GPIO_MODE_OUTPUT);
    digital_write(RS485_TXEN_PIN, 0);
}

static void configure_gpio(void)
{
    pin_mode(S1_ON_OFF_PIN, GPIO_MODE_INPUT);
    pin_mode(S2_CABIN_LIGHT_PIN, GPIO_MODE_INPUT);
    pin_mode(S4_STANDBY_PIN, GPIO_MODE_INPUT);
    pin_mode(S5_DOWN_PIN, GPIO_MODE_INPUT);
    pin_mode(S6_TEMP_TIME_PIN, GPIO_MODE_INPUT);
    pin_mode(S7_UP_PIN, GPIO_MODE_INPUT);
    pin_mode(S3_ADC_PIN, GPIO_MODE_INPUT);

    pin_mode(BUZZER_PIN, GPIO_MODE_OUTPUT);
    digital_write(BUZZER_PIN, 0);

    pin_mode(SEG_A_PIN, GPIO_MODE_OUTPUT);
    pin_mode(SEG_B_PIN, GPIO_MODE_OUTPUT);
    pin_mode(SEG_C_PIN, GPIO_MODE_OUTPUT);
    pin_mode(SEG_D_PIN, GPIO_MODE_OUTPUT);
    pin_mode(SEG_E_PIN, GPIO_MODE_OUTPUT);
    pin_mode(SEG_F_PIN, GPIO_MODE_OUTPUT);
    pin_mode(SEG_G_PIN, GPIO_MODE_OUTPUT);
    pin_mode(SEG_DP_PIN, GPIO_MODE_OUTPUT);

    pin_mode(COL1_PIN, GPIO_MODE_OUTPUT);
    pin_mode(COL2_PIN, GPIO_MODE_OUTPUT);
    pin_mode(COL3_PIN, GPIO_MODE_OUTPUT);
    pin_mode(COL4_PIN, GPIO_MODE_OUTPUT);
    pin_mode(COL5_PIN, GPIO_MODE_OUTPUT);

    digital_write(SEG_A_PIN, 0);
    digital_write(SEG_B_PIN, 0);
    digital_write(SEG_C_PIN, 0);
    digital_write(SEG_D_PIN, 0);
    digital_write(SEG_E_PIN, 0);
    digital_write(SEG_F_PIN, 0);
    digital_write(SEG_G_PIN, 0);
    digital_write(SEG_DP_PIN, 0);
    digital_write(COL1_PIN, 1);
    digital_write(COL2_PIN, 1);
    digital_write(COL3_PIN, 1);
    digital_write(COL4_PIN, 1);
    digital_write(COL5_PIN, 1);
}

static void send_rs485_byte(uint8_t byte)
{
    digital_write(RS485_TXEN_PIN, 1);
    uart_write_bytes(UART_PORT, (const char *)&byte, 1);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));
    digital_write(RS485_TXEN_PIN, 0);
}

static void send_heartbeat(void)
{
    static uint32_t last_heartbeat_ms = 0;
    uint32_t now = millis_ms();
    if (now - last_heartbeat_ms < 1000UL) {
        return;
    }
    last_heartbeat_ms = now;
    send_rs485_byte('H');
    ESP_LOGD(TAG, "Heartbeat sent over RS485");
}

static void setup(void)
{
    ESP_LOGI(TAG, "Starting STN UI firmware for ESP32-C5-WROOM-1");

    init_nvs_storage();
    configure_gpio();
    configure_uart_rs485();

    /* Example of reading a persisted value from the old EEPROM layout */
    uint8_t saved_value = eeprom_read_byte(7);
    ESP_LOGI(TAG, "Saved EEPROM/NVS byte at address 7 = %u", saved_value);
}

static void loop(void)
{
    send_heartbeat();

    /* Placeholder for the full original UI state machine. */
    /* The rest of the Arduino logic can be ported into this loop as needed. */
    vTaskDelay(pdMS_TO_TICKS(10));
}

void app_main(void)
{
    setup();
    while (true) {
        loop();
    }
}
