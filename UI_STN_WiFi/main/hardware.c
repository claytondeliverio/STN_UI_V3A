#include "hardware.h"

#include "esp_timer.h"

esp_err_t hardware_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO) | (1ULL << BLUE_LED_GPIO) | (1ULL << RS485_TXEN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_level(BLUE_LED_GPIO, 0);
    gpio_set_level(RS485_TXEN_GPIO, 0);
    return ESP_OK;
}

uint32_t hardware_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}