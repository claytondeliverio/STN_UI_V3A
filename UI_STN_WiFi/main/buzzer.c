#include "buzzer.h"
#include "hardware.h"

#include <stdbool.h>
#include "driver/ledc.h"

#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_DUTY_ON 512
#define BUZZER_DUTY_OFF 0
#define BUZZER_ERROR_INTERVAL_MS 1500U
#define BUZZER_ERROR_BEEP_MS 80U

static bool s_beep_active;
static bool s_error_pattern;
static uint32_t s_beep_end_ms;
static uint32_t s_next_error_ms;

static void buzzer_set(bool on)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, on ? BUZZER_DUTY_ON : BUZZER_DUTY_OFF);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

esp_err_t buzzer_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = BUZZER_LEDC_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = BUZZER_LEDC_TIMER,
        .freq_hz = 4000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    buzzer_set(false);
    return ESP_OK;
}

void buzzer_beep(uint32_t duration_ms)
{
    uint32_t now = hardware_millis();
    s_beep_active = true;
    s_beep_end_ms = now + duration_ms;
    buzzer_set(true);
}

void buzzer_error_pattern_set(bool enabled)
{
    s_error_pattern = enabled;
    if (enabled) s_next_error_ms = hardware_millis();
}

void buzzer_update(uint32_t now_ms)
{
    if (s_beep_active && (int32_t)(now_ms - s_beep_end_ms) >= 0) {
        s_beep_active = false;
        buzzer_set(false);
    }

    if (s_error_pattern && !s_beep_active && (int32_t)(now_ms - s_next_error_ms) >= 0) {
        buzzer_beep(BUZZER_ERROR_BEEP_MS);
        s_next_error_ms = now_ms + BUZZER_ERROR_INTERVAL_MS;
    }
}