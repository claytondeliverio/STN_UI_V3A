#include "hardware.h"
#include "buttons.h"
#include "display.h"
#include "buzzer.h"
#include "settings.h"
#include "pc_comm.h"
#include "ui_state.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_ERROR_CHECK(hardware_init());
    ESP_ERROR_CHECK(buzzer_init());
    settings_init();
    ESP_ERROR_CHECK(buttons_init());
    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(pc_comm_init());
    ui_state_init();

    ESP_LOGI(TAG, "STN UI ESP32-C5 started");

    while (true) {
        uint32_t now = hardware_millis();
        pc_comm_update(now);
        buttons_update(now);
        ui_state_update(now);
        buzzer_update(now);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}