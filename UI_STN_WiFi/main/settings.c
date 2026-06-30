#include "settings.h"

#define DEFAULT_TARGET_TEMP_C 40
#define DEFAULT_SESSION_TIME_MIN 30
#define DEFAULT_STANDBY_TIME_MIN 240
#define DEFAULT_SCENT_SETPOINT 1
#define DEFAULT_CABIN_LIGHT_PERCENT 100
#define DEFAULT_MAX_TEMP_C 50
#define DEFAULT_MAX_SESSION_MIN 30
#define DEFAULT_MAX_STANDBY_MIN 240

static ui_settings_t s_settings;

void settings_init(void)
{
    s_settings.targetTemperatureC = DEFAULT_TARGET_TEMP_C;
    s_settings.sessionTimeMin = DEFAULT_SESSION_TIME_MIN;
    s_settings.standbyTimeMin = DEFAULT_STANDBY_TIME_MIN;
    s_settings.scentPumpSetpoint = DEFAULT_SCENT_SETPOINT;
    s_settings.cabinLightPercent = DEFAULT_CABIN_LIGHT_PERCENT;
    s_settings.fanOn = false;
    s_settings.maxSessionTimeMin = DEFAULT_MAX_SESSION_MIN;
    s_settings.maxStandbyTimeMin = DEFAULT_MAX_STANDBY_MIN;
    s_settings.maxTargetTemperatureC = DEFAULT_MAX_TEMP_C;
    s_settings.standbyDisabled = false;
}

ui_settings_t *settings_get(void)
{
    return &s_settings;
}

esp_err_t settings_save(void)
{
    return ESP_OK;
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void settings_apply_dip_reply(const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < 11) return;

    s_settings.maxSessionTimeMin = read_u16_le(&data[1]);
    s_settings.maxStandbyTimeMin = read_u16_le(&data[3]);
    s_settings.standbyDisabled = data[5] != 0;
    s_settings.sessionTimeMin = read_u16_le(&data[6]);
    s_settings.standbyTimeMin = read_u16_le(&data[8]);
    s_settings.maxTargetTemperatureC = data[10];

    if (s_settings.maxTargetTemperatureC < 30) s_settings.maxTargetTemperatureC = DEFAULT_MAX_TEMP_C;
    if (s_settings.targetTemperatureC > s_settings.maxTargetTemperatureC) {
        s_settings.targetTemperatureC = s_settings.maxTargetTemperatureC;
    }
}