#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t targetTemperatureC;
    uint16_t sessionTimeMin;
    uint16_t standbyTimeMin;
    uint8_t scentPumpSetpoint;
    uint8_t cabinLightPercent;
    bool fanOn;
    uint16_t maxSessionTimeMin;
    uint16_t maxStandbyTimeMin;
    uint8_t maxTargetTemperatureC;
    bool standbyDisabled;
} ui_settings_t;

void settings_init(void);
ui_settings_t *settings_get(void);
esp_err_t settings_save(void);
void settings_apply_dip_reply(const uint8_t *data, uint8_t len);

#endif