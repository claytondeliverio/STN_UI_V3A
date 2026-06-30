#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t display_init(void);
void display_pause_for_button_scan(void);
void display_resume_after_button_scan(void);
void display_show_text(const char *text);
void display_show_off(void);
void display_show_error(uint8_t error_code);
void display_show_slave_error(uint8_t slave_id, uint8_t error_code);
void display_show_version(uint8_t major, uint8_t minor);
void display_show_temperature(int value);
void display_show_time_minutes(uint16_t minutes);
void display_show_service_status(uint8_t slave_id, uint8_t status, uint8_t error_code);

#endif