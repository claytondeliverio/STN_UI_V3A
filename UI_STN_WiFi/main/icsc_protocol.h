#ifndef ICSC_PROTOCOL_H
#define ICSC_PROTOCOL_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#define ICSC_MAX_MESSAGE 64
#define ICSC_MAX_COMMANDS 32
#define ICSC_BROADCAST 0

typedef void (*icsc_callback_t)(uint8_t station, char command, uint8_t len, uint8_t *data, void *ctx);

esp_err_t icsc_protocol_init(uint8_t station, uint32_t baud, uart_port_t uart_port, gpio_num_t tx_gpio, gpio_num_t rx_gpio, gpio_num_t txen_gpio);
esp_err_t icsc_protocol_send(uint8_t station, char command, uint8_t len, const uint8_t *data);
void icsc_protocol_process(void);
esp_err_t icsc_protocol_register(char command, icsc_callback_t cb, void *ctx);

#endif