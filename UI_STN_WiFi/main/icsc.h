#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#define ICSC_BROADCAST 0
#define ICSC_MAX_MESSAGE 50
#define ICSC_MAX_COMMANDS 25

typedef void (*icsc_callback_t)(uint8_t station, char command, uint8_t len, char *data);

typedef struct {
    uint32_t oob_bytes;
    uint32_t rx_packets;
    uint32_t rx_bytes;
    uint32_t tx_packets;
    uint32_t tx_bytes;
    uint32_t tx_fail;
    uint32_t cs_errors;
    uint32_t cb_run;
    uint32_t cb_bad;
    uint32_t collision;
} icsc_stats_t;

typedef struct {
    char command;
    icsc_callback_t callback;
} icsc_command_t;

typedef struct {
    uart_port_t uart_num;
    gpio_num_t de_pin;
    uint8_t station;
    uint32_t baud;
    icsc_command_t commands[ICSC_MAX_COMMANDS];
    char data[ICSC_MAX_MESSAGE];
    char header[6];
    uint8_t rec_phase;
    uint8_t rec_pos;
    uint8_t rec_command;
    uint8_t rec_len;
    uint8_t rec_station;
    uint8_t rec_sender;
    uint8_t rec_cs;
    uint8_t rec_calc_cs;
    int64_t last_byte_seen_us;
    icsc_stats_t stats;
} icsc_t;

esp_err_t icsc_init(icsc_t *bus,
                    uint8_t station,
                    uint32_t baud,
                    uart_port_t uart_num,
                    gpio_num_t tx_pin,
                    gpio_num_t rx_pin,
                    gpio_num_t de_pin);
bool icsc_send(icsc_t *bus, uint8_t station, char command, uint8_t len, const char *data);
void icsc_process(icsc_t *bus);
void icsc_register_command(icsc_t *bus, char command, icsc_callback_t callback);
