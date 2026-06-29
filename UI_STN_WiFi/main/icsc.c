#include "icsc.h"

#include <string.h>

#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SOH 1
#define STX 2
#define ETX 3
#define EOT 4
#define ICSC_SYS_PING 0x05
#define ICSC_SYS_PONG 0x06

static const char *TAG = "icsc";

static void reset_rx(icsc_t *bus)
{
    bus->rec_phase = 0;
    bus->rec_pos = 0;
    bus->rec_len = 0;
    bus->rec_command = 0;
    bus->rec_cs = 0;
    bus->rec_calc_cs = 0;
}

esp_err_t icsc_init(icsc_t *bus,
                    uint8_t station,
                    uint32_t baud,
                    uart_port_t uart_num,
                    gpio_num_t tx_pin,
                    gpio_num_t rx_pin,
                    gpio_num_t de_pin)
{
    memset(bus, 0, sizeof(*bus));
    bus->uart_num = uart_num;
    bus->de_pin = de_pin;
    bus->station = station;
    bus->baud = baud;

    const uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(uart_num, 1024, 1024, 0, NULL, 0), TAG, "uart driver");
    ESP_RETURN_ON_ERROR(uart_param_config(uart_num, &cfg), TAG, "uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart pins");

    if (de_pin != GPIO_NUM_NC) {
        const gpio_config_t de_cfg = {
            .pin_bit_mask = BIT64(de_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&de_cfg), TAG, "de pin");
        gpio_set_level(de_pin, 0);
    }

    reset_rx(bus);
    return ESP_OK;
}

static void de_set(const icsc_t *bus, int level)
{
    if (bus->de_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->de_pin, level);
        if (level) {
            esp_rom_delay_us(5);
        }
    }
}

bool icsc_send(icsc_t *bus, uint8_t station, char command, uint8_t len, const char *data)
{
    if (len > ICSC_MAX_MESSAGE) {
        bus->stats.tx_fail++;
        return false;
    }

    icsc_process(bus);
    for (uint8_t timeout = 10; timeout > 0; --timeout) {
        int64_t age_us = esp_timer_get_time() - bus->last_byte_seen_us;
        if (bus->last_byte_seen_us == 0 || age_us >= 1000) {
            break;
        }
        bus->stats.collision++;
        vTaskDelay(pdMS_TO_TICKS(1));
        if (timeout == 1) {
            bus->stats.tx_fail++;
            return false;
        }
    }

    uint8_t packet[5 + 4 + ICSC_MAX_MESSAGE];
    uint8_t cs = 0;
    size_t pos = 0;

    for (uint8_t i = 0; i < 5; ++i) {
        packet[pos++] = SOH;
    }
    packet[pos++] = station;
    cs += station;
    packet[pos++] = bus->station;
    cs += bus->station;
    packet[pos++] = (uint8_t)command;
    cs += (uint8_t)command;
    packet[pos++] = len;
    cs += len;
    packet[pos++] = STX;

    for (uint8_t i = 0; i < len; ++i) {
        packet[pos++] = (uint8_t)data[i];
        cs += (uint8_t)data[i];
    }
    packet[pos++] = ETX;
    packet[pos++] = cs;
    packet[pos++] = EOT;

    de_set(bus, 1);
    int written = uart_write_bytes(bus->uart_num, packet, pos);
    uart_wait_tx_done(bus->uart_num, pdMS_TO_TICKS(100));
    de_set(bus, 0);

    if (written != (int)pos) {
        bus->stats.tx_fail++;
        return false;
    }

    bus->stats.tx_packets++;
    bus->stats.tx_bytes += len;
    return true;
}

static void dispatch_packet(icsc_t *bus)
{
    if (bus->rec_command == ICSC_SYS_PING) {
        icsc_send(bus, bus->rec_sender, ICSC_SYS_PONG, bus->rec_len, bus->data);
    }

    bool handled = false;
    for (int i = 0; i < ICSC_MAX_COMMANDS; ++i) {
        if (bus->commands[i].command == (char)bus->rec_command && bus->commands[i].callback != NULL) {
            bus->commands[i].callback(bus->rec_sender, (char)bus->rec_command, bus->rec_len, bus->data);
            bus->stats.cb_run++;
            handled = true;
        }
    }
    if (!handled) {
        bus->stats.cb_bad++;
    }
    bus->stats.rx_packets++;
    bus->stats.rx_bytes += bus->rec_len;
}

void icsc_process(icsc_t *bus)
{
    uint8_t inch = 0;
    while (uart_read_bytes(bus->uart_num, &inch, 1, 0) == 1) {
        bus->last_byte_seen_us = esp_timer_get_time();

        switch (bus->rec_phase) {
        case 0:
            memmove(&bus->header[0], &bus->header[1], 5);
            bus->header[5] = (char)inch;
            if ((uint8_t)bus->header[0] == SOH && (uint8_t)bus->header[5] == STX && bus->header[1] != bus->header[2]) {
                bus->rec_calc_cs = 0;
                bus->rec_station = (uint8_t)bus->header[1];
                bus->rec_sender = (uint8_t)bus->header[2];
                bus->rec_command = (uint8_t)bus->header[3];
                bus->rec_len = (uint8_t)bus->header[4];
                for (uint8_t i = 1; i <= 4; ++i) {
                    bus->rec_calc_cs += (uint8_t)bus->header[i];
                }
                bus->rec_phase = bus->rec_len == 0 ? 2 : 1;
                bus->rec_pos = 0;
                if ((bus->rec_station != bus->station && bus->rec_station != ICSC_BROADCAST) || bus->rec_len > ICSC_MAX_MESSAGE) {
                    reset_rx(bus);
                }
            } else {
                bus->stats.oob_bytes++;
            }
            break;
        case 1:
            bus->data[bus->rec_pos++] = (char)inch;
            bus->rec_calc_cs += inch;
            if (bus->rec_pos == bus->rec_len) {
                bus->rec_phase = 2;
            }
            break;
        case 2:
            bus->rec_phase = inch == ETX ? 3 : 0;
            break;
        case 3:
            bus->rec_cs = inch;
            bus->rec_phase = 4;
            break;
        case 4:
            if (inch == EOT) {
                if (bus->rec_cs == bus->rec_calc_cs) {
                    dispatch_packet(bus);
                } else {
                    bus->stats.cs_errors++;
                }
            }
            reset_rx(bus);
            break;
        default:
            reset_rx(bus);
            break;
        }
    }
}

void icsc_register_command(icsc_t *bus, char command, icsc_callback_t callback)
{
    for (int i = 0; i < ICSC_MAX_COMMANDS; ++i) {
        if (bus->commands[i].command == command) {
            bus->commands[i].callback = callback;
            return;
        }
    }
    for (int i = 0; i < ICSC_MAX_COMMANDS; ++i) {
        if (bus->commands[i].command == 0) {
            bus->commands[i].command = command;
            bus->commands[i].callback = callback;
            return;
        }
    }
}
