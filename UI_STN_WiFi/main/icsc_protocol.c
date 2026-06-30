#include "icsc_protocol.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define SOH 1
#define STX 2
#define ETX 3
#define EOT 4

static const char *TAG = "icsc";

typedef struct {
    char command;
    icsc_callback_t cb;
    void *ctx;
} command_slot_t;

static command_slot_t s_commands[ICSC_MAX_COMMANDS];
static uint8_t s_station;
static uart_port_t s_uart;
static gpio_num_t s_txen_gpio;
static uint32_t s_baud;
static uint8_t s_header[6];
static uint8_t s_data[ICSC_MAX_MESSAGE];
static uint8_t s_phase;
static uint8_t s_pos;
static uint8_t s_rec_station;
static uint8_t s_rec_sender;
static char s_rec_command;
static uint8_t s_rec_len;
static uint8_t s_rec_checksum;
static uint8_t s_calc_checksum;
static int64_t s_last_byte_us;

static void reset_parser(void)
{
    s_phase = 0;
    s_pos = 0;
    s_rec_len = 0;
    s_rec_checksum = 0;
    s_calc_checksum = 0;
}

esp_err_t icsc_protocol_init(uint8_t station, uint32_t baud, uart_port_t uart_port, gpio_num_t tx_gpio, gpio_num_t rx_gpio, gpio_num_t txen_gpio)
{
    s_station = station;
    s_uart = uart_port;
    s_txen_gpio = txen_gpio;
    s_baud = baud;
    memset(s_commands, 0, sizeof(s_commands));
    memset(s_header, 0, sizeof(s_header));
    reset_parser();

    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(s_uart, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(s_uart, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_level(s_txen_gpio, 0);
    ESP_LOGI(TAG, "initialized station=%u baud=%lu", (unsigned)s_station, (unsigned long)s_baud);
    return ESP_OK;
}

esp_err_t icsc_protocol_register(char command, icsc_callback_t cb, void *ctx)
{
    for (int i = 0; i < ICSC_MAX_COMMANDS; i++) {
        if (s_commands[i].command == command || s_commands[i].command == 0) {
            s_commands[i].command = command;
            s_commands[i].cb = cb;
            s_commands[i].ctx = ctx;
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

esp_err_t icsc_protocol_send(uint8_t station, char command, uint8_t len, const uint8_t *data)
{
    uint8_t packet[5 + 4 + 1 + ICSC_MAX_MESSAGE + 3];
    uint8_t checksum = 0;
    size_t p = 0;

    if (len > ICSC_MAX_MESSAGE) return ESP_ERR_INVALID_SIZE;

    for (int i = 0; i < 5; i++) packet[p++] = SOH;
    packet[p++] = station; checksum += station;
    packet[p++] = s_station; checksum += s_station;
    packet[p++] = (uint8_t)command; checksum += (uint8_t)command;
    packet[p++] = len; checksum += len;
    packet[p++] = STX;
    for (uint8_t i = 0; i < len; i++) {
        packet[p++] = data ? data[i] : 0;
        checksum += data ? data[i] : 0;
    }
    packet[p++] = ETX;
    packet[p++] = checksum;
    packet[p++] = EOT;

    gpio_set_level(s_txen_gpio, 1);
    esp_rom_delay_us(5);
    int written = uart_write_bytes(s_uart, (const char *)packet, p);
    uart_wait_tx_done(s_uart, pdMS_TO_TICKS(20));
    gpio_set_level(s_txen_gpio, 0);

    return written == (int)p ? ESP_OK : ESP_FAIL;
}

static void dispatch_packet(void)
{
    for (int i = 0; i < ICSC_MAX_COMMANDS; i++) {
        if (s_commands[i].command == s_rec_command && s_commands[i].cb != NULL) {
            s_commands[i].cb(s_rec_sender, s_rec_command, s_rec_len, s_data, s_commands[i].ctx);
        }
    }
}

static void process_byte(uint8_t inch)
{
    s_last_byte_us = esp_timer_get_time();

    switch (s_phase) {
    case 0:
        memmove(&s_header[0], &s_header[1], 5);
        s_header[5] = inch;
        if (s_header[0] == SOH && s_header[5] == STX && s_header[1] != s_header[2]) {
            s_rec_station = s_header[1];
            s_rec_sender = s_header[2];
            s_rec_command = (char)s_header[3];
            s_rec_len = s_header[4];
            s_calc_checksum = s_header[1] + s_header[2] + s_header[3] + s_header[4];
            s_pos = 0;

            if (s_rec_station != s_station && s_rec_station != ICSC_BROADCAST) {
                reset_parser();
                return;
            }
            if (s_rec_len > ICSC_MAX_MESSAGE) {
                reset_parser();
                return;
            }
            s_phase = (s_rec_len == 0) ? 2 : 1;
        }
        break;

    case 1:
        s_data[s_pos++] = inch;
        s_calc_checksum += inch;
        if (s_pos >= s_rec_len) s_phase = 2;
        break;

    case 2:
        if (inch == ETX) s_phase = 3;
        else reset_parser();
        break;

    case 3:
        s_rec_checksum = inch;
        s_phase = 4;
        break;

    case 4:
        if (inch == EOT && s_rec_checksum == s_calc_checksum) {
            dispatch_packet();
        }
        reset_parser();
        break;

    default:
        reset_parser();
        break;
    }
}

void icsc_protocol_process(void)
{
    uint8_t buf[64];
    int got = 0;
    do {
        got = uart_read_bytes(s_uart, buf, sizeof(buf), 0);
        for (int i = 0; i < got; i++) process_byte(buf[i]);
    } while (got > 0);
}