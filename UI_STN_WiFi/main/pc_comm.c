#include "pc_comm.h"
#include "hardware.h"
#include "icsc_protocol.h"

#include <string.h>

#define HEARTBEAT_INTERVAL_MS 1000U
#define HEARTBEAT_ACK_TIMEOUT_MS 10000U

static pc_event_handler_t s_handler;
static void *s_handler_ctx;
static uint32_t s_last_heartbeat_send_ms;
static uint32_t s_last_heartbeat_ack_ms;
static uint32_t s_watchdog_start_ms;
static bool s_ack_received;
static bool s_timeout_reported;

static void emit(pc_event_type_t type, const uint8_t *data, uint8_t len)
{
    if (s_handler == NULL) return;
    pc_event_t event = {.type = type, .len = len > sizeof(event.data) ? sizeof(event.data) : len};
    if (data != NULL && event.len > 0) memcpy(event.data, data, event.len);
    s_handler(&event, s_handler_ctx);
}

static void mark_alive(uint32_t now_ms)
{
    s_ack_received = true;
    s_last_heartbeat_ack_ms = now_ms;
    s_timeout_reported = false;
}

static void on_packet(uint8_t station, char command, uint8_t len, uint8_t *data, void *ctx)
{
    (void)ctx;
    if (station != POWER_CONTROLLER_ADDRESS) return;

    switch (command) {
    case CMD_HEARTBEAT_ACK:
        mark_alive(hardware_millis());
        emit(PC_EVENT_HEARTBEAT_ACK, data, len);
        break;
    case CMD_VERSION_REPLY:
        emit(PC_EVENT_VERSION_REPLY, data, len);
        break;
    case CMD_POWER_ON_ACK:
        emit(PC_EVENT_POWER_ON_ACK, data, len);
        break;
    case CMD_ERROR_REPORT:
        emit(PC_EVENT_ERROR_REPORT, data, len);
        break;
    case CMD_SLAVE_STATUS_REPLY:
        emit(PC_EVENT_SLAVE_STATUS_REPLY, data, len);
        break;
    case CMD_SLAVE_SERVICE_ACK:
        emit(PC_EVENT_SLAVE_SERVICE_ACK, data, len);
        break;
    case CMD_CABIN_LIGHT_STATUS:
        emit(PC_EVENT_CABIN_LIGHT_STATUS, data, len);
        break;
    case CMD_DIP_REPLY:
        emit(PC_EVENT_DIP_REPLY, data, len);
        break;
    case CMD_UI_SETTINGS_ACK:
        emit(PC_EVENT_UI_SETTINGS_ACK, data, len);
        break;
    case CMD_FAN_STATUS:
        emit(PC_EVENT_FAN_STATUS, data, len);
        break;
    case CMD_SCENT_PUMP_STATUS:
        emit(PC_EVENT_SCENT_PUMP_STATUS, data, len);
        break;
    default:
        break;
    }
}

static void on_legacy_heartbeat(uint8_t station, char command, uint8_t len, uint8_t *data, void *ctx)
{
    (void)command;
    (void)len;
    (void)data;
    (void)ctx;
    if (station != POWER_CONTROLLER_ADDRESS) return;
    uint8_t payload[1] = {1};
    icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_HEARTBEAT_ACK, sizeof(payload), payload);
}

esp_err_t pc_comm_init(void)
{
    ESP_ERROR_CHECK(icsc_protocol_init(UI_ADDRESS, ICSC_BAUD_RATE, UART_PORT, UART_TX_GPIO, UART_RX_GPIO, RS485_TXEN_GPIO));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_HEARTBEAT, on_legacy_heartbeat, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_HEARTBEAT_ACK, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_VERSION_REPLY, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_POWER_ON_ACK, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_ERROR_REPORT, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_SLAVE_STATUS_REPLY, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_SLAVE_SERVICE_ACK, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_CABIN_LIGHT_STATUS, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_DIP_REPLY, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_UI_SETTINGS_ACK, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_FAN_STATUS, on_packet, NULL));
    ESP_ERROR_CHECK(icsc_protocol_register(CMD_SCENT_PUMP_STATUS, on_packet, NULL));

    uint32_t now = hardware_millis();
    s_watchdog_start_ms = now;
    s_last_heartbeat_send_ms = now - HEARTBEAT_INTERVAL_MS;
    s_last_heartbeat_ack_ms = now;
    s_ack_received = false;
    s_timeout_reported = false;
    return ESP_OK;
}

void pc_comm_set_event_handler(pc_event_handler_t handler, void *ctx)
{
    s_handler = handler;
    s_handler_ctx = ctx;
}

void pc_comm_update(uint32_t now_ms)
{
    icsc_protocol_process();

    if ((uint32_t)(now_ms - s_last_heartbeat_send_ms) >= HEARTBEAT_INTERVAL_MS) {
        uint8_t payload[1] = {1};
        icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_HEARTBEAT, sizeof(payload), payload);
        s_last_heartbeat_send_ms = now_ms;
    }

    bool lost = false;
    if (!s_ack_received) {
        lost = (uint32_t)(now_ms - s_watchdog_start_ms) >= HEARTBEAT_ACK_TIMEOUT_MS;
    } else {
        lost = (uint32_t)(now_ms - s_last_heartbeat_ack_ms) >= HEARTBEAT_ACK_TIMEOUT_MS;
    }

    if (lost && !s_timeout_reported) {
        s_timeout_reported = true;
        emit(PC_EVENT_COMM_TIMEOUT, NULL, 0);
    }
}

esp_err_t pc_comm_send_version_request(void)
{
    uint8_t payload[1] = {1};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_VERSION_REQUEST, sizeof(payload), payload);
}

esp_err_t pc_comm_send_dip_request(void)
{
    uint8_t payload[1] = {1};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_DIP_REQUEST, sizeof(payload), payload);
}

esp_err_t pc_comm_send_ui_settings(uint8_t target_temp_c, uint16_t session_min, uint16_t standby_min)
{
    uint8_t payload[5] = {
        target_temp_c,
        (uint8_t)(session_min & 0xFF),
        (uint8_t)((session_min >> 8) & 0xFF),
        (uint8_t)(standby_min & 0xFF),
        (uint8_t)((standby_min >> 8) & 0xFF),
    };
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_UI_SETTINGS, sizeof(payload), payload);
}

esp_err_t pc_comm_send_power_on(void)
{
    uint8_t payload[1] = {1};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_POWER_ON, sizeof(payload), payload);
}

esp_err_t pc_comm_send_power_off(void)
{
    uint8_t payload[1] = {0};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_POWER_OFF, sizeof(payload), payload);
}

esp_err_t pc_comm_send_operation_mode(uint8_t mode)
{
    uint8_t payload[1] = {mode};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_SET_OPERATION_MODE, sizeof(payload), payload);
}

esp_err_t pc_comm_send_cabin_light(uint8_t percent)
{
    uint8_t payload[1] = {percent};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_CABIN_LIGHT_SETPOINT, sizeof(payload), payload);
}

esp_err_t pc_comm_send_fan(bool fan_on)
{
    uint8_t payload[1] = {fan_on ? 1U : 0U};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_FAN_CONTROL, sizeof(payload), payload);
}

esp_err_t pc_comm_send_scent_pump(uint8_t setpoint)
{
    uint8_t payload[1] = {setpoint};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_SCENT_PUMP_SETPOINT, sizeof(payload), payload);
}

esp_err_t pc_comm_send_drain_fill_start(void)
{
    uint8_t payload[1] = {1};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_DRAIN_FILL_START, sizeof(payload), payload);
}

esp_err_t pc_comm_send_slave_status_request(uint8_t slave_id)
{
    uint8_t payload[1] = {slave_id};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_SLAVE_STATUS_REQUEST, sizeof(payload), payload);
}

esp_err_t pc_comm_send_slave_service_set(uint8_t slave_id, uint8_t requested_status)
{
    uint8_t payload[2] = {slave_id, requested_status};
    return icsc_protocol_send(POWER_CONTROLLER_ADDRESS, CMD_SLAVE_SERVICE_SET, sizeof(payload), payload);
}