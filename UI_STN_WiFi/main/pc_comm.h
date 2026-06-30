#ifndef PC_COMM_H
#define PC_COMM_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define CMD_HEARTBEAT              'H'
#define CMD_HEARTBEAT_ACK          'h'
#define CMD_VERSION_REQUEST        'V'
#define CMD_VERSION_REPLY          'v'
#define CMD_POWER_ON               'P'
#define CMD_POWER_ON_ACK           'p'
#define CMD_POWER_OFF              'X'
#define CMD_CABIN_LIGHT_SETPOINT   'C'
#define CMD_CABIN_LIGHT_STATUS     'c'
#define CMD_ERROR_REPORT           'E'
#define CMD_SLAVE_STATUS_REQUEST   'Z'
#define CMD_SLAVE_STATUS_REPLY     'z'
#define CMD_SLAVE_SERVICE_SET      'O'
#define CMD_SLAVE_SERVICE_ACK      'o'
#define CMD_DIP_REQUEST            'D'
#define CMD_DIP_REPLY              'd'
#define CMD_UI_SETTINGS            'U'
#define CMD_UI_SETTINGS_ACK        'u'
#define CMD_SET_OPERATION_MODE     'M'
#define CMD_DRAIN_FILL_START       'A'
#define CMD_DRAIN_FILL_STOP        'a'
#define CMD_SCENT_PUMP_SETPOINT    'N'
#define CMD_SCENT_PUMP_STATUS      'n'
#define CMD_FAN_CONTROL            'F'
#define CMD_FAN_STATUS             'f'

typedef enum {
    PC_EVENT_HEARTBEAT_ACK = 0,
    PC_EVENT_COMM_TIMEOUT,
    PC_EVENT_VERSION_REPLY,
    PC_EVENT_POWER_ON_ACK,
    PC_EVENT_ERROR_REPORT,
    PC_EVENT_SLAVE_STATUS_REPLY,
    PC_EVENT_SLAVE_SERVICE_ACK,
    PC_EVENT_CABIN_LIGHT_STATUS,
    PC_EVENT_DIP_REPLY,
    PC_EVENT_UI_SETTINGS_ACK,
    PC_EVENT_FAN_STATUS,
    PC_EVENT_SCENT_PUMP_STATUS
} pc_event_type_t;

typedef struct {
    pc_event_type_t type;
    uint8_t data[32];
    uint8_t len;
} pc_event_t;

typedef void (*pc_event_handler_t)(const pc_event_t *event, void *ctx);

esp_err_t pc_comm_init(void);
void pc_comm_set_event_handler(pc_event_handler_t handler, void *ctx);
void pc_comm_update(uint32_t now_ms);
esp_err_t pc_comm_send_version_request(void);
esp_err_t pc_comm_send_dip_request(void);
esp_err_t pc_comm_send_ui_settings(uint8_t target_temp_c, uint16_t session_min, uint16_t standby_min);
esp_err_t pc_comm_send_power_on(void);
esp_err_t pc_comm_send_power_off(void);
esp_err_t pc_comm_send_operation_mode(uint8_t mode);
esp_err_t pc_comm_send_cabin_light(uint8_t percent);
esp_err_t pc_comm_send_fan(bool fan_on);
esp_err_t pc_comm_send_scent_pump(uint8_t setpoint);
esp_err_t pc_comm_send_drain_fill_start(void);
esp_err_t pc_comm_send_slave_status_request(uint8_t slave_id);
esp_err_t pc_comm_send_slave_service_set(uint8_t slave_id, uint8_t requested_status);

#endif