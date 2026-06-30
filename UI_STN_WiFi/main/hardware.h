#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#define UI_VERSION_MAJOR 1
#define UI_VERSION_MINOR 8

#define UI_ADDRESS 1
#define POWER_CONTROLLER_ADDRESS 2
#define ICSC_BAUD_RATE 57600

/* PCB silkscreen labels. Firmware maps these to logical BTN_* functions. */
#define S1_TOP_GPIO     GPIO_NUM_0   /* PB3 / S1 top -> BTN_UP */
#define S2_LEFT_GPIO    GPIO_NUM_6   /* PB1 / S2 left -> BTN_LIGHT */
#define S3_RIGHT_GPIO   GPIO_NUM_13  /* SW3 / S3 right -> BTN_POWER_MODE */
#define S4_CENTER_GPIO  GPIO_NUM_1   /* PB2 / S4 center -> BTN_MENU_SAVE */
#define S5_BOTTOM_GPIO  GPIO_NUM_7   /* PB0 / S5 bottom -> BTN_DOWN */

#define UART_PORT          UART_NUM_0
#define UART_TX_GPIO       GPIO_NUM_NC
#define UART_RX_GPIO       GPIO_NUM_NC
#define RS485_TXEN_GPIO    GPIO_NUM_8

#define BUZZER_GPIO        GPIO_NUM_14
#define BLUE_LED_GPIO      GPIO_NUM_23
/* Multiplexed 7-segment display, active-low segments and active-low columns. */
#define DISPLAY_SEG_A_GPIO   GPIO_NUM_7
#define DISPLAY_SEG_B_GPIO   GPIO_NUM_6
#define DISPLAY_SEG_C_GPIO   GPIO_NUM_1
#define DISPLAY_SEG_D_GPIO   GPIO_NUM_0
#define DISPLAY_SEG_E_GPIO   GPIO_NUM_3
#define DISPLAY_SEG_F_GPIO   GPIO_NUM_2
#define DISPLAY_SEG_G_GPIO   GPIO_NUM_9
#define DISPLAY_SEG_DP_GPIO  GPIO_NUM_10

#define DISPLAY_COL1_GPIO    GPIO_NUM_5
#define DISPLAY_COL2_GPIO    GPIO_NUM_4
#define DISPLAY_COL3_GPIO    GPIO_NUM_26
#define DISPLAY_COL4_GPIO    GPIO_NUM_25
#define DISPLAY_LED_COL_GPIO GPIO_NUM_24

esp_err_t hardware_init(void);
uint32_t hardware_millis(void);

#endif