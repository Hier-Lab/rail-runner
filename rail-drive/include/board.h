#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#define BOARD_LED_GPIO_PORT GPIOA
#define BOARD_LED_PIN GPIO_PIN_5

#define BOARD_CAN1_RX_GPIO_PORT GPIOB
#define BOARD_CAN1_RX_PIN GPIO_PIN_8
#define BOARD_CAN1_TX_PIN GPIO_PIN_9
#define BOARD_CAN1_GPIO_AF GPIO_AF9_CAN1

#define BOARD_UART_GPIO_PORT GPIOA
#define BOARD_UART_TX_PIN GPIO_PIN_2
#define BOARD_UART_RX_PIN GPIO_PIN_3
#define BOARD_UART_GPIO_AF GPIO_AF7_USART2

#define BOARD_HARD_LIMIT_GPIO_PORT GPIOC
#define BOARD_HARD_LIMIT_PIN GPIO_PIN_9
#define BOARD_ESTOP_GPIO_PORT GPIOA
#define BOARD_ESTOP_PIN GPIO_PIN_1
#define BOARD_HOME_SWITCH_GPIO_PORT GPIOC
#define BOARD_HOME_SWITCH_PIN GPIO_PIN_8
#define BOARD_GATE_HOME_GPIO_PORT GPIOC
#define BOARD_GATE_HOME_PIN GPIO_PIN_10
#define BOARD_GATE_FAR_GPIO_PORT GPIOC
#define BOARD_GATE_FAR_PIN GPIO_PIN_11

/** Initializes the HAL and drive board's GPIO assignments. */
void board_init(void);

/** Returns true while the normally-open far (hard-limit) switch is inactive. */
bool board_hard_limits_ok(void);

/** Returns true while the normally-closed estop chain is healthy. */
bool board_estop_ok(void);

/** Returns true while the normally-open home switch is asserted. */
bool board_home_switch_active(void);

/** Returns true while the home-side optical gate is triggered. */
bool board_gate_home_active(void);

/** Returns true while the far-side optical gate is triggered. */
bool board_gate_far_active(void);

#endif
