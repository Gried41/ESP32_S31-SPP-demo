#ifndef _USER_UART_H_
#define _USER_UART_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/uart.h"
#include "driver/gpio.h"

void uart_init(void);
void uart_task(void *arg);
void uart_spp_set_connection(uint32_t handle, bool connected);

#endif