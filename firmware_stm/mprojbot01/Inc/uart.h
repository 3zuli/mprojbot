/*
 * uart.h
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 *      https://electronics.stackexchange.com/questions/206113/how-do-i-use-the-printf-function-on-stm32
 */

#ifndef UART_H_
#define UART_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal.h"

#define UART_RXBUF_SIZE 500
int UART2rxi;  // UART2/3 rx buffer index
volatile uint8_t UART2inChar; // UART2/3 last received char
// RX buffers
uint8_t UART2inBuff[UART_RXBUF_SIZE]; // Used for immediate rx data storage
uint8_t UART2LineBuff[UART_RXBUF_SIZE]; // UART2inBuff gets copied here once '\n' is received

bool _lineAvailable;

extern UART_HandleTypeDef huart2;

void uartInit();

void vprint(const char *fmt, va_list argp);

void uartPrintf(const char *fmt, ...); // custom printf() function

void uartRxCallback(UART_HandleTypeDef *huart);

bool uartLineAvailable();

#endif /* UART_H_ */
