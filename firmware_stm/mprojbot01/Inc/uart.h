/*
 * uart.h
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 *      https://electronics.stackexchange.com/questions/206113/how-do-i-use-the-printf-function-on-stm32
 */

#ifndef UART_H_
#define UART_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

void vprint(const char *fmt, va_list argp);

void uartPrintf(const char *fmt, ...); // custom printf() function

#endif /* UART_H_ */
