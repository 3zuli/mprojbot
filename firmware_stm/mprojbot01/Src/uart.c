/*
 * uart.c
 *
 *  Created on: 3. 11. 2018
 *      Author: Adam
 */

#include "uart.h"

void uartInit(){
	memset(UART2inBuff, 0, UART_RXBUF_SIZE);
	memset(UART2LineBuff, 0, UART_RXBUF_SIZE);
	_lineAvailable = false;
	huart2.Instance->CR1 = USART_CR1_RE|USART_CR1_TE|USART_CR1_UE|USART_CR1_RXNEIE;
}

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void uartPrintf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

void uartRxCallback(UART_HandleTypeDef *huart){
	uint8_t data = 0;
	if(huart->Instance->SR & USART_SR_RXNE){
		data = (uint8_t)(huart->Instance->DR & 0xFF);
	}
//	asm("nop");

//	if(data!='\n'){
		UART2inBuff[UART2rxi]=data;
		UART2rxi++;
//	}
	if(UART2rxi>=UART_RXBUF_SIZE){
		UART2rxi=0;
		memset(UART2inBuff, 0, UART_RXBUF_SIZE);
	}
	if(data=='\n'){
		UART2inBuff[UART2rxi]='\0';
		memcpy(&UART2LineBuff, &UART2inBuff, UART2rxi+1);
		UART2rxi=0;
		_lineAvailable = true;
	}
}

bool uartLineAvailable(){
	if(_lineAvailable){
		_lineAvailable = false;
		return true;
	}
	else {
		return false;
	}
}
