#ifndef __UART_SVC_H__
#define __UART_SVC_H__
#include "main.h"


#define UART_RX_PIN 				3
#define UART_TX_PIN  				4
#define UART_CTS_PIN				5
#define UART_RTS_PIN				6


void uart_init(void);
void uart_send(uint8_t* data, uint16_t size);
void uart_receive(uint8_t* buf, uint16_t size);
void uart_run(void);


#endif









