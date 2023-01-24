//
// Created by ilia.motornyi on 13-Dec-18.
//

#ifndef __SUPPORT_H
#define __SUPPORT_H

#include "main.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>


extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart3;
#define HEX_CHARS      "0123456789ABCDEF"


void UART_SendChar(char b) ;
void UART_SendStr(char *string);
void Toggle_LED();

void UART_SendBufHex(char *buf, uint16_t bufsize);
void UART_SendHex8(uint16_t num);
void UART_SendInt(int32_t num);

void nRF24_CE_L();
void nRF24_CE_H() ;
void nRF24_CSN_L();
void nRF24_CSN_H();
uint8_t nRF24_LL_RW(uint8_t data);
void Delay_ms(uint32_t ms);

#endif //__SUPPORT_H
