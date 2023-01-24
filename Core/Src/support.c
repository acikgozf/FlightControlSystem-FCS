/*
 * support.c
 *
 *  Created on: Jan 17, 2023
 *      Author: FURKAN
 */
#include "support.h"


__IO ITStatus UartReady = RESET;

void UART_SendChar(char b) {
	//HAL_UART_Transmit(&huart3, (uint8_t *) &b, 1, 200);

	//HAL_UART_Transmit_IT(huart3, &b, strlen(b));


		CDC_Transmit_FS(&b, strlen(b));
}

void UART_SendStr(char *string) {
	//HAL_UART_Transmit(&huart3, (uint8_t *) string, (uint16_t) strlen(string), 200);


	//HAL_UART_Transmit_IT(huart3, string, strlen(string));


	CDC_Transmit_FS(string, strlen(string));


}

void Toggle_LED() {
	//HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
}



void UART_SendBufHex(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(HEX_CHARS[(ch >> 4)   % 0x10]);
		UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}
void UART_SendHex8(uint16_t num) {
	UART_SendChar(HEX_CHARS[(num >> 4)   % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendInt(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	do str[i++] = (char) (num % 10 + '0'); while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}


/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
  Set transmission flag: transfer complete

 }



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	UartReady = SET;

}
*/
  void nRF24_CE_L() {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

  void nRF24_CE_H() {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

  void nRF24_CSN_L() {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

  void nRF24_CSN_H() {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}


  uint8_t nRF24_LL_RW(uint8_t data) {
    // Wait until TX buffer is empty
    uint8_t result;
    if(HAL_SPI_TransmitReceive(&hspi3,&data,&result,1,2000)!=HAL_OK) {
        Error_Handler();
    }
    return result;
}


  void Delay_ms(uint32_t ms) { HAL_Delay(ms); }
