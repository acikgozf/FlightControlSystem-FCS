/*
 * neo_m8n_gps.c
 *
 *  Created on: 19 Oca 2023
 *      Author: FURKAN
 */




#include "neo_m8n_gps.h"



extern UART_HandleTypeDef huart6;
m8n_ubx_nav_posllh posllh;

uint8_t uart6_rx_flag = 0;
uint8_t uart6_rx_data = 0;

uint8_t uart6_tx_flag = 0;

uint8_t m8n_rx_buf[36];
uint8_t m8n_rx_cplt_flag = 0;

int crc_control_value = 0;

uint8_t usb_data[100];

const unsigned char ubx_cfg_prt[] = {0xb5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xd0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x9a,0x79};
const unsigned char ubx_cfg_msg[] = {0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xbe};
const unsigned char ubx_cfg_rate[] = {0xb5,0x62,0x06,0x08,0x06,0x00,0xc8,0x00,0x01,0x00,0x01,0x00,0xde,0x6a};
const unsigned char ubx_cfg_cfg[] = {0xb5,0x62,0x06,0x09,0x0d,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0x31,0xbf};


void m8n_ubx_init(void){

	//m8n_ubx_default_config(&ubx_cfg_prt[0],sizeof(ubx_cfg_prt));
	//m8n_ubx_default_config(&ubx_cfg_msg[0],sizeof(ubx_cfg_msg));
	//m8n_ubx_default_config(&ubx_cfg_rate[0],sizeof(ubx_cfg_rate));
	//m8n_ubx_default_config(&ubx_cfg_cfg[0],sizeof(ubx_cfg_cfg));

	m8n_ubx_take_data();

}

config_status m8n_ubx_default_config(unsigned char *data, unsigned char len){

	for(int i=0;i<len;i++){

		HAL_UART_Transmit_IT(&huart6, data[i], 1);
		if(uart6_tx_flag == 1){
			uart6_tx_flag =0;
			 CDC_Transmit_FS("check ok", strlen("check ok"));

		}
		else
			CDC_Transmit_FS("check error", strlen("check error"));
	}
}


/* ozet: bu fonksiyon verilerin dogrulugunu kontrol eder. ilk 2 bayt haric diger baytları toplayarak checkum datası olusur.
 *
 */
unsigned char m8n_ubx_checksum_check(unsigned char *data, unsigned char len){

	unsigned char ck_a = 0;
	unsigned char ck_b = 0;

	for(int i =2;i<len-2;i++){

		ck_a = ck_a + data[i];
		ck_b = ck_b + ck_a;

	}
	if((ck_a = data[len-2]) && (ck_b == data[len-1]))
		return crc_ok;
	else
		return crc_error;
}

parsing_status m8n_ubx_parsing(unsigned char *data, m8n_ubx_nav_posllh *posllh){

	posllh->class		= data[2];
	posllh->id			= data[3];
	posllh->length		= data[4] | data[5] << 8; //little endian old icin kaydirildi.
	posllh->itow		= data[6] | data[7] << 8  | data[8] << 16  | data[9] << 24;
	posllh->longitude	= data[10] | data[11] << 8  | data[12] << 16  | data[13] << 24;
	posllh->latitude 	= data[14] | data[15] << 8  | data[16] << 16  | data[17] << 24;
	posllh->heigh		= data[18] | data[19] << 8  | data[20] << 16  | data[21] << 24;	//Elipsoid yükseligi gösterir.
	posllh->hmsl		= data[22] | data[23] << 8  | data[24] << 16  | data[25] << 24; //Deniz seviyesinden yüksekligi gösterir.
	posllh->hacc		= data[26] | data[27] << 8  | data[28] << 16  | data[29] << 24;	//Horizontal accuracy gösterir.
	posllh->vacc		= data[30] | data[31] << 8  | data[32] << 16  | data[33] << 24; //Vertical accuracy gösterir.

	posllh->latitude_f_64 = posllh->latitude /10000000.;
	posllh->longitude_f_64 = posllh->longitude /10000000.;

	sprintf(usb_data,"lat: %ld\tlon: %ld\theigh: %ld\r\n", posllh->latitude, posllh->longitude, posllh->heigh);
	CDC_Transmit_FS(usb_data,strlen(usb_data));

}


void m8n_ubx_take_data(void){

	HAL_UART_Receive_IT(&huart6, &uart6_rx_data, 1);


	   while( m8n_rx_cplt_flag = 1 ){

		   m8n_rx_cplt_flag = 0;
		   if (m8n_ubx_checksum_check(m8n_rx_buf[0], 36) == 1){

			   m8n_ubx_parsing(m8n_rx_buf[0],&posllh);
			   crc_control_value = 1;


		   }
	   }


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	static unsigned char cnt = 0;
	uart6_rx_flag = 1;
	m8n_rx_cplt_flag = 1;
  //HAL_UART_Transmit(huart5, gelen_gps_data, strlen(gelen_gps_data), HAL_MAX_DELAY);


  switch(cnt){

	  case 0:
		  if(uart6_rx_data == 0xb5)
		  {
			  m8n_rx_buf[cnt] = uart6_rx_data;
			  cnt++;

		  }
		  break;
	  case 1:
		  if(uart6_rx_data == 0x62){

			  m8n_rx_buf[cnt] = uart6_rx_data;
			  cnt++;
		  }
		  else
			  cnt = 0;
		  break;

	  case 35:
		  m8n_rx_buf[cnt] = uart6_rx_data;
		  cnt = 0;
		  m8n_rx_cplt_flag = 1;
		  break;
	  default:
		  	m8n_rx_buf[cnt] = uart6_rx_data;
		  	cnt++;
		  	break;
  }

  CDC_Transmit_FS(m8n_rx_buf, strlen(m8n_rx_buf));

 }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	uart6_tx_flag = 1;




}
