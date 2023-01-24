#include <memory.h>
#include "support.h"
#include "nrf24.h"
//
// Created by ilia.motornyi on 13-Dec-18.
//
// Buffer to store a payload of maximum width

#include "mpu6050.h"
imu_datalar *imu_data;






#define DEMO_RX_SINGLE      0 // Single address receiver (1 pipe)
#define DEMO_RX_MULTI       0 // Multiple address receiver (3 pipes)
#define DEMO_RX_SOLAR       0 // Solar temperature sensor receiver
#define DEMO_TX_SINGLE      0 // Single address transmitter (1 pipe)
#define DEMO_TX_MULTI       0 // Multiple address transmitter (3 pipes)
#define DEMO_RX_SINGLE_ESB  0 // Single address receiver with Enhanced ShockBurst (1 pipe)
#define DEMO_TX_SINGLE_ESB  0 // Single address transmitter with Enhanced ShockBurst (1 pipe)
#define DEMO_RX_ESB_ACK_PL  0 // Single address receiver with Enhanced ShockBurst (1 pipe) + payload sent back
#define DEMO_TX_ESB_ACK_PL  1 // Single address transmitter with Enhanced ShockBurst (1 pipe) + payload received in ACK


// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF


nRF24_TXResult tx_res;
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
	UART_SendStr("[");
	UART_SendHex8(status);
	UART_SendStr("] ");

	// Clear pending IRQ flags
    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}


int runRadio(void) {
	UART_SendStr("\r\FCS is online.\r\n");

	get_Imu(imu_data);
	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+
	UART_SendStr("nRF24L01+ check: ");

	if (!nRF24_Check()) {
		UART_SendStr("FAIL\r\n");
		while (1) {
			Toggle_LED();
			Delay_ms(50);
		}
	}

	UART_SendStr("OK\r\n");

	// Initialize the nRF24L01 to its default state
	nRF24_Init();

/***************************************************************************/

	// This is simple transmitter with Enhanced ShockBurst (to one logic address):
	//   - TX address: 'ESB'
	//   - payload: 10 bytes
	//   - RF channel: 40 (2440MHz)
	//   - data rate: 2Mbps
	//   - CRC scheme: 2 byte

    // The transmitter sends a 10-byte packets to the address 'ESB' with Auto-ACK (ShockBurst enabled)

    // Set RF channel
    nRF24_SetRFChannel(40);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_2Mbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Configure TX PIPE
    static const uint8_t nRF24_ADDR[] = { 'E', 'S', 'B' };
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address
    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR); // program address for pipe#0, must be same as TX (for Auto-ACK)

    // Set TX power (maximum)
    nRF24_SetTXPower(nRF24_TXPWR_12dBm);

    // Configure auto retransmit: 10 retransmissions with pause of 2500s in between
    nRF24_SetAutoRetr(nRF24_ARD_2500us, 10);

    // Enable Auto-ACK for pipe#0 (for ACK packets)
    nRF24_EnableAA(nRF24_PIPE0);

    // Set operational mode (PTX == transmitter)
    nRF24_SetOperationalMode(nRF24_MODE_TX);

    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();

    // Enable DPL
    nRF24_SetDynamicPayloadLength(nRF24_DPL_ON);
	nRF24_SetPayloadWithAck(1);


	// Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);


    uint8_t nRF24_payload[32];
    // Pipe number
    nRF24_RXResult pipe;
    uint32_t i,j,k;
    // Length of received payload
    uint8_t payload_length;

    // Some variables
    uint32_t packets_lost = 0; // global counter of lost packets
    uint8_t otx;
    uint8_t otx_plos_cnt; // lost packet count
	uint8_t otx_arc_cnt; // retransmit count


    // The main loop
    j = 0;

    while (1) {

/*
		payload_length = (uint8_t)(2 + (j + j /10)% 7);

		// Prepare data packet
    	for (i = 0; i < payload_length; i++) {
    		nRF24_payload[i] = (uint8_t) j++;
    		if (j > 0x000000FF) j = 0;
    	}
*/
    	nRF24_payload[0] = imu_data->pitch;
    	nRF24_payload[1] = imu_data->roll;
    	payload_length = 4;
    	// Print a payload
    	UART_SendStr("FCS DATAPAYLOAD:>\r\n");
    	//UART_SendBufHex((char *)nRF24_payload, payload_length);
    	//UART_SendStr()

    	UART_SendStr("< ... TX: ");

    	// Transmit a packet
    	tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
		otx = nRF24_GetRetransmitCounters();
		nRF24_ReadPayloadDpl(nRF24_payload, &payload_length );
		otx_plos_cnt = (otx & nRF24_MASK_PLOS_CNT) >> 4; // packets lost counter
		otx_arc_cnt  = (otx & nRF24_MASK_ARC_CNT); // auto retransmissions counter


    	switch (tx_res) {
			case nRF24_TX_SUCCESS:
				UART_SendStr("OK");
				break;
			case nRF24_TX_TIMEOUT:
				UART_SendStr("TIMEOUT");
				break;
			case nRF24_TX_MAXRT:
				UART_SendStr("MAX RETRANSMIT");
				packets_lost += otx_plos_cnt;
				nRF24_ResetPLOS();
				break;
			default:
				UART_SendStr("ERROR");
				break;
		}
		UART_SendStr("   ACK_PAYLOAD=>");
    	UART_SendBufHex((char *) nRF24_payload, payload_length);
    	UART_SendStr("<   ARC=");
		UART_SendInt(otx_arc_cnt);
		UART_SendStr(" LOST=");
		UART_SendInt(packets_lost);
		UART_SendStr("\r\n");

    	// Wait ~0.5s
    	Delay_ms(50);
		Toggle_LED();
    }


}
