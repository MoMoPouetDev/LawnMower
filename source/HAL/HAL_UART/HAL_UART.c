/*
 * HAL_UART.c
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_UART.h"
#include "HAL_GPS.h"
#include "LLD_UART.h"


/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define UART_STATUS_TX_IDLE		1302
#define UART_STATUS_RX_IDLE		1303

uint8_t gau8_uart_txBuff[8];
uint8_t gau8_uart_rxBuff[8];
volatile bool rxBufferEmpty            = true;
volatile bool txBufferFull             = false;
volatile bool txOnGoing                = false;
volatile bool rxOnGoing                = false;

static volatile uint8_t gu8_FlagUartGps;
static volatile uint8_t gu8_FlagUartBleTx;
static volatile uint8_t gu8_FlagUartBleRx;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
/**
* @brief		UART callback
* @param		e_Uart : UART number
* @return		void
* @details
**/
void HAL_UART_Callback(LPUART_Type *base, lld_uart_handle_t *handle, status_t status, void *userData)
{
	if (UART_STATUS_TX_IDLE == status)
	{
		gu8_FlagUartBleTx = 1;
	}

	if (UART_STATUS_RX_IDLE == status)
	{
		gu8_FlagUartGps = 1;
		gu8_FlagUartBleRx = 1;
	}
}

void HAL_UART_Init()
{
	gu8_FlagUartGps = 0;
	gu8_FlagUartBleTx = 0;
	gu8_FlagUartBleRx = 0;

	LLD_UART_Init(UART_GPS, GPS_BAUDRATE, HAL_UART_Callback);
	LLD_UART_Init(UART_BLE, BLE_BAUDRATE, HAL_UART_Callback);
}

void HAL_UART_BleInit()
{
	char commandAT[] = "AT",
		commandRole[] = "AT+ROLE0",
		commandUuid[] = "AT+UUID0xFFE0",
		commandChar[] = "AT+CHAR0xFFE1",
		commandName[] = "AT+NAMEMower";

	LLD_UART_Send(UART_BLE, commandAT, 2);
	while(!gu8_FlagUartBleTx);
	gu8_FlagUartBleTx = 0;
	LLD_UART_Send(UART_BLE, commandRole, 8);
	while(!gu8_FlagUartBleTx);
	gu8_FlagUartBleTx = 0;
	LLD_UART_Send(UART_BLE, commandUuid, 13);
	while(!gu8_FlagUartBleTx);
	gu8_FlagUartBleTx = 0;
	LLD_UART_Send(UART_BLE, commandChar, 13);
	while(!gu8_FlagUartBleTx);
	gu8_FlagUartBleTx = 0;
	LLD_UART_Send(UART_BLE, commandName, 12);
	while(!gu8_FlagUartBleTx);
	gu8_FlagUartBleTx = 0;
}

void HAL_UART_Reception()
{
	static uint8_t tu8_uart_rxBuff[1] = {0};
	static uint8_t u8_uartState = 0;

	switch (u8_uartState)
	{
	case 0:
		LLD_UART_Receive(UART_BLE, tu8_uart_rxBuff, 1);
		u8_uartState = 1;
		break;
	case 1:
		if (gu8_FlagUartBleRx == 1)
		{
			gu8_FlagUartBleRx = 0;

			u8_uartState = 0;
		}
		break;

	default:
		u8_uartState = 0;
		break;
	}
}

uint8_t HAL_UART_ReceptionGPS(char* tu8_RxBuffer, uint8_t u8_size)
{	
	static uint8_t _tu8_uart_rxBuff[BUFFER_SIZE_MAX] = {0};
	static uint8_t _tu8_rxBuff[BUFFER_SIZE] = {0};
	static uint8_t _u8_uartState = 0;
	uint8_t u8_flagNMEA = 0;
	uint8_t u8_ind = 0;
	uint8_t u8_returnValue = 0;

	switch (_u8_uartState)
	{
	case 0:
		LLD_UART_Receive(UART_GPS, _tu8_uart_rxBuff, BUFFER_SIZE_MAX);
		_u8_uartState = 1;
		break;
	case 1:
		if (gu8_FlagUartGps == 1)
		{
			gu8_FlagUartGps = 0;

			for (size_t i = 0; i < BUFFER_SIZE_MAX; i++)
			{
				if( _tu8_uart_rxBuff[i] == '$' )
				{
					_tu8_rxBuff[0] = _tu8_uart_rxBuff[i];
					u8_ind = i;
					u8_flagNMEA = 1;
				}
				else if( (u8_flagNMEA == 1) && (_tu8_uart_rxBuff[i] != '\n') )
				{
					_tu8_rxBuff[(i - u8_ind)] = _tu8_uart_rxBuff[i];
				}
				else if( _tu8_uart_rxBuff[i] == '\n')
				{
					_tu8_rxBuff[(i - u8_ind)] = _tu8_uart_rxBuff[i];
					u8_flagNMEA = 2;
					break;
				}
			}
			
			if (u8_flagNMEA == 2)
			{
				if ( _tu8_rxBuff[0] == '$' &&
					_tu8_rxBuff[1] == 'G' && 
					_tu8_rxBuff[3] == 'R' &&
					_tu8_rxBuff[4] == 'M' &&
					_tu8_rxBuff[5] == 'C' )
				{
					u8_returnValue = 1;
					memcpy(tu8_RxBuffer, _tu8_rxBuff, BUFFER_SIZE);
				}
			}

			_u8_uartState = 0;
		}
		break;
	
	default:
		_u8_uartState = 0;
		break;
	}

	return u8_returnValue;
}
void HAL_UART_SendStatus(uint8_t* tu8_uart_txBuff, uint8_t u8_size)
{
	static uint8_t u8_uartState = 0;

	switch (u8_uartState)
	{
		case 0:
			LLD_UART_Send(UART_BLE, tu8_uart_txBuff, u8_size);
			u8_uartState = 1;
			break;
		case 1:
			if (gu8_FlagUartBleTx == 1)
			{
				gu8_FlagUartBleTx = 0;
				u8_uartState = 0;
			}
			break;
		default:
			u8_uartState = 0;
			break;
	}
}

uint8_t HAL_UART_ReceptionBLE(uint8_t* tu8_RxBuffer, uint8_t u8_size)
{
	static uint8_t tu8_uart_rxBuff[1] = {0};
	static uint8_t u8_uartState = 0;
	uint8_t u8_returnValue = 0;

	switch (u8_uartState)
	{
		case 0:
			LLD_UART_Receive(UART_BLE, tu8_uart_rxBuff, 1);
			u8_uartState = 1;
			break;
		case 1:
			if (gu8_FlagUartBleRx == 1)
			{
				gu8_FlagUartBleRx = 0;
				u8_uartState = 0;
				u8_returnValue = 1;
				memcpy(tu8_RxBuffer, tu8_uart_rxBuff, BUFFER_SIZE);
			}
			break;
		default:
			u8_uartState = 0;
			break;
	}
	return u8_returnValue;
}
