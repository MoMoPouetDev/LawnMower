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
#include "LLD_UART.h"


/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
static volatile uint8_t gu8_FlagUartGps;
static volatile uint8_t gu8_FlagUartBle;
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
void HAL_UART_Callback(typ_Lld_Uart e_Uart)
{
	if(e_Uart == UART_GPS)
	{
		gu8_FlagUartGps = 1;
	}
	if(e_Uart == UART_BLE)
	{
		gu8_FlagUartBle = 1;
	}

}

void HAL_UART_Init()
{
	gu8_FlagUartGps = 0;
	gu8_FlagUartBle = 0;

	LLD_UART_Init(UART_GPS, UART_BAUDRATE, HAL_UART_Callback);
	LLD_UART_Init(UART_BLE, UART_BAUDRATE, HAL_UART_Callback);
}

void HAL_UART_BleInit()
{
	char commandAT[] = "AT",
		commandRole[] = "AT+ROLE0",
		commandUuid[] = "AT+UUID0xFFE0",
		commandChar[] = "AT+CHAR0xFFE1",
		commandName[] = "AT+NAMEMower";

	LLD_UART_Send(UART_BLE, commandAT, 2);
}

uint8_t HAL_UART_ReceptionGPS(char* tu8_RxBuffer, uint8_t u8_size) 
{	
	static uint8_t u8_uartState = 0;
	uint8_t u8_returnValue = 0;

	switch (u8_uartState)
	{
	case 0:
		LLD_UART_Receive(UART_GPS, tu8_RxBuffer, 1);
		u8_uartState = 1;
		break;
	case 1:
		if (gu8_FlagUartGps == 1)
		{
			gu8_FlagUartGps = 0;

			if (tu8_RxBuffer[0] == '$')
			{
				LLD_UART_Receive(UART_GPS, tu8_RxBuffer, u8_size);
				u8_uartState = 2;
			}
			else
			{
				u8_uartState = 0;
			}
		}
		break;
	case 2:
		if (gu8_FlagUartGps == 1)
		{
			gu8_FlagUartGps = 0;
			u8_uartState = 0;
			u8_returnValue = 1;
		}
		break;
	
	default:
		u8_uartState = 0;
		break;
	}

	return u8_returnValue;
}
