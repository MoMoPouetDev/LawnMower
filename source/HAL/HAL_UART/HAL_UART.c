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
#define UART_BAUDRATE   9600U

static volatile uint8_t gu8_FlagUart3;
static volatile uint8_t gu8_FlagUart4;
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
	if(e_Uart == LLD_UART_UART3)
	{
		gu8_FlagUart3 = 1;
	}
	if(e_Uart == LLD_UART_UART4)
	{
		gu8_FlagUart4 = 1;
	}

}

void HAL_UART_Init()
{
	gu8_FlagUart3 = 0;
	gu8_FlagUart4 = 0;

	LLD_UART_Init(LLD_UART_UART3, UART_BAUDRATE, HAL_UART_Callback);
	LLD_UART_Init(LLD_UART_UART4, UART_BAUDRATE, HAL_UART_Callback);
}

void HAL_UART_BleInit()
{
	char commandAT[] = "AT",
		commandRole[] = "AT+ROLE0",
		commandUuid[] = "AT+UUID0xFFE0",
		commandChar[] = "AT+CHAR0xFFE1",
		commandName[] = "AT+NAMEMower";

	LLD_UART_Send(LLD_UART_UART4, commandAT, 2);
}
