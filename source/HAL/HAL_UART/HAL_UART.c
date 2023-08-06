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
void HAL_UART_SendStatus() {
	Coordinates tLatitude;
	Coordinates tLongitude;
	uint8_t uHoursGPS;
	uint8_t uMinutesGPS;
	uint8_t uDaysGPS;
	uint8_t uMonthsGPS;
	uint8_t angleLSB, angleMSB;
	uint16_t angleW;
	float angle;

	
	//angle = MOWER_getAngleFromNorth();
	angleW = (uint16_t) angle;
	angleLSB = angleW & 0xFF;
	angleMSB = (angleW >> 8) & 0xFF;
	
	//LLD_UART_Send(UART_BLE, );
	/*
    UART_transmission(_eEtatMower);
    UART_transmission(_eErrorMower);
    UART_transmission(_uBattery);
	
	UART_transmission(tLatitude.degrees);
	UART_transmission(tLatitude.minutes);
	UART_transmission(tLatitude.decimalMSB);
	UART_transmission(tLatitude.decimalB);
	UART_transmission(tLatitude.decimalLSB);
	
	UART_transmission(tLongitude.degrees);
	UART_transmission(tLongitude.minutes);
	UART_transmission(tLongitude.decimalMSB);
	UART_transmission(tLongitude.decimalB);
	UART_transmission(tLongitude.decimalLSB);
	
	UART_transmission(uHoursGPS);
	UART_transmission(uMinutesGPS);
	UART_transmission(uDaysGPS);
	UART_transmission(uMonthsGPS);
	
	UART_transmission(angleMSB);
	UART_transmission(angleLSB);
	*/
}

void HAL_UART_ReceivedStatus() {
    //LLD_UART_Receive(UART_BLE, );
	/*
	_eCommandMower = UDR0;
    
    switch(_eCommandMower) {
        case START:
            _uBpStop = 0;
            _uBpStart ^= (1<<1);
            break;
            
        case STOP:
            _uBpStop = 1;
            if((_eEtatRain == ON) && (isDocking()))
                _eEtatRain = OFF;
            break;
            
        case FORCE_START:
            _uBpForceStart = 1;
            break;
			
		case DOCK_ON:
			_uDock = 1;
			break;
			
		case DOCK_OFF:
			_uDock = 0;
			break;
            
        default:
            break;
    }
	*/
}
