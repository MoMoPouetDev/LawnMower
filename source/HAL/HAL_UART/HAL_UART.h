/*
 * HAL_UART.h
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_UART_HAL_UART_H_
#define HAL_HAL_UART_HAL_UART_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define UART_BAUDRATE   9600U
#define UART_GPS        LLD_UART_UART3
#define UART_BLE        LLD_UART_UART4
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_UART_Init(void);
uint8_t HAL_UART_ReceptionGPS(char* tu8_RxBuffer, uint8_t u8_size);

#endif /* HAL_HAL_UART_HAL_UART_H_ */
