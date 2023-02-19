/*
 * HAL_PWM.h
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_PWM_HAL_PWM_H_
#define HAL_HAL_PWM_HAL_PWM_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "LLD_PWM.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define MOTOR_LEFT_CHANNEL  LLD_PWM_CHANNEL_A
#define MOTOR_LEFT_FORWARD LLD_PWM_PWM1_MODULE0
#define MOTOR_LEFT_BACKWARD LLD_PWM_PWM1_MODULE1
#define MOTOR_RIGHT_CHANNEL  LLD_PWM_CHANNEL_B
#define MOTOR_RIGHT_FORWARD LLD_PWM_PWM1_MODULE0
#define MOTOR_RIGHT_BACKWARD LLD_PWM_PWM1_MODULE1

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_UART_Init(void);

#endif /* HAL_HAL_PWM_HAL_PWM_H_ */
