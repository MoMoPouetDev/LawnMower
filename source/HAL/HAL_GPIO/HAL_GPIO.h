/*
 * HAL_GPIO.h
 *
 *  Created on: 19 août 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_GPIO_HAL_GPIO_H_
#define HAL_HAL_GPIO_HAL_GPIO_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define PIN_LED_RIGHT_BUTTON 			24
#define PIN_LED_LEFT_BUTTON 			22
#define PIN_SW_RIGHT_BUTTON 			9
#define PIN_SW_LEFT_BUTTON 				10
#define PIN_SW_RIGHT_JOYSTICK_BUTTON 	03
#define PIN_SW_LEFT_JOYSTICK_BUTTON 	18

#define LLD_GPIO_LED_RIGHT_BUTTON LLD_GPIO_B1_08
#define LLD_GPIO_LED_LEFT_BUTTON LLD_GPIO_B1_06
#define LLD_GPIO_SW_RIGHT_BUTTON LLD_GPIO_B0_09
#define LLD_GPIO_SW_LEFT_BUTTON LLD_GPIO_B0_10
#define LLD_GPIO_SW_RIGHT_JOYSTICK_BUTTON LLD_GPIO_B0_03
#define LLD_GPIO_SW_LEFT_JOYSTICK_BUTTON LLD_GPIO_B1_02

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_Init(void);

#endif /* HAL_HAL_GPIO_HAL_GPIO_H_ */
