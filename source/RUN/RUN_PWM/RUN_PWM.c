/*
 * RUN_PWM.c
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_GPIO.h"
#include "HAL_PWM.h"
#include "RUN_PWM.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_PWM_Init()
{
	HAL_PWM_Init();
}

void RUN_PWM_Stop()
{
	HAL_GPIO_UpdateWheelState(STOP);
	HAL_PWM_Stop();
}

void RUN_PWM_Backward(uint8_t u8_speed)
{
	HAL_GPIO_UpdateWheelState(BACKWARD);
	HAL_PWM_Backward(u8_speed);
}

void RUN_PWM_Right()
{
	HAL_GPIO_UpdateWheelState(RIGHT);
	HAL_PWM_Right();
}
