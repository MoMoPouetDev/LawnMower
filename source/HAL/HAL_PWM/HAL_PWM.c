/*
 * HAL_PWM.c
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_PWM.h"
#include "LLD_PWM.h"


/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_PWM_Init()
{
	LLD_PWM_Init(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 1000);
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 0); 
	LLD_PWM_Enable(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, true);
	LLD_PWM_Init(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, 1000);
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, 0);
	LLD_PWM_Enable(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, true);

	LLD_PWM_Init(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 1000);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 0);
	LLD_PWM_Enable(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, true);
	LLD_PWM_Init(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, 1000);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, 0);
	LLD_PWM_Enable(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, true);
}

void HAL_PWM_Stop()
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 0); 
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, 0);

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 0);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, 0);
}

void HAL_PWM_Backward(uint8_t u8_speed)
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 0); 
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, u8_speed);

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 0);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, u8_speed);
}

void HAL_PWM_Right()
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 100);
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, 0);

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 0);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, 100);
}

void HAL_PWM_Forward(uint8_t u8_speed)
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, u8_speed); 
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, 0);

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, u8_speed);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, 0);
}

void HAL_PWM_Left()
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 0);
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_CHANNEL, 100);

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 100);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_CHANNEL, 0);
}
