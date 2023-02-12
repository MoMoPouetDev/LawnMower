/*
 * HAL_GPIO.c
 *
 *  Created on: 19 août 2022
 *      Author: morgan.venandy
 */


/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "MIMXRT1061.h"
#include "HAL_GPIO.h"
#include "LLD_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
uint8_t gu8_flagLeftLed = 0;
uint8_t gu8_flagRightLed = 0;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_ReadLeftActivationButton(void);

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/

void HAL_GPIO_Init()
{
	LLD_GPIO_Init(LLD_GPIO_LED_RIGHT_BUTTON, GPIO1, PIN_LED_RIGHT_BUTTON, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(LLD_GPIO_LED_LEFT_BUTTON, GPIO1, PIN_LED_LEFT_BUTTON, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);

	LLD_GPIO_Init(LLD_GPIO_SW_RIGHT_BUTTON, GPIO1, PIN_SW_RIGHT_BUTTON, LLD_GPIO_NO_INT_MODE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(LLD_GPIO_SW_LEFT_BUTTON, GPIO1, PIN_SW_LEFT_BUTTON, LLD_GPIO_NO_INT_MODE, LLD_GPIO_INPUT, 0);

	LLD_GPIO_Init(LLD_GPIO_SW_RIGHT_JOYSTICK_BUTTON, GPIO1, PIN_SW_RIGHT_JOYSTICK_BUTTON, LLD_GPIO_NO_INT_MODE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(LLD_GPIO_SW_LEFT_JOYSTICK_BUTTON, GPIO1, PIN_SW_LEFT_JOYSTICK_BUTTON, LLD_GPIO_NO_INT_MODE, LLD_GPIO_INPUT, 0);

}

void HAL_GPIO_ReadLeftActivationButton()
{
	uint8_t u8_buttonValue = 1;
	static uint8_t u8_flagButton = 0;
	static uint8_t u8_flagLed = 0;
	uint8_t u8_ActivatedButton = 0;

	u8_buttonValue = LLD_GPIO_ReadPin(LLD_GPIO_SW_LEFT_BUTTON);

	if(u8_buttonValue == 0)
	{
		if(!u8_flagButton)
		{
			if(!gu8_flagLeftLed)
			{
				u8_ActivatedButton = 1;
				gu8_flagLeftLed = 1;
			}
			else
			{
				u8_ActivatedButton = 0;
				gu8_flagLeftLed = 0;
			}

			u8_flagButton = 1;
		}
	}
	else
	{
		u8_flagButton = 0;
	}
}
