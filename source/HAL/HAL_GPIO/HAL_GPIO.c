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
	LLD_GPIO_Init(E_GREEN_LED, GPIO2, E_GREEN_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_ORANGE_LED, GPIO2, E_ORANGE_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_RED_LED, GPIO2, E_RED_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_YELLOW_ONE_LED, GPIO2, E_YELLOW_ONE_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_YELLOW_TWO_LED, GPIO2, E_YELLOW_TWO_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_YELLOW_THREE_LED, GPIO2, E_YELLOW_THREE_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);

	LLD_GPIO_Init(E_STOP_BUTTON, GPIO2, E_STOP_BUTTON_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(E_START_BUTTON, GPIO2, E_START_BUTTON_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);

	LLD_GPIO_Init(E_LEFT_BUMPER, GPIO2, E_LEFT_BUMPER_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(E_CENTER_BUMPER, GPIO2, E_CENTER_BUMPER_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(E_RIGHT_BUMPER, GPIO2, E_RIGHT_BUMPER_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);

	LLD_GPIO_Init(E_MOTOR_BLADE_ENABLE, GPIO2, E_CENTER_BUMPER_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_ONE_FORWARD_ENABLE, GPIO2, E_MOTOR_ONE_FORWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_ONE_BACKWARD_ENABLE, GPIO2, E_MOTOR_ONE_BACKWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_TWO_FORWARD_ENABLE, GPIO2, E_MOTOR_TWO_FORWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_TWO_BACKWARD_ENABLE, GPIO2, E_MOTOR_TWO_BACKWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);

	LLD_GPIO_Init(E_LEFT_ECHO_SONAR, GPIO2, E_LEFT_ECHO_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(E_CENTER_ECHO_SONAR, GPIO2, E_CENTER_ECHO_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(E_RIGHT_ECHO_SONAR, GPIO2, E_RIGHT_ECHO_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_Init(E_LEFT_TRIGGER_SONAR, GPIO2, E_LEFT_TRIGGER_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_CENTER_TRIGGER_SONAR, GPIO2, E_CENTER_TRIGGER_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_RIGHT_TRIGGER_SONAR, GPIO2, E_RIGHT_TRIGGER_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
}

