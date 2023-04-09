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
#include "HAL_Timer.h"
#include "HAL_GPIO.h"
#include "LLD_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
EtatMower eEtatMower;
ErrorMower eErrorMower;

volatile uint8_t gu8_flagEcho;
volatile uint8_t gu8_risingEdgeGptValue;
volatile uint8_t gu8_fallingEdgeGptValue;

volatile uint8_t gu8_flagStartButton;
volatile uint8_t gu8_flagStopButton;

volatile uint8_t gu8_flagLeftBumper;
volatile uint8_t gu8_flagCenterBumper;
volatile uint8_t gu8_flagRightBumper;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_LeftEchoHandler(void)
{	
	if (gu8_flagEcho == 0)
	{
		gu8_risingEdgeGptValue = HAL_TIMER_ReadGptValue();
		gu8_flagEcho = 1;
	}
	else if (gu8_flagEcho == 1)
	{
		gu8_risingEdgeGptValue = HAL_TIMER_ReadGptValue();
		gu8_flagEcho = 2;
	}
	
}

void HAL_GPIO_CenterEchoHandler(void)
{
	if (gu8_flagEcho == 0)
	{
		gu8_risingEdgeGptValue = HAL_TIMER_ReadGptValue();
		gu8_flagEcho = 1;
	}
	else if (gu8_flagEcho == 1)
	{
		gu8_risingEdgeGptValue = HAL_TIMER_ReadGptValue();
		gu8_flagEcho = 2;
	}
}

void HAL_GPIO_RightEchoHandler(void)
{
	if (gu8_flagEcho == 0)
	{
		gu8_risingEdgeGptValue = HAL_TIMER_ReadGptValue();
		gu8_flagEcho = 1;
	}
	else if (gu8_flagEcho == 1)
	{
		gu8_risingEdgeGptValue = HAL_TIMER_ReadGptValue();
		gu8_flagEcho = 2;
	}
}

void HAL_GPIO_StopButtonHandler(void)
{
	gu8_flagStopButton = 1;
}

void HAL_GPIO_StartButtonHandler(void)
{
	gu8_flagStartButton = 1;
}

void HAL_GPIO_LeftBumperHandler(void)
{
	gu8_flagLeftBumper = 1;
}

void HAL_GPIO_CenterBumperHandler(void)
{
	gu8_flagCenterBumper = 1;
}

void HAL_GPIO_RightBumperHandler(void)
{
	gu8_flagRightBumper = 1;
}

void HAL_GPIO_Init()
{
	LLD_GPIO_Init(E_GREEN_LED, GPIO2, E_GREEN_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_ORANGE_LED, GPIO2, E_ORANGE_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_RED_LED, GPIO2, E_RED_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_YELLOW_ONE_LED, GPIO2, E_YELLOW_ONE_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_YELLOW_TWO_LED, GPIO2, E_YELLOW_TWO_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_YELLOW_THREE_LED, GPIO2, E_YELLOW_THREE_LED_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);

	LLD_GPIO_SetCallback(E_STOP_BUTTON, HAL_GPIO_StopButtonHandler);
	LLD_GPIO_Init(E_STOP_BUTTON, GPIO2, E_STOP_BUTTON_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_SetCallback(E_START_BUTTON, HAL_GPIO_StartButtonHandler);
	LLD_GPIO_Init(E_START_BUTTON, GPIO2, E_START_BUTTON_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);

	LLD_GPIO_SetCallback(E_LEFT_BUMPER, HAL_GPIO_LeftBumperHandler);
	LLD_GPIO_Init(E_LEFT_BUMPER, GPIO2, E_LEFT_BUMPER_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_SetCallback(E_CENTER_BUMPER, HAL_GPIO_CenterBumperHandler);
	LLD_GPIO_Init(E_CENTER_BUMPER, GPIO2, E_CENTER_BUMPER_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_SetCallback(E_RIGHT_BUMPER, HAL_GPIO_RightBumperHandler);
	LLD_GPIO_Init(E_RIGHT_BUMPER, GPIO2, E_RIGHT_BUMPER_PIN, LLD_GPIO_INT_RISING_EDGE, LLD_GPIO_INPUT, 0);

	LLD_GPIO_Init(E_MOTOR_BLADE_ENABLE, GPIO2, E_CENTER_BUMPER_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_ONE_FORWARD_ENABLE, GPIO2, E_MOTOR_ONE_FORWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_ONE_BACKWARD_ENABLE, GPIO2, E_MOTOR_ONE_BACKWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_TWO_FORWARD_ENABLE, GPIO2, E_MOTOR_TWO_FORWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_MOTOR_TWO_BACKWARD_ENABLE, GPIO2, E_MOTOR_TWO_BACKWARD_ENABLE_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);

	LLD_GPIO_SetCallback(E_LEFT_ECHO_SONAR, HAL_GPIO_LeftEchoHandler);
	LLD_GPIO_Init(E_LEFT_ECHO_SONAR, GPIO2, E_LEFT_ECHO_SONAR_PIN, LLD_GPIO_INT_RISING_OR_FALLING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_SetCallback(E_CENTER_ECHO_SONAR, HAL_GPIO_CenterEchoHandler);
	LLD_GPIO_Init(E_CENTER_ECHO_SONAR, GPIO2, E_CENTER_ECHO_SONAR_PIN, LLD_GPIO_INT_RISING_OR_FALLING_EDGE, LLD_GPIO_INPUT, 0);
	LLD_GPIO_SetCallback(E_RIGHT_ECHO_SONAR, HAL_GPIO_RightEchoHandler);
	LLD_GPIO_Init(E_RIGHT_ECHO_SONAR, GPIO2, E_RIGHT_ECHO_SONAR_PIN, LLD_GPIO_INT_RISING_OR_FALLING_EDGE, LLD_GPIO_INPUT, 0);

	LLD_GPIO_Init(E_LEFT_TRIGGER_SONAR, GPIO2, E_LEFT_TRIGGER_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_CENTER_TRIGGER_SONAR, GPIO2, E_CENTER_TRIGGER_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);
	LLD_GPIO_Init(E_RIGHT_TRIGGER_SONAR, GPIO2, E_RIGHT_TRIGGER_SONAR_PIN, LLD_GPIO_NO_INT_MODE, LLD_GPIO_OUTPUT, 0);

	eEtatMower = UNKNOWN_ETAT;
	eErrorMower = NTR;

	gu8_flagEcho = 0;
	gu8_risingEdgeGptValue = 0;
	gu8_fallingEdgeGptValue = 0;

	gu8_flagStartButton = 0;
	gu8_flagStopButton = 0;

	gu8_flagLeftBumper = 0;
  	gu8_flagCenterBumper = 0;
 	gu8_flagRightBumper = 0;
}

void HAL_GPIO_UpdateLed() {
    switch(eEtatMower) {
        case UNKNOWN_ETAT:
			LLD_GPIO_ClearPin(E_GREEN_LED);
			LLD_GPIO_ClearPin(E_ORANGE_LED);
			LLD_GPIO_ClearPin(E_RED_LED);
            break;
            
        case TACHE_EN_COURS:
			LLD_GPIO_WritePin(E_GREEN_LED);
			LLD_GPIO_ClearPin(E_ORANGE_LED);
			LLD_GPIO_ClearPin(E_RED_LED);
            break;
            
        case RETOUR_STATION:
			LLD_GPIO_ClearPin(E_GREEN_LED);
			LLD_GPIO_WritePin(E_ORANGE_LED);
			LLD_GPIO_ClearPin(E_RED_LED);
            break;
            
        case EN_CHARGE:
			LLD_GPIO_WritePin(E_GREEN_LED);
			LLD_GPIO_WritePin(E_ORANGE_LED);
			LLD_GPIO_WritePin(E_RED_LED);
            break;
            
        case PAS_DE_TACHE_EN_COURS:
			LLD_GPIO_ClearPin(E_GREEN_LED);
			LLD_GPIO_ClearPin(E_ORANGE_LED);
			LLD_GPIO_WritePin(E_RED_LED);
            break;
            
        case PAUSE:
			LLD_GPIO_WritePin(E_GREEN_LED);
			LLD_GPIO_WritePin(E_ORANGE_LED);
			LLD_GPIO_ClearPin(E_RED_LED);
            break;
            
        default:
			LLD_GPIO_ClearPin(E_GREEN_LED);
			LLD_GPIO_ClearPin(E_ORANGE_LED);
			LLD_GPIO_ClearPin(E_RED_LED);
            break;
    }
    
    switch(eErrorMower) {
        case NTR:
			LLD_GPIO_ClearPin(E_YELLOW_ONE_LED);
			LLD_GPIO_ClearPin(E_YELLOW_TWO_LED);
			LLD_GPIO_ClearPin(E_YELLOW_THREE_LED);
            break;
            
        case BLOCKED_MOWER:
			LLD_GPIO_ClearPin(E_YELLOW_ONE_LED);
			LLD_GPIO_ClearPin(E_YELLOW_TWO_LED);
			LLD_GPIO_WritePin(E_YELLOW_THREE_LED);
            break;
            
        case DETECTED_RAIN:
			LLD_GPIO_ClearPin(E_YELLOW_ONE_LED);
			LLD_GPIO_WritePin(E_YELLOW_TWO_LED);
			LLD_GPIO_ClearPin(E_YELLOW_THREE_LED);
            break;
            
        case WIRE_NOT_DETECTED:
			LLD_GPIO_ClearPin(E_YELLOW_ONE_LED);
			LLD_GPIO_WritePin(E_YELLOW_TWO_LED);
			LLD_GPIO_WritePin(E_YELLOW_THREE_LED);
            break;
            
        case LOW_BATTERY:
			LLD_GPIO_WritePin(E_YELLOW_ONE_LED);
			LLD_GPIO_ClearPin(E_YELLOW_TWO_LED);
			LLD_GPIO_ClearPin(E_YELLOW_THREE_LED);
            break;
            
        case VERY_LOW_BATTERY:
			LLD_GPIO_WritePin(E_YELLOW_ONE_LED);
			LLD_GPIO_ClearPin(E_YELLOW_TWO_LED);
			LLD_GPIO_WritePin(E_YELLOW_THREE_LED);
            break;
            
        case EMPTY_BATTERY:
			LLD_GPIO_WritePin(E_YELLOW_ONE_LED);
			LLD_GPIO_WritePin(E_YELLOW_TWO_LED);
			LLD_GPIO_ClearPin(E_YELLOW_THREE_LED);
            break;
            
        default:
			LLD_GPIO_ClearPin(E_YELLOW_ONE_LED);
			LLD_GPIO_ClearPin(E_YELLOW_TWO_LED);
			LLD_GPIO_ClearPin(E_YELLOW_THREE_LED);
            break;
    }
}

void HAL_GPIO_SetEtatMower(EtatMower _eEtatMower)
{
	eEtatMower = _eEtatMower;
}

void HAL_GPIO_SetErrorMower(ErrorMower _eErrorMower)
{
	eErrorMower = _eErrorMower;
}

void HAL_GPIO_WritePinSonar(uint8_t u8_sonarID, uint8_t u8_pinValue)
{
	if(u8_pinValue == 1)
	{
		LLD_GPIO_WritePin(u8_sonarID);
	}
	else
	{
		LLD_GPIO_ClearPin(u8_sonarID);
	}
}

uint8_t HAL_GPIO_GetEchoState()
{
	return gu8_flagEcho;
}

uint32_t HAL_GPIO_GetTimerValue()
{
	uint8_t u8_returnValue = 0;

    u8_returnValue = gu8_fallingEdgeGptValue - gu8_risingEdgeGptValue;

    return u8_returnValue;
}

void HAL_GPIO_UpdateBladeState(Etat e_bladeState)
{
	switch(e_bladeState) {
		case ON:
			LLD_GPIO_WritePin(E_MOTOR_BLADE_ENABLE);
			break;
		default:
		case OFF:
			LLD_GPIO_ClearPin(E_MOTOR_BLADE_ENABLE);
			break;
	}
}

void HAL_GPIO_UpdateWheelState(MotorState e_wheelState)
{
	switch(e_wheelState) {
		default:
		case STOP:
			LLD_GPIO_ClearPin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case FORWARD:
			LLD_GPIO_WritePin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case BACKWARD:
			LLD_GPIO_ClearPin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case LEFT:
			LLD_GPIO_ClearPin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case RIGHT:
			LLD_GPIO_WritePin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
	}
}

void HAL_GPIO_ClearFlagButton(GPIO e_flagButton)
{
	switch (e_flagButton)
	{
		case E_STOP_BUTTON :
			gu8_flagStopButton = 0;
			break;

		case E_START_BUTTON :
			gu8_flagStartButton = 0;
			break;
		
		default:
			break;
	}
}

void HAL_GPIO_ClearFlagBumper(GPIO e_flagBumper)
{
	switch (e_flagBumper)
	{
		case E_LEFT_BUMPER :
			gu8_flagLeftBumper = 0;
			break;

		case E_CENTER_BUMPER :
			gu8_flagCenterBumper = 0;
			break;
		
		case E_RIGHT_BUMPER :
			gu8_flagRightBumper = 0;
			break;

		default:
			break;
	}
}

uint8_t HAL_GPIO_GetFlagButton(GPIO e_flagButton)
{
	uint8_t u8_flagButton = 0;

	switch (e_flagButton)
	{
		case E_STOP_BUTTON :
			u8_flagButton = gu8_flagStopButton;
			gu8_flagStopButton = 0;
			break;

		case E_START_BUTTON :
			u8_flagButton = gu8_flagStartButton;
			gu8_flagStartButton = 0;
			break;
		
		default:
			break;
	}

	return u8_flagButton;
}

uint8_t HAL_GPIO_GetFlagBumper(GPIO e_flagBumper)
{
	uint8_t u8_flagBumper = 0;
	switch (e_flagBumper)
	{
		case E_LEFT_BUMPER :
			u8_flagBumper = gu8_flagLeftBumper;
			gu8_flagLeftBumper = 0;
			break;

		case E_CENTER_BUMPER :
			u8_flagBumper = gu8_flagCenterBumper;
			gu8_flagCenterBumper = 0;
			break;
		
		case E_RIGHT_BUMPER :
			u8_flagBumper = gu8_flagRightBumper;
			gu8_flagRightBumper = 0;
			break;

		default:
			break;
	}
	return u8_flagBumper;
}
