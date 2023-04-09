/*
 * RUN_Mower.c
 *
 *  Created on: 05 MAR 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "HAL_I2C.h"
#include "HAL_FIFO.h"
#include "HAL_Mower.h"
#include "HAL_Timer.h"
#include "HAL_Sonar.h"
#include "HAL_GPIO.h"
#include "RUN_PWM.h"
#include "RUN_Mower.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define DELTA_ANGLE 5

#define WIRE_DETECTION_LIMITE 600
#define WIRE_DETECTION_LOAD 750
#define WIRE_DETECTION_UNLOAD 450

#define HIGH_SPEED 100
#define MIDDLE_SPEED 90
#define LOW_SPEED 80

#define GPT_ONE_SECOND 100
#define GPT_FIVE_SECOND 500

uint8_t gu8_deltaAngle;
static uint8_t gu8_distanceSonarFC;
static uint8_t gu8_distanceSonarFL;
static uint8_t gu8_distanceSonarFR;
static uint16_t gu16_distanceWireLeft;
static uint16_t gu16_distanceWireRight;
static uint8_t gd_pitch;
static uint8_t gd_roll;
static uint16_t gu16_currentAngle;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Mower_Init()
{
	gu8_deltaAngle = DELTA_ANGLE;
	gu8_distanceSonarFC = 255;
  	gu8_distanceSonarFL = 255;
  	gu8_distanceSonarFR = 255;
  	gu16_distanceWireLeft = WIRE_DETECTION_UNLOAD;
  	gu16_distanceWireRight = WIRE_DETECTION_UNLOAD;
	gd_pitch = 0;
	gd_roll = 0;
	gu16_currentAngle = 0;
}

uint8_t RUN_Mower_LeaveDockCharger()
{
	static uint8_t _u8_leaveState = 0;
	static uint16_t _u16_randAngle = 0;
	static uint16_t _u16_startAngle = 0;
	static uint16_t _u16_endAngle = 0;
	static uint16_t _u16_cptValue = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_leaveState)
   	{
		default:
	  	case 0:
			_u16_randAngle = HAL_Mower_MyRandDeg(180);
			_u16_startAngle = gu16_currentAngle;
			_u8_leaveState = 1;

			break;
		case 1:
			_u16_endAngle = (_u16_startAngle + _u16_randAngle)%180;
			RUN_PWM_Backward(MIDDLE_SPEED);
			_u8_leaveState = 2;

			break;
		case 2 :
			if ( (_u16_cptValue) >= GPT_FIVE_SECOND )
			{
				RUN_PWM_Stop();
				RUN_PWM_Right();
				_u16_cptValue = 0;
				_u8_leaveState = 3;
			}
			else
			{
				_u16_cptValue++;
			}
			
			break;
		case 3 :
			if ( (gu16_currentAngle > ((_u16_endAngle - gu8_deltaAngle)%180)) && (gu16_currentAngle < ((_u16_endAngle + gu8_deltaAngle)%180)) )
			{
				_u8_leaveState = 4;
			}
			else
			{
				gu32_distanceWireRight = HAL_ADC_GetRightWireValue();
				if (gu32_distanceWireRight > WIRE_DETECTION_LIMITE)
				{
					_u8_leaveState = 4;
				}
			}

			break;
		case 4 :
			RUN_PWM_Stop();
			_u8_leaveState = 0;
			u8_returnValue = 1;

			break;
    }
	return u8_returnValue;
}

void RUN_Mower_GetAngles()
{
	static uint8_t _tu8_rxBuffCompass[6] = {0};
	static uint8_t _u8_rxBuffCompassSize = 0;
	static uint8_t _tu8_rxBuffAccel[6] = {0};
	static uint8_t _u8_rxBuffAccelSize = 0;
	static uint8_t _u8_getAngleState = 0;
	uint8_t u8_flagI2c = 0;

	switch (_u8_getAngleState)
	{
		default:
		case 0 :
			u8_flagI2c = HAL_I2C_ReadAccel(_tu8_rxBuffAccel, &_u8_rxBuffAccelSize);
			if (u8_flagI2c)
			{
				HAL_Mower_GetAnglePitchRoll(&gd_pitch, &gd_roll, _tu8_rxBuffAccel, &_u8_rxBuffAccelSize);
				_u8_getAngleState = 1;
			}

			break;
		case 1 :
			u8_flagI2c = HAL_I2C_ReadCompass(_tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
			if (u8_flagI2c)
			{
				gu16_currentAngle = HAL_Mower_getAngleFromNorth(gd_pitch, gd_roll, _tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
				_u8_getAngleState = 0;
			}

			break;
	}
}

void RUN_Mower_SonarDistance()
{
	HAL_Sonar_Distance();
}

void RUN_Mower_TiltProtection()
{
	HAL_Mower_TiltProtection(gd_pitch, gd_roll);
}

uint8_t RUN_Mower_WireDetection()
{
	static uint8_t _u8_wireState = 0;
	static uint16_t _u16_randAngle = 0;
	static uint16_t _u16_startAngle = 0;
	static uint16_t _u16_endAngle = 0;
	static uint8_t _u8_wireValue = 0;
	static uint16_t _u16_cptValue = 0;
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_wireState)
   	{
		default:
	  	case 0:
			_u16_randAngle = HAL_Mower_MyRandDeg(360);
			_u16_startAngle = gu16_currentAngle;
			_u16_endAngle = (_u16_startAngle + _u16_randAngle)%360;
			
			RUN_PWM_Backward(MIDDLE_SPEED);

			if (gu16_distanceWireLeft > WIRE_DETECTION_LIMITE)
			{
				_u8_wireValue = 1;
			}
			else if (gu16_distanceWireRight > WIRE_DETECTION_LIMITE)
			{
				_u8_wireValue = 2;
			}
			_u8_wireState = 1;
			
			break;
		case 1:
			if ( (_u16_cptValue) >= GPT_ONE_SECOND )
			{
				if (_u8_wireValue == 1)
				{
					RUN_PWM_Stop();
					RUN_PWM_Right();
					_u16_cptValue = 0;
					_u8_wireState = 2;
				}
				else if (_u8_wireValue == 2)
				{
					RUN_PWM_Stop();
					RUN_PWM_Left();
					_u16_cptValue = 0;
					_u8_wireState = 2;
				}
				else
				{
					_u16_cptValue = 0;
					_u8_wireState = 2;
				}
			}
			else
			{
				_u16_cptValue++;
			}
			
			break;
		case 2 :
			if ( (gu16_currentAngle > ((_u16_endAngle - gu8_deltaAngle)%180)) && (gu16_currentAngle < ((_u16_endAngle + gu8_deltaAngle)%180)) )
			{
				_u8_wireState = 3;
			}
			else
			{
				gu32_distanceWireRight = HAL_ADC_GetRightWireValue();
				gu32_distanceWireLeft = HAL_ADC_GetLeftWireValue();

				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

				if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_leftBumperState == 1) )
				{
					RUN_PWM_Stop();
					_u8_wireState = 0;
					u8_returnValue = 2;
				}
				else if ((gu32_distanceWireLeft > WIRE_DETECTION_LIMITE) || (gu32_distanceWireRight > WIRE_DETECTION_LIMITE) )
				{
					_u8_wireState = 0;
				}
			}

			break;
		case 3 :
			RUN_PWM_Stop();
			_u8_wireState = 0;
			u8_returnValue = 1;

			break;
    }
	return u8_returnValue;
}

uint8_t RUN_Mower_BumperDetection()
{
	static uint8_t _u8_bumperState = 0;
	static uint16_t _u16_randAngle = 0;
	static uint16_t _u16_startAngle = 0;
	static uint16_t _u16_endAngle = 0;
	static uint16_t _u16_cptValue = 0;
	uint8_t u8_leftBumperState = 0;
	uint8_t u8_centerBumperState = 0;
	uint8_t u8_rightBumperState = 0;
	uint8_t u8_returnValue = 0;

	switch(_u8_bumperState)
   	{
		default:
	  	case 0:
			_u16_randAngle = HAL_Mower_MyRandDeg(360);
			_u16_startAngle = gu16_currentAngle;
			_u16_endAngle = (_u16_startAngle + _u16_randAngle)%360;

			RUN_PWM_Backward(MIDDLE_SPEED);

			_u8_bumperState = 1;
			
			break;
		case 1 :
			if ( (_u16_cptValue) >= GPT_ONE_SECOND )
			{
				RUN_PWM_Stop();
				RUN_PWM_Left();
				_u16_cptValue = 0;
				_u8_bumperState = 2;
			}
			else
			{
				_u16_cptValue++;
			}
			
			break;
		case 2 :
			if ( (gu16_currentAngle > ((_u16_endAngle - gu8_deltaAngle)%180)) && (gu16_currentAngle < ((_u16_endAngle + gu8_deltaAngle)%180)) )
			{
				_u8_bumperState = 3;
			}
			else
			{
				gu32_distanceWireRight = HAL_ADC_GetRightWireValue();
				gu32_distanceWireLeft = HAL_ADC_GetLeftWireValue();

				u8_leftBumperState = HAL_GPIO_GetFlagBumper(E_LEFT_BUMPER);
				u8_centerBumperState = HAL_GPIO_GetFlagBumper(E_CENTER_BUMPER);
				u8_rightBumperState = HAL_GPIO_GetFlagBumper(E_RIGHT_BUMPER);

				if ( (u8_leftBumperState == 1) || (u8_centerBumperState == 1) || (u8_leftBumperState == 1) )
				{
					RUN_PWM_Stop();
					_u8_bumperState = 0;
				}
				else if ((gu32_distanceWireLeft > WIRE_DETECTION_LIMITE) || (gu32_distanceWireRight > WIRE_DETECTION_LIMITE) )
				{
					_u8_bumperState = 3;
				}
			}

			break;
		case 3 :
			RUN_PWM_Stop();
			_u8_bumperState = 0;
			u8_returnValue = 1;

			break;
    }
	return u8_returnValue;
}

uint8_t RUN_Mower_RunMower()
{
	uint8_t u8_returnValue = 0;
	HAL_Sonar_GetDistance(&gu8_distanceSonarFC, &gu8_distanceSonarFL, &gu8_distanceSonarFR);
	HAL_FIFO_GetSonarAverage(&gu8_distanceSonarFC, &gu8_distanceSonarFL, &gu8_distanceSonarFR);

	gu16_distanceWireLeft = HAL_ADC_GetLeftWireValue();
	gu16_distanceWireRight = HAL_ADC_GetRightWireValue();

	if (gu16_distanceWireLeft > WIRE_DETECTION_LIMITE) 
	{
		u8_returnValue = 1;
	}
	else if (gu16_distanceWireRight > WIRE_DETECTION_LIMITE) 
	{
		u8_returnValue = 2;
	}
	else if ((gu8_distanceSonarFC < SONAR_WARN) || (gu8_distanceSonarFL < SONAR_WARN) || (gu8_distanceSonarFR < SONAR_WARN))
	{
		RUN_PWM_Forward(MIDDLE_SPEED);
	}
	else 
	{
		RUN_PWM_Forward(HIGH_SPEED);
	}

	return u8_returnValue;
}
