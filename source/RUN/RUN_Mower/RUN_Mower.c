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
#include "HAL_Mower.h"
#include "HAL_Timer.h"
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

#define GPT_FIVE_SECOND 50000

uint8_t gu8_deltaAngle;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Mower_Init()
{
	gu8_deltaAngle = DELTA_ANGLE;
}

void RUN_Mower_LeaveDockCharger()
{
	static uint8_t _u8_leaveState = 0;
	static uint8_t _tu8_rxBuffCompass[6] = {0};
	static uint8_t _u8_rxBuffCompassSize = 0;
	static uint8_t _tu8_rxBuffAccel[6] = {0};
	static uint8_t _u8_rxBuffAccelSize = 0;
	static uint8_t _u8_getAngleState = 0;
	uint8_t _u8_flagI2c = 0;
	static uint16_t _u16_randAngle = 0;
	static uint16_t _u16_startAngle = 0;
	static uint16_t _u16_currentAngle = 0;
	static uint16_t _u16_endAngle = 0;
	static uint16_t _u16_distanceWireRight = WIRE_DETECTION_UNLOAD;
	static uint32_t _u32_gptValue = 0;
	uint32_t _u32_newGptValue = 0;
	static double _d_pitch = 0;
	static double _d_roll = 0;

	switch(_u8_leaveState)
   	{
	  	case 0:
			_u16_randAngle = HAL_Mower_MyRandDeg(180);

			break;

		case 1:
			_u8_flagI2c = HAL_I2C_ReadAccel(_tu8_rxBuffAccel, &_u8_rxBuffAccelSize);

			if (_u8_flagI2c)
			{
				HAL_Mower_GetAnglePitchRoll(&_d_pitch, &_d_roll, _tu8_rxBuffAccel, &_u8_rxBuffAccelSize);
				_u8_leaveState = 2;
			}

			break;

	  	case 2 :
			_u8_flagI2c = HAL_I2C_ReadCompass(_tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
			if (_u8_flagI2c)
			{
				_u16_startAngle = HAL_Mower_GetAngleFromNorth(_d_pitch, _d_roll, _tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
				_u16_endAngle = (_u16_startAngle + _u16_randAngle)%180;
				RUN_PWM_Backward(MIDDLE_SPEED);
				_u32_gptValue = HAL_TIMER_ReadGptValue();
				_u8_leaveState = 3;
			}

		 	break;

		case 3 :
			_u32_newGptValue = HAL_TIMER_ReadGptValue();
			if ( (_u32_newGptValue - _u32_gptValue) >= GPT_FIVE_SECOND )
			{
				RUN_PWM_Stop();
				RUN_PWM_Right();
				_u8_leaveState = 4;
			}
			
			break;

		case 4 :
			if ( (_u16_currentAngle > ((_u16_endAngle - gu8_deltaAngle)%180)) && (_u16_currentAngle < ((_u16_endAngle + gu8_deltaAngle)%180)) )
			{
				_u8_leaveState = 5;
			}
			else
			{
				switch (_u8_getAngleState)
				{
					default:
					case 0 :
						_u8_flagI2c = HAL_I2C_ReadAccel(_tu8_rxBuffAccel, &_u8_rxBuffAccelSize);
						if (_u8_flagI2c)
						{
							HAL_Mower_GetAnglePitchRoll(&_d_pitch, &_d_roll, _tu8_rxBuffAccel, &_u8_rxBuffAccelSize);
							_u8_getAngleState = 1;
						}

						break;

					case 1 :
						_u8_flagI2c = HAL_I2C_ReadCompass(_tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
						if (_u8_flagI2c)
						{
							_u16_currentAngle = MOWER_getAngleFromNorth(_d_pitch, _d_roll, _tu8_rxBuffCompass, &_u8_rxBuffCompassSize);
							_u8_getAngleState = 0;
						}

						break;
				}
			}

			break;

		case 5 :

			break;

		case 6 :

			break;

		default:
			break;
    }
}
