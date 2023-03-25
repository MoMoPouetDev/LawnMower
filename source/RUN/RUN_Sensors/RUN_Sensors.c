/*
 * RUN_Sensors.c
 *
 *  Created on: 25 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_GPS.h"
#include "HAL_Sonar.h"
#include "HAL_GPIO.h"
#include "HAL_ADC.h"
#include "RUN_Sensors.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
Etat ge_rain;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
uint8_t RUN_Sensors_GetBatteryPercent(void);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Sensors_Init()
{
	ge_rain = ON;

	HAL_GPS_Init();
	HAL_Sonar_Init();
}

uint8_t RUN_Sensors_IsTimeToMow()
{
	uint8_t u8_returnValue = 0;
	uint8_t u8_hours;

	u8_hours = HAL_GPS_GetHours();

	if ((THRESHOLD_HOUR_MIN <= u8_hours) && (u8_hours < THRESHOLD_HOUR_MAX))
	{
		u8_returnValue = 1;
	}
	else
	{
		u8_returnValue = 0;
	}

	return u8_returnValue;
}

uint8_t RUN_Sensors_IsCharging()
{
	uint8_t u8_returnValue = 0;
	uint32_t u8_chargeValue;

	u8_chargeValue = HAL_ADC_GetChargeValue();

	if (u8_chargeValue <= CHARGING_THRESHOLD)
	{
		u8_returnValue = 1;
	}
	else
	{
		u8_returnValue = 0;
	}

	return u8_returnValue;
}

int8_t RUN_Sensors_IsEnoughCharged()
{
	uint8_t battery = 0;
	int8_t u8_returnValue = 0;

	battery = RUN_Sensors_GetBatteryPercent();
	
	if (battery <= SENSOR_V_FAIBLE_WARN) {
		if(battery <= SENSOR_V_FAIBLE_WARN)
		{
			HAL_GPIO_SetErrorMower(LOW_BATTERY);
		}
		else if (battery <= SENSOR_V_FAIBLE_ERR)
		{
			HAL_GPIO_SetErrorMower(VERY_LOW_BATTERY);
		}
		else if (battery <= SENSOR_V_EMPTY) 
		{
			HAL_GPIO_SetErrorMower(EMPTY_BATTERY);
			u8_returnValue = -1;
		}
		else
		{
			HAL_GPIO_SetErrorMower(VERY_LOW_BATTERY);
		}
		
		u8_returnValue = 0;
	}
	else
		u8_returnValue = 1;

	return u8_returnValue;
}

/**********************************************/
//	0%   | 9     | 1,8   | 368,0981595
//	5%   | 9,9   | 1,98  | 404,9079755
//	10%  | 10,8  | 2,16  | 441,7177914
//	15%  | 10,95 |  2,19 | 447,8527607
//	20%  | 11,1  | 2,22  | 453,9877301
//	25%  | 11,18 | 2,236 | 457,2597137
//	30%  | 11,25 | 2,25  | 460,1226994
//	40%  | 11,37 | 2,274 | 465,0306748
//	50%  | 11,49 | 2,298 | 469,9386503
//	60%  | 11,61 | 2,322 | 474,8466258
//	70%  | 11,76 | 2,352 | 480,9815951
//	75%  | 11,84 | 2,368 | 484,2535787
//	80%  | 11,91 | 2,382 | 487,1165644
//	85%  | 12,1  | 2,42  | 494,8875256
//	90%  | 12,3  | 2,46  | 503,0674847
//	95%  | 12,45 | 2,49  | 509,202454
//	100% | 12,6  | 2,52  | 515,3374233
/*********************************************/
uint8_t RUN_Sensors_GetBatteryPercent() 
{
	uint32_t uTension;
	uint8_t uPourcentage = 0;

	uTension = HAL_ADC_GetBatteryValue();
	
	if(uTension < 370) { uPourcentage = 0; }
	else if(uTension >= 370 && uTension < 405) { uPourcentage = 5; }
	else if(uTension >= 405 && uTension < 442) { uPourcentage = 10; }
	else if(uTension >= 442 && uTension < 448) { uPourcentage = 15; }
	else if(uTension >= 448 && uTension < 454) { uPourcentage = 20; }
	else if(uTension >= 454 && uTension < 457) { uPourcentage = 25; }
	else if(uTension >= 457 && uTension < 460) { uPourcentage = 30; }
	else if(uTension >= 460 && uTension < 465) { uPourcentage = 40; }
	else if(uTension >= 465 && uTension < 470) { uPourcentage = 50; }
	else if(uTension >= 470 && uTension < 475) { uPourcentage = 60; }
	else if(uTension >= 475 && uTension < 481) { uPourcentage = 70; }
	else if(uTension >= 481 && uTension < 484) { uPourcentage = 75; }
	else if(uTension >= 484 && uTension < 487) { uPourcentage = 80; }
	else if(uTension >= 487 && uTension < 495) { uPourcentage = 85; }
	else if(uTension >= 495 && uTension < 503) { uPourcentage = 90; }
	else if(uTension >= 503 && uTension < 509) { uPourcentage = 95; }
	else if(uTension >= 509) { uPourcentage = 100; }
	
	return uPourcentage;
}

Etat RUN_Sensors_GetRainState()
{
	return ge_rain;
}

void RUN_Sensors_SetRainState(Etat e_rainState)
{
	ge_rain = e_rainState;
}
