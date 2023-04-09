/*
 * FSM_Dock.c
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"
#include "RUN_Sensors.h"
#include "RUN_GPIO.h"
#include "RUN_PWM.h"

#include "FSM_Enum.h"
#include "FSM_Dock.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
uint8_t gu8_isCharging;
uint8_t gu8_leavingDockState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Dock_ADCReadValue(uint32_t u32_CyclicTask);
void FSM_Dock_LeavingDockCharger(uint32_t u32_CyclicTask);
void FSM_Dock_DisableAllMotor(void);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Dock_Init()
{
	gu8_isCharging = 0;
	gu8_isCharging = 0;
}

void FSM_Dock(S_MOWER_FSM_STATE e_FSM_Dock_State)
{
	uint32_t u32_CyclicTask = 0;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/
	u32_CyclicTask = RUN_Task_GetCyclicTask();

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

    switch( e_FSM_Dock_State )
   {

		FSM_Dock_ADCReadValue(u32_CyclicTask);
		FSM_Dock_AnglesRead(u32_CyclicTask);

	  	default:
	  	case S_SUP_DOCK_Init:
			FSM_Dock_Init();
			FSM_Dock_DisableAllMotor();

			if (gu8_isCharging)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_In_Charge);
			}
			else
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Mow);
			}

			break;
	  	case S_SUP_DOCK_In_Charge :

			if ( (RUN_Sensors_GetRainState() == OFF) && (RUN_Sensors_IsEnoughCharged() == 1) && (RUN_GPIO_GetStartButton() == 1) )
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Leaving_Dock);
			}
			else if (gu8_isCharging == 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Mow);
			}
			
			
			RUN_GPIO_SetEtatMowerInCharge();

		 	break;
		case S_SUP_DOCK_Waiting_For_Mow :

			if ( (RUN_Sensors_IsTimeToMow() == 1) && (RUN_Sensors_GetRainState() == OFF) )
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Leaving_Dock);
			}

			RUN_GPIO_SetEtatMowerWaitingForMow();

			if (gu8_isCharging)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_In_Charge);
				RUN_GPIO_ClearStartButton();
				RUN_GPIO_SetErrorMowerNtr();
		  		RUN_GPIO_SetEtatMowerInTask();
			}

			break;
		case S_SUP_DOCK_Waiting_For_Leaving_Dock :
			FSM_Dock_LeavingDockCharger(u32_CyclicTask);
			if (gu8_leavingDockState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Init);
			}

			break;
   }
}

void FSM_Dock_ADCReadValue(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_ADC_READ_VALUE) != 0) {
		RUN_ADC_ReadValue();
		gu8_isCharging = RUN_Sensors_IsCharging();

		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
}

void FSM_Dock_AnglesRead(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_ANGLE_READ) != 0) {
		RUN_Mower_GetAngles();

		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ANGLE_READ);
	}
}

void FSM_Dock_LeavingDockCharger(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_ADC_READ_VALUE) != 0) {
		gu8_isCharging = RUN_Mower_LeaveDockCharger();

		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
}

void FSM_Dock_DisableAllMotor()
{
	RUN_GPIO_DisableAllMotor();
	RUN_PWM_Stop();
}
