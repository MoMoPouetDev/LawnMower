/*
 * FSM_Operative.c
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"
#include "RUN_ADC.h"
#include "RUN_GPIO.h"

#include "FSM_Enum.h"
#include "FSM_Operative.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
uint8_t gu8_startButtonState;
uint8_t gu8_stopButtonState;
uint8_t gu8_runMowerState;
uint8_t gu8_wireDetectionState;
uint8_t gu8_bumperDetectionState;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Operative_ADCReadValue(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Operative_Init()
{
	gu8_startButtonState = 0;
	gu8_stopButtonState = 0;
	gu8_runMowerState = 0;
	gu8_wireDetectionState = 0;
	gu8_bumperDetectionState = 0;
}

void FSM_Operative(S_MOWER_FSM_STATE e_FSM_Operative_State)
{
	uint32_t u32_CyclicTask;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/

	u32_CyclicTask = RUN_Task_GetCyclicTask();

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

	switch( e_FSM_Operative_State )
   	{
	  	default:
	  	case S_SUP_OPERATIVE_Init:
		 	FSM_Operative_Init();

			FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);

			break;
	  	case S_SUP_OPERATIVE_Moving :

			FSM_Operative_ADCReadValue(u32_CyclicTask);
			FSM_Operative_AnglesRead(u32_CyclicTask);
			FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_Operative_TiltProtection(u32_CyclicTask);

			FSM_Operative_RunMower(u32_CyclicTask);	

			if (gu8_runMowerState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Wire_Detection_Left);
			}
			else if (gu8_runMowerState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Wire_Detection_Right);
			}

			RUN_GPIO_SetEtatMowerInTask();
			RUN_GPIO_SetErrorMowerNtr();
			
			break;
	  	case S_SUP_OPERATIVE_Wire_Detection :
			FSM_Operative_DisableMotor();

			FSM_Operative_ADCReadValue(u32_CyclicTask);
			FSM_Operative_AnglesRead(u32_CyclicTask);
			FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_Operative_TiltProtection(u32_CyclicTask);

			FSM_Operative_WireDetection(u32_CyclicTask);

			if (gu8_wireDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
			else if (gu8_wireDetectionState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Bumper_Detection);
			}
			
		 	break;
	  	case S_SUP_OPERATIVE_Bumper_Detection:
			FSM_Operative_DisableMotor();

			FSM_Operative_ADCReadValue(u32_CyclicTask);
			FSM_Operative_AnglesRead(u32_CyclicTask);
			FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_Operative_TiltProtection(u32_CyclicTask);

			FSM_Operative_BumperDetection(u32_CyclicTask);

			if (gu8_bumperDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}

		 	break;
		case S_SUP_OPERATIVE_Waiting:
			FSM_Operative_DisableAllMotor();
			FSM_Operative_GetFlagStartButton(u32_CyclicTask);

			RUN_GPIO_SetEtatMowerInWait();
			RUN_GPIO_SetErrorMowerNtr();

			if (gu8_startButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
			else if (gu8_stopButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			
		 	break;
	  	case S_SUP_OPERATIVE_Waiting_For_Return_To_Base :
			FSM_Operative_DisableAllMotor();
			RUN_GPIO_SetEtatMowerReturnToBase();

			FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Init);

			break;
   	}
}

void FSM_Operative_ADCReadValue(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_ADC_READ_VALUE) != 0) {
		RUN_ADC_ReadValue();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
}

void FSM_Operative_AnglesRead(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_READ_ANGLES) != 0) {
		RUN_Mower_GetAngles();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_READ_ANGLES);
	}
}

void FSM_Operative_SonarDistance(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SONAR) != 0) {
		RUN_Mower_SonarDistance();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SONAR);
	}
}

void FSM_Operative_RunMower(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_RUN_MOWER) != 0) {
		gu8_runMowerState = RUN_Mower_RunMower();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_RUN_MOWER);
	}
}

void FSM_Operative_GetFlagStartButton(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_READ_START_BUTTON) != 0) {
		gu8_startButtonState = RUN_GPIO_GetStartButton();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_READ_START_BUTTON);
	}
}

void FSM_Operative_GetFlagStopButton(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_READ_STOP_BUTTON) != 0) {
		gu8_stopButtonState = RUN_GPIO_GetStopButton();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_READ_STOP_BUTTON);
	}
}

void FSM_Operative_TiltProtection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_TILT_PROTECTION) != 0) {
		RUN_Mower_TiltProtection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_TILT_PROTECTION);
	}
}

void FSM_Operative_WireDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_WIRE_DETECTION) != 0) {
		gu8_wireDetectionState = RUN_Mower_WireDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_WIRE_DETECTION);
	}
}

void FSM_Operative_BumperDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_BUMPER_DETECTION) != 0) {
		gu8_wireDetectionState = RUN_Mower_BumperDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_BUMPER_DETECTION);
	}
}

void FSM_Operative_DisableAllMotor()
{
	RUN_GPIO_DisableAllMotor();
	RUN_PWM_Stop();
}

void FSM_Operative_DisableMotor()
{
	RUN_GPIO_DisableMotor();
	RUN_PWM_Stop();
}
