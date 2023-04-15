/*
 * FSM_ReturnToBase.c
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"

#include "FSM_Enum.h"
#include "FSM_ReturnToBase.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
uint8_t gu8_angleToBaseState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_ReturnToBase_ADCReadValue(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_ReturnToBase_Init()
{
	gu8_angleToBaseState = 0;
}

void FSM_ReturnToBase(S_MOWER_FSM_STATE e_FSM_ReturnToBase_State)
{
	uint32_t u32_CyclicTask;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/

	u32_CyclicTask = RUN_Task_GetCyclicTask();

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

    switch( e_FSM_ReturnToBase_State )
	{
		default:
	  	case S_SUP_RETURN_TO_BASE_Init:
			FSM_ReturnToBase_Init();
			FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Angle_To_Base);

			break;
		case S_SUP_RETURN_TO_BASE_Angle_To_Base:
			FSM_ReturnToBase_ADCReadValue(u32_CyclicTask);
		  	FSM_ReturnToBase_AnglesRead(u32_CyclicTask);
			FSM_ReturnToBase_GetCoordinates(u32_CyclicTask);
			FSM_ReturnToBase_SonarDistance(u32_CyclicTask);
			FSM_ReturnToBase_SensorRead(u32_CyclicTask);
			FSM_ReturnToBase_TiltProtection(u32_CyclicTask);

		 	FSM_ReturnToBase_GetAngleToBase(u32_CyclicTask);

		 	if (gu8_angleToBaseState == 1)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Moving);	
		 	}
			else if (gu8_angleToBaseState == 2)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Bumper_Detection);	
		 	}
			else if (gu8_angleToBaseState == 3)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Wire_Detection);	
		 	}

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Moving :
			FSM_ReturnToBase_ADCReadValue(u32_CyclicTask);
		  	FSM_ReturnToBase_AnglesRead(u32_CyclicTask);
			FSM_ReturnToBase_SonarDistance(u32_CyclicTask);
			FSM_ReturnToBase_SensorRead(u32_CyclicTask);
			FSM_ReturnToBase_TiltProtection(u32_CyclicTask);

			FSM_Operative_RunMower(u32_CyclicTask);

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Wire_Detection :
		 	FSM_ReturnToBase_ADCReadValue(u32_CyclicTask);

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Bumper_Detection :
		 	FSM_ReturnToBase_ADCReadValue(u32_CyclicTask);

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Wire_Guiding:
		 /* Insert init code */

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Wainting_For_Docking :
		  /* Insert init code */

		 	break;
   	}
}

void FSM_ReturnToBase_ADCReadValue(uint32_t u32_CyclicTask)
{
	/*
	if ( (u32_CyclicTask & CYCLIC_TASK_ADC_READ_VALUE) != 0) {
		RUN_ADC_ReadValue();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
	*/
}
