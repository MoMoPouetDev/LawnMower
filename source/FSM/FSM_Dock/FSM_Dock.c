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

#include "FSM_Enum.h"
#include "FSM_Dock.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Dock_ADCReadValue(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Dock_Init()
{

}

void FSM_Dock(S_MOWER_FSM_STATE e_FSM_Dock_State)
{
	uint32_t u32_CyclicTask;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/
	u32_CyclicTask = RUN_Task_GetCyclicTask();

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

    switch( e_FSM_Dock_State )
   {
	  	default:
	  	case S_SUP_DOCK_Init:
		 /* Insert init code */

			break;
	  	case S_SUP_DOCK_In_Charge :
		  /* Insert init code */

		 	break;
		case S_SUP_DOCK_Waiting_For_Mow :
		/* Insert init code */

			break;
		case S_SUP_DOCK_Waiting_For_Leaving_Dock :
		  /* Insert init code */

			break;
   }
}

void FSM_Dock_ADCReadValue(uint32_t u32_CyclicTask)
{
	/*
	if ( (u32_CyclicTask & CYCLIC_TASK_ADC_READ_VALUE) != 0) {
		RUN_ADC_ReadValue();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
	*/
}
