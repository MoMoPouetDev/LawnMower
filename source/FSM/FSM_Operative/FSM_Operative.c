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

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Operative_ADCReadValue(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Operative_Init()
{

}

void FSM_Operative(S_ACU_FSM_STATE e_FSM_Operative_State)
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
	  case S_ACU_OPERATIVE_Init:
		 /* Insert init code */

		 break;
	  case S_ACU_OPERATIVE_Loading_Device :
		  /* Insert init code */

		 break;
	  case S_ACU_OPERATIVE_Moving_KTG :
		  /* Insert init code */
		  FSM_Operative_ADCReadValue(u32_CyclicTask);

		 break;
	  case S_ACU_OPERATIVE_Moving_MKT:
		 /* Insert init code */

		 break;
	  case S_ACU_OPERATIVE_Waiting_For_Track_Change :
		  /* Insert init code */

		 break;
	  case S_ACU_OPERATIVE_Sequential_Mode :
		  /* Insert init code */

		 break;
	  case S_ACU_OPERATIVE_Waiting_For_Unloading_Conso:
		 /* Insert init code */

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
