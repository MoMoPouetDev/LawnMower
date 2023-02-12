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
		 /* Insert init code */

			break;
	  	case S_SUP_OPERATIVE_Moving :
		  /* Insert init code */
			FSM_Operative_ADCReadValue(u32_CyclicTask);	
			
			break;
	  	case S_SUP_OPERATIVE_Wire_Detection_Left :
		  /* Insert init code */
		  

		 	break;
	  	case S_SUP_OPERATIVE_Wire_Detection_Right:
		 /* Insert init code */

		 	break;
	  	case S_SUP_OPERATIVE_Waiting_For_Return_To_Base :
		  /* Insert init code */

			break;
   }
}

void FSM_Operative_ADCReadValue(uint32_t u32_CyclicTask)
{
	/*
	if ( (u32_CyclicTask & CYCLIC_TASK_ADC_READ_VALUE) != 0) {
		RUN_ADC_ReadValue();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
	*/
}
