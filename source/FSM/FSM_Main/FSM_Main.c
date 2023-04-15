/*
 * FSM_Main.c
 *
 *  Created on: 7 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Init.h"
#include "RUN_Task.h"

#include "FSM_Enum.h"
#include "FSM_Dock.h"
#include "FSM_Operative.h"
#include "FSM_ReturnToBase.h"
#include "FSM_Error.h"
#include "FSM_Main.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
S_MOWER_FSM_STATE ge_FSM_Phase;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Main_UpdateFsmMower(void);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
/**
 * \fn void FSM_Main_Init( void )
 * \brief This function initialize all finite state machine
 * \details - Initialize Event finite state machine
 * - by calling Events_Init()
 * - by calling FSM_Init_Init()
 * - by calling FSM_StopButton_Init()
**/
void FSM_Main_Init( void )
{
	/*** FSM Init ***/
	ge_FSM_Phase = FSM_Enum_GetFsmPhase();
	FSM_Dock_Init();
	FSM_Operative_Init();
	FSM_ReturnToBase_Init();
	FSM_Error_Init();

	/*** RUN Init ***/
	RUN_Init();
}

void FSM_Main( void )
{
   while(1)
   {
      /***************************************************************************************************************/
      /*                                      MANAGE GLOBALE CYCLE TASK FUNCTION                                     */
      /***************************************************************************************************************/
		//HAL_GPS_startGpsAcquisition()
		//HAL_GPIO_UpdateLed()
		//RUN_Mower_GetAngles()
		//FSM_Dock_ADCReadValue(u32_CyclicTask);
		//RUN_BLE
      /***************************************************************************************************************/
      /*                                   FINITE STATE MACHINE                                                      */
      /***************************************************************************************************************/


		if ( (ge_FSM_Phase >= PHASE_DOCK_INIT) && (ge_FSM_Phase <= PHASE_DOCK_WAITING_FOR_LEAVING_DOCK))
		{
			FSM_Dock( ge_FSM_Phase );
		}
		else if ( (ge_FSM_Phase >= PHASE_OPERATIVE_INIT) && (ge_FSM_Phase <= PHASE_OPERATIVE_WAITING_FOR_RETURN_TO_BASE) )
		{
			FSM_Operative( ge_FSM_Phase );
		}
		else if ( (ge_FSM_Phase >= PHASE_RETURN_TO_BASE_INIT) && (ge_FSM_Phase <= PHASE_RETURN_TO_BASE_WAITING_DOCKING) )
		{
			FSM_ReturnToBase( ge_FSM_Phase );
		}
		else if ( (ge_FSM_Phase >= PHASE_ERROR_INIT) && (ge_FSM_Phase <= PHASE_ERROR_INIT) )
		{
			FSM_Error( ge_FSM_Phase );
		}
		FSM_Main_UpdateFsmMower();
   }
}

void FSM_Main_UpdateFsmMower()
{
	ge_FSM_Phase = FSM_Enum_GetFsmPhase();
}
