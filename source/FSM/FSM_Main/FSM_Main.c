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
#include "FSM_Operative.h"
#include "FSM_Main.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
S_SUP_FSM_STATE ge_FSM_Phase;
S_ACU_FSM_STATE ge_FSM_Acu_State;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Main_UpdateFsmAcu(void);
void FSM_Main_CANReceive(void);
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
	ge_FSM_Phase = PHASE_INITIALIZATION_INIT;
	ge_FSM_Acu_State = S_ACU_OPERATIVE_Moving_KTG;
	FSM_Main_UpdateFsmAcu();
	//FSM_Init_Init();
	//FSM_Calibration_Init();
	//FSM_LoadingConso_Init();
	//FSM_GettingReady_Init();
	FSM_Operative_Init();
	/* ... */

	/*** RUN Init ***/
	RUN_Init();
}

void FSM_Main( void )
{
   while(1)
   {
      /***************************************************************************************************************/
      /*                                      MANAGE CAN FRAME                                                       */
      /***************************************************************************************************************/
	   FSM_Main_CANReceive();

      /***************************************************************************************************************/
      /*                                   FINITE STATE MACHINE                                                      */
      /***************************************************************************************************************/


		if ( (ge_FSM_Acu_State >= PHASE_INITIALIZATION_INIT) && (ge_FSM_Acu_State <= PHASE_INITIALIZATION_WAITING_FOR_CALIBRATION))
		{
			//FSM_Init( ge_FSM_PositioningSystem_State );
		}
		else if ( (ge_FSM_Acu_State >= PHASE_CALIBRATION_INIT) && (ge_FSM_Acu_State <= PHASE_CALIBRATION_WAITING_FOR_LOADING_CONSO) )
		{
			//FSM_Calibration( ge_FSM_PositioningSystem_State );
		}
		else if ( (ge_FSM_Acu_State >= PHASE_LOADING_CONSO_INIT) && (ge_FSM_Acu_State <= PHASE_LOADING_CONSO_WAITING_FOR_GETTING_READY) )
		{
			//FSM_LoadingConso( ge_FSM_PositioningSystem_State );
		}
		else if ( (ge_FSM_Acu_State >= PHASE_GETTING_READY_INIT) && (ge_FSM_Acu_State <= PHASE_GETTING_READY_WAITING_FOR_OPERATIVE) )
		{
			//FSM_GettingReady( ge_FSM_PositioningSystem_State );
		}
		else if ( (ge_FSM_Acu_State >= PHASE_OPERATIVE_INIT) && (ge_FSM_Acu_State <= PHASE_OPERATIVE_WAITING_FOR_UNLOADING_CONSO) )
		{
			FSM_Operative( ge_FSM_Acu_State);
		}
		else if ( (ge_FSM_Acu_State >= PHASE_UNLOADING_CONSO_INIT) && (ge_FSM_Acu_State <= PHASE_UNLOADING_CONSO_WAITING_FOR_SHUT_DOWN) )
		{
			//FSM_UnloadingConso( ge_FSM_PositioningSystem_State );
		}
		else if ( (ge_FSM_Acu_State >= PHASE_RESTART_INIT) && (ge_FSM_Acu_State <= PHASE_RESTART_WAITING_FOR_OPERATIVE) )
		{
			//FSM_Restart( ge_FSM_PositioningSystem_State );
		}
		FSM_Main_UpdateFsmAcu();
   }
}

void FSM_Main_UpdateFsmAcu()
{
	//S_FSM_Positioning_System_Modu_U.S_SUP_FSM = ge_FSM_Phase;
	//S_FSM_Positioning_System_Modu_U.Tilting_angle_setpoint_deg = ge_Tilting;
	//S_FSM_Positioning_System_Module_step();
	//ge_FSM_Acu_State = S_FSM_Positioning_System_Modu_Y.S_POSITIONING_SYSTEM_FSM;
}

void FSM_Main_CANReceive()
{
}

