/*
 * RUN_Task_Interface.c
 *
 *  Created on: 5 oct. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
static uint32_t gArraySlotTask[NB_SLOT_CYCLIC_TASK] =
{
      CYCLIC_TASK_TOUCH_SCREEN , // 0
	  CYCLIC_TASK_ADC_READ_VALUE,  // 1
	  CYCLIC_TASK_TURBO_BUTTON_VALUE | CYCLIC_TASK_ACTIVATION_BUTTON_VALUE,  // 2
	  CYCLIC_TASK_JOYSTICK_VALUE,  // 3
	  0,  // 4
	  CYCLIC_TASK_SEND_CAN_ACTIVATED_RUIP | CYCLIC_TASK_SEND_CAN_KT_RUIP | CYCLIC_TASK_SEND_CAN_MAIN_RUIP | CYCLIC_TASK_SEND_CAN_SECOND_RUIP,  // 5
	  0,  // 6
	  CYCLIC_TASK_SEND_CAN_CUID,  // 7
	  0,  // 8
	  0,  // 9
};

/*--------------------------------------------------------------------------*/
/* ... DATAS ...                                                            */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* ... LOCAL FUNCTIONS DECLARATIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* ... FUNCTIONS IMPLEMENTATIONS...                                         */
/*--------------------------------------------------------------------------*/
void RUN_Task_Interface_Init()
{
	RUN_Task_Sequencer_Init();
	RUN_Task_SetArraySlotTask(gArraySlotTask, NB_SLOT_CYCLIC_TASK);
}

