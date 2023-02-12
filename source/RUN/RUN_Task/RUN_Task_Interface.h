/*
 * RUN_Task_Interface.h
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_TASK_RUN_TASK_INTERFACE_H_
#define RUN_RUN_TASK_RUN_TASK_INTERFACE_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define NB_SLOT_CYCLIC_TASK 10u

#define CYCLIC_TASK_TOUCH_SCREEN	0x00000001
#define CYCLIC_TASK_ADC_READ_VALUE	0x00000002
#define CYCLIC_TASK_TURBO_BUTTON_VALUE	0x00000004
#define CYCLIC_TASK_ACTIVATION_BUTTON_VALUE	0x00000008
#define CYCLIC_TASK_JOYSTICK_VALUE	0x00000010
#define CYCLIC_TASK_SEND_CAN_CUID	0x00000020
#define CYCLIC_TASK_SEND_CAN_ACTIVATED_RUIP	0x00000040
#define CYCLIC_TASK_SEND_CAN_KT_RUIP	0x00000080
#define CYCLIC_TASK_SEND_CAN_MAIN_RUIP	0x00000100
#define CYCLIC_TASK_SEND_CAN_SECOND_RUIP	0x00000200

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_Task_Interface_Init(void);

#endif /* RUN_RUN_TASK_RUN_TASK_INTERFACE_H_ */
