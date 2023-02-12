/*
 * HAL_Timer.c
 *
 *  Created on: 17 août 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "RUN_Task.h"
#include "HAL_Timer.h"
#include "HAL_GPIO.h"
#include "LLD_Timer.h"

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_Timer_Init()
{
    LLD_TIMER_Init(LLD_TIMER_PIT0, 0);
    LLD_TIMER_PIT_SetTimerInitialValue(LLD_TIMER_PIT0, 10000);
    LLD_TIMER_Init(LLD_TIMER_PIT1, 0);
    LLD_TIMER_PIT_SetTimerInitialValue(LLD_TIMER_PIT1, 10000);

    LLD_TIMER_Init(LLD_TIMER_PIT2, 0);
    LLD_TIMER_SetCallback(LLD_TIMER_PIT2, RUN_Task_TickIncrement);
    LLD_TIMER_EnableIrq(LLD_TIMER_PIT2, 1);
    LLD_TIMER_PIT_SetTimerInitialValue(LLD_TIMER_PIT2, 1000);
    LLD_TIMER_Start(LLD_TIMER_PIT2);


}

