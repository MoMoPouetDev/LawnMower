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
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
volatile uint8_t u8_flagPit;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_Timer_PitHandler()
{
    u8_flagPit = 1;
}

void HAL_Timer_Init()
{
    LLD_TIMER_Init(LLD_TIMER_PIT0, 0);
    LLD_TIMER_PIT_SetTimerInitialValue(LLD_TIMER_PIT0, ADC_CONVERSION_TIME);
    LLD_TIMER_Init(LLD_TIMER_PIT1, 0);
    LLD_TIMER_PIT_SetTimerInitialValue(LLD_TIMER_PIT1, ADC_CONVERSION_TIME);

    LLD_TIMER_Init(LLD_TIMER_PIT2, 0);
    LLD_TIMER_SetCallback(LLD_TIMER_PIT2, RUN_Task_TickIncrement);
    LLD_TIMER_EnableIrq(LLD_TIMER_PIT2, 1);
    LLD_TIMER_PIT_SetTimerInitialValue(LLD_TIMER_PIT2, TICK_TIME);
    LLD_TIMER_Start(LLD_TIMER_PIT2);

    LLD_TIMER_Init(LLD_TIMER_PIT3, 0);
    LLD_TIMER_SetCallback(LLD_TIMER_PIT3, HAL_Timer_PitHandler);
    LLD_TIMER_EnableIrq(LLD_TIMER_PIT3, 1);
    LLD_TIMER_PIT_SetTimerInitialValue(LLD_TIMER_PIT3, PULSE_TIME);

    LLD_TIMER_Init(LLD_TIMER_GPT1, 1);
    LLD_TIMER_Start(LLD_TIMER_GPT1);
}

void HAL_TIMER_StartSonarPulse()
{
    LLD_TIMER_Start(LLD_TIMER_PIT3);
    while (!u8_flagPit);
    LLD_TIMER_Stop(LLD_TIMER_PIT3);
    u8_flagPit = 0;
}

uint32_t HAL_TIMER_ReadGptValue()
{
    uint32_t u8_gptValue;

    LLD_TIMER_Stop(LLD_TIMER_GPT1);
    LLD_TIMER_GPT_Read(LLD_TIMER_GPT1, &u8_gptValue);
    LLD_TIMER_Start(LLD_TIMER_GPT1);

    return u8_gptValue;
}
