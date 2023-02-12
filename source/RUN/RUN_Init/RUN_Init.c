/*
 * RUN_Init.c
 *
 *  Created on: 16 août 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "RUN_Task_Interface.h"
#include "RUN_Init.h"
#include "RUN_Timer.h"
#include "RUN_ADC.h"
#include "RUN_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Init()
{
	RUN_Task_Interface_Init();
	RUN_GPIO_Init();
	RUN_Timer_Init();
	RUN_ADC_Init();
	RUN_INIT_I2C();
}
