/*
 * RUN_Mower.h
 *
 *  Created on: 05 MAR 2023
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_MOWER_RUN_MOWER_H_
#define RUN_RUN_MOWER_RUN_MOWER_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_Mower_Init(void);
uint8_t RUN_Mower_LeaveDockCharger(void);
void RUN_Mower_GetAngles(void);
void RUN_Mower_RunMower(void);

#endif /* RUN_RUN_MOWER_RUN_MOWER_H_ */
