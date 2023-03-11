/*
 * HAL_Mower.h
 *
 *  Created on: 05 MAR 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_MOWER_HAL_MOWER_H_
#define HAL_HAL_MOWER_HAL_MOWER_H_

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
void HAL_Mower_Init(void);
int16_t HAL_Mower_GetAngleFromNorth(double d_pitch, double d_roll, uint8_t* pu8_rxBuffCompass, uint8_t* pu8_rxBuffCompassSize);
void HAL_Mower_GetAnglePitchRoll(double* pd_pitch, double* pd_roll, uint8_t* pu8_rxBuffAccel, uint8_t* pu8_rxBuffAccelSize);

#endif /* HAL_HAL_MOWER_HAL_MOWER_H_ */
