/*
 * HAL_Sonar.h
 *
 *  Created on: 26 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_SONAR_HAL_SONAR_H_
#define HAL_HAL_SONAR_HAL_SONAR_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define SONAR_DIST_ERR 999
#define TIMER1_OVERFLOW 65535
/*** Calcul of value timer 343 m/s -> 34300 cm/s
 dist = (speedSound*TIMER)/2 = (34300*TIMER)/2 = 17150*TIMER = 17150 * (TIMER_VALUE * 0.125 * 10^-6)
 ***/
#define TIMER_DISTANCE 466.47
#define THRESHOLD_8_BITS 0xFE
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_Sonar_Init(void);
void HAL_Sonar_Distance(void);

#endif /* HAL_HAL_SONAR_HAL_SONAR_H_ */