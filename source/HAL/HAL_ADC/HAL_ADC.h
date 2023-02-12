/*
 * HAL_ADC.h
 *
 *  Created on: 17 août 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_ADC_HAL_ADC_H_
#define HAL_HAL_ADC_HAL_ADC_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_ADC_Init(void);
void HAL_ADC_ReadValue(void);
void HAL_ADC_GetAdcValue(uint32_t *pu32Etc0Adc1, uint32_t *pu32Etc1Adc1, uint32_t *pu32Etc0Adc2, uint32_t *pu32Etc1Adc2);

#endif /* HAL_HAL_ADC_HAL_ADC_H_ */
