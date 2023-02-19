/*
 * HAL_ADC.c
 *
 *  Created on: 17 août 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "MIMXRT1061.h"

#include "HAL_ADC.h"
#include "LLD_ADC.h"
#include "LLD_Timer.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
/* TODO RBC : Complete 8 ADC pins array and put it in order of priority, the last one triggers the interruption. */
#define NB_CHANNEL_ETC1_ADC1		2
const uint32_t cs_Etc1_Adc1[NB_CHANNEL_ETC1_ADC1] = { 12, 13 };

/* TODO RBC : Complete 8 ADC pins array and put it in order of priority, the last one triggers the interruption. */
#define NB_CHANNEL_ETC1_ADC2		3
const uint32_t cs_Etc1_Adc2[NB_CHANNEL_ETC1_ADC2] = { 9, 10, 11 };

static uint8_t u8_flagIntEtc1;

static uint8_t u8_adcValueLeftWire;
static uint8_t u8_adcValueRightWire;
static uint8_t u8_adcValueBattVolt;
static uint8_t u8_adcValueBattAmp;
static uint8_t u8_adcValueMotorAmp;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
/**
* @brief		ADC TRIG0 interrupt
* @param		void
* @return		void
* @details
**/
void callbackETC1(void)
{
	u8_flagIntEtc1 = 1;
}

void HAL_ADC_Init()
{
	u8_flagIntEtc1 = 0;

	u8_adcValueLeftWire = 0;
	u8_adcValueRightWire = 0;
  	u8_adcValueBattVolt = 0;
  	u8_adcValueBattAmp = 0;
  	u8_adcValueMotorAmp = 0;

    /* Test ADC */
	LLD_ADC_Init(ADC_ETC_PIT1, cs_Etc1_Adc1, NB_CHANNEL_ETC1_ADC1, cs_Etc1_Adc2, NB_CHANNEL_ETC1_ADC2);
	LLD_ADC_PriorityIrq(ADC_ETC_PIT1, ADC_PRIORITY0);
	LLD_ADC_SetCallback(ADC_ETC_PIT1, callbackETC1);

	LLD_ADC_SetConversionRate(ADC_SAMPLE_TIME_3, ADC_RESOLUTION_12BITS, ADC_AVERAGE_COUNT8);
	LLD_ADC_EnableConversion(ADC_ENABLE);
	LLD_ADC_EnableIrq(ADC_ENABLE);

	LLD_TIMER_Start(LLD_TIMER_PIT1);
}

void HAL_ADC_ReadValue()
{
	if (u8_flagIntEtc1)
	{
		u8_adcValueLeftWire = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC1, 0);
		u8_adcValueRightWire = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC1, 1);

		u8_adcValueBattVolt = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC2, 0);
		u8_adcValueBattAmp = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC2, 1);
		u8_adcValueMotorAmp = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC2, 2);

		u8_flagIntEtc1 = 0;
	}
}

void HAL_ADC_GetAdcValue()
{

}