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
#define NB_CHANNEL_ETC0_ADC1		1
const uint32_t cs_Etc0_Adc1[NB_CHANNEL_ETC0_ADC1] = { 0 };
#define NB_CHANNEL_ETC1_ADC1		1
const uint32_t cs_Etc1_Adc1[NB_CHANNEL_ETC1_ADC1] = { 15 };

/* TODO RBC : Complete 8 ADC pins array and put it in order of priority, the last one triggers the interruption. */
#define NB_CHANNEL_ETC0_ADC2		1
const uint32_t cs_Etc0_Adc2[NB_CHANNEL_ETC0_ADC2] = { 0 };
#define NB_CHANNEL_ETC1_ADC2		2
const uint32_t cs_Etc1_Adc2[NB_CHANNEL_ETC1_ADC2] = { 9, 10 };

static uint8_t u8_flagIntEtc0;
static uint8_t u8_flagIntEtc1;

typedef struct
{
	uint32_t u32_Translation;
	uint32_t u32_Rotation;
}typ_AdcConversionValue;

static typ_AdcConversionValue gtyp_AdcConversionValueRightJoystick;
static typ_AdcConversionValue gtyp_AdcConversionValueLeftJoystick;

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
void callbackETC0(void)
{
	u8_flagIntEtc0 = 1;
}
void callbackETC1(void)
{
	u8_flagIntEtc1 = 1;
}

void HAL_ADC_Init()
{
	u8_flagIntEtc0 = 0;
	u8_flagIntEtc1 = 0;

	gtyp_AdcConversionValueRightJoystick.u32_Rotation = 0;
	gtyp_AdcConversionValueRightJoystick.u32_Translation = 0;
	gtyp_AdcConversionValueLeftJoystick.u32_Rotation = 0;
	gtyp_AdcConversionValueLeftJoystick.u32_Translation = 0;

    /* Test ADC */
	LLD_ADC_Init(ADC_ETC_PIT0, cs_Etc0_Adc1, NB_CHANNEL_ETC0_ADC1, cs_Etc0_Adc2, NB_CHANNEL_ETC0_ADC2);
	LLD_ADC_PriorityIrq(ADC_ETC_PIT0, ADC_PRIORITY0);
	LLD_ADC_SetCallback(ADC_ETC_PIT0, callbackETC0);

	LLD_ADC_Init(ADC_ETC_PIT1, cs_Etc1_Adc1, NB_CHANNEL_ETC1_ADC1, cs_Etc1_Adc2, NB_CHANNEL_ETC1_ADC2);
	LLD_ADC_PriorityIrq(ADC_ETC_PIT1, ADC_PRIORITY0);
	LLD_ADC_SetCallback(ADC_ETC_PIT1, callbackETC1);

	LLD_ADC_SetConversionRate(ADC_SAMPLE_TIME_3, ADC_RESOLUTION_12BITS, ADC_AVERAGE_COUNT8);
	LLD_ADC_EnableConversion(ADC_ENABLE);
	LLD_ADC_EnableIrq(ADC_ENABLE);

	LLD_TIMER_Start(LLD_TIMER_PIT0);
	LLD_TIMER_Start(LLD_TIMER_PIT1);
}

void HAL_ADC_ReadValue()
{
	if (u8_flagIntEtc0)
	{
		gtyp_AdcConversionValueRightJoystick.u32_Translation = LLD_ADC_ReadConversionValue(ADC_ETC_PIT0, ADC_ETC_ADC1, 0);

		u8_flagIntEtc0 = 0;
	}

	if (u8_flagIntEtc1)
	{
		gtyp_AdcConversionValueRightJoystick.u32_Rotation = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC1, 0);

		gtyp_AdcConversionValueLeftJoystick.u32_Translation = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC2, 0);
		gtyp_AdcConversionValueLeftJoystick.u32_Rotation = LLD_ADC_ReadConversionValue(ADC_ETC_PIT1, ADC_ETC_ADC2, 1);

		u8_flagIntEtc1 = 0;
	}
}

void HAL_ADC_GetAdcValue(uint32_t *pu32_AdcConversionValueRightJoystickTranslation, uint32_t *pu32_AdcConversionValueRightJoystickRotation, uint32_t *pu32_AdcConversionValueLeftJoystickTranslation, uint32_t *pu32_AdcConversionValueLeftJoystickRotation)
{
	*pu32_AdcConversionValueRightJoystickTranslation = gtyp_AdcConversionValueRightJoystick.u32_Translation;
	*pu32_AdcConversionValueRightJoystickRotation = gtyp_AdcConversionValueRightJoystick.u32_Rotation;
	*pu32_AdcConversionValueLeftJoystickTranslation = gtyp_AdcConversionValueLeftJoystick.u32_Translation;
	*pu32_AdcConversionValueLeftJoystickRotation = gtyp_AdcConversionValueLeftJoystick.u32_Rotation;
}
