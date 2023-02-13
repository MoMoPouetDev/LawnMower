/**
 * @file LLD_GPIO.h
 * @author ACR
 * @brief Header file for GPIO peripheral
 * @details
**/

#ifndef LLD_GPIO_H_
#define LLD_GPIO_H_

#include "stdbool.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

typedef enum
{
	LLD_GPIO_AD_B0_00,		/* GPIO1_IN00 or GPIO6_IN00 */
	LLD_GPIO_AD_B0_01,		/* GPIO1_IN01 or GPIO6_IN01 */
	LLD_GPIO_AD_B0_02,		/* GPIO1_IN02 or GPIO6_IN02 */
	LLD_GPIO_AD_B0_03,		/* GPIO1_IN03 or GPIO6_IN03 */
	LLD_GPIO_AD_B0_04,		/* GPIO1_IN04 or GPIO6_IN04 */
	LLD_GPIO_AD_B0_05,		/* GPIO1_IN05 or GPIO6_IN05 */
	LLD_GPIO_AD_B0_06,		/* GPIO1_IN06 or GPIO6_IN06 */
	LLD_GPIO_AD_B0_07,		/* GPIO1_IN07 or GPIO6_IN07 */
	LLD_GPIO_AD_B0_08,		/* GPIO1_IN08 or GPIO6_IN08 */
	LLD_GPIO_AD_B0_09,		/* GPIO1_IN09 or GPIO6_IN09 */
	LLD_GPIO_AD_B0_10,		/* GPIO1_IN10 or GPIO6_IN10 */
	LLD_GPIO_AD_B0_11,		/* GPIO1_IN11 or GPIO6_IN11 */
	LLD_GPIO_AD_B0_12,		/* GPIO1_IN12 or GPIO6_IN12 */
	LLD_GPIO_AD_B0_13,		/* GPIO1_IN13 or GPIO6_IN13 */
	LLD_GPIO_AD_B0_14,		/* GPIO1_IN14 or GPIO6_IN14 */
	LLD_GPIO_AD_B0_15,		/* GPIO1_IN15 or GPIO6_IN15 */

	LLD_GPIO_AD_B1_00,		/* GPIO1_IN16 or GPIO6_IN16 */
	LLD_GPIO_AD_B1_01,		/* GPIO1_IN17 or GPIO6_IN17 */
	LLD_GPIO_AD_B1_02,		/* GPIO1_IN18 or GPIO6_IN18 */
	LLD_GPIO_AD_B1_03,		/* GPIO1_IN19 or GPIO6_IN19 */
	LLD_GPIO_AD_B1_04,		/* GPIO1_IN20 or GPIO6_IN20 */ // ADC2 Volt
	LLD_GPIO_AD_B1_05,		/* GPIO1_IN21 or GPIO6_IN21 */ // ADC2 Amp
	LLD_GPIO_AD_B1_06,		/* GPIO1_IN22 or GPIO6_IN22 */ // ADC2 Motor Amp
	LLD_GPIO_AD_B1_07,		/* GPIO1_IN23 or GPIO6_IN23 */ // ADC1 Wire Left
	LLD_GPIO_AD_B1_08,		/* GPIO1_IN24 or GPIO6_IN24 */ // ADC1 Wire Right
	LLD_GPIO_AD_B1_09,		/* GPIO1_IN25 or GPIO6_IN25 */
	LLD_GPIO_AD_B1_10,		/* GPIO1_IN26 or GPIO6_IN26 */
	LLD_GPIO_AD_B1_11,		/* GPIO1_IN27 or GPIO6_IN27 */
	LLD_GPIO_AD_B1_12,		/* GPIO1_IN28 or GPIO6_IN28 */
	LLD_GPIO_AD_B1_13,		/* GPIO1_IN29 or GPIO6_IN29 */
	LLD_GPIO_AD_B1_14,		/* GPIO1_IN30 or GPIO6_IN30 */
	LLD_GPIO_AD_B1_15,		/* GPIO1_IN31 or GPIO6_IN31 */

	LLD_GPIO_B0_00,			/* GPIO2_IN00 or GPIO7_IN00 */ // LED Green
	LLD_GPIO_B0_01,			/* GPIO2_IN01 or GPIO7_IN01 */ // LED Orange
	LLD_GPIO_B0_02,			/* GPIO2_IN02 or GPIO7_IN02 */ // LED Red
	LLD_GPIO_B0_03,			/* GPIO2_IN03 or GPIO7_IN03 */ // LED Yellow 1
	LLD_GPIO_B0_04,			/* GPIO2_IN04 or GPIO7_IN04 */ // LED Yellow 2
	LLD_GPIO_B0_05,			/* GPIO2_IN05 or GPIO7_IN05 */ // LED Yellow 3
	LLD_GPIO_B0_06,			/* GPIO2_IN06 or GPIO7_IN06 */
	LLD_GPIO_B0_07,			/* GPIO2_IN07 or GPIO7_IN07 */
	LLD_GPIO_B0_08,			/* GPIO2_IN08 or GPIO7_IN08 */
	LLD_GPIO_B0_09,			/* GPIO2_IN09 or GPIO7_IN09 */
	LLD_GPIO_B0_10,			/* GPIO2_IN10 or GPIO7_IN10 */
	LLD_GPIO_B0_11,			/* GPIO2_IN11 or GPIO7_IN11 */
	LLD_GPIO_B0_12,			/* GPIO2_IN12 or GPIO7_IN12 */
	LLD_GPIO_B0_13,			/* GPIO2_IN13 or GPIO7_IN13 */
	LLD_GPIO_B0_14,			/* GPIO2_IN14 or GPIO7_IN14 */
	LLD_GPIO_B0_15,			/* GPIO2_IN15 or GPIO7_IN15 */

	LLD_GPIO_B1_00,			/* GPIO2_IN16 or GPIO7_IN16 */ // Stop Button
	LLD_GPIO_B1_01,			/* GPIO2_IN17 or GPIO7_IN17 */ // Start button
	LLD_GPIO_B1_02,			/* GPIO2_IN18 or GPIO7_IN18 */ // Bumper Left
	LLD_GPIO_B1_03,			/* GPIO2_IN19 or GPIO7_IN19 */ // Bumper Center
	LLD_GPIO_B1_04,			/* GPIO2_IN20 or GPIO7_IN20 */ // Bumper Right
	LLD_GPIO_B1_05,			/* GPIO2_IN21 or GPIO7_IN21 */ // Motor EN Blade
	LLD_GPIO_B1_06,			/* GPIO2_IN22 or GPIO7_IN22 */ // Motor 1 EN Forward
	LLD_GPIO_B1_07,			/* GPIO2_IN23 or GPIO7_IN23 */ // Motor 1 EN Backward
	LLD_GPIO_B1_08,			/* GPIO2_IN24 or GPIO7_IN24 */ // Motor 2 EN Forward
	LLD_GPIO_B1_09,			/* GPIO2_IN25 or GPIO7_IN25 */ // Motor 2 EN BackWard
	LLD_GPIO_B1_10,			/* GPIO2_IN26 or GPIO7_IN26 */ // Echo Center
	LLD_GPIO_B1_11,			/* GPIO2_IN27 or GPIO7_IN27 */ // Echo Left
	LLD_GPIO_B1_12,			/* GPIO2_IN28 or GPIO7_IN28 */ // Echo Right
	LLD_GPIO_B1_13,			/* GPIO2_IN29 or GPIO7_IN29 */ // Trig Center
	LLD_GPIO_B1_14,			/* GPIO2_IN30 or GPIO7_IN30 */ // Trig Left
	LLD_GPIO_B1_15,			/* GPIO2_IN31 or GPIO7_IN31 */ // Trig Right

	LLD_GPIO_SD_B1_00,		/* GPIO3_IN00 or GPIO8_IN00 */
	LLD_GPIO_SD_B1_01,		/* GPIO3_IN01 or GPIO8_IN01 */
	LLD_GPIO_SD_B1_02,		/* GPIO3_IN02 or GPIO8_IN02 */
	LLD_GPIO_SD_B1_03,		/* GPIO3_IN03 or GPIO8_IN03 */
	LLD_GPIO_SD_B1_04,		/* GPIO3_IN04 or GPIO8_IN04 */
	LLD_GPIO_SD_B1_05,		/* GPIO3_IN05 or GPIO8_IN05 */
	LLD_GPIO_SD_B1_06,		/* GPIO3_IN06 or GPIO8_IN06 */
	LLD_GPIO_SD_B1_07,		/* GPIO3_IN07 or GPIO8_IN07 */
	LLD_GPIO_SD_B1_08,		/* GPIO3_IN08 or GPIO8_IN08 */
	LLD_GPIO_SD_B1_09,		/* GPIO3_IN09 or GPIO8_IN09 */
	LLD_GPIO_SD_B1_10,		/* GPIO3_IN10 or GPIO8_IN10 */
	LLD_GPIO_SD_B1_11,		/* GPIO3_IN11 or GPIO8_IN11 */

	LLD_GPIO_SD_B0_00,		/* GPIO3_IN12 or GPIO8_IN12 */
	LLD_GPIO_SD_B0_01,		/* GPIO3_IN13 or GPIO8_IN13 */
	LLD_GPIO_SD_B0_02,		/* GPIO3_IN14 or GPIO8_IN14 */
	LLD_GPIO_SD_B0_03,		/* GPIO3_IN15 or GPIO8_IN15 */
	LLD_GPIO_SD_B0_04,		/* GPIO3_IN16 or GPIO8_IN16 */
	LLD_GPIO_SD_B0_05,		/* GPIO3_IN17 or GPIO8_IN17 */

	LLD_GPIO_EMC_32,		/* GPIO3_IN18 or GPIO8_IN18 */
	LLD_GPIO_EMC_33,		/* GPIO3_IN19 or GPIO8_IN19 */
	LLD_GPIO_EMC_34,		/* GPIO3_IN20 or GPIO8_IN20 */
	LLD_GPIO_EMC_35,		/* GPIO3_IN21 or GPIO8_IN21 */
	LLD_GPIO_EMC_36,		/* GPIO3_IN22 or GPIO8_IN22 */
	LLD_GPIO_EMC_37,		/* GPIO3_IN23 or GPIO8_IN23 */
	LLD_GPIO_EMC_38,		/* GPIO3_IN24 or GPIO8_IN24 */
	LLD_GPIO_EMC_39,		/* GPIO3_IN25 or GPIO8_IN25 */
	LLD_GPIO_EMC_40,		/* GPIO3_IN26 or GPIO8_IN26 */
	LLD_GPIO_EMC_41,		/* GPIO3_IN27 or GPIO8_IN27 */

	LLD_GPIO_EMC_00,		/* GPIO4_IN00 or GPIO9_IN00 */
	LLD_GPIO_EMC_01,		/* GPIO4_IN01 or GPIO9_IN01 */
	LLD_GPIO_EMC_02,		/* GPIO4_IN02 or GPIO9_IN02 */
	LLD_GPIO_EMC_03,		/* GPIO4_IN03 or GPIO9_IN03 */
	LLD_GPIO_EMC_04,		/* GPIO4_IN04 or GPIO9_IN04 */
	LLD_GPIO_EMC_05,		/* GPIO4_IN05 or GPIO9_IN05 */
	LLD_GPIO_EMC_06,		/* GPIO4_IN06 or GPIO9_IN06 */
	LLD_GPIO_EMC_07,		/* GPIO4_IN07 or GPIO9_IN07 */
	LLD_GPIO_EMC_08,		/* GPIO4_IN08 or GPIO9_IN08 */
	LLD_GPIO_EMC_09,		/* GPIO4_IN09 or GPIO9_IN09 */
	LLD_GPIO_EMC_10,		/* GPIO4_IN10 or GPIO9_IN10 */
	LLD_GPIO_EMC_11,		/* GPIO4_IN11 or GPIO9_IN11 */ // SDA I2C4
	LLD_GPIO_EMC_12,		/* GPIO4_IN12 or GPIO9_IN12 */ // SCL I2C4
	LLD_GPIO_EMC_13,		/* GPIO4_IN13 or GPIO9_IN13 */ // Tx UART3
	LLD_GPIO_EMC_14,		/* GPIO4_IN14 or GPIO9_IN14 */ // Rx UART3
	LLD_GPIO_EMC_15,		/* GPIO4_IN15 or GPIO9_IN15 */ // Tx UART4
	LLD_GPIO_EMC_16,		/* GPIO4_IN16 or GPIO9_IN16 */ // Rx UART4
	LLD_GPIO_EMC_17,		/* GPIO4_IN17 or GPIO9_IN17 */
	LLD_GPIO_EMC_18,		/* GPIO4_IN18 or GPIO9_IN18 */
	LLD_GPIO_EMC_19,		/* GPIO4_IN19 or GPIO9_IN19 */
	LLD_GPIO_EMC_20,		/* GPIO4_IN20 or GPIO9_IN20 */
	LLD_GPIO_EMC_21,		/* GPIO4_IN21 or GPIO9_IN21 */
	LLD_GPIO_EMC_22,		/* GPIO4_IN22 or GPIO9_IN22 */
	LLD_GPIO_EMC_23,		/* GPIO4_IN23 or GPIO9_IN23 */ // PWM Motor 1
	LLD_GPIO_EMC_24,		/* GPIO4_IN24 or GPIO9_IN24 */
	LLD_GPIO_EMC_25,		/* GPIO4_IN25 or GPIO9_IN25 */ // PWM Motor 2
	LLD_GPIO_EMC_26,		/* GPIO4_IN26 or GPIO9_IN26 */
	LLD_GPIO_EMC_27,		/* GPIO4_IN27 or GPIO9_IN27 */
	LLD_GPIO_EMC_28,		/* GPIO4_IN28 or GPIO9_IN28 */
	LLD_GPIO_EMC_29,		/* GPIO4_IN29 or GPIO9_IN29 */
	LLD_GPIO_EMC_30,		/* GPIO4_IN30 or GPIO9_IN30 */
	LLD_GPIO_EMC_31,		/* GPIO4_IN31 or GPIO9_IN31 */

	LLD_GPIO_WAKEUP,		/* GPIO5_IN00 */
	LLD_GPIO_PMIC_ON,		/* GPIO5_IN01 */
	LLD_GPIO_PMIC_STBY,		/* GPIO5_IN02 */

	LLD_NB_GPIO
}lld_gpio_t;

typedef enum
{
	LLD_GPIO_INPUT,
	LLD_GPIO_OUTPUT
}lld_gpio_pin_direction_t;

typedef enum
{
	LLD_GPIO_NO_INT_MODE,
	LLD_GPIO_INT_LOW_LEVEL,
	LLD_GPIO_INT_HIGH_LEVEL,
	LLD_GPIO_INT_RISING_EDGE,
	LLD_GPIO_INT_FALLING_EDGE,
	LLD_GPIO_INT_RISING_OR_FALLING_EDGE,
}lld_gpio_interrupt_mode_t;

typedef void (*lld_gpio_transfer_callback_t)(void);

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/

void LLD_GPIO_Init( lld_gpio_t e_Gpio, GPIO_Type * ps_Port, uint32_t u32_Pin, lld_gpio_interrupt_mode_t e_ItMode,
					lld_gpio_pin_direction_t e_Direction, uint8_t u8_OutputValue);
void LLD_GPIO_WritePin(lld_gpio_t e_Gpio);
void LLD_GPIO_ClearPin(lld_gpio_t e_Gpio);
uint8_t LLD_GPIO_ReadPin(lld_gpio_t e_Gpio);
void LLD_GPIO_Toggle(lld_gpio_t e_Gpio);
void LLD_GPIO_EnableIt(lld_gpio_t e_Gpio, bool b_Enable);
void LLD_GPIO_SetCallback(lld_gpio_t e_Gpio, lld_gpio_transfer_callback_t pf_Callback);


#endif /* LLD_GPIO_H_ */
