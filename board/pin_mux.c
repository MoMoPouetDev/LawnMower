/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v13.0
processor: MIMXRT1061xxxxA
package_id: MIMXRT1061CVL5A
mcu_data: ksdk2_0
processor_version: 13.0.1
pin_labels:
- {pin_num: M11, pin_signal: GPIO_AD_B0_02, label: 'USB_OTG1_PWR/J24[2]', identifier: SW_RIGHT}
- {pin_num: F14, pin_signal: GPIO_AD_B0_09, label: 'JTAG_TDI/J21[5]/ENET_RST/J22[5]/USER_LED', identifier: USER_LED}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
    BOARD_InitI2C();
    BOARD_InitADC();
    BOARD_InitGPIO();
    BOARD_InitUART();
    BOARD_InitPWM();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitI2C:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: H1, peripheral: LPI2C4, signal: SCL, pin_signal: GPIO_EMC_12}
  - {pin_num: G3, peripheral: LPI2C4, signal: SDA, pin_signal: GPIO_EMC_11}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitI2C
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitI2C(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           

  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_11_LPI2C4_SDA, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_12_LPI2C4_SCL, 0U); 
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitADC:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: K10, peripheral: ADC1, signal: 'IN, 12', pin_signal: GPIO_AD_B1_07}
  - {pin_num: H13, peripheral: ADC1, signal: 'IN, 13', pin_signal: GPIO_AD_B1_08}
  - {pin_num: L12, peripheral: ADC2, signal: 'IN, 9', pin_signal: GPIO_AD_B1_04}
  - {pin_num: K12, peripheral: ADC2, signal: 'IN, 10', pin_signal: GPIO_AD_B1_05}
  - {pin_num: J12, peripheral: ADC2, signal: 'IN, 11', pin_signal: GPIO_AD_B1_06}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitADC
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitADC(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           

  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_GPIO1_IO20, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_GPIO1_IO21, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_GPIO1_IO22, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_GPIO1_IO23, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_GPIO1_IO24, 0U); 
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitGPIO:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: A11, peripheral: GPIO2, signal: 'gpio_io, 16', pin_signal: GPIO_B1_00}
  - {pin_num: B11, peripheral: GPIO2, signal: 'gpio_io, 17', pin_signal: GPIO_B1_01}
  - {pin_num: C11, peripheral: GPIO2, signal: 'gpio_io, 18', pin_signal: GPIO_B1_02}
  - {pin_num: D11, peripheral: GPIO2, signal: 'gpio_io, 19', pin_signal: GPIO_B1_03}
  - {pin_num: E12, peripheral: GPIO2, signal: 'gpio_io, 20', pin_signal: GPIO_B1_04}
  - {pin_num: D12, peripheral: GPIO2, signal: 'gpio_io, 21', pin_signal: GPIO_B1_05}
  - {pin_num: C12, peripheral: GPIO2, signal: 'gpio_io, 22', pin_signal: GPIO_B1_06}
  - {pin_num: B12, peripheral: GPIO2, signal: 'gpio_io, 23', pin_signal: GPIO_B1_07}
  - {pin_num: A12, peripheral: GPIO2, signal: 'gpio_io, 24', pin_signal: GPIO_B1_08}
  - {pin_num: A13, peripheral: GPIO2, signal: 'gpio_io, 25', pin_signal: GPIO_B1_09}
  - {pin_num: B13, peripheral: GPIO2, signal: 'gpio_io, 26', pin_signal: GPIO_B1_10}
  - {pin_num: C13, peripheral: GPIO2, signal: 'gpio_io, 27', pin_signal: GPIO_B1_11}
  - {pin_num: D13, peripheral: GPIO2, signal: 'gpio_io, 28', pin_signal: GPIO_B1_12}
  - {pin_num: D14, peripheral: GPIO2, signal: 'gpio_io, 29', pin_signal: GPIO_B1_13}
  - {pin_num: C14, peripheral: GPIO2, signal: 'gpio_io, 30', pin_signal: GPIO_B1_14}
  - {pin_num: B14, peripheral: GPIO2, signal: 'gpio_io, 31', pin_signal: GPIO_B1_15}
  - {pin_num: D7, peripheral: GPIO2, signal: 'gpio_io, 00', pin_signal: GPIO_B0_00}
  - {pin_num: E7, peripheral: GPIO2, signal: 'gpio_io, 01', pin_signal: GPIO_B0_01}
  - {pin_num: E8, peripheral: GPIO2, signal: 'gpio_io, 02', pin_signal: GPIO_B0_02}
  - {pin_num: D8, peripheral: GPIO2, signal: 'gpio_io, 03', pin_signal: GPIO_B0_03}
  - {pin_num: C8, peripheral: GPIO2, signal: 'gpio_io, 04', pin_signal: GPIO_B0_04}
  - {pin_num: B8, peripheral: GPIO2, signal: 'gpio_io, 05', pin_signal: GPIO_B0_05}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitGPIO
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitGPIO(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           

  IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_GPIO2_IO00, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B0_01_GPIO2_IO01, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B0_02_GPIO2_IO02, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B0_03_GPIO2_IO03, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B0_04_GPIO2_IO04, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B0_05_GPIO2_IO05, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_00_GPIO2_IO16, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_01_GPIO2_IO17, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_02_GPIO2_IO18, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_03_GPIO2_IO19, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_04_GPIO2_IO20, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_05_GPIO2_IO21, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_06_GPIO2_IO22, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_07_GPIO2_IO23, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_08_GPIO2_IO24, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_09_GPIO2_IO25, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_10_GPIO2_IO26, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_11_GPIO2_IO27, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_12_GPIO2_IO28, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_13_GPIO2_IO29, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_14_GPIO2_IO30, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_B1_15_GPIO2_IO31, 0U); 
  IOMUXC_GPR->GPR27 = ((IOMUXC_GPR->GPR27 &
    (~(BOARD_INITGPIO_IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL_MASK))) 
      | IOMUXC_GPR_GPR27_GPIO_MUX2_GPIO_SEL(0x00U) 
    );
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitUART:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: A6, peripheral: LPUART3, signal: TX, pin_signal: GPIO_EMC_13}
  - {pin_num: B6, peripheral: LPUART3, signal: RX, pin_signal: GPIO_EMC_14}
  - {pin_num: A3, peripheral: LPUART4, signal: RX, pin_signal: GPIO_EMC_20}
  - {pin_num: B4, peripheral: LPUART4, signal: TX, pin_signal: GPIO_EMC_19}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitUART
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitUART(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           

  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_13_LPUART3_TX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_14_LPUART3_RX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_19_LPUART4_TX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_20_LPUART4_RX, 0U); 
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPWM:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: G2, peripheral: PWM1, signal: 'A, 0', pin_signal: GPIO_EMC_23}
  - {pin_num: D3, peripheral: PWM1, signal: 'B, 0', pin_signal: GPIO_EMC_24}
  - {pin_num: D2, peripheral: PWM1, signal: 'A, 1', pin_signal: GPIO_EMC_25}
  - {pin_num: B3, peripheral: PWM1, signal: 'B, 1', pin_signal: GPIO_EMC_26}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPWM
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPWM(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           

  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_23_FLEXPWM1_PWMA00, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_24_FLEXPWM1_PWMB00, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_25_FLEXPWM1_PWMA01, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_26_FLEXPWM1_PWMB01, 0U); 
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/