/*
 * initialisation.c
 *
 * Created: 20/10/2021 00:04:54
 *  Author: morgan
 */ 

#include "sam.h"
#include "constant.h"
#include "initialisation.h"

void Initialisation() {
	INIT_io();
	INIT_variable();
	INIT_pwm();
	INIT_timer();
	INIT_i2c();
	INIT_uart();
	INIT_adc();
	INIT_wdt();
	INIT_interrupt();
	INIT_compass();
	INIT_accel();
	INIT_ble();
}

void INIT_io() {
	/*** Bouton Poussoir ***/
	//STOP EXTINT10
	PORT->Group[0].PINCFG[10].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA10;
	PORT->Group[0].PINCFG[10].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[5].bit.PMUXO = PORT_PMUX_PMUXO_A;
	//START EXTINT11
	PORT->Group[0].PINCFG[11].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA11;
	PORT->Group[0].PINCFG[11].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[5].bit.PMUXE = PORT_PMUX_PMUXO_A;
	
	/*** Bumper ***/
	//PA17 PA18 PA19 EXTINT1 EXTINT2 EXTINT3
	PORT->Group[0].PINCFG[17].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA17;
	PORT->Group[0].PINCFG[17].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[8].bit.PMUXE = PORT_PMUX_PMUXO_A;

	PORT->Group[0].PINCFG[18].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA18;
	PORT->Group[0].PINCFG[18].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[9].bit.PMUXE = PORT_PMUX_PMUXO_A;

	PORT->Group[0].PINCFG[19].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	REG_PORT_OUTSET0 |= PORT_PA19;
	PORT->Group[0].PINCFG[19].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[9].bit.PMUXE = PORT_PMUX_PMUXO_A;
	
	/*** Sonar ***/
	// echo PA22 PA23 PA24
	PORT->Group[0].PINCFG[22].bit.INEN = 1;
	PORT->Group[0].PINCFG[23].bit.INEN = 1;
	PORT->Group[0].PINCFG[24].bit.INEN = 1;
	
	// trigger PA25 PA27 PA28
	REG_PORT_DIRSET0 |= PORT_PA25 | PORT_PA27 | PORT_PA28;
	REG_PORT_OUTCLR0 |= PORT_PA25 | PORT_PA27 | PORT_PA28;
	
	/*** LED ***/
	// PB22 PB23 PA30 PA31 PB02 PB03
	REG_PORT_DIRSET0 |= PORT_PA30 | PORT_PA31;
	REG_PORT_DIRSET1 |= PORT_PB22 | PORT_PB23 | PORT_PB02 | PORT_PB03;
	REG_PORT_OUTCLR0 |= PORT_PA30 | PORT_PA31;
	REG_PORT_OUTCLR1 |= PORT_PB22 | PORT_PB23 | PORT_PB02 | PORT_PB03;
	
	/*** PWM ***/
	/*** Moteur 1 ***/
	// PWM -> PA07; PA12 PA13
	REG_PORT_DIRSET0 |= PORT_PA07 | PORT_PA12 | PORT_PA13;
	REG_PORT_OUTCLR0 |= PORT_PA07 | PORT_PA12 | PORT_PA13;
	PORT->Group[0].PINCFG[7].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[3].bit.PMUXO = PORT_PMUX_PMUXO_E;
	
	/*** Moteur 2 ***/
	// PWM -> PA16; PA20 PA21
	REG_PORT_DIRSET0 |= PORT_PA16 | PORT_PA20 | PORT_PA21;
	REG_PORT_OUTCLR0 |= PORT_PA16 | PORT_PA20 | PORT_PA21;
	PORT->Group[0].PINCFG[16].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[8].bit.PMUXE = PORT_PMUX_PMUXE_E;
	
	/*** Moteur Lame ***/
	// PB10
	REG_PORT_DIRSET1 |= PORT_PB10;
	REG_PORT_OUTCLR1 |= PORT_PB10;
	
	/*** ADC ***/
	// ADC -> PA02 .. PA06
	PORT->Group[0].PINCFG[2].bit.INEN = 1;
	PORT->Group[0].PINCFG[3].bit.INEN = 1;
	PORT->Group[0].PINCFG[4].bit.INEN = 1;	
	PORT->Group[0].PINCFG[5].bit.INEN = 1;
	PORT->Group[0].PINCFG[6].bit.INEN = 1;
		
	/*** I2C ***/
	//PA08 SDA PA09 SCL
	PORT->Group[0].PINCFG[8].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[8].bit.PMUXE = PORT_PMUX_PMUXE_C;
	PORT->Group[0].PINCFG[9].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[8].bit.PMUXO = PORT_PMUX_PMUXO_C;
	
	/*** UART ***/
	/*** BLE ***/
	//PA12 PA13 SERCOM2
	PORT->Group[0].PINCFG[12].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[6].bit.PMUXE = PORT_PMUX_PMUXE_C;
	PORT->Group[0].PINCFG[13].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[6].bit.PMUXO = PORT_PMUX_PMUXO_C;
	
	/*** GPS ***/
	//PA14 PA15 SERCOM4
	PORT->Group[0].PINCFG[14].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[7].bit.PMUXE = PORT_PMUX_PMUXE_D;
	PORT->Group[0].PINCFG[15].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[7].bit.PMUXO = PORT_PMUX_PMUXO_D;
		
}

void INIT_interrupt() {
	__disable_irq();

	NVIC_EnableIRQ(EIC_IRQn);

	GCLK->CLKCTRL.bit.ID = EIC;
	EIC->CTRL.bit.ENABLE = 1;
	EIC->INTENSET.reg |= EIC_INTENSET_EXTINT10 | EIC_INTENSET_EXTINT11 | EIC_INTENSET_EXTINT1 | EIC_INTENSET_EXTINT2 | EIC_INTENSET_EXTINT3;
	EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE1_RISE | EIC_CONFIG_SENSE2_RISE | EIC_CONFIG_SENSE3_RISE;
	EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE2_RISE | EIC_CONFIG_SENSE3_RISE;

	__enable_irq();
}

void INIT_pwm() {
	/* see help in
	https://community.atmel.com/forum/sam-d21-problem-buffered-pwm-tcc?skey=sam%20d21%20CC[0]
	*/
	/***** Moteur 1 - Gauche *****/ 
	GCLK->CLKCTRL.bit.ID = TCC1; // TCC1/WO[1]
	while(GCLK->STATUS.bit.SYNCBUSY);
	//TCC1->WAVE.bit.DIR = 1; // Single Slope
	TCC1->WAVE.bit.WAVEGEN = 2;
	TCC1->WAVE.bit.POL1 = 1; // Clear on match
	TCC1->PER.reg = 100;
	TCC1->CC[1].reg = 0;
	PM->APBCMASK.bit.TCC1_ = 1;
	TCC1->CTRLA.bit.ENABLE = 1;
	
	/***** Moteur 2 - Droit *****/
	GCLK->CLKCTRL.bit.ID = TCC2; // TCC2/WO[0]
	while(GCLK->STATUS.bit.SYNCBUSY);
	//TCC2->WAVE.bit.DIR = 1; // Single Slope
	TCC2->WAVE.bit.WAVEGEN = 2;
	TCC2->WAVE.bit.POL0 = 1; // Clear on match
	TCC2->PER.reg = 100;
	TCC2->CC[0].reg = 0;
	PM->APBCMASK.bit.TCC2_ = 1;
	TCC2->CTRLA.bit.ENABLE = 1;
}

void INIT_uart() {
	/* see help in
	https://www.avrfreaks.net/forum/no-asf-issue-samd21-lin-slave-implementation-uart-using-breaksync-detection
	*/
	PM->APBCMASK.bit.SERCOM2_= 1;
	GCLK->CLKCTRL.bit.ID = SERCOM2;
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	SERCOM2->USART.CTRLA.bit.RXPO = 1;
	SERCOM2->USART.CTRLA.bit.TXPO = 0;
	SERCOM2->USART.CTRLA.bit.MODE = 1;
	SERCOM2->USART.CTRLB.bit.RXEN = 1;
	SERCOM2->USART.CTRLB.bit.TXEN = 1;
	SERCOM2->USART.BAUD.FRAC.FP   = (BAUD_PRESCALE % 8);
    SERCOM2->USART.BAUD.FRAC.BAUD = (BAUD_PRESCALE / 8);
	SERCOM2->USART.INTENSET.bit.RXS = 1;
	NVIC_EnableIRQ(SERCOM2_IRQn);
	SERCOM2->USART.CTRLA.bit.ENABLE = 1;
	while(SERCOM2->USART.SYNCBUSY.bit.ENABLE);
	
	PM->APBCMASK.bit.SERCOM4_ = 1;
	GCLK->CLKCTRL.bit.ID = SERCOM4;
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	SERCOM4->USART.CTRLA.bit.RXPO = 3;
	SERCOM4->USART.CTRLA.bit.TXPO = 1;
	SERCOM4->USART.CTRLA.bit.MODE = 1;
	SERCOM4->USART.CTRLB.bit.RXEN = 1;
	SERCOM4->USART.CTRLB.bit.TXEN = 1;
	SERCOM4->USART.BAUD.FRAC.FP   = (BAUD_PRESCALE % 8);
    SERCOM4->USART.BAUD.FRAC.BAUD = (BAUD_PRESCALE / 8);
	SERCOM4->USART.INTENSET.bit.RXS = 1;
	NVIC_EnableIRQ(SERCOM4_IRQn);
	SERCOM4->USART.CTRLA.bit.ENABLE = 1;
	while(SERCOM4->USART.SYNCBUSY.bit.ENABLE);
}

void INIT_i2c() {
	/* see help in 
	https://www.avrfreaks.net/sites/default/files/forum_attachments/Atmel-42631-SAM-D21-SERCOM-I2C-Configura.pdf
	*/
	PM->APBCMASK.bit.SERCOM0_ = 1;
	GCLK->CLKCTRL.bit.ID = SERCOM0;
	while(GCLK->STATUS.bit.SYNCBUSY);

	SERCOM0->I2C.CTRLA.bit.SDAHOLD = 2;
	SERCOM0->I2C.CTRLA.bit.MODE = 5;
	SERCOM0->I2C.CTRLB.bit.SMEN = 1;
	SERCOM0->I2C.CTRLA.bit.ENABLE = 1;
	while(SERCOM0->I2C.SYNCBUSY.bit.ENABLE);
	SERCOM0->STATUS.bit.BUSSTATE = 1;
	while(SERCOM0->I2C.SYNCBUSY.bit.SYSOP);
}