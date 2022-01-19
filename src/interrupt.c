//
//  Interrupt.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 29/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/interrupt.h>
#include "sam.h"

#include "constant.h"
#include "uart.h"
//#include "status.h"
//#include "mower.h"

/*
ISR(USART_RX_vect)
{
    STATUS_receivedStatus();
}

ISR(WDT_vect) {
	_uFlagWatchdog = 1;
    WDTCSR |= (1<<WDIE);
}
*/

void EIC_Handler(){
    //BP Stop
    if(EIC->INTFLAG.bit.EXTINT10) {
        _uBpStop = 1;
        if ((_eEtatRain == ON) && (isDocking()))
	{
            _eEtatRain = OFF;
        }
        
        REG_EIC_INTFLAG = EIC_INTFLAG_EXTINT10; //clear flag
    }

    //BP Start
    if(EIC->INTFLAG.bit.EXTINT11) {
        if(!isDocking()) {
		_uBpStop = 0;	
		_uBpStart ^= (1<<1);
	}
	else {
		_uBpForceStart = 1;
        }

        REG_EIC_INTFLAG = EIC_INTFLAG_EXTINT11; //clear flag
    }

    //BP Left
    if(EIC->INTFLAG.bit.EXTINT1) {
        MOWER_bumperDetect(FL);

        REG_EIC_INTFLAG = EIC_INTFLAG_EXTINT1; //clear flag
    }

    //BP Center
    if(EIC->INTFLAG.bit.EXTINT2) {
        MOWER_bumperDetect(FC);

        REG_EIC_INTFLAG = EIC_INTFLAG_EXTINT2; //clear flag
    }

    //BP Right
    if(EIC->INTFLAG.bit.EXTINT3) {
        MOWER_bumperDetect(FR);

        REG_EIC_INTFLAG = EIC_INTFLAG_EXTINT3; //clear flag
    }
}
