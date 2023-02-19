/*
 * HAL_I2C.c
 *
 *  Created on: 12 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_I2C.h"
#include "LLD_I2C.h"


/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define I2C_BAUDRATE   400000U
#define COMPASS_ADDR (0x1E<<1)
#define ACCELEROMETRE_ADDR (0x53<<1)

#define ADDR_DATA_COMPASS_X_LSB 0x04
#define ADDR_DATA_COMPASS_X_MSB 0x03
#define ADDR_DATA_COMPASS_Y_LSB 0x08
#define ADDR_DATA_COMPASS_Y_MSB 0x07
#define ADDR_DATA_COMPASS_Z_LSB 0x06
#define ADDR_DATA_COMPASS_Z_MSB 0x05

#define ADDR_DATA_ACCELEROMETER_X_LSB 0x32
#define ADDR_DATA_ACCELEROMETER_X_MSB 0x33
#define ADDR_DATA_ACCELEROMETER_Y_LSB 0x34
#define ADDR_DATA_ACCELEROMETER_Y_MSB 0x35
#define ADDR_DATA_ACCELEROMETER_Z_LSB 0x36
#define ADDR_DATA_ACCELEROMETER_Z_MSB 0x37

static volatile uint8_t gu8_FlagRx;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
/**
* @brief		I2C callback
* @param		e_I2c : I2C number
* @return		void
* @details
**/
void HAL_I2C_Callback(lldI2c_t e_I2c)
{
	if(e_I2c == LLD_I2C_I2C4)
	{
		gu8_FlagRx = 1;
	}

}

void HAL_I2C_Init()
{
	gu8_FlagRx = 0;

	LLD_I2C_Init(LLD_I2C_I2C4, I2C_BAUDRATE, HAL_I2C_Callback);
}

void HAL_I2C_CompassInit()
{
	uint8_t tu8_txBufferInit1[1] = {0x70};
	uint8_t tu8_txBufferInit2[1] = {0x00};
	LLD_I2C_Write(LLD_I2C_I2C4, COMPASS_ADDR, 0x00, 1, tu8_txBufferInit1, 1);
	while (!gu8_FlagRx);
	gu8_FlagRx = 0;
	
	LLD_I2C_Write(LLD_I2C_I2C4, COMPASS_ADDR, 0xA0, 1, tu8_txBufferInit2, 1);
	while (!gu8_FlagRx);
	gu8_FlagRx = 0;
}

void HAL_I2C_AccelInit()
{
	uint8_t tu8_txBufferInit1[1] = {0x08};
	LLD_I2C_Write(LLD_I2C_I2C4, ACCELEROMETRE_ADDR, 0x2D, 1, tu8_txBufferInit1, 1);
	while (!gu8_FlagRx);
	gu8_FlagRx = 0;
}

void HAL_I2C_Write()
{

}

uint8_t HAL_I2C_Read(uint8_t* pu8_RxBuff, uint8_t* pu8_Size)
{
	static uint8_t u8_FlagReadSend = 0;
	static uint8_t tu8_RxBuff[5] = {0};
	uint8_t u8_ReturnValue;
	uint8_t u8_Size = 4;
	uint8_t i;

	*pu8_Size = u8_Size;

	if (!u8_FlagReadSend)
	{
		//LLD_I2C_Read(LLD_I2C_I2C1, TOUCH_PANEL_ADDR, TOUCH_X_POSITION_MSB, u8_Size, tu8_RxBuff, u8_Size);
		u8_FlagReadSend = 1;
		u8_ReturnValue = 0;
	}
	else if (gu8_FlagRx)
	{
		for(i = 0; i < u8_Size; i++)
		{
			pu8_RxBuff[i] = tu8_RxBuff[i];
		}
		gu8_FlagRx = 0;
		u8_FlagReadSend = 0;
		u8_ReturnValue = 1;
	}
	else
	{
		u8_ReturnValue = 0;
	}

	return u8_ReturnValue;
}
