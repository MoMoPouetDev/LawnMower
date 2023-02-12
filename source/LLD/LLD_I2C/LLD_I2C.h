/**
 * @file LLD_I2C.h
 * @author ACR
 * @brief Header file for I2C peripheral
 * @details
**/

#ifndef LLD_I2C_H_
#define LLD_I2C_H_

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

typedef enum
{
	LLD_I2C_I2C1,
	LLD_I2C_I2C2,
	LLD_I2C_I2C3,
	LLD_I2C_I2C4,
	LLD_I2C_NB
}lldI2c_t;

typedef enum
{
	LLD_I2C_STATUS_ERROR,
	LLD_I2C_STATUS_SUCCESS,
}lldI2cStatus_t;

/* Forward declaration of the transfer descriptor and handle typedefs. */
typedef struct _lld_i2c_master_transfer lld_i2c_master_transfer_t;
typedef struct _lld_i2c_master_handle lld_i2c_master_handle_t;

/*!
 * @brief Master completion callback function pointer type.
 *
 * This callback is used only for the non-blocking master transfer API. Specify the callback you wish to use
 * in the call to LPI2C_MasterTransferCreateHandle().
 *
 * @param base The LPI2C peripheral base address.
 * @param completionStatus Either LLD_STATUS_Success or an error code describing how the transfer completed.
 * @param userData Arbitrary pointer-sized value passed from the application.
 */
typedef void (*lld_i2c_master_transfer_callback_t)(lldI2c_t e_I2c);

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/

void LLD_I2C_Init(lldI2c_t e_I2c, uint32_t u32_Frequency, lld_i2c_master_transfer_callback_t pf_Callback);
lldI2cStatus_t LLD_I2C_Read(lldI2c_t e_I2c, uint8_t u8_SlaveAddress, uint8_t u8_DataAddress,
					uint8_t u8_DataAddressSize, uint8_t* pu8_MasterRxBuff, uint8_t u8_RxBuffSize);
lldI2cStatus_t LLD_I2C_Write(lldI2c_t e_I2c, uint8_t u8_SlaveAddress, uint8_t u8_DataAddress,
					uint8_t u8_DataAddressSize, uint8_t* pu8_MasterTxBuff, uint8_t u8_TxBuffSize);

#endif /* LLD_I2C_H_ */
