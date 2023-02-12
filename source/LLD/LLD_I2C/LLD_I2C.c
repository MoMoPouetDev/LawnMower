/**
 * @file LLD_I2C.c
 * @author ACR
 * @brief Specific I2C driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_I2C_USE_LOCALS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "MIMXRT1061.h"
#include "MIMXRT1061_features.h"
#include "fsl_clock.h"
#include "LLD_I2C.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES LPI2C ...                                                  */
/*--------------------------------------------------------------------------*/

/*! @brief LPI2C status return codes. */
enum
{
    kStatus_LPI2C_Busy = 900, /*!< The master is already performing a transfer. */
    kStatus_LPI2C_Idle = 901, /*!< The slave driver is idle. */
    kStatus_LPI2C_Nak  = 902, /*!< The slave device sent a NAK in response to a byte. */
    kStatus_LPI2C_FifoError       = 903, /*!< FIFO under run or overrun. */
    kStatus_LPI2C_BitError        = 904, /*!< Transferred bit was not seen on the bus. */
    kStatus_LPI2C_ArbitrationLost = 905, /*!< Arbitration lost error. */
    kStatus_LPI2C_PinLowTimeout   = 906, /*!< SCL or SDA were held low longer than the timeout. */
    kStatus_LPI2C_NoTransferInProgress = 907, /*!< Attempt to abort a transfer when one is not in progress. */
    kStatus_LPI2C_DmaRequestFail = 908, /*!< DMA request failed. */
    kStatus_LPI2C_Timeout        = 909, /*!< Timeout polling status flags. */
};

/*!
 * @brief LPI2C master peripheral flags.
 *
 * The following status register flags can be cleared:
 * - #kLPI2C_MasterEndOfPacketFlag
 * - #kLPI2C_MasterStopDetectFlag
 * - #kLPI2C_MasterNackDetectFlag
 * - #kLPI2C_MasterArbitrationLostFlag
 * - #kLPI2C_MasterFifoErrFlag
 * - #kLPI2C_MasterPinLowTimeoutFlag
 * - #kLPI2C_MasterDataMatchFlag
 *
 * All flags except #kLPI2C_MasterBusyFlag and #kLPI2C_MasterBusBusyFlag can be enabled as
 * interrupts.
 *
 * @note These enums are meant to be OR'd together to form a bit mask.
 */
enum _lld_i2c_master_flags
{
    kLPI2C_MasterTxReadyFlag         = LPI2C_MSR_TDF_MASK,  /*!< Transmit data flag */
    kLPI2C_MasterRxReadyFlag         = LPI2C_MSR_RDF_MASK,  /*!< Receive data flag */
    kLPI2C_MasterEndOfPacketFlag     = LPI2C_MSR_EPF_MASK,  /*!< End Packet flag */
    kLPI2C_MasterStopDetectFlag      = LPI2C_MSR_SDF_MASK,  /*!< Stop detect flag */
    kLPI2C_MasterNackDetectFlag      = LPI2C_MSR_NDF_MASK,  /*!< NACK detect flag */
    kLPI2C_MasterArbitrationLostFlag = LPI2C_MSR_ALF_MASK,  /*!< Arbitration lost flag */
    kLPI2C_MasterFifoErrFlag         = LPI2C_MSR_FEF_MASK,  /*!< FIFO error flag */
    kLPI2C_MasterPinLowTimeoutFlag   = LPI2C_MSR_PLTF_MASK, /*!< Pin low timeout flag */
    kLPI2C_MasterDataMatchFlag       = LPI2C_MSR_DMF_MASK,  /*!< Data match flag */
    kLPI2C_MasterBusyFlag            = LPI2C_MSR_MBF_MASK,  /*!< Master busy flag */
    kLPI2C_MasterBusBusyFlag         = LPI2C_MSR_BBF_MASK,  /*!< Bus busy flag */

    /*! All flags which are cleared by the driver upon starting a transfer. */
    kLPI2C_MasterClearFlags = kLPI2C_MasterEndOfPacketFlag | kLPI2C_MasterStopDetectFlag | kLPI2C_MasterNackDetectFlag |
                              kLPI2C_MasterArbitrationLostFlag | kLPI2C_MasterFifoErrFlag |
                              kLPI2C_MasterPinLowTimeoutFlag | kLPI2C_MasterDataMatchFlag,
    /*! IRQ sources enabled by the non-blocking transactional API. */
    kLPI2C_MasterIrqFlags = kLPI2C_MasterArbitrationLostFlag | kLPI2C_MasterTxReadyFlag | kLPI2C_MasterRxReadyFlag |
                            kLPI2C_MasterStopDetectFlag | kLPI2C_MasterNackDetectFlag | kLPI2C_MasterPinLowTimeoutFlag |
                            kLPI2C_MasterFifoErrFlag,
    /*! Errors to check for. */
    kLPI2C_MasterErrorFlags = kLPI2C_MasterNackDetectFlag | kLPI2C_MasterArbitrationLostFlag |
                              kLPI2C_MasterFifoErrFlag | kLPI2C_MasterPinLowTimeoutFlag
};

/*! @brief Direction of master and slave transfers. */
typedef enum _lld_i2c_direction
{
    kLPI2C_Write = 0U, /*!< Master transmit. */
    kLPI2C_Read  = 1U  /*!< Master receive. */
} lld_i2c_direction_t;

/*! @brief LPI2C pin configuration. */
typedef enum _lld_i2c_master_pin_config
{
    kLPI2C_2PinOpenDrain  = 0x0U, /*!< LPI2C Configured for 2-pin open drain mode */
    kLPI2C_2PinOutputOnly = 0x1U, /*!< LPI2C Configured for 2-pin output only mode (ultra-fast mode) */
    kLPI2C_2PinPushPull   = 0x2U, /*!< LPI2C Configured for 2-pin push-pull mode */
    kLPI2C_4PinPushPull   = 0x3U, /*!< LPI2C Configured for 4-pin push-pull mode */
    kLPI2C_2PinOpenDrainWithSeparateSlave =
        0x4U, /*!< LPI2C Configured for 2-pin open drain mode with separate LPI2C slave */
    kLPI2C_2PinOutputOnlyWithSeparateSlave =
        0x5U, /*!< LPI2C Configured for 2-pin output only mode(ultra-fast mode) with separate LPI2C slave */
    kLPI2C_2PinPushPullWithSeparateSlave =
        0x6U, /*!< LPI2C Configured for 2-pin push-pull mode with separate LPI2C slave */
    kLPI2C_4PinPushPullWithInvertedOutput = 0x7U /*!< LPI2C Configured for 4-pin push-pull mode(inverted outputs) */
} lld_i2c_master_pin_config_t;

/*! @brief LPI2C master host request pin polarity configuration. */
typedef enum _lld_i2c_host_request_polarity
{
    kLPI2C_HostRequestPinActiveLow  = 0x0U, /*!< Configure the LPI2C_HREQ pin active low */
    kLPI2C_HostRequestPinActiveHigh = 0x1U  /*!< Configure the LPI2C_HREQ pin active high */
} lld_i2c_host_request_polarity_t;

/*!
 * @brief Structure with settings to initialize the LPI2C master module.
 *
 * This structure holds configuration settings for the LPI2C peripheral. To initialize this
 * structure to reasonable defaults, call the LPI2C_MasterGetDefaultConfig() function and
 * pass a pointer to your configuration structure instance.
 *
 * The configuration structure can be made constant so it resides in flash.
 */
typedef struct _lld_i2c_master_config
{
    uint32_t u32_BaudRate_Hz;                /*!< Desired baud rate in Hertz. */
} lld_i2c_master_config_t;

/*!
 * @brief Transfer option flags.
 *
 * @note These enumerations are intended to be OR'd together to form a bit mask of options for
 * the #_lld_i2c_master_transfer::flags field.
 */
enum _lld_i2c_master_transfer_flags
{
    kLPI2C_TransferDefaultFlag       = 0x00U, /*!< Transfer starts with a start signal, stops with a stop signal. */
    kLPI2C_TransferNoStartFlag       = 0x01U, /*!< Don't send a start condition, address, and sub address */
    kLPI2C_TransferRepeatedStartFlag = 0x02U, /*!< Send a repeated start condition */
    kLPI2C_TransferNoStopFlag        = 0x04U, /*!< Don't send a stop condition. */
};

/*!
 * @brief Non-blocking transfer descriptor structure.
 *
 * This structure is used to pass transaction parameters to the LLD_I2C_MasterTransferNonBlocking() API.
 */
struct _lld_i2c_master_transfer
{
    uint32_t u32_Flags;        		/*!< Bit mask of options for the transfer. See enumeration #_lpi2c_master_transfer_flags for
                              	  	  available options. Set to 0 or #kLPI2C_TransferDefaultFlag for normal transfers. */
    uint16_t u16_SlaveAddress; 		/*!< The 7-bit slave address. */
    lld_i2c_direction_t e_Direction; 	/*!< Either #kLPI2C_Read or #kLPI2C_Write. */
    uint32_t u32_Subaddress;       	/*!< Sub address. Transferred MSB first. */
    size_t u32_SubaddressSize;		/*!< Length of sub address to send in bytes. Maximum size is 4 bytes. */
    void *p_Data;                  	/*!< Pointer to data to transfer. */
    size_t u32_DataSize;          	/*!< Number of bytes to transfer. */
};

/*!
 * @brief Driver handle for master non-blocking APIs.
 * @note The contents of this structure are private and subject to change.
 */
struct _lld_i2c_master_handle
{
    uint8_t u8_State;                                       /*!< Transfer state machine current state. */
    uint16_t u16_RemainingBytes;                             /*!< Remaining byte count in current state. */
    uint8_t *pu8_buf;                                        /*!< Buffer pointer for current state. */
    uint16_t tu16_CommandBuffer[6];                           /*!< LPI2C command sequence. When all 6 command words are used:
         Start&addr&write[1 word] + subaddr[4 words] + restart&addr&read[1 word] */
    lld_i2c_master_transfer_t s_Transfer;                    /*!< Copy of the current transfer info. */
    lld_i2c_master_transfer_callback_t pf_CompletionCallback; /*!< Callback function pointer. */
    void *p_UserData;                                      /*!< Application data passed to callback. */
};

/*! @brief Typedef for master interrupt handler, used internally for LPI2C master interrupt and EDMA transactional APIs.
 */
typedef void (*lld_i2c_master_isr_t)(LPI2C_Type *ps_Base, void *p_Handle);

/* ! @brief LPI2C master fifo commands. */
enum
{
    kTxDataCmd = LPI2C_MTDR_CMD(0x0U), /*!< Transmit DATA[7:0] */
    kRxDataCmd = LPI2C_MTDR_CMD(0X1U), /*!< Receive (DATA[7:0] + 1) bytes */
    kStopCmd   = LPI2C_MTDR_CMD(0x2U), /*!< Generate STOP condition */
    kStartCmd  = LPI2C_MTDR_CMD(0x4U), /*!< Generate(repeated) START and transmit address in DATA[[7:0] */
};

/*!
 * @brief Default watermark values.
 *
 * The default watermarks are set to zero.
 */
enum
{
    kDefaultTxWatermark = 0,
    kDefaultRxWatermark = 0,
};

/*! @brief States for the state machine used by transactional APIs. */
enum
{
    kIdleState = 0,
    kSendCommandState,
    kIssueReadCommandState,
    kTransferDataState,
    kStopState,
    kWaitForCompletionState,
};

/*! @brief Pointers to master handles for each instance, used internally for LPI2C master interrupt and EDMA
transactional APIs. */
void *tp_I2cMasterHandle[5];


/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD I2C ...                                                */
/*--------------------------------------------------------------------------*/

#define LPI2C_CLOCK_FREQUENCY	60000000UL

lld_i2c_master_handle_t ts_MasterHandle[LLD_I2C_NB];

typedef struct
{
	LPI2C_Type * ps_Base;
	uint32_t u32_frequency;
}lld_i2c_manager_t;

lld_i2c_manager_t ts_lld_i2c_manager[LLD_I2C_NB];

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

status_t LLD_I2C_MasterCheckAndClearError(LPI2C_Type *ps_Base, uint32_t u32_Status);
status_t LLD_I2C_CheckForBusyBus(LPI2C_Type *ps_Base);
void LLD_I2C_MasterInit(LPI2C_Type *ps_Base, const lld_i2c_master_config_t *pcs_MasterConfig, uint32_t u32_sourceClock_Hz);
void LLD_I2C_MasterSetBaudRate(LPI2C_Type *ps_Base, uint32_t u32_sourceClock_Hz, uint32_t u32_BaudRate_Hz);
void LLD_I2C_MasterTransferCreateHandle(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle,
                                      lld_i2c_master_transfer_callback_t pf_Callback, void *p_UserData);
static status_t LLD_I2C_RunTransferStateMachine(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle, bool *pb_IsDone);
static void LLD_I2C_InitTransferStateMachine(lld_i2c_master_handle_t *ps_handle);
status_t LLD_I2C_MasterTransferNonBlocking(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle, lld_i2c_master_transfer_t *ps_Transfer);
void LLD_I2C_MasterTransferAbort(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle);
void LLD_I2C_MasterTransferHandleIRQ(LPI2C_Type *ps_Base, void *p_I2cMasterHandle);
static inline void LLD_I2C_MasterEnable(LPI2C_Type *ps_Base, bool b_Enable);
static inline uint32_t LLD_I2C_MasterGetStatusFlags(LPI2C_Type *ps_Base);
static inline void LLD_I2C_MasterClearStatusFlags(LPI2C_Type *ps_Base, uint32_t u32_StatusMask);
static inline void LLD_I2C_MasterEnableInterrupts(LPI2C_Type *ps_Base, uint32_t u32_InterruptMask);
static inline void LLD_I2C_MasterDisableInterrupts(LPI2C_Type *ps_Base, uint32_t u32_InterruptMask);
static inline void LLD_I2C_MasterSetWatermarks(LPI2C_Type *ps_Base, size_t u32_TxWords, size_t u32_RxWords);
static inline void LLD_I2C_MasterGetFifoCounts(LPI2C_Type *ps_Base, size_t *pu32_RxCount, size_t *pu32_TxCount);

static void LLD_I2C_IRQHandler(LPI2C_Type *ps_Base, uint32_t u32_Instance);

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*!
 * @brief Convert provided flags to status code, and clear any errors if present.
 * @param ps_Base The LPI2C peripheral base address.
 * @param u32_Status Current status flags value that will be checked.
 * @retval #LLD_STATUS_Success
 * @retval #kStatus_LPI2C_PinLowTimeout
 * @retval #kStatus_LPI2C_ArbitrationLost
 * @retval #kStatus_LPI2C_Nak
 * @retval #kStatus_LPI2C_FifoError
 */
/* Not static so it can be used from fsl_lpi2c_edma.c. */
status_t LLD_I2C_MasterCheckAndClearError(LPI2C_Type *ps_Base, uint32_t u32_Status)
{
    status_t s32_Result = kStatus_Success;

    /* Check for error. These errors cause a stop to automatically be sent. We must */
    /* clear the errors before a new transfer can start. */
    u32_Status &= (uint32_t)kLPI2C_MasterErrorFlags;
    if (0U != u32_Status)
    {
        /* Select the correct error code. Ordered by severity, with bus issues first. */
        if (0U != (u32_Status & (uint32_t)kLPI2C_MasterPinLowTimeoutFlag))
        {
        	s32_Result = kStatus_LPI2C_PinLowTimeout;
        }
        else if (0U != (u32_Status & (uint32_t)kLPI2C_MasterArbitrationLostFlag))
        {
        	s32_Result = kStatus_LPI2C_ArbitrationLost;
        }
        else if (0U != (u32_Status & (uint32_t)kLPI2C_MasterNackDetectFlag))
        {
        	s32_Result = kStatus_LPI2C_Nak;
        }
        else if (0U != (u32_Status & (uint32_t)kLPI2C_MasterFifoErrFlag))
        {
        	s32_Result = kStatus_LPI2C_FifoError;
        }
        else
        {
            ; /* Intentional empty */
        }

        /* Clear the flags. */
        LLD_I2C_MasterClearStatusFlags(ps_Base, u32_Status);

        /* Reset fifos. These flags clear automatically. */
        ps_Base->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;
    }
    else
    {
        ; /* Intentional empty */
    }

    return s32_Result;
}

/*!
 * @brief Make sure the bus isn't already busy.
 *
 * A busy bus is allowed if we are the one driving it.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @retval #LLD_STATUS_Success
 * @retval #kStatus_LPI2C_Busy
 */
/* Not static so it can be used from fsl_lpi2c_edma.c. */
status_t LLD_I2C_CheckForBusyBus(LPI2C_Type *ps_Base)
{
    status_t s32_Ret = kStatus_Success;

    uint32_t status = LLD_I2C_MasterGetStatusFlags(ps_Base);
    if ((0U != (status & (uint32_t)kLPI2C_MasterBusBusyFlag)) && (0U == (status & (uint32_t)kLPI2C_MasterBusyFlag)))
    {
    	s32_Ret = kStatus_LPI2C_Busy;
    }

    return s32_Ret;
}

/*!
 * brief Initializes the LPI2C master peripheral.
 *
 * This function enables the peripheral clock and initializes the LPI2C master peripheral as described by the user
 * provided configuration. A software reset is performed prior to configuration.
 *
 * param ps_Base The LPI2C peripheral base address.
 * param pcs_MasterConfig User provided peripheral configuration. Use LPI2C_MasterGetDefaultConfig() to get a set of
 * defaults
 *      that you can override.
 * param u32_sourceClock_Hz Frequency in Hertz of the LPI2C functional clock. Used to calculate the baud rate divisors,
 *      filter widths, and timeout periods.
 */
void LLD_I2C_MasterInit(LPI2C_Type *ps_Base, const lld_i2c_master_config_t *pcs_MasterConfig, uint32_t u32_sourceClock_Hz)
{
    uint32_t u32_Value;

    /* Ungate the clock. */
    if(ps_Base == LPI2C1)
    {
    	CLOCK_EnableClock(kCLOCK_Lpi2c1);
    }
    else if(ps_Base == LPI2C2)
    {
    	CLOCK_EnableClock(kCLOCK_Lpi2c2);
    }
    else if(ps_Base == LPI2C3)
    {
    	CLOCK_EnableClock(kCLOCK_Lpi2c3);
    }
    else /* if(ps_Base == LPI2C4) */
    {
    	CLOCK_EnableClock(kCLOCK_Lpi2c4);
    }

    /* Reset peripheral before configuring it. */
    ps_Base->MCR = 0;

    /* host request pin polarity configuration : active high */
    u32_Value = ps_Base->MCFGR0;
    u32_Value &= (~(LPI2C_MCFGR0_HREN_MASK | LPI2C_MCFGR0_HRPOL_MASK | LPI2C_MCFGR0_HRSEL_MASK));
    u32_Value |= LPI2C_MCFGR0_HRPOL(kLPI2C_HostRequestPinActiveHigh);
    ps_Base->MCFGR0 = u32_Value;

    /* pin config and ignore ack */
    u32_Value = ps_Base->MCFGR1;
    u32_Value &= ~(LPI2C_MCFGR1_PINCFG_MASK | LPI2C_MCFGR1_IGNACK_MASK);
    u32_Value |= LPI2C_MCFGR1_PINCFG(kLPI2C_2PinOpenDrain);
    ps_Base->MCFGR1 = u32_Value;

    LLD_I2C_MasterSetWatermarks(ps_Base, (size_t)kDefaultTxWatermark, (size_t)kDefaultRxWatermark);

    /* Configure baudrate after the SDA/SCL glitch filter setting,
       since the baudrate calculation needs them as parameter. */
    LLD_I2C_MasterSetBaudRate(ps_Base, u32_sourceClock_Hz, pcs_MasterConfig->u32_BaudRate_Hz);

    LLD_I2C_MasterEnable(ps_Base, true);
}

/*!
 * brief Sets the I2C bus frequency for master transactions.
 *
 * The LPI2C master is automatically disabled and re-enabled as necessary to configure the baud
 * rate. Do not call this function during a transfer, or the transfer is aborted.
 *
 * note Please note that the second parameter is the clock frequency of LPI2C module, the third
 * parameter means user configured bus baudrate, this implementation is different from other I2C drivers
 * which use baudrate configuration as second parameter and source clock frequency as third parameter.
 *
 * param ps_Base The LPI2C peripheral base address.
 * param u32_sourceClock_Hz LPI2C functional clock frequency in Hertz.
 * param u32_BaudRate_Hz Requested bus frequency in Hertz.
 */
void LLD_I2C_MasterSetBaudRate(LPI2C_Type *ps_Base, uint32_t u32_sourceClock_Hz, uint32_t u32_BaudRate_Hz)
{
    bool b_WasEnabled;
    uint8_t u8_FiltScl = (uint8_t)((ps_Base->MCFGR2 & LPI2C_MCFGR2_FILTSCL_MASK) >> LPI2C_MCFGR2_FILTSCL_SHIFT);

    uint8_t u8_Divider     = 1U;
    uint8_t u8_BestDivider = 1U;
    uint8_t u8_Prescale    = 0U;
    uint8_t u8_BestPre     = 0U;

    uint8_t u8_ClkCycle;
    uint8_t u8_BestclkCycle = 0U;

    uint32_t u32_AbsError  = 0U;
    uint32_t u32_BestError = 0xffffffffu;
    uint32_t u32_ComputedRate;

    uint32_t u32_TmpReg = 0U;

    /* Disable master mode. */
    b_WasEnabled = (0U != ((ps_Base->MCR & LPI2C_MCR_MEN_MASK) >> LPI2C_MCR_MEN_SHIFT));
    LLD_I2C_MasterEnable(ps_Base, false);

    /* Baud rate = (u32_sourceClock_Hz / 2 ^ prescale) / (CLKLO + 1 + CLKHI + 1 + SCL_LATENCY)
     * SCL_LATENCY = ROUNDDOWN((2 + FILTSCL) / (2 ^ prescale))
     */
    for (u8_Prescale = 0U; u8_Prescale <= 7U; u8_Prescale++)
    {
        /* Calculate the clkCycle, clkCycle = CLKLO + CLKHI, divider = 2 ^ prescale */
    	u8_ClkCycle = (uint8_t)((10U * u32_sourceClock_Hz / u8_Divider / u32_BaudRate_Hz + 5U) / 10U - (2U + u8_FiltScl) / u8_Divider - 2U);
        /* According to register description, The max value for CLKLO and CLKHI is 63.
           however to meet the I2C specification of tBUF, CLKHI should be less than
           clkCycle - 0.52 x u32_sourceClock_Hz / u32_BaudRate_Hz / divider + 1U. Refer to the comment of the tmpHigh's
           calculation for details. So we have:
           CLKHI < clkCycle - 0.52 x u32_sourceClock_Hz / u32_BaudRate_Hz / divider + 1U,
           clkCycle = CLKHI + CLKLO and
           u32_sourceClock_Hz / u32_BaudRate_Hz / divider = clkCycle + 2 + ROUNDDOWN((2 + FILTSCL) / divider),
           we can come up with: CLKHI < 0.92 x CLKLO - ROUNDDOWN(2 + FILTSCL) / divider
           so the max boundary of CLKHI should be 0.92 x 63 - ROUNDDOWN(2 + FILTSCL) / divider,
           and the max boundary of clkCycle is 1.92 x 63 - ROUNDDOWN(2 + FILTSCL) / divider. */
        if (u8_ClkCycle > (120U - (2U + u8_FiltScl) / u8_Divider))
        {
        	u8_Divider *= 2U;
            continue;
        }
        /* Calculate the computed baudrate and compare it with the desired baudrate */
        u32_ComputedRate = (u32_sourceClock_Hz / (uint32_t)u8_Divider) /
                       ((uint32_t)u8_ClkCycle + 2U + (2U + (uint32_t)u8_FiltScl) / (uint32_t)u8_Divider);
        u32_AbsError = u32_BaudRate_Hz > u32_ComputedRate ? u32_BaudRate_Hz - u32_ComputedRate : u32_ComputedRate - u32_BaudRate_Hz;
        if (u32_AbsError < u32_BestError)
        {
        	u8_BestPre      = u8_Prescale;
            u8_BestDivider  = u8_Divider;
            u8_BestclkCycle = u8_ClkCycle;
            u32_BestError    = u32_AbsError;

            /* If the error is 0, then we can stop searching because we won't find a better match. */
            if (u32_AbsError == 0U)
            {
                break;
            }
        }
        u8_Divider *= 2U;
    }

    /* SCL low time tLO should be larger than or equal to SCL high time tHI:
       tLO = ((CLKLO + 1) x (2 ^ PRESCALE)) >= tHI = ((CLKHI + 1 + SCL_LATENCY) x (2 ^ PRESCALE)),
       which is CLKLO >= CLKHI + (2U + filtScl) / bestDivider.
       Also since bestclkCycle = CLKLO + CLKHI, bestDivider = 2 ^ PRESCALE
       which makes CLKHI <= (bestclkCycle - (2U + filtScl) / bestDivider) / 2U.

       The max tBUF should be at least 0.52 times of the SCL clock cycle:
       tBUF = ((CLKLO + 1) x (2 ^ PRESCALE) / u32_sourceClock_Hz) > (0.52 / u32_BaudRate_Hz),
       plus bestDivider = 2 ^ PRESCALE, bestclkCycle = CLKLO + CLKHI we can come up with
       CLKHI <= (bestclkCycle - 0.52 x u32_sourceClock_Hz / u32_BaudRate_Hz / bestDivider + 1U).
       In this case to get a safe CLKHI calculation, we can assume:
    */
    uint8_t u8_TmpHigh = (u8_BestclkCycle - (2U + u8_FiltScl) / u8_BestDivider) / 2U;
    while (u8_TmpHigh > (u8_BestclkCycle - 52U * u32_sourceClock_Hz / u32_BaudRate_Hz / u8_BestDivider / 100U + 1U))
    {
    	u8_TmpHigh = u8_TmpHigh - 1U;
    }

    /* Calculate DATAVD and SETHOLD.
       To meet the timing requirement of I2C spec for standard mode, fast mode and fast mode plus: */
    /* The min tHD:STA/tSU:STA/tSU:STO should be at least 0.4 times of the SCL clock cycle, use 0.5 to be safe:
       tHD:STA = ((SETHOLD + 1) x (2 ^ PRESCALE) / u32_sourceClock_Hz) > (0.5 / u32_BaudRate_Hz), bestDivider = 2 ^ PRESCALE */
    uint8_t u8_TmpHold = (uint8_t)(u32_sourceClock_Hz / u32_BaudRate_Hz / u8_BestDivider / 2U) - 1U;

    /* The max tVD:DAT/tVD:ACK/tHD:DAT should be at most 0.345 times of the SCL clock cycle, use 0.25 to be safe:
       tVD:DAT = ((DATAVD + 1) x (2 ^ PRESCALE) / u32_sourceClock_Hz) < (0.25 / u32_BaudRate_Hz), bestDivider = 2 ^ PRESCALE */
    uint8_t u8_TmpDataVd = (uint8_t)(u32_sourceClock_Hz / u32_BaudRate_Hz / u8_BestDivider / 4U) - 1U;

    /* The min tSU:DAT should be at least 0.05 times of the SCL clock cycle:
       tSU:DAT = ((2 + FILTSDA + 2 ^ PRESCALE) / u32_sourceClock_Hz) >= (0.05 / baud),
       plus bestDivider = 2 ^ PRESCALE, we can come up with:
       FILTSDA >= (0.05 x u32_sourceClock_Hz / u32_BaudRate_Hz - bestDivider - 2) */
    if ((u32_sourceClock_Hz / u32_BaudRate_Hz / 20U) > (u8_BestDivider + 2U))
    {
        /* Read out the FILTSDA configuration, if it is smaller than expected, change the setting. */
        uint8_t u8_FiltSda = (uint8_t)((ps_Base->MCFGR2 & LPI2C_MCFGR2_FILTSDA_MASK) >> LPI2C_MCFGR2_FILTSDA_SHIFT);
        if (u8_FiltSda < (u32_sourceClock_Hz / u32_BaudRate_Hz / 20U - u8_BestDivider - 2U))
        {
        	u8_FiltSda = (uint8_t)(u32_sourceClock_Hz / u32_BaudRate_Hz / 20U) - u8_BestDivider - 2U;
        }
        ps_Base->MCFGR2 = (ps_Base->MCFGR2 & ~LPI2C_MCFGR2_FILTSDA_MASK) | LPI2C_MCFGR2_FILTSDA(u8_FiltSda);
    }

    /* Set CLKHI, CLKLO, SETHOLD, DATAVD value. */
    u32_TmpReg = LPI2C_MCCR0_CLKHI((uint32_t)u8_TmpHigh) |
             LPI2C_MCCR0_CLKLO((uint32_t)((uint32_t)u8_BestclkCycle - (uint32_t)u8_TmpHigh)) |
             LPI2C_MCCR0_SETHOLD((uint32_t)u8_TmpHold) | LPI2C_MCCR0_DATAVD((uint32_t)u8_TmpDataVd);
    ps_Base->MCCR0 = u32_TmpReg;

    /* Set PRESCALE value. */
    ps_Base->MCFGR1 = (ps_Base->MCFGR1 & ~LPI2C_MCFGR1_PRESCALE_MASK) | LPI2C_MCFGR1_PRESCALE(u8_BestPre);

    /* Restore master mode. */
    if (b_WasEnabled)
    {
        LLD_I2C_MasterEnable(ps_Base, true);
    }
}

/*!
 * brief Creates a new handle for the LPI2C master non-blocking APIs.
 *
 * The creation of a handle is for use with the non-blocking APIs. Once a handle
 * is created, there is not a corresponding destroy handle. If the user wants to
 * terminate a transfer, the LLD_I2C_MasterTransferAbort() API shall be called.
 *
 *
 * note The function also enables the NVIC IRQ for the input LPI2C. Need to notice
 * that on some SoCs the LPI2C IRQ is connected to INTMUX, in this case user needs to
 * enable the associated INTMUX IRQ in application.
 *
 * param ps_Base The LPI2C peripheral base address.
 * param[out] ps_handle Pointer to the LPI2C master driver handle.
 * param pf_Callback User provided pointer to the asynchronous callback function.
 * param p_UserData User provided pointer to the application callback data.
 */
void LLD_I2C_MasterTransferCreateHandle(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle, lld_i2c_master_transfer_callback_t pf_Callback, void *p_UserData)
{
    assert(NULL != ps_handle);

    /* Clear out the handle. */
    (void)memset(ps_handle, 0, sizeof(*ps_handle));

    /* Save base and instance. */
    ps_handle->pf_CompletionCallback = pf_Callback;
    ps_handle->p_UserData           = p_UserData;

    /* Save this handle for IRQ use. */
    if (ps_Base == LPI2C1)
	{
    	tp_I2cMasterHandle[1] = ps_handle;
	}
	else if (ps_Base == LPI2C2)
	{
		tp_I2cMasterHandle[2] = ps_handle;
	}
	else if (ps_Base == LPI2C3)
	{
		tp_I2cMasterHandle[3] = ps_handle;
	}
	else /* if (ps_Base == LPI2C4) */
	{
		tp_I2cMasterHandle[4] = ps_handle;
	}

    /* Clear internal IRQ enables and enable NVIC IRQ. */
    LLD_I2C_MasterDisableInterrupts(ps_Base, (uint32_t)kLPI2C_MasterIrqFlags);

    /* Enable NVIC IRQ, this only enables the IRQ directly connected to the NVIC.
     In some cases the LPI2C IRQ is configured through INTMUX, user needs to enable
     INTMUX IRQ in application code. */
    if (ps_Base == LPI2C1)
    {
    	EnableIRQ(LPI2C1_IRQn);
    }
    else if (ps_Base == LPI2C2)
	{
    	EnableIRQ(LPI2C2_IRQn);
	}
    else if (ps_Base == LPI2C3)
	{
    	EnableIRQ(LPI2C3_IRQn);
	}
    else /* if (ps_Base == LPI2C4) */
	{
    	EnableIRQ(LPI2C4_IRQn);
	}
}

/*!
 * @brief Execute states until FIFOs are exhausted.
 * @param ps_handle Master nonblocking driver handle.
 * @param[out] pb_IsDone Set to true if the transfer has completed.
 * @retval #LLD_STATUS_Success
 * @retval #kStatus_LPI2C_PinLowTimeout
 * @retval #kStatus_LPI2C_ArbitrationLost
 * @retval #kStatus_LPI2C_Nak
 * @retval #kStatus_LPI2C_FifoError
 */
static status_t LLD_I2C_RunTransferStateMachine(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle, bool *pb_IsDone)
{
    uint32_t u32_Status;
    status_t s32_Result = kStatus_Success;
    lld_i2c_master_transfer_t *ps_Xfer;
    size_t u32_TxCount;
    size_t u32_RxCount;
    size_t u32_TxFifoSize   = (size_t)FSL_FEATURE_LPI2C_FIFO_SIZEn(ps_Base);
    bool b_StateComplete = false;
    uint16_t u16_Sendval;

    /* Set default pb_IsDone return value. */
    *pb_IsDone = false;

    /* Check for errors. */
    u32_Status = LLD_I2C_MasterGetStatusFlags(ps_Base);

    /* Get fifo counts. */
    LLD_I2C_MasterGetFifoCounts(ps_Base, &u32_RxCount, &u32_TxCount);

    /* Get pointer to private data. */
    ps_Xfer = &ps_handle->s_Transfer;

    /* For the last byte, nack flag is expected.
       Do not check and clear kLPI2C_MasterNackDetectFlag for the last byte,
       in case FIFO is emptied when stop command has not been sent. */
    if (ps_handle->u16_RemainingBytes == 0U)
    {
        /* When data size is not zero which means it is not only one byte of address is sent, and */
        /* when the txfifo is empty, or have one byte which is the stop command, then the nack status can be ignored. */
        if ((ps_Xfer->u32_DataSize != 0U) &&
            ((u32_TxCount == 0U) || ((u32_TxCount == 1U) && (ps_handle->u8_State == (uint8_t)kWaitForCompletionState) &&
                                 ((ps_Xfer->u32_Flags & (uint32_t)kLPI2C_TransferNoStopFlag) == 0U))))
        {
        	u32_Status &= ~(uint32_t)kLPI2C_MasterNackDetectFlag;
        }
    }

    s32_Result = LLD_I2C_MasterCheckAndClearError(ps_Base, u32_Status);

    if (kStatus_Success == s32_Result)
    {
        /* Compute room in tx fifo */
        u32_TxCount = u32_TxFifoSize - u32_TxCount;

        while (!b_StateComplete)
        {
            /* Execute the state. */
            switch (ps_handle->u8_State)
            {
                case (uint8_t)kSendCommandState:
                    /* Make sure there is room in the tx fifo for the next command. */
                    if (0U == u32_TxCount--)
                    {
                    	b_StateComplete = true;
                        break;
                    }

                    /* Issue command. buf is a uint8_t* pointing at the uint16 command array. */
                	u16_Sendval    = ((uint16_t)ps_handle->pu8_buf[0]) | (((uint16_t)ps_handle->pu8_buf[1]) << 8U);
                    ps_Base->MTDR = u16_Sendval;
                    ps_handle->pu8_buf++;
                    ps_handle->pu8_buf++;

                    /* Count down until all commands are sent. */
                    if (--ps_handle->u16_RemainingBytes == 0U)
                    {
                        /* Choose next state and set up buffer pointer and count. */
                        if (0U != ps_Xfer->u32_DataSize)
                        {
                            /* Either a send or receive transfer is next. */
                            ps_handle->u8_State          = (uint8_t)kTransferDataState;
                            ps_handle->pu8_buf            = (uint8_t *)ps_Xfer->p_Data;
                            ps_handle->u16_RemainingBytes = (uint16_t)ps_Xfer->u32_DataSize;
                            if (ps_Xfer->e_Direction == kLPI2C_Read)
                            {
                                /* Disable TX interrupt */
                                LLD_I2C_MasterDisableInterrupts(ps_Base, (uint32_t)kLPI2C_MasterTxReadyFlag);
                                /* Issue command to receive data. A single write to MTDR can issue read operation of
                                   0xFFU + 1 byte of data at most, so when the dataSize is larger than 0x100U, push
                                   multiple read commands to MTDR until dataSize is reached. */
                                size_t u32_TmpRxSize = ps_Xfer->u32_DataSize;
                                while (u32_TmpRxSize != 0U)
                                {
                                    LLD_I2C_MasterGetFifoCounts(ps_Base, NULL, &u32_TxCount);
                                    while (u32_TxFifoSize == u32_TxCount)
                                    {
                                        LLD_I2C_MasterGetFifoCounts(ps_Base, NULL, &u32_TxCount);
                                    }

                                    if (u32_TmpRxSize > 256U)
                                    {
                                        ps_Base->MTDR = (uint32_t)(kRxDataCmd) | (uint32_t)LPI2C_MTDR_DATA(0xFFU);
                                        u32_TmpRxSize -= 256U;
                                    }
                                    else
                                    {
                                        ps_Base->MTDR = (uint32_t)(kRxDataCmd) | (uint32_t)LPI2C_MTDR_DATA(u32_TmpRxSize - 1U);
                                        u32_TmpRxSize  = 0U;
                                    }
                                }
                            }
                        }
                        else
                        {
                            /* No transfer, so move to stop state. */
                            ps_handle->u8_State = (uint8_t)kStopState;
                        }
                    }
                    break;

                case (uint8_t)kIssueReadCommandState:
                    /* Make sure there is room in the tx fifo for the read command. */
                    if (0U == u32_TxCount--)
                    {
                    	b_StateComplete = true;
                        break;
                    }

                    ps_Base->MTDR = (uint32_t)kRxDataCmd | LPI2C_MTDR_DATA(ps_Xfer->u32_DataSize - 1U);

                    /* Move to transfer state. */
                    ps_handle->u8_State = (uint8_t)kTransferDataState;
                    if (ps_Xfer->e_Direction == kLPI2C_Read)
                    {
                        /* Disable TX interrupt */
                        LLD_I2C_MasterDisableInterrupts(ps_Base, (uint32_t)kLPI2C_MasterTxReadyFlag);
                    }
                    break;

                case (uint8_t)kTransferDataState:
                    if (ps_Xfer->e_Direction == kLPI2C_Write)
                    {
                        /* Make sure there is room in the tx fifo. */
                        if (0U == u32_TxCount--)
                        {
                        	b_StateComplete = true;
                            break;
                        }

                        /* Put byte to send in fifo. */
                        ps_Base->MTDR = *(ps_handle->pu8_buf)++;
                    }
                    else
                    {
                        /* XXX handle receive sizes > 256, use kIssueReadCommandState */
                        /* Make sure there is data in the rx fifo. */
                        if (0U == u32_RxCount--)
                        {
                        	b_StateComplete = true;
                            break;
                        }

                        /* Read byte from fifo. */
                        *(ps_handle->pu8_buf)++ = (uint8_t)(ps_Base->MRDR & LPI2C_MRDR_DATA_MASK);
                    }

                    /* Move to stop when the transfer is done. */
                    if (--ps_handle->u16_RemainingBytes == 0U)
                    {
                        if (ps_Xfer->e_Direction == kLPI2C_Write)
                        {
                        	b_StateComplete = true;
                        }
                        ps_handle->u8_State = (uint8_t)kStopState;
                    }
                    break;

                case (uint8_t)kStopState:
                    /* Only issue a stop transition if the caller requested it. */
                    if ((ps_Xfer->u32_Flags & (uint32_t)kLPI2C_TransferNoStopFlag) == 0U)
                    {
                        /* Make sure there is room in the tx fifo for the stop command. */
                        if (0U == u32_TxCount--)
                        {
                        	b_StateComplete = true;
                            break;
                        }

                        ps_Base->MTDR = (uint32_t)kStopCmd;
                    }
                    else
                    {
                        /* If all data is read and no stop flag is required to send, we are done. */
                        if (ps_Xfer->e_Direction == kLPI2C_Read)
                        {
                            *pb_IsDone = true;
                        }
                        b_StateComplete = true;
                    }
                    ps_handle->u8_State = (uint8_t)kWaitForCompletionState;
                    break;

                case (uint8_t)kWaitForCompletionState:
                    if ((ps_Xfer->u32_Flags & (uint32_t)kLPI2C_TransferNoStopFlag) == 0U)
                    {
                        /* We stay in this state until the stop state is detected. */
                        if (0U != (u32_Status & (uint32_t)kLPI2C_MasterStopDetectFlag))
                        {
                            *pb_IsDone = true;
                        }
                    }
                    else
                    {
                        /* If all data is pushed to FIFO and no stop flag is required to send, we need to make sure they
                           are all send out to bus. */
                        if ((ps_Xfer->e_Direction == kLPI2C_Write) && ((ps_Base->MFSR & LPI2C_MFSR_TXCOUNT_MASK) == 0U))
                        {
                            /* We stay in this state until the data is sent out to bus. */
                            *pb_IsDone = true;
                        }
                    }
                	b_StateComplete = true;
                    break;
                default:
                    assert(false);
                    break;
            }
        }
    }
    return s32_Result;
}

/*!
 * @brief Prepares the transfer state machine and fills in the command buffer.
 * @param ps_handle Master nonblocking driver handle.
 */
static void LLD_I2C_InitTransferStateMachine(lld_i2c_master_handle_t *ps_handle)
{
    lld_i2c_master_transfer_t *ps_Xfer = &ps_handle->s_Transfer;

    /* Handle no start option. */
    if (0U != (ps_Xfer->u32_Flags & (uint32_t)kLPI2C_TransferNoStartFlag))
    {
        if (ps_Xfer->e_Direction == kLPI2C_Read)
        {
            /* Need to issue read command first. */
            ps_handle->u8_State = (uint8_t)kIssueReadCommandState;
        }
        else
        {
            /* Start immediately in the data transfer state. */
            ps_handle->u8_State = (uint8_t)kTransferDataState;
        }

        ps_handle->pu8_buf            = (uint8_t *)ps_Xfer->p_Data;
        ps_handle->u16_RemainingBytes = (uint16_t)ps_Xfer->u32_DataSize;
    }
    else
    {
        uint16_t *pu16_Cmd     = (uint16_t *)&ps_handle->tu16_CommandBuffer;
        uint32_t u32_CmdCount = 0U;

        /* Initial direction depends on whether a subaddress was provided, and of course the actual */
        /* data transfer direction. */
        lld_i2c_direction_t e_Direction = (0U != ps_Xfer->u32_SubaddressSize) ? kLPI2C_Write : ps_Xfer->e_Direction;

        /* Start command. */
        pu16_Cmd[u32_CmdCount++] =
            (uint16_t)kStartCmd | (uint16_t)((uint16_t)((uint16_t)ps_Xfer->u16_SlaveAddress << 1U) | (uint16_t)e_Direction);

        /* Subaddress, MSB first. */
        if (0U != ps_Xfer->u32_SubaddressSize)
        {
            uint32_t u32_SubaddressRemaining = ps_Xfer->u32_SubaddressSize;
            while (0U != (u32_SubaddressRemaining--))
            {
                uint8_t u8_SubaddressByte = (uint8_t)((ps_Xfer->u32_Subaddress >> (8U * u32_SubaddressRemaining)) & 0xffU);
                pu16_Cmd[u32_CmdCount++]        = u8_SubaddressByte;
            }
        }

        /* Reads need special handling. */
        if ((0U != ps_Xfer->u32_DataSize) && (ps_Xfer->e_Direction == kLPI2C_Read))
        {
            /* Need to send repeated start if switching directions to read. */
            if (e_Direction == kLPI2C_Write)
            {
            	pu16_Cmd[u32_CmdCount++] = (uint16_t)kStartCmd |
                                  (uint16_t)((uint16_t)((uint16_t)ps_Xfer->u16_SlaveAddress << 1U) | (uint16_t)kLPI2C_Read);
            }
        }

        /* Set up state machine for transferring the commands. */
        ps_handle->u8_State       = (uint8_t)kSendCommandState;
        ps_handle->u16_RemainingBytes = (uint16_t)u32_CmdCount;
        ps_handle->pu8_buf            = (uint8_t *)&ps_handle->tu16_CommandBuffer;
    }
}

/*!
 * brief Performs a non-blocking transaction on the I2C bus.
 *
 * param ps_Base The LPI2C peripheral base address.
 * param ps_handle Pointer to the LPI2C master driver handle.
 * param ps_Transfer The pointer to the transfer descriptor.
 * retval #LLD_STATUS_Success The transaction was started successfully.
 * retval #kStatus_LPI2C_Busy Either another master is currently utilizing the bus, or a non-blocking
 *      transaction is already in progress.
 */
status_t LLD_I2C_MasterTransferNonBlocking(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle, lld_i2c_master_transfer_t *ps_Transfer)
{
    assert(NULL != ps_handle);
    assert(NULL != ps_Transfer);
    assert(ps_Transfer->u32_SubaddressSize <= sizeof(ps_Transfer->u32_Subaddress));

    status_t s32_Result;

    /* Check transfer data size in read operation. */
    if ((ps_Transfer->e_Direction == kLPI2C_Read) &&
        (ps_Transfer->u32_DataSize > (256U * (uint32_t)FSL_FEATURE_LPI2C_FIFO_SIZEn(ps_Base))))
    {
        return kStatus_InvalidArgument;
    }

    /* Return busy if another transaction is in progress. */
    if (ps_handle->u8_State != (uint8_t)kIdleState)
    {
    	s32_Result = kStatus_LPI2C_Busy;
    }
    else
    {
    	s32_Result = LLD_I2C_CheckForBusyBus(ps_Base);
    }

    if ((status_t)kStatus_Success == s32_Result)
    {
        /* Disable LPI2C IRQ sources while we configure stuff. */
        LLD_I2C_MasterDisableInterrupts(ps_Base, (uint32_t)kLPI2C_MasterIrqFlags);

        /* Reset FIFO in case there are data. */
        ps_Base->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;

        /* Save transfer into handle. */
        ps_handle->s_Transfer = *ps_Transfer;

        /* Generate commands to send. */
        LLD_I2C_InitTransferStateMachine(ps_handle);

        /* Clear all flags. */
        LLD_I2C_MasterClearStatusFlags(ps_Base, (uint32_t)kLPI2C_MasterClearFlags);

        /* Turn off auto-stop option. */
        ps_Base->MCFGR1 &= ~LPI2C_MCFGR1_AUTOSTOP_MASK;

        /* Enable LPI2C internal IRQ sources. NVIC IRQ was enabled in CreateHandle() */
        LLD_I2C_MasterEnableInterrupts(ps_Base, (uint32_t)kLPI2C_MasterIrqFlags);
    }

    return s32_Result;
}

/*!
 * brief Terminates a non-blocking LPI2C master transmission early.
 *
 * note It is not safe to call this function from an IRQ handler that has a higher priority than the
 *      LPI2C peripheral's IRQ priority.
 *
 * param ps_Base The LPI2C peripheral base address.
 * param ps_handle Pointer to the LPI2C master driver handle.
 * retval #LLD_STATUS_Success A transaction was successfully aborted.
 * retval #kStatus_LPI2C_Idle There is not a non-blocking transaction currently in progress.
 */
void LLD_I2C_MasterTransferAbort(LPI2C_Type *ps_Base, lld_i2c_master_handle_t *ps_handle)
{
    if (ps_handle->u8_State != (uint8_t)kIdleState)
    {
        /* Disable internal IRQ enables. */
        LLD_I2C_MasterDisableInterrupts(ps_Base, (uint32_t)kLPI2C_MasterIrqFlags);

        /* Reset fifos. */
        ps_Base->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;

        /* If master is still busy and has not send out stop signal yet. */
        if ((LLD_I2C_MasterGetStatusFlags(ps_Base) & ((uint32_t)kLPI2C_MasterStopDetectFlag |
                                                 (uint32_t)kLPI2C_MasterBusyFlag)) == (uint32_t)kLPI2C_MasterBusyFlag)
        {
            /* Send a stop command to finalize the transfer. */
            ps_Base->MTDR = (uint32_t)kStopCmd;
        }

        /* Reset handle. */
        ps_handle->u8_State = (uint8_t)kIdleState;
    }
}

/*!
 * brief Reusable routine to handle master interrupts.
 * note This function does not need to be called unless you are reimplementing the
 *  nonblocking API's interrupt handler routines to add special functionality.
 * param ps_Base The LPI2C peripheral base address.
 * param p_I2cMasterHandle Pointer to the LPI2C master driver handle.
 */
void LLD_I2C_MasterTransferHandleIRQ(LPI2C_Type *ps_Base, void *p_I2cMasterHandle)
{
    assert(p_I2cMasterHandle != NULL);

    lld_i2c_master_handle_t *ps_handle = (lld_i2c_master_handle_t *)p_I2cMasterHandle;
    bool b_IsDone                   = false;
    status_t s32_Result;
    lldI2c_t e_I2c;

    if (ps_Base == LPI2C1)
	{
    	e_I2c = LLD_I2C_I2C1;
	}
	else if (ps_Base == LPI2C2)
	{
    	e_I2c = LLD_I2C_I2C2;
	}
	else if (ps_Base == LPI2C3)
	{
    	e_I2c = LLD_I2C_I2C3;
	}
	else /* if (ps_Base == LPI2C4) */
	{
    	e_I2c = LLD_I2C_I2C4;
	}

    /* Don't do anything if we don't have a valid handle. */
    if (NULL != ps_handle)
    {
        if (ps_handle->u8_State != (uint8_t)kIdleState)
        {
        	s32_Result = LLD_I2C_RunTransferStateMachine(ps_Base, ps_handle, &b_IsDone);

            if ((s32_Result != kStatus_Success) || b_IsDone)
            {
                /* Handle error, terminate xfer */
                if (s32_Result != kStatus_Success)
                {
                    LLD_I2C_MasterTransferAbort(ps_Base, ps_handle);
                }

                /* Disable internal IRQ enables. */
                LLD_I2C_MasterDisableInterrupts(ps_Base, (uint32_t)kLPI2C_MasterIrqFlags);

                /* Set handle to idle state. */
                ps_handle->u8_State = (uint8_t)kIdleState;

                /* Invoke callback. */
                if (NULL != ps_handle->pf_CompletionCallback)
                {
                    ps_handle->pf_CompletionCallback(e_I2c);
                }
            }
        }
    }
}

/*!
 * @brief Enables or disables the LPI2C module as master.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @param b_Enable Pass true to enable or false to disable the specified LPI2C as master.
 */
static inline void LLD_I2C_MasterEnable(LPI2C_Type *ps_Base, bool b_Enable)
{
    ps_Base->MCR = (ps_Base->MCR & ~LPI2C_MCR_MEN_MASK) | LPI2C_MCR_MEN(b_Enable);
}

/*!
 * @brief Gets the LPI2C master status flags.
 *
 * A bit mask with the state of all LPI2C master status flags is returned. For each flag, the corresponding bit
 * in the return value is set if the flag is asserted.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @return State of the status flags:
 *         - 1: related status flag is set.
 *         - 0: related status flag is not set.
 * @see _lpi2c_master_flags
 */
static inline uint32_t LLD_I2C_MasterGetStatusFlags(LPI2C_Type *ps_Base)
{
    return ps_Base->MSR;
}

/*!
 * @brief Clears the LPI2C master status flag state.
 *
 * The following status register flags can be cleared:
 * - #kLPI2C_MasterEndOfPacketFlag
 * - #kLPI2C_MasterStopDetectFlag
 * - #kLPI2C_MasterNackDetectFlag
 * - #kLPI2C_MasterArbitrationLostFlag
 * - #kLPI2C_MasterFifoErrFlag
 * - #kLPI2C_MasterPinLowTimeoutFlag
 * - #kLPI2C_MasterDataMatchFlag
 *
 * Attempts to clear other flags has no effect.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @param u32_StatusMask A bitmask of status flags that are to be cleared. The mask is composed of
 *  _lpi2c_master_flags enumerators OR'd together. You may pass the result of a previous call to
 *  LLD_I2C_MasterGetStatusFlags().
 * @see _lpi2c_master_flags.
 */
static inline void LLD_I2C_MasterClearStatusFlags(LPI2C_Type *ps_Base, uint32_t u32_StatusMask)
{
    ps_Base->MSR = u32_StatusMask;
}

/*!
 * @brief Enables the LPI2C master interrupt requests.
 *
 * All flags except #kLPI2C_MasterBusyFlag and #kLPI2C_MasterBusBusyFlag can be enabled as
 * interrupts.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @param u32_InterruptMask Bit mask of interrupts to enable. See _lpi2c_master_flags for the set
 *      of constants that should be OR'd together to form the bit mask.
 */
static inline void LLD_I2C_MasterEnableInterrupts(LPI2C_Type *ps_Base, uint32_t u32_InterruptMask)
{
    ps_Base->MIER |= u32_InterruptMask;
}

/*!
 * @brief Disables the LPI2C master interrupt requests.
 *
 * All flags except #kLPI2C_MasterBusyFlag and #kLPI2C_MasterBusBusyFlag can be enabled as
 * interrupts.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @param u32_InterruptMask Bit mask of interrupts to disable. See _lpi2c_master_flags for the set
 *      of constants that should be OR'd together to form the bit mask.
 */
static inline void LLD_I2C_MasterDisableInterrupts(LPI2C_Type *ps_Base, uint32_t u32_InterruptMask)
{
    ps_Base->MIER &= ~u32_InterruptMask;
}

/*!
 * @brief Sets the watermarks for LPI2C master FIFOs.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @param u32_TxWords Transmit FIFO watermark value in words. The #kLPI2C_MasterTxReadyFlag flag is set whenever
 *      the number of words in the transmit FIFO is equal or less than @a u32_TxWords. Writing a value equal or
 *      greater than the FIFO size is truncated.
 * @param u32_RxWords Receive FIFO watermark value in words. The #kLPI2C_MasterRxReadyFlag flag is set whenever
 *      the number of words in the receive FIFO is greater than @a u32_RxWords. Writing a value equal or greater
 *      than the FIFO size is truncated.
 */
static inline void LLD_I2C_MasterSetWatermarks(LPI2C_Type *ps_Base, size_t u32_TxWords, size_t u32_RxWords)
{
    ps_Base->MFCR = LPI2C_MFCR_TXWATER(u32_TxWords) | LPI2C_MFCR_RXWATER(u32_RxWords);
}

/*!
 * @brief Gets the current number of words in the LPI2C master FIFOs.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @param[out] pu32_TxCount Pointer through which the current number of words in the transmit FIFO is returned.
 *      Pass NULL if this value is not required.
 * @param[out] pu32_RxCount Pointer through which the current number of words in the receive FIFO is returned.
 *      Pass NULL if this value is not required.
 */
static inline void LLD_I2C_MasterGetFifoCounts(LPI2C_Type *ps_Base, size_t *pu32_RxCount, size_t *pu32_TxCount)
{
    if (NULL != pu32_TxCount)
    {
        *pu32_TxCount = (ps_Base->MFSR & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT;
    }
    if (NULL != pu32_RxCount)
    {
        *pu32_RxCount = (ps_Base->MFSR & LPI2C_MFSR_RXCOUNT_MASK) >> LPI2C_MFSR_RXCOUNT_SHIFT;
    }
}

/*!
 * @brief Shared IRQ handler that can call both master and slave ISRs.
 *
 * The master and slave ISRs are called through function pointers in order to decouple
 * this code from the ISR functions. Without this, the linker would always pull in both
 * ISRs and every function they call, even if only the functional API was used.
 *
 * @param ps_Base The LPI2C peripheral base address.
 * @param u32_Instance The LPI2C peripheral instance number.
 */
static void LLD_I2C_IRQHandler(LPI2C_Type *ps_Base, uint32_t u32_Instance)
{
    /* Check for master IRQ. */
    if(0U != (ps_Base->MCR & LPI2C_MCR_MEN_MASK))
    {
        /* Master mode. */
    	LLD_I2C_MasterTransferHandleIRQ(ps_Base, tp_I2cMasterHandle[u32_Instance]);
    }

    SDK_ISR_EXIT_BARRIER;
}


/* Implementation of LPI2C1 handler named in startup code. */
void LPI2C1_DriverIRQHandler(void)
{
    LLD_I2C_IRQHandler(LPI2C1, 1U);
}

/* Implementation of LPI2C2 handler named in startup code. */
void LPI2C2_DriverIRQHandler(void)
{
    LLD_I2C_IRQHandler(LPI2C2, 2U);
}

/* Implementation of LPI2C3 handler named in startup code. */
void LPI2C3_DriverIRQHandler(void)
{
    LLD_I2C_IRQHandler(LPI2C3, 3U);
}

/* Implementation of LPI2C4 handler named in startup code. */
void LPI2C4_DriverIRQHandler(void)
{
    LLD_I2C_IRQHandler(LPI2C4, 4U);
}

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/**
* @brief		I2C initialization
* @param		e_I2c : I2C number
* @param		u32_Frequency : frequency to configure
* @param		pf_callback : pointer on callback function
* @return		void
* @details
**/
void LLD_I2C_Init(lldI2c_t e_I2c, uint32_t u32_Frequency, lld_i2c_master_transfer_callback_t pf_Callback)
{
	lld_i2c_master_config_t s_MasterConfig;

    /*Clock setting for LPI2C*/
	CLOCK_SetMux(kCLOCK_Lpi2cMux, 0);
	CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 0);

    if (e_I2c == LLD_I2C_I2C1)
	{
		ts_lld_i2c_manager[e_I2c].ps_Base = LPI2C1;
	}
	else if (e_I2c == LLD_I2C_I2C2)
	{
		ts_lld_i2c_manager[e_I2c].ps_Base = LPI2C2;
	}
	else if (e_I2c == LLD_I2C_I2C3)
	{
		ts_lld_i2c_manager[e_I2c].ps_Base = LPI2C3;
	}
	else /* if (e_I2c == LLD_I2C_I2C4) */
	{
		ts_lld_i2c_manager[e_I2c].ps_Base = LPI2C4;
	}

    /* Change the default baudrate configuration */
    s_MasterConfig.u32_BaudRate_Hz = u32_Frequency;

    /* Initialize the LPI2C master peripheral */
    LLD_I2C_MasterInit(ts_lld_i2c_manager[e_I2c].ps_Base, &s_MasterConfig, LPI2C_CLOCK_FREQUENCY);

    /* Create the LPI2C handle for the non-blocking transfer */
    LLD_I2C_MasterTransferCreateHandle(ts_lld_i2c_manager[e_I2c].ps_Base, &ts_MasterHandle[e_I2c], pf_Callback, NULL);
}

/**
* @brief		Write data to slave
* @param		e_I2c : I2C number
* @param		u8_SlaveAddress : slave address
* @param		u8_DataAddress : data address
* @param		u8_DataAddressSize : data address
* @param		pu8_MasterTxBuff : pointer on tx buffer
* @param		u8_TxBuffSize : size of tx buffer
* @return		e_Result
* @details
**/
lldI2cStatus_t LLD_I2C_Write(lldI2c_t e_I2c, uint8_t u8_SlaveAddress, uint8_t u8_DataAddress,
					uint8_t u8_DataAddressSize, uint8_t* pu8_MasterTxBuff, uint8_t u8_TxBuffSize)
{
	lldI2cStatus_t e_Result;
	status_t s32_ReVal                     = kStatus_Fail;
	lld_i2c_master_transfer_t s_MasterXfer = {0};

	/* subAddress = 0x01, data = g_master_txBuff - write to slave.
	  start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
	s_MasterXfer.u16_SlaveAddress   = (uint16_t)u8_SlaveAddress;
	s_MasterXfer.e_Direction      = kLPI2C_Write;
	s_MasterXfer.u32_Subaddress     = (uint32_t)u8_DataAddress;
	s_MasterXfer.u32_SubaddressSize = (uint32_t)u8_DataAddressSize;
	s_MasterXfer.p_Data           = pu8_MasterTxBuff;
	s_MasterXfer.u32_DataSize       = u8_TxBuffSize;
	s_MasterXfer.u32_Flags          = kLPI2C_TransferDefaultFlag;

	/* Send master non-blocking data to slave */
	s32_ReVal = LLD_I2C_MasterTransferNonBlocking(ts_lld_i2c_manager[e_I2c].ps_Base, &ts_MasterHandle[e_I2c], &s_MasterXfer);

	if (s32_ReVal != kStatus_Success)
	{
		e_Result = LLD_I2C_STATUS_ERROR;
	}
	else
	{
		e_Result = LLD_I2C_STATUS_SUCCESS;
	}

	return e_Result;
}

/**
* @brief		Read data from slave
* @param		e_I2c : I2C number
* @param		u8_SlaveAddress : slave address (7 bits)
* @param		u8_DataAddress : data address
* @param		u8_DataAddressSize : size of data address
* @param		pu8_MasterRxBuff : pointer on rx buffer
* @param		u8_RxBuffSize : size of rx buffer
* @return		e_Result
* @details
**/
lldI2cStatus_t LLD_I2C_Read(lldI2c_t e_I2c, uint8_t u8_SlaveAddress, uint8_t u8_DataAddress,
					uint8_t u8_DataAddressSize, uint8_t* pu8_MasterRxBuff, uint8_t u8_RxBuffSize)
{
	lldI2cStatus_t e_Result;
	status_t s32_ReVal                     = kStatus_Fail;
	lld_i2c_master_transfer_t s_MasterXfer = {0};

	/* subAddress = 0x01, data = g_master_rxBuff - read from slave.
	  start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
	s_MasterXfer.u16_SlaveAddress   = (uint16_t)u8_SlaveAddress;
	s_MasterXfer.e_Direction      = kLPI2C_Read;
	s_MasterXfer.u32_Subaddress     = (uint32_t)u8_DataAddress;
	s_MasterXfer.u32_SubaddressSize = (uint32_t)u8_DataAddressSize;
	s_MasterXfer.p_Data           = pu8_MasterRxBuff;
	s_MasterXfer.u32_DataSize       = u8_RxBuffSize;
	s_MasterXfer.u32_Flags          = kLPI2C_TransferDefaultFlag;

	s32_ReVal = LLD_I2C_MasterTransferNonBlocking(ts_lld_i2c_manager[e_I2c].ps_Base, &ts_MasterHandle[e_I2c], &s_MasterXfer);

	if (s32_ReVal != kStatus_Success)
	{
		e_Result = LLD_I2C_STATUS_ERROR;
	}
	else
	{
		e_Result = LLD_I2C_STATUS_SUCCESS;
	}

	return e_Result;
}
