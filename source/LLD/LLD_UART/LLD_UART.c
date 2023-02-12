/**
 * @file LLD_UART.c
 * @author ACR
 * @brief Specific UART driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_UART_USE_LOCALS
#include <stdint.h>
#include <string.h>

#include "MIMXRT1061.h"
#include "MIMXRT1061_features.h"
#include "fsl_clock.h"
#include "LLD_UART.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES LPUART ...                                                 */
/*--------------------------------------------------------------------------*/

/*! @brief Error codes for the LPUART driver. */
enum
{
    kStatus_LPUART_TxBusy = 1300,			 /*!< TX busy */
    kStatus_LPUART_RxBusy,					 /*!< RX busy */
    kStatus_LPUART_TxIdle,					 /*!< LPUART transmitter is idle. */
    kStatus_LPUART_RxIdle, 					 /*!< LPUART receiver is idle. */
    kStatus_LPUART_TxWatermarkTooLarge,		 /*!< TX FIFO watermark too large  */
    kStatus_LPUART_RxWatermarkTooLarge, 	 /*!< RX FIFO watermark too large  */
    kStatus_LPUART_FlagCannotClearManually,  /*!< Some flag can't manually clear */
    kStatus_LPUART_Error,					 /*!< Error happens on LPUART. */
    kStatus_LPUART_RxRingBufferOverrun, 	 /*!< LPUART RX software ring buffer overrun. */
    kStatus_LPUART_RxHardwareOverrun,  		 /*!< LPUART RX receiver overrun. */
    kStatus_LPUART_NoiseError,				 /*!< LPUART noise error. */
    kStatus_LPUART_FramingError,			 /*!< LPUART framing error. */
    kStatus_LPUART_ParityError,				 /*!< LPUART parity error. */
    kStatus_LPUART_BaudrateNotSupport, 		 /*!< Baudrate is not support in current clock source */
    kStatus_LPUART_IdleLineDetected, 		 /*!< IDLE flag. */
    kStatus_LPUART_Timeout,					 /*!< LPUART times out. */
};

/*! @brief LPUART parity mode. */
typedef enum _lld_uart_parity_mode
{
    kLPUART_ParityDisabled = 0x0U, /*!< Parity disabled */
    kLPUART_ParityEven     = 0x2U, /*!< Parity enabled, type even, bit setting: PE|PT = 10 */
    kLPUART_ParityOdd      = 0x3U, /*!< Parity enabled, type odd,  bit setting: PE|PT = 11 */
} lld_uart_parity_mode_t;

/*! @brief LPUART data bits count. */
typedef enum _lld_uart_data_bits
{
    kLPUART_EightDataBits = 0x0U, /*!< Eight data bit */
    kLPUART_SevenDataBits = 0x1U, /*!< Seven data bit */
} lld_uart_data_bits_t;

/*! @brief LPUART stop bit count. */
typedef enum _lld_uart_stop_bit_count
{
    kLPUART_OneStopBit = 0U, /*!< One stop bit */
    kLPUART_TwoStopBit = 1U, /*!< Two stop bits */
} lld_uart_stop_bit_count_t;

/*! @brief LPUART transmit CTS source. */
typedef enum _lld_uart_transmit_cts_source
{
    kLPUART_CtsSourcePin         = 0U, /*!< CTS resource is the LPUART_CTS pin. */
    kLPUART_CtsSourceMatchResult = 1U, /*!< CTS resource is the match result. */
} lld_uart_transmit_cts_source_t;

/*! @brief LPUART transmit CTS configure. */
typedef enum _lld_uart_transmit_cts_config
{
    kLPUART_CtsSampleAtStart = 0U, /*!< CTS input is sampled at the start of each character. */
    kLPUART_CtsSampleAtIdle  = 1U, /*!< CTS input is sampled when the transmitter is idle */
} lld_uart_transmit_cts_config_t;

/*! @brief LPUART idle flag type defines when the receiver starts counting. */
typedef enum _lld_uart_idle_type_select
{
    kLPUART_IdleTypeStartBit = 0U, /*!< Start counting after a valid start bit. */
    kLPUART_IdleTypeStopBit  = 1U, /*!< Start counting after a stop bit. */
} lld_uart_idle_type_select_t;

/*! @brief LPUART idle detected configuration.
 *  This structure defines the number of idle characters that must be received before
 *  the IDLE flag is set.
 */
typedef enum _lld_uart_idle_config
{
    kLPUART_IdleCharacter1   = 0U, /*!< the number of idle characters. */
    kLPUART_IdleCharacter2   = 1U, /*!< the number of idle characters. */
    kLPUART_IdleCharacter4   = 2U, /*!< the number of idle characters. */
    kLPUART_IdleCharacter8   = 3U, /*!< the number of idle characters. */
    kLPUART_IdleCharacter16  = 4U, /*!< the number of idle characters. */
    kLPUART_IdleCharacter32  = 5U, /*!< the number of idle characters. */
    kLPUART_IdleCharacter64  = 6U, /*!< the number of idle characters. */
    kLPUART_IdleCharacter128 = 7U, /*!< the number of idle characters. */
} lld_uart_idle_config_t;

/*!
 * @brief LPUART interrupt configuration structure, default settings all disabled.
 *
 * This structure contains the settings for all LPUART interrupt configurations.
 */
enum _lld_uart_interrupt_enable
{
    kLPUART_LinBreakInterruptEnable = (LPUART_BAUD_LBKDIE_MASK >> 8U), /*!< LIN break detect. bit 7 */
    kLPUART_RxActiveEdgeInterruptEnable         = (LPUART_BAUD_RXEDGIE_MASK >> 8U), /*!< Receive Active Edge. bit 6 */
    kLPUART_TxDataRegEmptyInterruptEnable       = (LPUART_CTRL_TIE_MASK),  /*!< Transmit data register empty. bit 23 */
    kLPUART_TransmissionCompleteInterruptEnable = (LPUART_CTRL_TCIE_MASK), /*!< Transmission complete. bit 22 */
    kLPUART_RxDataRegFullInterruptEnable        = (LPUART_CTRL_RIE_MASK),  /*!< Receiver data register full. bit 21 */
    kLPUART_IdleLineInterruptEnable             = (LPUART_CTRL_ILIE_MASK), /*!< Idle line. bit 20 */
    kLPUART_RxOverrunInterruptEnable            = (LPUART_CTRL_ORIE_MASK), /*!< Receiver Overrun. bit 27 */
    kLPUART_NoiseErrorInterruptEnable           = (LPUART_CTRL_NEIE_MASK), /*!< Noise error flag. bit 26 */
    kLPUART_FramingErrorInterruptEnable         = (LPUART_CTRL_FEIE_MASK), /*!< Framing error flag. bit 25 */
    kLPUART_ParityErrorInterruptEnable          = (LPUART_CTRL_PEIE_MASK), /*!< Parity error flag. bit 24 */
    kLPUART_Match1InterruptEnable = (LPUART_CTRL_MA1IE_MASK), /*!< Parity error flag. bit 15 */
    kLPUART_Match2InterruptEnable = (LPUART_CTRL_MA2IE_MASK), /*!< Parity error flag. bit 14 */
    kLPUART_TxFifoOverflowInterruptEnable  = (LPUART_FIFO_TXOFE_MASK), /*!< Transmit FIFO Overflow. bit 9 */
    kLPUART_RxFifoUnderflowInterruptEnable = (LPUART_FIFO_RXUFE_MASK), /*!< Receive FIFO Underflow. bit 8 */

    kLPUART_AllInterruptEnable = kLPUART_RxActiveEdgeInterruptEnable | kLPUART_TxDataRegEmptyInterruptEnable |
                                 kLPUART_TransmissionCompleteInterruptEnable | kLPUART_RxDataRegFullInterruptEnable |
                                 kLPUART_IdleLineInterruptEnable | kLPUART_RxOverrunInterruptEnable |
                                 kLPUART_NoiseErrorInterruptEnable | kLPUART_FramingErrorInterruptEnable |
                                 kLPUART_ParityErrorInterruptEnable | kLPUART_LinBreakInterruptEnable |
								 kLPUART_Match1InterruptEnable | kLPUART_Match2InterruptEnable |
								 kLPUART_TxFifoOverflowInterruptEnable | kLPUART_RxFifoUnderflowInterruptEnable ,
};

/*!
 * @brief LPUART status flags.
 *
 * This provides constants for the LPUART status flags for use in the LPUART functions.
 */
enum _lld_uart_flags
{
    kLPUART_TxDataRegEmptyFlag =
        (LPUART_STAT_TDRE_MASK), /*!< Transmit data register empty flag, sets when transmit buffer is empty. bit 23 */
    kLPUART_TransmissionCompleteFlag =
        (LPUART_STAT_TC_MASK), /*!< Transmission complete flag, sets when transmission activity complete. bit 22 */
    kLPUART_RxDataRegFullFlag = (LPUART_STAT_RDRF_MASK), /*!< Receive data register full flag, sets when the receive
                                                            data buffer is full. bit 21 */
    kLPUART_IdleLineFlag  = (LPUART_STAT_IDLE_MASK), /*!< Idle line detect flag, sets when idle line detected. bit 20 */
    kLPUART_RxOverrunFlag = (LPUART_STAT_OR_MASK),   /*!< Receive Overrun, sets when new data is received before data is
                                                        read from receive register. bit 19 */
    kLPUART_NoiseErrorFlag = (LPUART_STAT_NF_MASK),  /*!< Receive takes 3 samples of each received bit.  If any of these
                                                        samples differ, noise flag sets. bit 18 */
    kLPUART_FramingErrorFlag =
        (LPUART_STAT_FE_MASK), /*!< Frame error flag, sets if logic 0 was detected where stop bit expected. bit 17 */
    kLPUART_ParityErrorFlag = (LPUART_STAT_PF_MASK), /*!< If parity enabled, sets upon parity error detection. bit 16 */
    kLPUART_LinBreakFlag = (LPUART_STAT_LBKDIF_MASK), /*!< LIN break detect interrupt flag, sets when LIN break
                                                         char detected and LIN circuit enabled. bit 31 */
    kLPUART_RxActiveEdgeFlag = (LPUART_STAT_RXEDGIF_MASK), /*!< Receive pin active edge interrupt flag, sets when active
                                                              edge detected. bit 30 */
    kLPUART_RxActiveFlag =
        (LPUART_STAT_RAF_MASK), /*!< Receiver Active Flag (RAF), sets at beginning of valid start. bit 24 */
    kLPUART_DataMatch1Flag =
        LPUART_STAT_MA1F_MASK, /*!< The next character to be read from LPUART_DATA matches MA1. bit 15 */
    kLPUART_DataMatch2Flag =
        LPUART_STAT_MA2F_MASK, /*!< The next character to be read from LPUART_DATA matches MA2. bit 14 */
    kLPUART_TxFifoEmptyFlag =
        (LPUART_FIFO_TXEMPT_MASK >> 16), /*!< TXEMPT bit, sets if transmit buffer is empty. bit 7 */
    kLPUART_RxFifoEmptyFlag =
        (LPUART_FIFO_RXEMPT_MASK >> 16), /*!< RXEMPT bit, sets if receive buffer is empty. bit 6 */
    kLPUART_TxFifoOverflowFlag =
        (LPUART_FIFO_TXOF_MASK >> 16), /*!< TXOF bit, sets if transmit buffer overflow occurred. bit 1 */
    kLPUART_RxFifoUnderflowFlag =
        (LPUART_FIFO_RXUF_MASK >> 16), /*!< RXUF bit, sets if receive buffer underflow occurred. bit 0 */

    kLPUART_AllClearFlags = kLPUART_RxActiveEdgeFlag | kLPUART_IdleLineFlag | kLPUART_RxOverrunFlag |
                            kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag |
							kLPUART_DataMatch1Flag | kLPUART_DataMatch2Flag | kLPUART_TxFifoOverflowFlag |
							kLPUART_RxFifoUnderflowFlag | kLPUART_LinBreakFlag ,

    kLPUART_AllFlags =
        kLPUART_RxActiveEdgeFlag | kLPUART_IdleLineFlag | kLPUART_RxOverrunFlag | kLPUART_TxDataRegEmptyFlag |
        kLPUART_TransmissionCompleteFlag | kLPUART_RxDataRegFullFlag | kLPUART_RxActiveFlag | kLPUART_NoiseErrorFlag |
        kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag | kLPUART_DataMatch1Flag | kLPUART_DataMatch2Flag |
		kLPUART_TxFifoOverflowFlag | kLPUART_RxFifoUnderflowFlag | kLPUART_TxFifoEmptyFlag | kLPUART_RxFifoEmptyFlag |
		kLPUART_LinBreakFlag ,
};

/*! @brief LPUART transfer structure. */
typedef struct _lld_uart_transfer
{
    /*
     * Use separate TX and RX data pointer, because TX data is const data.
     * The member data is kept for backward compatibility.
     */
    union
    {
        uint8_t *pu8_Data;         /*!< The buffer of data to be transfer.*/
        uint8_t *pu8_RxData;       /*!< The buffer to receive data. */
        const uint8_t *pcu8_TxData; /*!< The buffer of data to be sent. */
    };
    size_t u32_DataSize; /*!< The byte count to be transfer. */
} lld_uart_transfer_t;

/*! @brief LPUART handle structure. */
struct _lld_uart_handle
{
    const uint8_t *volatile pcu8_TxData; /*!< Address of remaining data to send. */
    volatile size_t u32_TxDataSize;     /*!< Size of the remaining data to send. */
    size_t u32_TxDataSizeAll;           /*!< Size of the data to send out. */
    uint8_t *volatile pu8_RxData;       /*!< Address of remaining data to receive. */
    volatile size_t u32_RxDataSize;     /*!< Size of the remaining data to receive. */
    size_t u32_RxDataSizeAll;           /*!< Size of the data to receive. */

    uint8_t *pu8_RxRingBuffer;              /*!< Start address of the receiver ring buffer. */
    size_t u32_RxRingBufferSize;            /*!< Size of the ring buffer. */
    volatile uint16_t u16_RxRingBufferHead; /*!< Index for the driver to store received data into ring buffer. */
    volatile uint16_t u16_RxRingBufferTail; /*!< Index for the user to get data from the ring buffer. */

    lld_uart_transfer_callback_t pf_Callback; /*!< Callback function. */
    void *p_UserData;                      /*!< LPUART callback function parameter.*/

    volatile uint8_t u8_TxState; /*!< TX transfer state. */
    volatile uint8_t u8_RxState; /*!< RX transfer state. */

    bool b_IsSevenDataBits; /*!< Seven data bits flag. */
};

/* LPUART transfer state. */
enum
{
    kLPUART_TxIdle, /*!< TX idle. */
    kLPUART_TxBusy, /*!< TX busy. */
    kLPUART_RxIdle, /*!< RX idle. */
    kLPUART_RxBusy  /*!< RX busy. */
};

/* Typedef for interrupt handler. */
typedef void (*lld_uart_isr_t)(LPUART_Type *ps_Base, void *ps_Handle);

/* Array of LPUART handle. */
void *s_lpuartHandle[9];

/* LPUART ISR for transactional APIs. */
lld_uart_isr_t s_lpuartIsr;

/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD UART ...                                               */
/*--------------------------------------------------------------------------*/

lld_uart_handle_t g_lpuartHandle;

typedef struct
{
	LPUART_Type * ps_Base;
}typ_Lld_Uart_manager;

typ_Lld_Uart_manager ts_lld_Uart_manager[LLD_UART_NB];

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

static inline void LLD_UART_SoftwareReset(LPUART_Type *ps_Base);
size_t LLD_UART_TransferGetRxRingBufferLength(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle);
static bool LLD_UART_TransferIsRxRingBufferFull(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle);
static void LLD_UART_WriteNonBlocking(LPUART_Type *ps_Base, const uint8_t *pcu8_Data, size_t u32_Length);
static void LLD_UART_ReadNonBlocking(LPUART_Type *ps_Base, uint8_t *pu8_Data, size_t u32_Length);
status_t LLD_UART_Initialization(LPUART_Type *ps_Base, const lld_uart_config_t *pcs_Config, uint32_t u32_SrcClock_Hz);
uint32_t LLD_UART_GetEnabledInterrupts(LPUART_Type *ps_Base);
uint32_t LLD_UART_GetStatusFlags(LPUART_Type *ps_Base);
void LLD_UART_TransferCreateHandle(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle, lld_uart_transfer_callback_t pf_Callback,
                                 void *p_UserData);
status_t LLD_UART_TransferSendNonBlocking(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle, lld_uart_transfer_t *ps_Xfer);
status_t LLD_UART_TransferReceiveNonBlocking(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle, lld_uart_transfer_t *ps_Xfer,
                                           size_t *pu32_ReceivedBytes);
void LLD_UART_TransferHandleIRQ(LPUART_Type *ps_Base, void *p_IrqHandle);

void LPUART1_DriverIRQHandler(void);
void LPUART2_DriverIRQHandler(void);
void LPUART3_DriverIRQHandler(void);
void LPUART4_DriverIRQHandler(void);
void LPUART5_DriverIRQHandler(void);
void LPUART6_DriverIRQHandler(void);
void LPUART7_DriverIRQHandler(void);
void LPUART8_DriverIRQHandler(void);

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*!
 * @brief Resets the LPUART using software.
 *
 * This function resets all internal logic and registers except the Global Register.
 * Remains set until cleared by software.
 *
 * @param ps_Base LPUART peripheral base address.
 */
static inline void LLD_UART_SoftwareReset(LPUART_Type *ps_Base)
{
    ps_Base->GLOBAL |= LPUART_GLOBAL_RST_MASK;
    ps_Base->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;
}

/*!
 * brief Get the length of received pu8_Data in RX ring buffer.
 *
 * userData handle LPUART handle pointer.
 * return Length of received data in RX ring buffer.
 */
size_t LLD_UART_TransferGetRxRingBufferLength(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle)
{
    assert(NULL != ps_Handle);

    size_t u32_Size;
    size_t u32_TmpRxRingBufferSize   = ps_Handle->u32_RxRingBufferSize;
    uint16_t u16_TmpRxRingBufferTail = ps_Handle->u16_RxRingBufferTail;
    uint16_t u16_TmpRxRingBufferHead = ps_Handle->u16_RxRingBufferHead;

    if (u16_TmpRxRingBufferTail > u16_TmpRxRingBufferHead)
    {
        u32_Size = ((size_t)u16_TmpRxRingBufferHead + u32_TmpRxRingBufferSize - (size_t)u16_TmpRxRingBufferTail);
    }
    else
    {
        u32_Size = ((size_t)u16_TmpRxRingBufferHead - (size_t)u16_TmpRxRingBufferTail);
    }

    return u32_Size;
}

static bool LLD_UART_TransferIsRxRingBufferFull(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle)
{
    assert(NULL != ps_Handle);

    bool b_Full;

    if (LLD_UART_TransferGetRxRingBufferLength(ps_Base, ps_Handle) == (ps_Handle->u32_RxRingBufferSize - 1U))
    {
        b_Full = true;
    }
    else
    {
        b_Full = false;
    }
    return b_Full;
}

static void LLD_UART_WriteNonBlocking(LPUART_Type *ps_Base, const uint8_t *pcu8_Data, size_t u32_Length)
{
    assert(NULL != pcu8_Data);

    size_t u32_i;

    /* The Non Blocking write data API assume user have ensured there is enough space in
    peripheral to write. */
    for (u32_i = 0; u32_i < u32_Length; u32_i++)
    {
        ps_Base->DATA = pcu8_Data[u32_i];
    }
}

static void LLD_UART_ReadNonBlocking(LPUART_Type *ps_Base, uint8_t *pu8_Data, size_t u32_Length)
{
    assert(NULL != pu8_Data);

    size_t u32_i;
    uint32_t u32_Ctrl        = ps_Base->CTRL;
    bool b_IsSevenDataBits = (((u32_Ctrl & LPUART_CTRL_M7_MASK) != 0U) ||
                            (((u32_Ctrl & LPUART_CTRL_M_MASK) == 0U) && ((u32_Ctrl & LPUART_CTRL_PE_MASK) != 0U)));

    /* The Non Blocking read data API assume user have ensured there is enough space in
    peripheral to write. */
    for (u32_i = 0; u32_i < u32_Length; u32_i++)
    {
        if (b_IsSevenDataBits)
        {
            pu8_Data[u32_i] = (uint8_t)(ps_Base->DATA & 0x7FU);
        }
        else
        {
            pu8_Data[u32_i] = (uint8_t)ps_Base->DATA;
        }
    }
}

/*!
 * brief Initializes an LPUART instance with the user configuration structure and the peripheral clock.
 *
 * param ps_Base LPUART peripheral base address.
 * param pcs_Config Pointer to a user-defined configuration structure.
 * param u32_SrcClock_Hz LPUART clock source frequency in HZ.
 * retval kStatus_LPUART_BaudrateNotSupport Baudrate is not support in current clock source.
 * retval LLD_STATUS_Success LPUART initialize succeed
 */
status_t LLD_UART_Initialization(LPUART_Type *ps_Base, const lld_uart_config_t *pcs_Config, uint32_t u32_SrcClock_Hz)
{
    assert(NULL != pcs_Config);
    assert(0U < pcs_Config->u32_BaudRate_Bps);

    assert((uint8_t)FSL_FEATURE_LPUART_FIFO_SIZEn(ps_Base) > 0);
    assert((uint8_t)FSL_FEATURE_LPUART_FIFO_SIZEn(ps_Base) > 0);

    status_t u32_Status = kStatus_Success;
    uint32_t u32_Temp;
    uint16_t u16_Sbr, u16_SbrTemp;
    uint8_t u8_Osr, u8_OsrTemp;
    uint32_t u32_TempDiff, u32_CalculatedBaud, u32_BaudDiff;

    /* This LPUART instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, OSR is typically hard-set to 16 in other LPUART instantiations
     * loop to find the best OSR value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of OSR */

    u32_BaudDiff = pcs_Config->u32_BaudRate_Bps;
    u8_Osr      = 0U;
    u16_Sbr      = 0U;
    for (u8_OsrTemp = 4U; u8_OsrTemp <= 32U; u8_OsrTemp++)
    {
        /* calculate the temporary sbr value   */
        u16_SbrTemp = (uint16_t)((u32_SrcClock_Hz * 10U / (pcs_Config->u32_BaudRate_Bps * (uint32_t)u8_OsrTemp) + 5U) / 10U);
        /*set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate*/
        if (u16_SbrTemp == 0U)
        {
            u16_SbrTemp = 1U;
        }
        /* Calculate the baud rate based on the temporary OSR and SBR values */
        u32_CalculatedBaud = (u32_SrcClock_Hz / ((uint32_t)u8_OsrTemp * (uint32_t)u16_SbrTemp));
        u32_TempDiff       = u32_CalculatedBaud > pcs_Config->u32_BaudRate_Bps ? (u32_CalculatedBaud - pcs_Config->u32_BaudRate_Bps) :
                                                           (pcs_Config->u32_BaudRate_Bps - u32_CalculatedBaud);

        if (u32_TempDiff <= u32_BaudDiff)
        {
            u32_BaudDiff = u32_TempDiff;
            u8_Osr      = u8_OsrTemp; /* update and store the best OSR value calculated */
            u16_Sbr      = u16_SbrTemp; /* update store the best SBR value calculated */
        }
    }

    /* Check to see if actual baud rate is within 3% of desired baud rate
     * based on the best calculate OSR value */
    if (u32_BaudDiff > ((pcs_Config->u32_BaudRate_Bps / 100U) * 3U))
    {
        /* Unacceptable baud rate difference of more than 3%*/
        u32_Status = kStatus_LPUART_BaudrateNotSupport;
    }
    else
    {
        /* Enable lpuart clock */
        if(ps_Base == LPUART1)
		{
        	CLOCK_EnableClock(kCLOCK_Lpuart1);
		}
		else if(ps_Base == LPUART2)
		{
			CLOCK_EnableClock(kCLOCK_Lpuart2);
		}
		else if(ps_Base == LPUART3)
		{
			CLOCK_EnableClock(kCLOCK_Lpuart3);
		}
		else if(ps_Base == LPUART4)
		{
			CLOCK_EnableClock(kCLOCK_Lpuart4);
		}
		else if(ps_Base == LPUART5)
		{
			CLOCK_EnableClock(kCLOCK_Lpuart5);
		}
		else if(ps_Base == LPUART6)
		{
			CLOCK_EnableClock(kCLOCK_Lpuart6);
		}
		else if(ps_Base == LPUART7)
		{
			CLOCK_EnableClock(kCLOCK_Lpuart7);
		}
		else /* if(ps_Base == LPUART8) */
		{
			CLOCK_EnableClock(kCLOCK_Lpuart8);
		}

        /*Reset all internal logic and registers, except the Global Register */
        LLD_UART_SoftwareReset(ps_Base);

        u32_Temp = ps_Base->BAUD;

        /* Acceptable baud rate, check if OSR is between 4x and 7x oversampling.
         * If so, then "BOTHEDGE" sampling must be turned on */
        if ((u8_Osr > 3U) && (u8_Osr < 8U))
        {
            u32_Temp |= LPUART_BAUD_BOTHEDGE_MASK;
        }

        /* program the osr value (bit value is one less than actual value) */
        u32_Temp &= ~LPUART_BAUD_OSR_MASK;
        u32_Temp |= LPUART_BAUD_OSR((uint32_t)u8_Osr - 1UL);

        /* write the sbr value to the BAUD registers */
        u32_Temp &= ~LPUART_BAUD_SBR_MASK;
        ps_Base->BAUD = u32_Temp | LPUART_BAUD_SBR(u16_Sbr);

        /* Set bit count and parity mode. */
        ps_Base->BAUD &= ~LPUART_BAUD_M10_MASK;

        u32_Temp = ps_Base->CTRL & ~(LPUART_CTRL_PE_MASK | LPUART_CTRL_PT_MASK | LPUART_CTRL_M_MASK | LPUART_CTRL_ILT_MASK |
                              LPUART_CTRL_IDLECFG_MASK);

        u32_Temp |= (uint8_t)kLPUART_ParityDisabled | LPUART_CTRL_IDLECFG(kLPUART_IdleCharacter1) |
                LPUART_CTRL_ILT(kLPUART_IdleTypeStartBit);

        ps_Base->CTRL = u32_Temp;

        /* set stop bit per char */
        u32_Temp       = ps_Base->BAUD & ~LPUART_BAUD_SBNS_MASK;
        ps_Base->BAUD = u32_Temp | LPUART_BAUD_SBNS((uint8_t)kLPUART_OneStopBit);


        ps_Base->WATER = 0;

        /* Enable tx/rx FIFO */
        ps_Base->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);

        /* Flush FIFO */
        ps_Base->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);

        /* Clear all status flags */
        u32_Temp = (LPUART_STAT_RXEDGIF_MASK | LPUART_STAT_IDLE_MASK | LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK |
                LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK);

        u32_Temp |= LPUART_STAT_LBKDIF_MASK;

        u32_Temp |= (LPUART_STAT_MA1F_MASK | LPUART_STAT_MA2F_MASK);

        /* Set the CTS configuration/TX CTS source. */
        ps_Base->MODIR |= LPUART_MODIR_TXCTSC(kLPUART_CtsSampleAtStart) | LPUART_MODIR_TXCTSSRC(kLPUART_CtsSourcePin);

        /* Set data bits order : LSB */
        u32_Temp &= ~LPUART_STAT_MSBF_MASK;

        ps_Base->STAT |= u32_Temp;

        /* Enable TX/RX base on configure structure. */
        u32_Temp = ps_Base->CTRL;
        u32_Temp |= LPUART_CTRL_TE_MASK;
        u32_Temp |= LPUART_CTRL_RE_MASK;
        ps_Base->CTRL = u32_Temp;
    }

    return u32_Status;
}

/*!
 * brief Gets enabled LPUART interrupts.
 *
 * This function gets the enabled LPUART interrupts. The enabled interrupts are returned
 * as the logical OR value of the enumerators ref _lld_uart_interrupt_enable. To check
 * a specific interrupt enable status, compare the return value with enumerators
 * in ref _lld_uart_interrupt_enable.
 * For example, to check whether the TX empty interrupt is enabled:
 * code
 *     uint32_t enabledInterrupts = LLD_UART_GetEnabledInterrupts(LPUART1);
 *
 *     if (kLPUART_TxDataRegEmptyInterruptEnable & enabledInterrupts)
 *     {
 *         ...
 *     }
 * endcode
 *
 * param ps_Base LPUART peripheral base address.
 * return LPUART interrupt flags which are logical OR of the enumerators in ref _lld_uart_interrupt_enable.
 */
uint32_t LLD_UART_GetEnabledInterrupts(LPUART_Type *ps_Base)
{
    /* Check int enable bits in ps_Base->CTRL */
    uint32_t u32_Temp = (uint32_t)(ps_Base->CTRL & (uint32_t)kLPUART_AllInterruptEnable);

    /* Check int enable bits in ps_Base->BAUD */
    u32_Temp = (u32_Temp & ~(uint32_t)kLPUART_RxActiveEdgeInterruptEnable) | ((ps_Base->BAUD & LPUART_BAUD_RXEDGIE_MASK) >> 8U);
    u32_Temp = (u32_Temp & ~(uint32_t)kLPUART_LinBreakInterruptEnable) | ((ps_Base->BAUD & LPUART_BAUD_LBKDIE_MASK) >> 8U);


    /* Check int enable bits in ps_Base->FIFO */
    u32_Temp =
        (u32_Temp & ~((uint32_t)kLPUART_TxFifoOverflowInterruptEnable | (uint32_t)kLPUART_RxFifoUnderflowInterruptEnable)) |
        (ps_Base->FIFO & (LPUART_FIFO_TXOFE_MASK | LPUART_FIFO_RXUFE_MASK));

    return u32_Temp;
}

/*!
 * brief Gets LPUART status flags.
 *
 * This function gets all LPUART status flags. The flags are returned as the logical
 * OR value of the enumerators ref _lld_uart_flags. To check for a specific status,
 * compare the return value with enumerators in the ref _lld_uart_flags.
 * For example, to check whether the TX is empty:
 * code
 *     if (kLPUART_TxDataRegEmptyFlag & LLD_UART_GetStatusFlags(LPUART1))
 *     {
 *         ...
 *     }
 * endcode
 *
 * param ps_Base LPUART peripheral base address.
 * return LPUART status flags which are ORed by the enumerators in the _lld_uart_flags.
 */
uint32_t LLD_UART_GetStatusFlags(LPUART_Type *ps_Base)
{
    uint32_t u32_Temp;
    u32_Temp = ps_Base->STAT;
    u32_Temp |= (ps_Base->FIFO &
             (LPUART_FIFO_TXEMPT_MASK | LPUART_FIFO_RXEMPT_MASK | LPUART_FIFO_TXOF_MASK | LPUART_FIFO_RXUF_MASK)) >>
            16U;
    /* Only keeps the status bits */
    u32_Temp &= (uint32_t)kLPUART_AllFlags;
    return u32_Temp;
}

/*!
 * brief Initializes the LPUART handle.
 *
 * This function initializes the LPUART handle, which can be used for other LPUART
 * transactional APIs. Usually, for a specified LPUART instance,
 * call this API once to get the initialized handle.
 *
 * The LPUART driver supports the "background" receiving, which means that user can set up
 * an RX ring buffer optionally. Data received is stored into the ring buffer even when the
 * user doesn't call the LLD_UART_TransferReceiveNonBlocking() API. If there is already data received
 * in the ring buffer, the user can get the received data from the ring buffer directly.
 * The ring buffer is disabled if passing NULL as p ringBuffer.
 *
 * param ps_Base LPUART peripheral base address.
 * param ps_Handle LPUART handle pointer.
 * param pf_Callback Callback function.
 * param p_UserData User data.
 */
void LLD_UART_TransferCreateHandle(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle, lld_uart_transfer_callback_t pf_Callback,
                                 void *p_UserData)
{
    assert(NULL != ps_Handle);

    uint32_t u32_Ctrl        = ps_Base->CTRL;
    bool b_IsSevenDataBits = (((u32_Ctrl & LPUART_CTRL_M7_MASK) != 0U) ||
                            (((u32_Ctrl & LPUART_CTRL_M_MASK) == 0U) && ((u32_Ctrl & LPUART_CTRL_PE_MASK) != 0U)));

    /* Zero the handle. */
    (void)memset(ps_Handle, 0, sizeof(lld_uart_handle_t));

    /* Set the TX/RX state. */
    ps_Handle->u8_RxState = (uint8_t)kLPUART_RxIdle;
    ps_Handle->u8_TxState = (uint8_t)kLPUART_TxIdle;

    /* Set the callback and user data. */
    ps_Handle->pf_Callback = pf_Callback;
    ps_Handle->p_UserData = p_UserData;

    /* Initial seven data bits flag */
    ps_Handle->b_IsSevenDataBits = b_IsSevenDataBits;

    /* Save the handle in global variables to support the double weak mechanism. */
    if(ps_Base == LPUART1)
	{
    	s_lpuartHandle[1] = ps_Handle;
	}
	else if(ps_Base == LPUART2)
	{
		s_lpuartHandle[2] = ps_Handle;
	}
	else if(ps_Base == LPUART3)
	{
		s_lpuartHandle[3] = ps_Handle;
	}
	else if(ps_Base == LPUART4)
	{
		s_lpuartHandle[4] = ps_Handle;
	}
	else if(ps_Base == LPUART5)
	{
		s_lpuartHandle[5] = ps_Handle;
	}
	else if(ps_Base == LPUART6)
	{
		s_lpuartHandle[6] = ps_Handle;
	}
	else if(ps_Base == LPUART7)
	{
		s_lpuartHandle[7] = ps_Handle;
	}
	else /* if(ps_Base == LPUART8) */
	{
		s_lpuartHandle[8] = ps_Handle;
	}

    s_lpuartIsr = LLD_UART_TransferHandleIRQ;

/* Enable interrupt in NVIC. */
    if(ps_Base == LPUART1)
	{
    	EnableIRQ(LPUART1_IRQn);
	}
	else if(ps_Base == LPUART2)
	{
		EnableIRQ(LPUART2_IRQn);
	}
	else if(ps_Base == LPUART3)
	{
		EnableIRQ(LPUART3_IRQn);
	}
	else if(ps_Base == LPUART4)
	{
		EnableIRQ(LPUART4_IRQn);
	}
	else if(ps_Base == LPUART5)
	{
		EnableIRQ(LPUART5_IRQn);
	}
	else if(ps_Base == LPUART6)
	{
		EnableIRQ(LPUART6_IRQn);
	}
	else if(ps_Base == LPUART7)
	{
		EnableIRQ(LPUART7_IRQn);
	}
	else /* if(ps_Base == LPUART8) */
	{
		EnableIRQ(LPUART8_IRQn);
	}
}

/*!
 * brief Transmits a buffer of data using the interrupt method.
 *
 * This function send data using an interrupt method. This is a non-blocking function, which
 * returns directly without waiting for all data written to the transmitter register. When
 * all data is written to the TX register in the ISR, the LPUART driver calls the callback
 * function and passes the ref kStatus_LPUART_TxIdle as status parameter.
 *
 * note The kStatus_LPUART_TxIdle is passed to the upper layer when all data are written
 * to the TX register. However, there is no check to ensure that all the data sent out. Before disabling the TX,
 * check the kLPUART_TransmissionCompleteFlag to ensure that the transmit is finished.
 *
 * param ps_Base LPUART peripheral base address.
 * param ps_Handle LPUART handle pointer.
 * param ps_Xfer LPUART transfer structure, see #lld_uart_transfer_t.
 * retval LLD_STATUS_Success Successfully start the data transmission.
 * retval kStatus_LPUART_TxBusy Previous transmission still not finished, data not all written to the TX register.
 * retval kStatus_InvalidArgument Invalid argument.
 */
status_t LLD_UART_TransferSendNonBlocking(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle, lld_uart_transfer_t *ps_Xfer)
{
    assert(NULL != ps_Handle);
    assert(NULL != ps_Xfer);
    assert(NULL != ps_Xfer->pcu8_TxData);
    assert(0U != ps_Xfer->u32_DataSize);

    status_t u32_Status;

    /* Return error if current TX busy. */
    if ((uint8_t)kLPUART_TxBusy == ps_Handle->u8_TxState)
    {
        u32_Status = kStatus_LPUART_TxBusy;
    }
    else
    {
        ps_Handle->pcu8_TxData        = ps_Xfer->pcu8_TxData;
        ps_Handle->u32_TxDataSize    = ps_Xfer->u32_DataSize;
        ps_Handle->u32_TxDataSizeAll = ps_Xfer->u32_DataSize;
        ps_Handle->u8_TxState       = (uint8_t)kLPUART_TxBusy;

        /* Disable and re-enable the global interrupt to protect the interrupt enable register during read-modify-wrte.
         */
        uint32_t u32_IrqMask = DisableGlobalIRQ();
        /* Enable transmitter interrupt. */
        ps_Base->CTRL |= (uint32_t)LPUART_CTRL_TIE_MASK;
        EnableGlobalIRQ(u32_IrqMask);

        u32_Status = kStatus_Success;
    }

    return u32_Status;
}

/*!
 * brief Receives a buffer of data using the interrupt method.
 *
 * This function receives data using an interrupt method. This is a non-blocking function
 * which returns without waiting to ensure that all data are received.
 * If the RX ring buffer is used and not empty, the data in the ring buffer is copied and
 * the parameter p pu32_ReceivedBytes shows how many bytes are copied from the ring buffer.
 * After copying, if the data in the ring buffer is not enough for read, the receive
 * request is saved by the LPUART driver. When the new data arrives, the receive request
 * is serviced first. When all data is received, the LPUART driver notifies the upper layer
 * through a callback function and passes a status parameter ref kStatus_UART_RxIdle.
 * For example, the upper layer needs 10 bytes but there are only 5 bytes in ring buffer.
 * The 5 bytes are copied to ps_Xfer->data, which returns with the
 * parameter p pu32_ReceivedBytes set to 5. For the remaining 5 bytes, the newly arrived data is
 * saved from ps_Xfer->data[5]. When 5 bytes are received, the LPUART driver notifies the upper layer.
 * If the RX ring buffer is not enabled, this function enables the RX and RX interrupt
 * to receive data to ps_Xfer->data. When all data is received, the upper layer is notified.
 *
 * param ps_Base LPUART peripheral base address.
 * param ps_Handle LPUART handle pointer.
 * param ps_Xfer LPUART transfer structure, see #uart_transfer_t.
 * param pu32_ReceivedBytes Bytes received from the ring buffer directly.
 * retval LLD_STATUS_Success Successfully queue the transfer into the transmit queue.
 * retval kStatus_LPUART_RxBusy Previous receive request is not finished.
 * retval kStatus_InvalidArgument Invalid argument.
 */
status_t LLD_UART_TransferReceiveNonBlocking(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle, lld_uart_transfer_t *ps_Xfer,
                                           size_t *pu32_ReceivedBytes)
{
    assert(NULL != ps_Handle);
    assert(NULL != ps_Xfer);
    assert(NULL != ps_Xfer->pu8_RxData);
    assert(0U != ps_Xfer->u32_DataSize);

    uint32_t u32_i;
    status_t u32_Status;
    uint32_t u32_IrqMask;
    /* How many bytes to copy from ring buffer to user memory. */
    size_t u32_BytesToCopy = 0U;
    /* How many bytes to receive. */
    size_t u32_BytesToReceive;
    /* How many bytes currently have received. */
    size_t u32_BytesCurrentReceived;

    /* How to get data:
       1. If RX ring buffer is not enabled, then save ps_Xfer->data and ps_Xfer->u32_DataSize
          to lpuart handle, enable interrupt to store received data to ps_Xfer->data. When
          all data received, trigger callback.
       2. If RX ring buffer is enabled and not empty, get data from ring buffer first.
          If there are enough data in ring buffer, copy them to ps_Xfer->data and return.
          If there are not enough data in ring buffer, copy all of them to ps_Xfer->data,
          save the ps_Xfer->data remained empty space to lpuart handle, receive data
          to this empty space and trigger callback when finished. */

    if ((uint8_t)kLPUART_RxBusy == ps_Handle->u8_RxState)
    {
        u32_Status = kStatus_LPUART_RxBusy;
    }
    else
    {
        u32_BytesToReceive       = ps_Xfer->u32_DataSize;
        u32_BytesCurrentReceived = 0;

        /* If RX ring buffer is used. */
        if (NULL != ps_Handle->pu8_RxRingBuffer)
        {
            /* Disable and re-enable the global interrupt to protect the interrupt enable register during
             * read-modify-wrte. */
            u32_IrqMask = DisableGlobalIRQ();
            /* Disable LPUART RX IRQ, protect ring buffer. */
            ps_Base->CTRL &= ~(uint32_t)(LPUART_CTRL_RIE_MASK | LPUART_CTRL_ORIE_MASK);
            EnableGlobalIRQ(u32_IrqMask);

            /* How many bytes in RX ring buffer currently. */
            u32_BytesToCopy = LLD_UART_TransferGetRxRingBufferLength(ps_Base, ps_Handle);

            if (0U != u32_BytesToCopy)
            {
                u32_BytesToCopy = MIN(u32_BytesToReceive, u32_BytesToCopy);

                u32_BytesToReceive -= u32_BytesToCopy;

                /* Copy data from ring buffer to user memory. */
                for (u32_i = 0U; u32_i < u32_BytesToCopy; u32_i++)
                {
                    ps_Xfer->pu8_RxData[u32_BytesCurrentReceived] = ps_Handle->pu8_RxRingBuffer[ps_Handle->u16_RxRingBufferTail];
                    u32_BytesCurrentReceived++;

                    /* Wrap to 0. Not use modulo (%) because it might be large and slow. */
                    if (((uint32_t)ps_Handle->u16_RxRingBufferTail + 1U) == ps_Handle->u32_RxRingBufferSize)
                    {
                        ps_Handle->u16_RxRingBufferTail = 0U;
                    }
                    else
                    {
                        ps_Handle->u16_RxRingBufferTail++;
                    }
                }
            }

            /* If ring buffer does not have enough data, still need to read more data. */
            if (0U != u32_BytesToReceive)
            {
                /* No data in ring buffer, save the request to LPUART handle. */
                ps_Handle->pu8_RxData        = &ps_Xfer->pu8_RxData[u32_BytesCurrentReceived];
                ps_Handle->u32_RxDataSize    = u32_BytesToReceive;
                ps_Handle->u32_RxDataSizeAll = ps_Xfer->u32_DataSize;
                ps_Handle->u8_RxState       = (uint8_t)kLPUART_RxBusy;
            }

            /* Disable and re-enable the global interrupt to protect the interrupt enable register during
             * read-modify-wrte. */
            u32_IrqMask = DisableGlobalIRQ();
            /* Re-enable LPUART RX IRQ. */
            ps_Base->CTRL |= (uint32_t)(LPUART_CTRL_RIE_MASK | LPUART_CTRL_ORIE_MASK);
            EnableGlobalIRQ(u32_IrqMask);

            /* Call user callback since all data are received. */
            if (0U == u32_BytesToReceive)
            {
                if (NULL != ps_Handle->pf_Callback)
                {
                    ps_Handle->pf_Callback(ps_Base, ps_Handle, kStatus_LPUART_RxIdle, ps_Handle->p_UserData);
                }
            }
        }
        /* Ring buffer not used. */
        else
        {
            ps_Handle->pu8_RxData        = &ps_Xfer->pu8_RxData[u32_BytesCurrentReceived];
            ps_Handle->u32_RxDataSize    = u32_BytesToReceive;
            ps_Handle->u32_RxDataSizeAll = u32_BytesToReceive;
            ps_Handle->u8_RxState       = (uint8_t)kLPUART_RxBusy;

            /* Disable and re-enable the global interrupt to protect the interrupt enable register during
             * read-modify-wrte. */
            u32_IrqMask = DisableGlobalIRQ();
            /* Enable RX interrupt. */
            ps_Base->CTRL |= (uint32_t)(LPUART_CTRL_RIE_MASK | LPUART_CTRL_ILIE_MASK | LPUART_CTRL_ORIE_MASK);
            EnableGlobalIRQ(u32_IrqMask);
        }

        /* Return the how many bytes have read. */
        if (NULL != pu32_ReceivedBytes)
        {
            *pu32_ReceivedBytes = u32_BytesCurrentReceived;
        }

        u32_Status = kStatus_Success;
    }

    return u32_Status;
}

/*!
 * brief LPUART IRQ handle function.
 *
 * This function handles the LPUART transmit and receive IRQ request.
 *
 * param ps_Base LPUART peripheral base address.
 * param p_IrqHandle LPUART handle pointer.
 */
void LLD_UART_TransferHandleIRQ(LPUART_Type *ps_Base, void *p_IrqHandle)
{
    assert(NULL != p_IrqHandle);

    uint8_t u8_Count;
    uint8_t u8_TempCount;
    uint32_t u32_Status            = LLD_UART_GetStatusFlags(ps_Base);
    uint32_t u32_EnabledInterrupts = LLD_UART_GetEnabledInterrupts(ps_Base);
    uint16_t u16_TpmRxRingBufferHead;
    uint32_t u32_TpmData;
    uint32_t u32_IrqMask;
    lld_uart_handle_t *ps_Handle = (lld_uart_handle_t *)p_IrqHandle;

    /* If RX overrun. */
    if ((uint32_t)kLPUART_RxOverrunFlag == ((uint32_t)kLPUART_RxOverrunFlag & u32_Status))
    {
        /* Clear overrun flag, otherwise the RX does not work. */
        ps_Base->STAT = ((ps_Base->STAT & 0x3FE00000U) | LPUART_STAT_OR_MASK);

        /* Trigger callback. */
        if (NULL != (ps_Handle->pf_Callback))
        {
            ps_Handle->pf_Callback(ps_Base, ps_Handle, kStatus_LPUART_RxHardwareOverrun, ps_Handle->p_UserData);
        }
    }

    /* If IDLE flag is set and the IDLE interrupt is enabled. */
    if ((0U != ((uint32_t)kLPUART_IdleLineFlag & u32_Status)) &&
        (0U != ((uint32_t)kLPUART_IdleLineInterruptEnable & u32_EnabledInterrupts)))
    {
        u8_Count = ((uint8_t)((ps_Base->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT));

        while ((0U != ps_Handle->u32_RxDataSize) && (0U != u8_Count))
        {
            u8_TempCount = (uint8_t)MIN(ps_Handle->u32_RxDataSize, u8_Count);

            /* Using non block API to read the data from the registers. */
            LLD_UART_ReadNonBlocking(ps_Base, ps_Handle->pu8_RxData, u8_TempCount);
            ps_Handle->pu8_RxData = &ps_Handle->pu8_RxData[u8_TempCount];
            ps_Handle->u32_RxDataSize -= u8_TempCount;
            u8_Count -= u8_TempCount;

            /* If u32_RxDataSize is 0, invoke rx idle callback.*/
            if (0U == (ps_Handle->u32_RxDataSize))
            {
                ps_Handle->u8_RxState = (uint8_t)kLPUART_RxIdle;

                if (NULL != ps_Handle->pf_Callback)
                {
                    ps_Handle->pf_Callback(ps_Base, ps_Handle, kStatus_LPUART_RxIdle, ps_Handle->p_UserData);
                }
            }
        }
        /* Clear IDLE flag.*/
        ps_Base->STAT = ((ps_Base->STAT & 0x3FE00000U) | LPUART_STAT_IDLE_MASK);

        /* If u32_RxDataSize is 0, disable rx ready, overrun and idle line interrupt.*/
        if (0U == ps_Handle->u32_RxDataSize)
        {
            /* Disable and re-enable the global interrupt to protect the interrupt enable register during
             * read-modify-wrte. */
            u32_IrqMask = DisableGlobalIRQ();
            ps_Base->CTRL &= ~(uint32_t)(LPUART_CTRL_RIE_MASK | LPUART_CTRL_ILIE_MASK | LPUART_CTRL_ORIE_MASK);
            EnableGlobalIRQ(u32_IrqMask);
        }
        /* Invoke callback if callback is not NULL and u32_RxDataSize is not 0. */
        else if (NULL != ps_Handle->pf_Callback)
        {
            ps_Handle->pf_Callback(ps_Base, ps_Handle, kStatus_LPUART_IdleLineDetected, ps_Handle->p_UserData);
        }
        else
        {
            /* Avoid MISRA 15.7 */
        }
    }
    /* Receive data register full */
    if ((0U != ((uint32_t)kLPUART_RxDataRegFullFlag & u32_Status)) &&
        (0U != ((uint32_t)kLPUART_RxDataRegFullInterruptEnable & u32_EnabledInterrupts)))
    {
        /* Get the size that can be stored into buffer for this interrupt. */
        u8_Count = ((uint8_t)((ps_Base->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT));

        /* If ps_Handle->u32_RxDataSize is not 0, first save data to ps_Handle->rxData. */
        while ((0U != ps_Handle->u32_RxDataSize) && (0U != u8_Count))
        {
            u8_TempCount = (uint8_t)MIN(ps_Handle->u32_RxDataSize, u8_Count);

            /* Using non block API to read the data from the registers. */
            LLD_UART_ReadNonBlocking(ps_Base, ps_Handle->pu8_RxData, u8_TempCount);
            ps_Handle->pu8_RxData = &ps_Handle->pu8_RxData[u8_TempCount];
            ps_Handle->u32_RxDataSize -= u8_TempCount;
            u8_Count -= u8_TempCount;

            /* If all the data required for upper layer is ready, trigger callback. */
            if (0U == ps_Handle->u32_RxDataSize)
            {
                ps_Handle->u8_RxState = (uint8_t)kLPUART_RxIdle;

                if (NULL != ps_Handle->pf_Callback)
                {
                    ps_Handle->pf_Callback(ps_Base, ps_Handle, kStatus_LPUART_RxIdle, ps_Handle->p_UserData);
                }
            }
        }

        /* If use RX ring buffer, receive data to ring buffer. */
        if (NULL != ps_Handle->pu8_RxRingBuffer)
        {
            while (0U != u8_Count--)
            {
                /* If RX ring buffer is full, trigger callback to notify over run. */
                if (LLD_UART_TransferIsRxRingBufferFull(ps_Base, ps_Handle))
                {
                    if (NULL != ps_Handle->pf_Callback)
                    {
                        ps_Handle->pf_Callback(ps_Base, ps_Handle, kStatus_LPUART_RxRingBufferOverrun, ps_Handle->p_UserData);
                    }
                }

                /* If ring buffer is still full after callback function, the oldest data is overridden. */
                if (LLD_UART_TransferIsRxRingBufferFull(ps_Base, ps_Handle))
                {
                    /* Increase ps_Handle->u16_RxRingBufferTail to make room for new data. */
                    if (((uint32_t)ps_Handle->u16_RxRingBufferTail + 1U) == ps_Handle->u32_RxRingBufferSize)
                    {
                        ps_Handle->u16_RxRingBufferTail = 0U;
                    }
                    else
                    {
                        ps_Handle->u16_RxRingBufferTail++;
                    }
                }

                /* Read data. */
                u16_TpmRxRingBufferHead = ps_Handle->u16_RxRingBufferHead;
                u32_TpmData             = ps_Base->DATA;
                if (ps_Handle->b_IsSevenDataBits)
                {
                    ps_Handle->pu8_RxRingBuffer[u16_TpmRxRingBufferHead] = (uint8_t)(u32_TpmData & 0x7FU);
                }
                else
                {
                    ps_Handle->pu8_RxRingBuffer[u16_TpmRxRingBufferHead] = (uint8_t)u32_TpmData;
                }

                /* Increase ps_Handle->u16_RxRingBufferHead. */
                if (((uint32_t)ps_Handle->u16_RxRingBufferHead + 1U) == ps_Handle->u32_RxRingBufferSize)
                {
                    ps_Handle->u16_RxRingBufferHead = 0U;
                }
                else
                {
                    ps_Handle->u16_RxRingBufferHead++;
                }
            }
        }
        /* If no receive requst pending, stop RX interrupt. */
        else if (0U == ps_Handle->u32_RxDataSize)
        {
            /* Disable and re-enable the global interrupt to protect the interrupt enable register during
             * read-modify-wrte. */
            u32_IrqMask = DisableGlobalIRQ();
            ps_Base->CTRL &= ~(uint32_t)(LPUART_CTRL_RIE_MASK | LPUART_CTRL_ORIE_MASK | LPUART_CTRL_ILIE_MASK);
            EnableGlobalIRQ(u32_IrqMask);
        }
        else
        {
        }
    }

    /* Send data register empty and the interrupt is enabled. */
    if ((0U != ((uint32_t)kLPUART_TxDataRegEmptyFlag & u32_Status)) &&
        (0U != ((uint32_t)kLPUART_TxDataRegEmptyInterruptEnable & u32_EnabledInterrupts)))
    {
/* Get the bytes that available at this moment. */
        u8_Count = (uint8_t)FSL_FEATURE_LPUART_FIFO_SIZEn(ps_Base) -
                (uint8_t)((ps_Base->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXCOUNT_SHIFT);

        while ((0U != ps_Handle->u32_TxDataSize) && (0U != u8_Count))
        {
            u8_TempCount = (uint8_t)MIN(ps_Handle->u32_TxDataSize, u8_Count);

            /* Using non block API to write the data to the registers. */
            LLD_UART_WriteNonBlocking(ps_Base, ps_Handle->pcu8_TxData, u8_TempCount);
            ps_Handle->pcu8_TxData = &ps_Handle->pcu8_TxData[u8_TempCount];
            ps_Handle->u32_TxDataSize -= u8_TempCount;
            u8_Count -= u8_TempCount;

            /* If all the data are written to data register, notify user with the callback, then TX finished. */
            if (0U == ps_Handle->u32_TxDataSize)
            {
                /* Disable and re-enable the global interrupt to protect the interrupt enable register during
                 * read-modify-wrte. */
                u32_IrqMask = DisableGlobalIRQ();
                /* Disable TX register empty interrupt and enable transmission completion interrupt. */
                ps_Base->CTRL = (ps_Base->CTRL & ~LPUART_CTRL_TIE_MASK) | LPUART_CTRL_TCIE_MASK;
                EnableGlobalIRQ(u32_IrqMask);
            }
        }
    }

    /* Transmission complete and the interrupt is enabled. */
    if ((0U != ((uint32_t)kLPUART_TransmissionCompleteFlag & u32_Status)) &&
        (0U != ((uint32_t)kLPUART_TransmissionCompleteInterruptEnable & u32_EnabledInterrupts)))
    {
        /* Set u8_TxState to idle only when all data has been sent out to bus. */
        ps_Handle->u8_TxState = (uint8_t)kLPUART_TxIdle;

        /* Disable and re-enable the global interrupt to protect the interrupt enable register during read-modify-wrte.
         */
        u32_IrqMask = DisableGlobalIRQ();
        /* Disable transmission complete interrupt. */
        ps_Base->CTRL &= ~(uint32_t)LPUART_CTRL_TCIE_MASK;
        EnableGlobalIRQ(u32_IrqMask);

        /* Trigger callback. */
        if (NULL != ps_Handle->pf_Callback)
        {
            ps_Handle->pf_Callback(ps_Base, ps_Handle, kStatus_LPUART_TxIdle, ps_Handle->p_UserData);
        }
    }
}

void LPUART1_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART1, s_lpuartHandle[1]);
    SDK_ISR_EXIT_BARRIER;
}

void LPUART2_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART2, s_lpuartHandle[2]);
    SDK_ISR_EXIT_BARRIER;
}

void LPUART3_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART3, s_lpuartHandle[3]);
    SDK_ISR_EXIT_BARRIER;
}

void LPUART4_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART4, s_lpuartHandle[4]);
    SDK_ISR_EXIT_BARRIER;
}

void LPUART5_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART5, s_lpuartHandle[5]);
    SDK_ISR_EXIT_BARRIER;
}

void LPUART6_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART6, s_lpuartHandle[6]);
    SDK_ISR_EXIT_BARRIER;
}

void LPUART7_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART7, s_lpuartHandle[7]);
    SDK_ISR_EXIT_BARRIER;
}

void LPUART8_DriverIRQHandler(void)
{
    s_lpuartIsr(LPUART8, s_lpuartHandle[8]);
    SDK_ISR_EXIT_BARRIER;
}

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/**
* @brief		UART initialization
* @param		e_Uart : UART number
* @param		u32_Baudrate : baudrate to configure
* @param		pf_callback : pointer on callback function
* @return		void
* @details
**/
void LLD_UART_Init(typ_Lld_Uart e_Uart, uint32_t u32_Baudrate, lld_uart_transfer_callback_t pf_Callback)
{
    lld_uart_config_t pcs_Config;

    pcs_Config.u32_BaudRate_Bps = u32_Baudrate;

    if(e_Uart == LLD_UART_UART1)
    {
    	ts_lld_Uart_manager[e_Uart].ps_Base = LPUART1;
    }
    else if(e_Uart == LLD_UART_UART2)
	{
		ts_lld_Uart_manager[e_Uart].ps_Base = LPUART2;
	}
    else if(e_Uart == LLD_UART_UART3)
	{
		ts_lld_Uart_manager[e_Uart].ps_Base = LPUART3;
	}
    else if(e_Uart == LLD_UART_UART4)
	{
		ts_lld_Uart_manager[e_Uart].ps_Base = LPUART4;
	}
    else if(e_Uart == LLD_UART_UART5)
	{
		ts_lld_Uart_manager[e_Uart].ps_Base = LPUART5;
	}
    else if(e_Uart == LLD_UART_UART6)
	{
		ts_lld_Uart_manager[e_Uart].ps_Base = LPUART6;
	}
    else if(e_Uart == LLD_UART_UART7)
	{
		ts_lld_Uart_manager[e_Uart].ps_Base = LPUART7;
	}
    else /* if(e_Uart == LLD_UART_UART8) */
	{
		ts_lld_Uart_manager[e_Uart].ps_Base = LPUART8;
	}

    uint32_t u32_Frequency;
	if (CLOCK_GetMux(kCLOCK_UartMux) == 0) /* PLL3 div6 80M */
	{
		u32_Frequency = (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
	}
	else
	{
		u32_Frequency = CLOCK_GetOscFreq() / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
	}

    LLD_UART_Initialization(ts_lld_Uart_manager[e_Uart].ps_Base, &pcs_Config, u32_Frequency);

    LLD_UART_TransferCreateHandle(ts_lld_Uart_manager[e_Uart].ps_Base, &g_lpuartHandle, pf_Callback, NULL);
}

/**
* @brief		Receive data
* @param		e_Uart : UART number
* @param		pu8_RxBuff : pointer on rx buffer
* @param		u32_RxBuffSize : size of rx buffer
* @return		void
* @details
**/
void LLD_UART_Receive(typ_Lld_Uart e_Uart, uint8_t* pu8_RxBuff, uint32_t u32_RxBuffSize)
{
    lld_uart_transfer_t receiveXfer;
    receiveXfer.pu8_Data     = pu8_RxBuff;
    receiveXfer.u32_DataSize = u32_RxBuffSize;
	LLD_UART_TransferReceiveNonBlocking(ts_lld_Uart_manager[e_Uart].ps_Base, &g_lpuartHandle, &receiveXfer, NULL);
}

/**
* @brief		Send data
* @param		e_Uart : UART number
* @param		pu8_TxBuff : pointer on tx buffer
* @param		u32_TxBuffSize : size of tx buffer
* @return		void
* @details
**/
void LLD_UART_Send(typ_Lld_Uart e_Uart, uint8_t* pu8_TxBuff, uint32_t u32_TxBuffSize)
{
	lld_uart_transfer_t sendXfer;
	sendXfer.pu8_Data        = pu8_TxBuff;
	sendXfer.u32_DataSize    = u32_TxBuffSize;
	LLD_UART_TransferSendNonBlocking(ts_lld_Uart_manager[e_Uart].ps_Base, &g_lpuartHandle, &sendXfer);
}
