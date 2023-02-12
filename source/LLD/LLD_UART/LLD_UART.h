/**
 * @file LLD_UART.h
 * @author ACR
 * @brief Header file for UART peripheral
 * @details
**/

#ifndef LLD_UART_H_
#define LLD_UART_H_

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

typedef enum
{
	LLD_UART_UART1,
	LLD_UART_UART2,
	LLD_UART_UART3,
	LLD_UART_UART4,
	LLD_UART_UART5,
	LLD_UART_UART6,
	LLD_UART_UART7,
	LLD_UART_UART8,
	LLD_UART_NB,
}typ_Lld_Uart;

/* Forward declaration of the handle typedef. */
typedef struct _lld_uart_handle lld_uart_handle_t;

/*! @brief LPUART configuration structure. */
typedef struct _lld_uart_config
{
    uint32_t u32_BaudRate_Bps;            /*!< LPUART baud rate  */
} lld_uart_config_t;

/*! @brief LPUART transfer callback function. */
typedef void (*lld_uart_transfer_callback_t)(LPUART_Type *ps_Base, lld_uart_handle_t *ps_Handle, status_t u32_Status, void *p_UserData);

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/
void LLD_UART_Init(typ_Lld_Uart e_Uart, uint32_t u32_Baudrate, lld_uart_transfer_callback_t pf_Callback);
void LLD_UART_Receive(typ_Lld_Uart e_Uart, uint8_t* pu8_RxBuff, uint32_t u32_RxBuffSize);
void LLD_UART_Send(typ_Lld_Uart e_Uart, uint8_t* pu8_TxBuff, uint32_t u32_TxBuffSize);

#endif /* LLD_UART_H_ */
