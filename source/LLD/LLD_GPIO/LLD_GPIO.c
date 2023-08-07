/**
 * @file LLD_GPIO.c
 * @author ACR
 * @brief Specific GPIO driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_GPIO_USE_LOCALS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "fsl_clock.h"
#include "LLD_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES GPIO ...                                                   */
/*--------------------------------------------------------------------------*/

/*! @brief GPIO direction definition. */
typedef enum
{
    kGPIO_DigitalInput  = 0U, /*!< Set current pin as digital input.*/
    kGPIO_DigitalOutput = 1U, /*!< Set current pin as digital output.*/
} gpio_pin_direction_t;

/*! @brief GPIO interrupt mode definition. */
typedef enum
{
    kGPIO_NoIntmode              = 0U, /*!< Set current pin general IO functionality.*/
    kGPIO_IntLowLevel            = 1U, /*!< Set current pin interrupt is low-level sensitive.*/
    kGPIO_IntHighLevel           = 2U, /*!< Set current pin interrupt is high-level sensitive.*/
    kGPIO_IntRisingEdge          = 3U, /*!< Set current pin interrupt is rising-edge sensitive.*/
    kGPIO_IntFallingEdge         = 4U, /*!< Set current pin interrupt is falling-edge sensitive.*/
    kGPIO_IntRisingOrFallingEdge = 5U, /*!< Enable the edge select bit to override the ICR register's configuration.*/
} gpio_interrupt_mode_t;

/*! @brief GPIO Init structure definition. */
typedef struct
{
    gpio_pin_direction_t e_Direction; 		/*!< Specifies the pin direction. */
    uint8_t outputLogic;            		/*!< Set a default output logic, which has no use in input */
    gpio_interrupt_mode_t e_InterruptMode; 	/*!< Specifies the pin interrupt mode, a value of @ref gpio_interrupt_mode_t. */
} gpio_pin_config_t;

/*--------------------------------------------------------------------------*/
/* ... DATATYPES LLD GPIO ...                                               */
/*--------------------------------------------------------------------------*/

/*! @brief LLD GPIO handle structure definition. */
typedef struct
{
    lld_gpio_transfer_callback_t pf_Callback; /*!< Callback function. */
}lld_gpio_handle_t;

typedef struct
{
	GPIO_Type * ps_Port;		/* Port : GPIO1 to GPIO5 */
	uint32_t u32_Pin;			/* Pin : 0 to 31 */
	lld_gpio_handle_t s_ldd_gpio_handle;
}lld_gpio_manager_t;

lld_gpio_manager_t ts_lld_gpio_manager[LLD_NB_GPIO];

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

static inline void LLD_GPIO_PortToggle(GPIO_Type *ps_Base, uint32_t u32_Mask);
static inline uint32_t LLD_GPIO_PinRead(GPIO_Type *ps_Base, uint32_t u32_Pin);
static inline void LLD_GPIO_SetPinInterruptConfig(GPIO_Type *ps_Base, uint32_t u32_Pin, gpio_interrupt_mode_t e_PinInterruptMode);
static inline void LLD_GPIO_PortEnableInterrupts(GPIO_Type *ps_Base, uint32_t u32_Mask);
static inline void LLD_GPIO_PortDisableInterrupts(GPIO_Type *ps_Base, uint32_t u32_Mask);
static inline uint32_t LLD_GPIO_GetPinsInterruptFlags(GPIO_Type *ps_Base);
static inline void LLD_GPIO_PortClearInterruptFlags(GPIO_Type *ps_Base, uint32_t u32_Mask);
void LLD_GPIO_PinInit(GPIO_Type *ps_Base, uint32_t u32_Pin, const gpio_pin_config_t *cs_Config);
void LLD_GPIO_PinWrite(GPIO_Type *ps_Base, uint32_t u32_Pin, uint8_t u8_Output);

void LLD_GPIO_HandleIRQ(GPIO_Type * ps_Base, uint32_t u32_Pin);
void GPIO1_Combined_0_15_IRQHandler(void);
void GPIO1_Combined_16_31_IRQHandler(void);
void GPIO2_Combined_0_15_IRQHandler(void);
void GPIO2_Combined_16_31_IRQHandler(void);
void GPIO3_Combined_0_15_IRQHandler(void);
void GPIO3_Combined_16_31_IRQHandler(void);
void GPIO4_Combined_0_15_IRQHandler(void);
void GPIO4_Combined_16_31_IRQHandler(void);
void GPIO5_Combined_0_15_IRQHandler(void);
void GPIO5_Combined_16_31_IRQHandler(void);

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*!
 * @brief Reverses the current output logic of the multiple GPIO pins.
 *
 * @param ps_Base GPIO peripheral base pointer (GPIO1, GPIO2, GPIO3, and so on.)
 * @param u32_Mask GPIO pin number macro
 */
static inline void LLD_GPIO_PortToggle(GPIO_Type *ps_Base, uint32_t u32_Mask)
{
    ps_Base->DR_TOGGLE = u32_Mask;
}

/*!
 * @brief Reads the current input value of the GPIO port.
 *
 * @param ps_Base GPIO base pointer.
 * @param u32_Pin GPIO port pin number.
 * @retval GPIO port input value.
 */
static inline uint32_t LLD_GPIO_PinRead(GPIO_Type *ps_Base, uint32_t u32_Pin)
{
    assert(u32_Pin < 32U);

    return (((ps_Base->DR) >> u32_Pin) & 0x1U);
}

/*!
 * brief Sets the current pin interrupt mode.
 *
 * param ps_Base GPIO base pointer.
 * param u32_Pin GPIO port pin number.
 * param e_PinInterruptMode Pointer to a ref gpio_interrupt_mode_t structure
 *        that contains the interrupt mode information.
 */
static inline void LLD_GPIO_SetPinInterruptConfig(GPIO_Type *ps_Base, uint32_t u32_Pin, gpio_interrupt_mode_t e_PinInterruptMode)
{
	volatile uint32_t *icr;
	uint32_t icrShift;

	icrShift = u32_Pin;

	/* Register reset to default value */
	ps_Base->EDGE_SEL &= ~(1UL << u32_Pin);

	if (u32_Pin < 16U)
	{
		icr = &(ps_Base->ICR1);
	}
	else
	{
		icr = &(ps_Base->ICR2);
		icrShift -= 16U;
	}
	switch (e_PinInterruptMode)
	{
		case (kGPIO_IntLowLevel):
			*icr &= ~(3UL << (2UL * icrShift));
			break;
		case (kGPIO_IntHighLevel):
			*icr = (*icr & (~(3UL << (2UL * icrShift)))) | (1UL << (2UL * icrShift));
			break;
		case (kGPIO_IntRisingEdge):
			*icr = (*icr & (~(3UL << (2UL * icrShift)))) | (2UL << (2UL * icrShift));
			break;
		case (kGPIO_IntFallingEdge):
			*icr |= (3UL << (2UL * icrShift));
			break;
		case (kGPIO_IntRisingOrFallingEdge):
			ps_Base->EDGE_SEL |= (1UL << u32_Pin);
			break;
		default:; /* Intentional empty default */
			break;
	}
}

/*!
 * @brief Enables the specific pin interrupt.
 *
 * @param ps_Base GPIO base pointer.
 * @param u32_Mask GPIO pin number macro.
 */
static inline void LLD_GPIO_PortEnableInterrupts(GPIO_Type *ps_Base, uint32_t u32_Mask)
{
    ps_Base->IMR |= u32_Mask;
}

/*!
 * @brief Disables the specific pin interrupt.
 *
 * @param ps_Base GPIO base pointer.
 * @param u32_Mask GPIO pin number macro.
 */
static inline void LLD_GPIO_PortDisableInterrupts(GPIO_Type *ps_Base, uint32_t u32_Mask)
{
    ps_Base->IMR &= ~u32_Mask;
}

/*!
 * @brief Reads individual pin interrupt status.
 *
 * @param ps_Base GPIO base pointer.
 * @retval current pin interrupt status flag.
 */
static inline uint32_t LLD_GPIO_GetPinsInterruptFlags(GPIO_Type *ps_Base)
{
    return ps_Base->ISR;
}

/*!
 * @brief Clears pin interrupt flag. Status flags are cleared by
 *        writing a 1 to the corresponding bit position.
 *
 * @param ps_Base GPIO base pointer.
 * @param u32_Mask GPIO pin number macro.
 */
static inline void LLD_GPIO_PortClearInterruptFlags(GPIO_Type *ps_Base, uint32_t u32_Mask)
{
    ps_Base->ISR = u32_Mask;
}

/*!
 * brief Initializes the GPIO peripheral according to the specified
 *        parameters in the initConfig.
 *
 * param ps_Base GPIO base pointer.
 * param u32_Pin Specifies the pin number
 * param initConfig pointer to a ref gpio_pin_config_t structure that
 *        contains the configuration information.
 */
void LLD_GPIO_PinInit(GPIO_Type *ps_Base, uint32_t u32_Pin, const gpio_pin_config_t *cs_Config)
{
    /* Enable GPIO clock. */
    if(ps_Base == GPIO1)
    {
    	CLOCK_EnableClock(kCLOCK_Gpio1);
    }
    else if(ps_Base == GPIO2)
    {
    	CLOCK_EnableClock(kCLOCK_Gpio2);
    }
    else if(ps_Base == GPIO3)
	{
    	CLOCK_EnableClock(kCLOCK_Gpio3);
	}
    else if(ps_Base == GPIO4)
	{
    	CLOCK_EnableClock(kCLOCK_Gpio4);
	}
    else if(ps_Base == GPIO5)
	{
    	CLOCK_EnableClock(kCLOCK_Gpio5);
	}


    /* Register reset to default value */
    ps_Base->IMR &= ~(1UL << u32_Pin);

    /* Configure GPIO pin direction */
    if (cs_Config->e_Direction == kGPIO_DigitalInput)
    {
        ps_Base->GDIR &= ~(1UL << u32_Pin);
    }
    else
    {
        LLD_GPIO_PinWrite(ps_Base, u32_Pin, cs_Config->outputLogic);
        ps_Base->GDIR |= (1UL << u32_Pin);
    }

    /* Configure GPIO pin interrupt mode */
    LLD_GPIO_SetPinInterruptConfig(ps_Base, u32_Pin, cs_Config->e_InterruptMode);
}

/*!
 * brief Sets the output level of the individual GPIO pin to logic 1 or 0.
 *
 * param ps_Base GPIO base pointer.
 * param u32_Pin GPIO port pin number.
 * param output GPIOpin output logic level.
 *        - 0: corresponding pin output low-logic level.
 *        - 1: corresponding pin output high-logic level.
 */
void LLD_GPIO_PinWrite(GPIO_Type *ps_Base, uint32_t u32_Pin, uint8_t u8_Output)
{
    assert(u32_Pin < 32U);
    if (u8_Output == 0U)
    {
        ps_Base->DR_CLEAR = (1UL << u32_Pin);
    }
    else
    {
        ps_Base->DR_SET = (1UL << u32_Pin);
    }
}

/**
* @brief		LLD GPIO IRQ handle function.
* @param		ps_Base GPIO port
* @param		u32_Pin Pin number
* @return		void
* @details
**/
void LLD_GPIO_HandleIRQ(GPIO_Type * ps_Base, uint32_t u32_Pin)
{
	uint8_t u8_Idx;

	for (u8_Idx = 0; u8_Idx < LLD_NB_GPIO; u8_Idx++)
	{
		if( (ts_lld_gpio_manager[u8_Idx].ps_Port == ps_Base) && (ts_lld_gpio_manager[u8_Idx].u32_Pin == u32_Pin) )
		{
			/* Calling Callback Function if has one. */
			if (ts_lld_gpio_manager[u8_Idx].s_ldd_gpio_handle.pf_Callback != NULL)
			{
				ts_lld_gpio_manager[u8_Idx].s_ldd_gpio_handle.pf_Callback();
			}
		}
	}
}

/**
* @brief		IRQ GPIO1 0 to 15 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO1_Combined_0_15_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00000001;
	uint8_t u8_PinNum;

	for(u8_PinNum = 0; u8_PinNum < 16; u8_PinNum++)
	{
		if((GPIO1->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO1)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO1, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO1, u8_PinNum);
				u8_PinNum = 16;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO1 16 to 31 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO1_Combined_16_31_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00010000;
	uint8_t u8_PinNum;

	for(u8_PinNum = 16; u8_PinNum < 32; u8_PinNum++)
	{
		if((GPIO1->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO1)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO1, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO1, u8_PinNum);
				u8_PinNum = 32;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO2 0 to 15 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO2_Combined_0_15_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00000001;
	uint8_t u8_PinNum;

	for(u8_PinNum = 0; u8_PinNum < 16; u8_PinNum++)
	{
		if((GPIO2->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO2)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO2, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO2, u8_PinNum);
				u8_PinNum = 16;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO2 16 to 31 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO2_Combined_16_31_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00010000;
	uint8_t u8_PinNum;

	for(u8_PinNum = 16; u8_PinNum < 32; u8_PinNum++)
	{
		if((GPIO2->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO2)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO2, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO2, u8_PinNum);
				u8_PinNum = 32;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO3 0 to 15 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO3_Combined_0_15_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00000001;
	uint8_t u8_PinNum;

	for(u8_PinNum = 0; u8_PinNum < 16; u8_PinNum++)
	{
		if((GPIO3->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO3)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO3, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO3, u8_PinNum);
				u8_PinNum = 16;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO3 16 to 31 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO3_Combined_16_31_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00010000;
	uint8_t u8_PinNum;

	for(u8_PinNum = 16; u8_PinNum < 32; u8_PinNum++)
	{
		if((GPIO3->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO3)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO3, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO3, u8_PinNum);
				u8_PinNum = 32;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO4 0 to 15 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO4_Combined_0_15_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00000001;
	uint8_t u8_PinNum;

	for(u8_PinNum = 0; u8_PinNum < 16; u8_PinNum++)
	{
		if((GPIO4->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO4)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO4, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO4, u8_PinNum);
				u8_PinNum = 16;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO4 16 to 31 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO4_Combined_16_31_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00010000;
	uint8_t u8_PinNum;

	for(u8_PinNum = 16; u8_PinNum < 32; u8_PinNum++)
	{
		if((GPIO4->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO4)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO4, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO4, u8_PinNum);
				u8_PinNum = 32;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO5 0 to 15 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO5_Combined_0_15_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00000001;
	uint8_t u8_PinNum;

	for(u8_PinNum = 0; u8_PinNum < 16; u8_PinNum++)
	{
		if((GPIO5->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO5)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO5, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO5, u8_PinNum);
				u8_PinNum = 16;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO5 16 to 31 pins
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO5_Combined_16_31_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00010000;
	uint8_t u8_PinNum;

	for(u8_PinNum = 16; u8_PinNum < 32; u8_PinNum++)
	{
		if((GPIO5->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO5)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO5, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO5, u8_PinNum);
				u8_PinNum = 32;
			}
		}
		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}

/**
* @brief		IRQ GPIO6-GPIO9
* @param		void
* @return		void
* @details		Clear IRQ and search corresponding GPIO number
**/
void GPIO6_7_8_9_IRQHandler(void)
{
	uint32_t u32_Mask = 0x00000001;
	uint8_t u8_PinNum;

	for(u8_PinNum = 0; u8_PinNum < 32; u8_PinNum++)
	{
		if((GPIO6->IMR & u32_Mask) != 0)
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO6)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO6, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO6, u8_PinNum);
				u8_PinNum = 32;
			}
		}

		if((u8_PinNum != 32) && ((GPIO7->IMR & u32_Mask) != 0))
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO7)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO7, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO7, u8_PinNum);
				u8_PinNum = 32;
			}
		}

		if((u8_PinNum != 32) && ((GPIO8->IMR & u32_Mask) != 0))
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO8)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO8, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO8, u8_PinNum);
				u8_PinNum = 32;
			}
		}

		if((u8_PinNum != 32) && ((GPIO9->IMR & u32_Mask) != 0))
		{
			if(((LLD_GPIO_GetPinsInterruptFlags(GPIO9)) & u32_Mask) != 0)
			{
				/* clear the interrupt status */
				LLD_GPIO_PortClearInterruptFlags(GPIO9, u32_Mask);

				LLD_GPIO_HandleIRQ(GPIO9, u8_PinNum);
				u8_PinNum = 32;
			}
		}

		u32_Mask = u32_Mask << 1;
	}

    __DSB();
}


/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/**
* @brief		GPIO initialization
* @param		e_Gpio Name of GPIO pin to initialize
* @param		ps_Port GPIO port
* @param		u32_Pin Pin number
* @param		e_ItMode Pin interrupt mode
* @param		e_Direction Pin direction.
* @param		u8_OutputValue Default output direction. Needed but not used for input.
* @return		void
* @details
**/
void LLD_GPIO_Init( lld_gpio_t e_Gpio, GPIO_Type * ps_Port, uint32_t u32_Pin, lld_gpio_interrupt_mode_t e_ItMode,
					lld_gpio_pin_direction_t e_Direction, uint8_t u8_OutputValue)
{
	gpio_pin_config_t led_config;

	/* GPIO initialization */
	ts_lld_gpio_manager[e_Gpio].ps_Port = ps_Port;
	ts_lld_gpio_manager[e_Gpio].u32_Pin = u32_Pin;

	led_config.e_InterruptMode = e_ItMode;
	led_config.e_Direction = e_Direction;
	if(led_config.e_Direction == kGPIO_DigitalOutput)
	{
		led_config.outputLogic = u8_OutputValue;
	}
	LLD_GPIO_PinInit(ts_lld_gpio_manager[e_Gpio].ps_Port, ts_lld_gpio_manager[e_Gpio].u32_Pin, &led_config);

	/* Configure interrupt */
	if(led_config.e_InterruptMode != kGPIO_NoIntmode)
	{
		if(ts_lld_gpio_manager[e_Gpio].ps_Port == GPIO1)
		{
			if(ts_lld_gpio_manager[e_Gpio].u32_Pin < 16)
			{
				EnableIRQ(GPIO1_Combined_0_15_IRQn);
			}
			else
			{
				EnableIRQ(GPIO1_Combined_16_31_IRQn);
			}
		}
		else if(ts_lld_gpio_manager[e_Gpio].ps_Port == GPIO2)
		{
			if(ts_lld_gpio_manager[e_Gpio].u32_Pin < 16)
			{
				EnableIRQ(GPIO2_Combined_0_15_IRQn);
			}
			else
			{
				EnableIRQ(GPIO2_Combined_16_31_IRQn);
			}
		}
		else if(ts_lld_gpio_manager[e_Gpio].ps_Port == GPIO3)
		{
			if(ts_lld_gpio_manager[e_Gpio].u32_Pin < 16)
			{
				EnableIRQ(GPIO3_Combined_0_15_IRQn);
			}
			else
			{
				EnableIRQ(GPIO3_Combined_16_31_IRQn);
			}
		}
		else if(ts_lld_gpio_manager[e_Gpio].ps_Port == GPIO4)
		{
			if(ts_lld_gpio_manager[e_Gpio].u32_Pin < 16)
			{
				EnableIRQ(GPIO4_Combined_0_15_IRQn);
			}
			else
			{
				EnableIRQ(GPIO4_Combined_16_31_IRQn);
			}
		}
		else if(ts_lld_gpio_manager[e_Gpio].ps_Port == GPIO5)
		{
			if(ts_lld_gpio_manager[e_Gpio].u32_Pin < 16)
			{
				EnableIRQ(GPIO5_Combined_0_15_IRQn);
			}
			else
			{
				EnableIRQ(GPIO5_Combined_16_31_IRQn);
			}
		}
	}
}

/**
* @brief		Sets the output level of GPIO pin to logic 1
* @param[in]	e_Gpio Name of GPIO pin to set the output level
* @return		void
* @details
**/
void LLD_GPIO_WritePin(lld_gpio_t e_Gpio)
{
	LLD_GPIO_PinWrite(ts_lld_gpio_manager[e_Gpio].ps_Port, ts_lld_gpio_manager[e_Gpio].u32_Pin, 1);
}

/**
* @brief		Sets the output level of GPIO pin to logic 0
* @param[in]	e_Gpio Name of GPIO pin to set the output level
* @return		void
* @details
**/
void LLD_GPIO_ClearPin(lld_gpio_t e_Gpio)
{
	LLD_GPIO_PinWrite(ts_lld_gpio_manager[e_Gpio].ps_Port, ts_lld_gpio_manager[e_Gpio].u32_Pin, 0);
}

/**
* @brief		Reads the current input value of the GPIO pin.
* @param[in]	e_Gpio Name of GPIO pin to read input value
* @return		Input value of GPIO pin
* @details
**/
uint8_t LLD_GPIO_ReadPin(lld_gpio_t e_Gpio)
{
	uint8_t u8_Value;
	u8_Value = LLD_GPIO_PinRead(ts_lld_gpio_manager[e_Gpio].ps_Port, ts_lld_gpio_manager[e_Gpio].u32_Pin);
	return u8_Value;
}

/**
* @brief		Reverses the current output logic of the GPIO pin
* @param[in]	e_Gpio Name of GPIO pin to reverse value
* @return		void
* @details
**/
void LLD_GPIO_Toggle(lld_gpio_t e_Gpio)
{
	LLD_GPIO_PortToggle(ts_lld_gpio_manager[e_Gpio].ps_Port, 1u << ts_lld_gpio_manager[e_Gpio].u32_Pin);
}

/**
* @brief		Enable/Disable interrupt of the GPIO pin
* @param[in]	e_Gpio Name of GPIO pin
* @param		b_Enable Enable or disable IT
* @return		void
* @details
**/
void LLD_GPIO_EnableIt(lld_gpio_t e_Gpio, bool b_Enable)
{
	if(b_Enable == true)
	{
		/* Enable GPIO pin interrupt */
		LLD_GPIO_PortEnableInterrupts(ts_lld_gpio_manager[e_Gpio].ps_Port, 1u << ts_lld_gpio_manager[e_Gpio].u32_Pin);
	}
	else
	{
		/* Disable GPIO pin interrupt */
		LLD_GPIO_PortDisableInterrupts(ts_lld_gpio_manager[e_Gpio].ps_Port, 1u << ts_lld_gpio_manager[e_Gpio].u32_Pin);
	}
}

/**
* @brief		Set callback function
* @param		e_Gpio Name of GPIO pin
* @param		pf_Callback Pointer on function
* @return		void
* @details
**/
void LLD_GPIO_SetCallback(lld_gpio_t e_Gpio, lld_gpio_transfer_callback_t pf_Callback)
{
	ts_lld_gpio_manager[e_Gpio].s_ldd_gpio_handle.pf_Callback = pf_Callback;
}
