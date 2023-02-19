/*
 * HAL_GPIO.h
 *
 *  Created on: 19 août 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_GPIO_HAL_GPIO_H_
#define HAL_HAL_GPIO_HAL_GPIO_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "LLD_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
typedef enum
{
    E_GREEN_LED                     = LLD_GPIO_B0_00,
    E_ORANGE_LED                    = LLD_GPIO_B0_01,
    E_RED_LED                       = LLD_GPIO_B0_02,
    E_YELLOW_ONE_LED                = LLD_GPIO_B0_03,
    E_YELLOW_TWO_LED                = LLD_GPIO_B0_04,
    E_YELLOW_THREE_LED              = LLD_GPIO_B0_05,

    E_STOP_BUTTON                   = LLD_GPIO_B1_00,
    E_START_BUTTON                  = LLD_GPIO_B1_01,

    E_LEFT_BUMPER                   = LLD_GPIO_B1_02,
    E_CENTER_BUMPER                 = LLD_GPIO_B1_03,
    E_RIGHT_BUMPER                  = LLD_GPIO_B1_04,

    E_MOTOR_BLADE_ENABLE            = LLD_GPIO_B1_05,
    E_MOTOR_ONE_FORWARD_ENABLE      = LLD_GPIO_B1_06,
    E_MOTOR_ONE_BACKWARD_ENABLE     = LLD_GPIO_B1_07,
    E_MOTOR_TWO_FORWARD_ENABLE      = LLD_GPIO_B1_08,
    E_MOTOR_TWO_BACKWARD_ENABLE     = LLD_GPIO_B1_09,

    E_LEFT_ECHO_SONAR               = LLD_GPIO_B1_10,
    E_CENTER_ECHO_SONAR             = LLD_GPIO_B1_11,
    E_RIGHT_ECHO_SONAR              = LLD_GPIO_B1_12,
    E_LEFT_TRIGGER_SONAR            = LLD_GPIO_B1_13,
    E_CENTER_TRIGGER_SONAR          = LLD_GPIO_B1_14,
    E_RIGHT_TRIGGER_SONAR           = LLD_GPIO_B1_15
}GPIO;

typedef enum
{
    E_GREEN_LED_PIN                     = 0,
    E_ORANGE_LED_PIN                    = 1,
    E_RED_LED_PIN                       = 2,
    E_YELLOW_ONE_LED_PIN                = 3,
    E_YELLOW_TWO_LED_PIN                = 4,
    E_YELLOW_THREE_LED_PIN              = 5,

    E_STOP_BUTTON_PIN                   = 16,
    E_START_BUTTON_PIN                  = 17,

    E_LEFT_BUMPER_PIN                   = 18,
    E_CENTER_BUMPER_PIN                 = 19,
    E_RIGHT_BUMPER_PIN                  = 20,

    E_MOTOR_BLADE_ENABLE_PIN            = 21,
    E_MOTOR_ONE_FORWARD_ENABLE_PIN      = 22,
    E_MOTOR_ONE_BACKWARD_ENABLE_PIN     = 23,
    E_MOTOR_TWO_FORWARD_ENABLE_PIN      = 24,
    E_MOTOR_TWO_BACKWARD_ENABLE_PIN     = 25,

    E_LEFT_ECHO_SONAR_PIN               = 26,
    E_CENTER_ECHO_SONAR_PIN             = 27,
    E_RIGHT_ECHO_SONAR_PIN              = 28,
    E_LEFT_TRIGGER_SONAR_PIN            = 29,
    E_CENTER_TRIGGER_SONAR_PIN          = 30,
    E_RIGHT_TRIGGER_SONAR_PIN           = 31
}PIN;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_Init(void);

#endif /* HAL_HAL_GPIO_HAL_GPIO_H_ */
