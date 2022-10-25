//
// Created by Brad on 10/16/2022.
//

#ifndef STM324XX_GPIO_DRIVER_H
#define STM324XX_GPIO_DRIVER_H

#include "stm32fxx_drivers.h"

/*********************************
 * Data Structures
 *********************************/

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;			/*! possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;       		/*!< This holds the base address of the GPIO port to which the pin belongs >*/
    GPIO_PinConfig_t GPIO_PinConfig;   /*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;

/*
 * GPIO MACROS
 */

/**
 * @GPIO_Pin_Modes
 * GPIO Pin Modes
 **/
#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFUNC   2
#define GPIO_MODE_ANALOG    3

#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

/**
 * Pin Output Types
 **/
#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1

/**
 * Pin Output Speeds
 **/
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MED      1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3

/**
 * Pin Pull Up and Pull Down Configuration
 **/
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 *                  API Declarations
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t state);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number);

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t state);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t state);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number);

void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority, uint8_t state);

void GPIO_IRQHandling(uint8_t pin_number);


#endif //STM324XX_GPIO_DRIVER_H
