#include "../inc/stm324xx_gpio_driver.h"


/*
 *                  API Implementations
 */

/**
 * @Function GPIO_Init
 *
 * @brief
 *
 * @param[in] GPIO_Handle_t *pGPIOHandle
 *
 * @return None
 *
 * @Note Initializes the GPIO port
 *
 **/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    // Configure Mode
    uint32_t temp = 0;
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        // Non-Interrupt Mode
        pGPIOHandle->pGPIOx->ModeReg &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->ModeReg |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
    }
    else
    {
        // Interrupt Mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // Configuring the FTSR
            EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            //Clearing RTSR bit
            EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // Configuring the RTSR
            EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            //Clearing FTSR bit
            EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // Configuring the FTSR and RTSR
            EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

            EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // Configuring GPIO Port Selection in SYSCFG_EXTICR
        SYSCFG_PCLK_EN();

        // Determining register index
        uint32_t index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;

        // Determining shift amount in register
        uint32_t shift = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

        uint8_t GPIO_Port_Code = GPIO_BaseAddr_To_Code(pGPIOHandle->pGPIOx);
        SYSCFG->EXTICR[index] |= (GPIO_Port_Code << (shift * 4) );

        // Enabling EXTI interrupt using Interrupt Mask Register (IMR)
        EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    // Configure Speed
    pGPIOHandle->pGPIOx->OSpeedReg &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSpeedReg |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

    // Configure PuPd
    pGPIOHandle->pGPIOx->PuPdReg &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PuPdReg |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

    // Configure Output Type
    pGPIOHandle->pGPIOx->OTypeReg &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTypeReg |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

    // Configure Alternate Functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
    {
        // Determining Low or High Register
        uint32_t index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;

        //Determining starting bit position in register
        uint32_t shift_amount = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

        pGPIOHandle->pGPIOx->AFReg[index] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->AFReg[index] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * shift_amount) );
    }
}

/**
 * @Function GPIO_DeInit
 *
 * @brief
 *
 * @param[in] GPIO_RegDef_t *pGPIOx
 *
 * @return None
 *
 * @Note De-initializes the GPIO port
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET;
    }
    else if(pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET;
    }
    else if(pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET;
    }
    else if(pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET;
    }
    else if(pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET;
    }
    else if(pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET;
    }
    else if(pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET;
    }
    else if(pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET;
    }
    else if(pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET;
    }
}

/**
 * @Function GPIO_PCLK_Control
 *
 * @brief
 *
 * @param[in] GPIO_RegDef_t *pGPIOx
 *
 * @return None
 *
 * @Note Enables the GPIO Peripheral Clock
 *
 */
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t state)
{
    if (state == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DIS();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DIS();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DIS();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DIS();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DIS();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DIS();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DIS();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DIS();
        }
        else if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DIS();
        }
    }
}

/**
 * @Function GPIO_ReadFromInputPin
 *
 * @brief
 *
 * @param[in] GPIO_RegDef_t *pGPIOx
 * @param[in] uint8_t pin_number
 *
 * @return None
 *
 * @Note Reads 8-bits from a GPIO Pin
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
    uint8_t value;

    value = (uint8_t) ((pGPIOx->IDReg >> pin_number) & 0x00000001);

    return value;
}

/**
 * @Function GPIO_ReadFromInputPort
 *
 * @brief
 *
 * @param[in] GPIO_RegDef_t *pGPIOx
 *
 * @return None
 *
 * @Note Reads 16-bits from GPIO Port
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t) pGPIOx->IDReg;

    return value;
}

/**
 * @Function GPIO_ReadInput
 *
 * @brief
 *
 * @param[in] GPIO_Handle_t *pGPIOx
 *
 * @return None
 *
 * @Note Reads from GPIO Pin based on handle configuration
 *
 */
uint8_t GPIO_ReadInput(GPIO_Handle_t *pGPIO_Handle)
{
    return (uint8_t) ((pGPIO_Handle->pGPIOx->IDReg >> pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) & 0x00000001);
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t state)
{
    pGPIOx->ODReg = state;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t state)
{
    if (state == SET)
    {
        pGPIOx->ODReg |= (SET << pin_number);
    }
    else
    {
        pGPIOx->ODReg &= ~(SET << pin_number);
    }
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
    pGPIOx->ODReg ^= (SET << pin_number);
}

void GPIO_ToggleOutput(GPIO_Handle_t *pGPIO_Handle)
{
    pGPIO_Handle->pGPIOx->ODReg ^= (SET << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
}

void GPIO_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state)
{
     if(state == ENABLE)
     {
         if(IRQ_Number <= 31)
         {
            // ISER0 Register
            *NVIC_ISER0 |= (1 << IRQ_Number);
         }
         else if (IRQ_Number > 31 && IRQ_Number < 64)
         {
             // ISER1 Register
             *NVIC_ISER1 |= (1 << IRQ_Number % 32);
         }
         else if (IRQ_Number >= 64 && IRQ_Number < 96)
         {
             // ISER2 Register
             *NVIC_ISER2 |= (1 << IRQ_Number % 64);
         }
     }
     else
     {
         if(IRQ_Number <= 31)
         {
             // ICER0 Register
             *NVIC_ICER0 |= (1 << IRQ_Number);
         }
         else if (IRQ_Number > 31 && IRQ_Number < 64)
         {
             // ICER1 Register
             *NVIC_ICER1 |= (1 << IRQ_Number % 32);
         }
         else if (IRQ_Number >= 64 && IRQ_Number < 96)
         {
             // ICER2 Register
             *NVIC_ICER2 |= (1 << IRQ_Number % 64);
         }
     }
}

void GPIO_IRQHandling(uint8_t pin_number)
{
     // Clear EXTI PR Reg
    if (EXTI->PR & (1 << pin_number))
    {
        EXTI->PR |= (1 << pin_number);
    }
}

void GPIO_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
    uint8_t iprx            = IRQ_Number / 4;
    uint8_t iprx_section    = IRQ_Number % 4;

    // Only upper half of 8 bits (4 bits) are usable
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= IRQ_Priority << shift_amount;
}