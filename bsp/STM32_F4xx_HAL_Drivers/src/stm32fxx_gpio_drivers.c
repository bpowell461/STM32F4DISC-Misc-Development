#include "../inc/stm324xx_gpio_driver.h"


/*
 *                  API Implementations
 */

/**
 * @Function GPIO_Init()
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

        pGPIOHandle->pGPIOx->ModeReg.value &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->ModeReg.value |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
    }
    else
    {
        // Interrupt Mode
    }

    // Configure Speed
    pGPIOHandle->pGPIOx->OSpeedReg.value &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSpeedReg.value |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

    // Configure PuPd
    pGPIOHandle->pGPIOx->PuPdReg.value &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PuPdReg.value |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

    // Configure Output Type
    pGPIOHandle->pGPIOx->OTypeReg.value &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTypeReg.value |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

    // Configure Alternate Functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
    {
        // Determining Low or High Register
        uint32_t index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;

        //Determining starting bit position in register
        uint32_t shift_amount = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

        pGPIOHandle->pGPIOx->AFReg[index].value &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->AFReg[index].value |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * shift_amount) );
    }
}

/**
 * @Function GPIO_DeInit()
 *
 * @brief
 *
 * @param[in] GPIO_RegDef_t *pGPIOx
 *
 * @return None
 *
 * @Note De-initializes the GPIO port
 *
 **/
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

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
    uint8_t value;

    value = (uint8_t) ((pGPIOx->IDReg.value >> pin_number) & 0x00000001);

    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t) pGPIOx->IDReg.value;

    return value;
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t state)
{
    pGPIOx->ODReg.value = state;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t state)
{
    if (state == SET)
    {
        pGPIOx->ODReg.value |= (SET << pin_number);
    }
    else
    {
        pGPIOx->ODReg.value &= ~(SET << pin_number);
    }
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
    pGPIOx->ODReg.value ^= (SET << pin_number);
}

void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority, uint8_t state);

void GPIO_IRQHandling(uint8_t pin_number);