//
// Created by Brad on 12/4/2022.
//
#include "stm32f4xx_i2c_driver.h"
static uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

void I2C_PClKControl(I2C_RegDef_t *pI2Cx, uint8_t state)
{
    if (state == ENABLE)
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_DIS();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_DIS();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_DIS();
        }
    }
}
static uint32_t RCC_Get_PLL_Output_Clock(void)
{
    return 1;
}
static uint32_t RCC_Get_PCLK1_Value(void)
{
    uint32_t pclk1,  SystemClock;
    uint8_t clksrc, temp, ahb_prescale, apb1_prescale;

    // Masking to get 2 bits
    clksrc = (RCC->CFGR >> 2) & 0x3;

    switch (clksrc) {
        case 0:
            SystemClock = 16000000;
            break;
        case 1:
            SystemClock = 8000000;
            break;
        case 2:
            SystemClock = RCC_Get_PLL_Output_Clock();
            break;
    }

    // Get AHB Prescale
    temp = (RCC->CFGR >> 4) & 0xF;
    if(temp < 8)
    {
        ahb_prescale = 1;
    }
    else
    {
        ahb_prescale = AHB_PreScaler[temp-8];
    }

    // Get APB Prescale

    // Masking to get 3 bits
    temp = (RCC->CFGR >> 10) & 0x7;
    if(temp < 4)
    {
        apb1_prescale = 1;
    }
    else
    {
        apb1_prescale = APB1_PreScaler[temp-4];
    }

    pclk1 = (SystemClock / ahb_prescale) / apb1_prescale;

    return pclk1;
}
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    // Configure ACK Field of CR1
    tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
    pI2CHandle->pI2Cx->I2C_CR1 = tempreg;

    // Configure FREQ Field of CR2
    tempreg = 0;
    tempreg = RCC_Get_PCLK1_Value() / 1000000U ;
    pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

    // Store Device Own Address
    tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

    // Must be kept at 1
    tempreg |= 1 << 14;
    pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

    // CCR Calculations
    uint16_t  ccr_value = 0;
    tempreg             = 0;

    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // Configure Standard Mode
        ccr_value = RCC_Get_PCLK1_Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
    }
    else
    {
        // Configure 2k Fast Mode
        tempreg |= 1 << 15;
        tempreg |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14;
        if ( pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = RCC_Get_PCLK1_Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        }
        else
        {
            ccr_value = RCC_Get_PCLK1_Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        }
    }

    // Masking to get 12 bits
    tempreg |= (ccr_value & 0xFFF);
    pI2CHandle->pI2Cx->I2C_CCR = tempreg;

    // Configure T Rise


}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if (pI2Cx == I2C1)
    {
        I2C1_REG_RESET;
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_REG_RESET;
    }
    else if (pI2Cx == I2C3)
    {
        I2C3_REG_RESET;
    }
}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t state)
{

}

uint32_t I2C_SendData(I2C_RegDef_t *pI2Cx, uint8_t *pTXBuffer, uint32_t payload_length)
{

}
uint32_t I2C_ReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRXBuffer, uint32_t payload_length)
{

}

uint32_t I2C_SendData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length);
uint32_t I2C_ReceiveData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t state);

void I2C_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state);
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority);

uint32_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t flag);