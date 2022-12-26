//
// Created by Brad on 12/6/2022.
//

#include "stm32f4xx_rcc_driver.h"


static uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_Get_PLL_Output_Clock(void)
{
    return 1;
}
uint32_t RCC_Get_PCLK1_Value(void)
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