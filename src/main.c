//
// Created by Brad on 10/19/2022.
//

#include "../bsp/STM32_F4xx_HAL_Drivers/inc/stm32fxx_drivers.h"

void delay()
{
    //Busy Loop
    int i;
    for (i = 0; i < 5000000; i++);
}
 int main(void)
{

    GPIO_Handle_t GPIO_Led_Handle, GPIO_Button_Handle;

    GPIO_Led_Handle.pGPIOx = GPIOD;
    GPIO_Button_Handle.pGPIOx = GPIOA;

    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinNumber       = 0;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IN;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;

    GPIO_PCLK_Control(GPIOA, ENABLE);
    GPIO_PCLK_Control(GPIOD, ENABLE);

    GPIO_Init(&GPIO_Led_Handle);

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOA, 0) == 1)
        {
            for(int i = 0; i < 2; i++)
            {
                // Button debouncing
                delay();
                GPIO_ToggleOutputPin(GPIOD, 12);
            }
        }
    }
    return 0;
}