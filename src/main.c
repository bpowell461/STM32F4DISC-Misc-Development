//
// Created by Brad on 10/19/2022.
//

#include "../bsp/STM32_F4xx_HAL_Drivers/inc/stm32fxx_drivers.h"

void delay()
{
    //Busy Loop
    int i;
    for (i = 0; i < 500000; i++);
}
int main(void)
{

    GPIO_Handle_t GPIO_Led_Handle, GPIO_Button_Handle;

    GPIO_Led_Handle.pGPIOx = GPIOA;
    GPIO_Button_Handle.pGPIOx = GPIOB;

    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinNumber       = 8;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IN;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;

    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinNumber       = 8;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;

    GPIO_PCLK_Control(GPIOB, ENABLE);
    GPIO_PCLK_Control(GPIOA, ENABLE);

    GPIO_Init(&GPIO_Led_Handle);

    GPIO_Init(&GPIO_Button_Handle);

    while(1)
    {
        if(GPIO_ReadInput(&GPIO_Button_Handle) == 1)
        {
            for(int i = 0; i < 2; i++)
            {
                // Button debouncing
                delay();
                GPIO_ToggleOutput(&GPIO_Led_Handle);
            }
        }
    }
    return 0;
}
