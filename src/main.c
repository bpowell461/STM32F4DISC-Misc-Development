//
// Created by Brad on 10/19/2022.
//

#include "../bsp/STM32_F4xx_HAL_Drivers/inc/stm32fxx_drivers.h"
#include "../bsp/STM32_F4xx_HAL_Drivers/inc/stm324xx_gpio_driver.h"
#include <string.h>

#define LOW     0
#define HIGH    1

#ifdef ARDUINO
    #define BUTTON_PRESSED  HIGH
#else
    #define BUTTON_PRESSED  LOW
#endif


void delay()
{
    //Busy Loop
    int i;
    for (i = 0; i < 500000/2; i++);
}
int main(void)
{

    GPIO_Handle_t GPIO_Led_Handle, GPIO_Button_Handle;

    memset(&GPIO_Led_Handle, 0, sizeof(GPIO_Led_Handle));
    memset(&GPIO_Button_Handle, 0, sizeof(GPIO_Button_Handle));

    // Button Init
    GPIO_Button_Handle.pGPIOx = GPIOB;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinNumber       = 8;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;

    GPIO_PCLK_Control(GPIOB, ENABLE);
    GPIO_Init(&GPIO_Button_Handle);

    // Button Interrupt Config
    GPIO_IRQ_Priority_Config(IRQ_NO_EXTI9_5, 15);
    GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI9_5, ENABLE);


    // LED Init
    GPIO_Led_Handle.pGPIOx = GPIOA;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinNumber       = 8;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LOW;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
    GPIO_Led_Handle.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;

    GPIO_PCLK_Control(GPIOA, ENABLE);
    GPIO_Init(&GPIO_Led_Handle);

    while(1);

    return 0;
}

void EXTI9_5_IRQHandler(void)
{
    // Handle IRQ
    delay();
    GPIO_IRQHandling(8);
    GPIO_ToggleOutputPin(GPIOA, 8);

}
