//
// Created by Brad on 11/2/2022.
//
#include "../inc/stm32fxx_drivers.h"

uint32_t GPIO_BaseAddr_To_Code(GPIO_RegDef_t *pGPIOx_Handle){
uint32_t port_code = -1;

if (pGPIOx_Handle == GPIOA)
{
port_code = 0;
}
else if (pGPIOx_Handle == GPIOB)
{
port_code = 1;
}
else if (pGPIOx_Handle == GPIOC)
{
port_code = 2;
}
else if (pGPIOx_Handle == GPIOD)
{
port_code = 3;
}
else if (pGPIOx_Handle == GPIOE)
{
port_code = 4;
}
else if (pGPIOx_Handle == GPIOF)
{
port_code = 5;
}
else if (pGPIOx_Handle == GPIOG)
{
port_code = 6;
}
else if (pGPIOx_Handle == GPIOH)
{
port_code = 7;
}

return port_code;
}