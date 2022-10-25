#include "../inc/stm32fxx_drivers.h"
#include "../inc/stm324xx_gpio_driver.h"

#include <stdio.h>

int main(void) {
    printf("Hello, World!\n");

    GPIO_RegDef_t *pGPIOA = GPIOA;

    return 0;
}
