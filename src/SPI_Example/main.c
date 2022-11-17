//
// Created by Brad on 10/19/2022.
//

#include "../../bsp/STM32_F4xx_HAL_Drivers/inc/stm32fxx_drivers.h"
#include <string.h>

void delay(void)
{
    //Busy Loop
    int i;
    for (i = 0; i < 500000/2; i++);
}
void SPI2_Init_GPIO_Pins(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_ALTFUNC;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode   = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;

    //Init Serial Clock
    SPIPins.GPIO_PinConfig.GPIO_PinNumber       = 13;
    GPIO_Init(&SPIPins);

    //Init MOSI Line
    SPIPins.GPIO_PinConfig.GPIO_PinNumber       = 15;
    GPIO_Init(&SPIPins);

    //Init MISO Line
    SPIPins.GPIO_PinConfig.GPIO_PinNumber       = 14;
    GPIO_Init(&SPIPins);

    //Init NSS Line
    SPIPins.GPIO_PinConfig.GPIO_PinNumber       = 12;
    GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
    SPI_Handle_t SPI2Handle;
    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig      = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode     = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed      = SPI_SCLK_PRESCALE_2;
    SPI2Handle.SPIConfig.SPI_DFF            = SPI_DFF_8BIT;
    SPI2Handle.SPIConfig.SPI_CPOL           = LOW;
    SPI2Handle.SPIConfig.SPI_CPHA           = LOW;
    SPI2Handle.SPIConfig.SPI_SSM            = SPI_SSM_EN;

    SPI_Init(&SPI2Handle);

}

int main(void)
{

    SPI2_Init_GPIO_Pins();

    SPI2_Init();

    // Pulls SSI to high to avoid multi-master mode fault
    SPI_SSIConfig(SPI2, ENABLE);

    // Enables SPI Peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    char dummy_payload[]    = "Hello World";
    uint8_t  payload_length = strlen(dummy_payload);

    SPI_SendData(SPI2, (uint8_t *)payload_length, 1);
    SPI_SendData(SPI2, (uint8_t *)dummy_payload, strlen(dummy_payload));

    SPI_PeripheralControl(SPI2, DISABLE);

    while(1);

    return 0;
}
