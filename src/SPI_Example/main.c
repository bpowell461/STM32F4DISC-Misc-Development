//
// Created by Brad on 10/19/2022.
//

#include "../../bsp/STM32_F4xx_HAL_Drivers/inc/stm32fxx_drivers.h"
#include <string.h>

void delay(void)
{
    //Busy Loop (200ms)
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
    //Configures GPIO pins for SPI
    SPI2_Init_GPIO_Pins();

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

    // Pulls SSI to high to avoid multi-master mode fault
    SPI_SSIConfig(SPI2, ENABLE);
    SPI_SSOEConfig(SPI2, ENABLE);

    // Enables SPI Peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

}
void Button_Init()
{
    GPIO_Handle_t GPIO_Button_Handle;

    memset(&GPIO_Button_Handle, 0, sizeof(GPIO_Button_Handle));

    // Button Init
    GPIO_Button_Handle.pGPIOx = GPIOB;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinNumber       = 8;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
    GPIO_Button_Handle.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;

    GPIO_Init(&GPIO_Button_Handle);
}

int main(void)
{
    SPI2_Init();
    Button_Init();

    char dummy_payload[]    = "Hello World";
    uint8_t  payload_length = strlen(dummy_payload);

    // Wait until button is pressed
    while( !GPIO_ReadFromInputPin(GPIOB, 8));

    SPI_SendData(SPI2, (uint8_t *)&payload_length, 1);
    SPI_SendData(SPI2, (uint8_t *)dummy_payload, strlen(dummy_payload));

    //Only close SPI when BSY flag is cleared
    while(SPI_Get_Flag_Status(SPI2, SPI_SR_BSY));

    SPI_PeripheralControl(SPI2, DISABLE);

    while(1);

    return 0;
}