//
// Created by Brad on 12/4/2022.
//
#include<string.h>
#include "../../bsp/STM32_F4xx_HAL_Drivers/inc/stm32fxx_drivers.h"

SPI_Handle_t SPI2handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;


volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
    for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUNC;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
    GPIO_Init(&SPIPins);

    //MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
    GPIO_Init(&SPIPins);

    //MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
    GPIO_Init(&SPIPins);


    //NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
    GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{
    SPI2handle.pSPIx = SPI2;
    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_PRESCALE_32;
    SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
    SPI2handle.SPIConfig.SPI_CPOL = LOW;
    SPI2handle.SPIConfig.SPI_CPHA = LOW;
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DIS; //Hardware slave management enabled for NSS pin

    SPI_Init(&SPI2handle);
}


/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
    GPIO_Handle_t spiIntPin;
    memset(&spiIntPin,0,sizeof(spiIntPin));

    //this is led gpio configuration
    spiIntPin.pGPIOx = GPIOD;
    spiIntPin.GPIO_PinConfig.GPIO_PinNumber = 6;
    spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&spiIntPin);

    GPIO_IRQ_Priority_Config(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
    GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI9_5,ENABLE);

}


int main(void)
{

    uint8_t dummy = 0xff;

    Slave_GPIO_InterruptPinInit();

    //this function is used to initialize the GPIO pins to behave as SPI2 pins
    SPI2_GPIOInits();

    //This function is used to initialize the SPI2 peripheral parameters
    SPI2_Inits();

    /*
    * making SSOE 1 does NSS output enable.
    * The NSS pin is automatically managed by the hardware.
    * i.e when SPE=1 , NSS will be pulled to low
    * and NSS pin will be high when SPE=0
    */
    SPI_SSOEConfig(SPI2,ENABLE);

    SPI_IRQ_Interrupt_Config(IRQ_NO_SPI2,ENABLE);

    while(1){

        rcvStop = 0;

        while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

        GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI9_5,DISABLE);

        //enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2,ENABLE);


        while(!rcvStop)
        {
            /* fetch the data from the SPI peripheral byte by byte in interrupt mode */
            while ( SPI_SendData_NonBlocking(&SPI2handle,&dummy,1) == SPI_BUSY_TX);
            while ( SPI_ReceiveData_NonBlocking(&SPI2handle,&ReadByte,1) == SPI_BUSY_RX );
        }


        // confirm SPI is not busy
        while(SPI_Get_Flag_Status(SPI2,SPI_BUSY_FLAG) );

        //Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2,DISABLE);

        dataAvailable = 0;

        GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI9_5,ENABLE);


    }

    return 0;

}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{

    SPI_IRQHandling(&SPI2handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
    static uint32_t i = 0;
    /* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
    if(AppEv == SPI_EVENT_RX_COMPLETE)
    {
        RcvBuff[i++] = ReadByte;
        if(ReadByte == '\0' || ( i == MAX_LEN)){
            rcvStop = 1;
            RcvBuff[i-1] = '\0';
            i = 0;
        }
    }

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
    GPIO_IRQHandling(6);
    dataAvailable = 1;
}