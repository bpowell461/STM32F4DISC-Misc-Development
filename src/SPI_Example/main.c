//
// Created by Brad on 10/19/2022.
//

#include "../../bsp/STM32_F4xx_HAL_Drivers/inc/stm32fxx_drivers.h"
#include <string.h>

//Arduino Command/Response Codes
#define ARDUINO_COMMAND_LED_CTRL        0x50
#define ARDUINO_COMMAND_SENSOR_READ     0x51
#define ARDUINO_COMMAND_LED_READ        0x52
#define ARDUINO_COMMAND_PRINT           0x53
#define ARDUINO_COMMAND_ID_READ         0x54

#define LED_ON      1
#define LED_OFF     0
#define LED_PIN     9

#define ACK         0xF5
#define NACK        0xA5

static uint8_t isLedOn = 0;
uint8_t analog_read;
uint8_t  id[11];

static void delay()
{
    //Busy Loop (200ms)
    int i;
    for (i = 0; i < 500000/2; i++);
}

static void SPI2_Init_GPIO_Pins(void)
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

static void SPI2_Init(void)
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

}
static void Button_Init()
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

static uint8_t SPI_Verify_Response(uint8_t data)
{
    switch (data)
    {
        case ACK:
            return 1;
        case NACK:
            return 0;
        default:
            return -1;
    }
}

uint8_t SPI_Command_Arduino(SPI_RegDef_t *pSPIx, uint8_t command_code)
{
    uint8_t ack_byte;

    uint8_t dummy_read;
    uint8_t dummy_write = 0xFF;

    //Sending initial command
    SPI_SendData(pSPIx, &command_code, 1);

    //Dummy read to clear RXNE flag
    SPI_ReceiveData(pSPIx, &dummy_read, 1);

    // Send dummy data to get ACK or NACK response
    SPI_SendData(pSPIx, &dummy_write, 1);

    // Receiving ACK or NACK
    SPI_ReceiveData(pSPIx, &ack_byte, 1);

    if(SPI_Verify_Response(ack_byte))
    {
        uint8_t args[2];
        switch(command_code) {
            // COMMAND_LED_CTRL <Pin_Number> <Pin_Value>
            case ARDUINO_COMMAND_LED_CTRL: {
                args[0] = LED_PIN;
                if (isLedOn) {
                    args[1] = LED_OFF;
                } else {
                    args[1] = LED_ON;
                }

                //Sending args
                SPI_SendData(pSPIx, (uint8_t *) &args, 2);

                //Dummy read to clear RXNE
                SPI_ReceiveData(pSPIx, (uint8_t *) &args, 2);
                break;
            }

            // COMMAND_SENSOR_READ <Pin_Number>
            case ARDUINO_COMMAND_SENSOR_READ: {
                // Analog Pin 0
                args[0] = 0;

                //Sending args
                SPI_SendData(pSPIx, (uint8_t *) &args, 1);

                //Dummy read to clear RXNE
                SPI_ReceiveData(pSPIx, (uint8_t *) &args, 1);

                // Need delay to ready data
                delay();

                // Send dummy data to get sensor analog value
                SPI_SendData(pSPIx, &dummy_write, 1);

                // Receiving Analog Sensor Data
                SPI_ReceiveData(pSPIx, &analog_read, 1);
                break;
            }

            // COMMAND_LED_READ <Pin_Number>
            case ARDUINO_COMMAND_LED_READ: {
                args[0] = LED_PIN;

                //Sending args
                SPI_SendData(pSPIx, (uint8_t *) &args, 1);

                //Dummy read to clear RXNE
                SPI_ReceiveData(pSPIx, (uint8_t *) &args, 1);

                // Need delay to ready data
                delay();

                // Send dummy data to get LED value
                SPI_SendData(pSPIx, &dummy_write, 1);

                uint8_t led_read;

                // Receiving LED Pin Value
                SPI_ReceiveData(pSPIx, &led_read, 1);

                // Ternary Operation
                isLedOn = led_read ? 1 : 0;
                break;
            }

            // Command_Print <Len> <Message(Len)>
            case ARDUINO_COMMAND_PRINT: {

                uint8_t message[] = "Hello!";

                uint8_t message_len = strlen((char *) message);
                args[0] = message_len;

                //Sending args
                SPI_SendData(pSPIx, (uint8_t *) &args, 1);

                //Dummy read to clear RXNE
                SPI_ReceiveData(pSPIx, (uint8_t *) &args, 1);

                // Transmitting one byte of message at a time
                for(int i = 0; i < message_len; i++)
                {
                    SPI_SendData(pSPIx, &message[i], 1);
                    SPI_ReceiveData(pSPIx, &dummy_read, 1);
                }
                break;
            }
            case ARDUINO_COMMAND_ID_READ: {

                for(int i = 0; i < 10; i++)
                {
                    // Send dummy data to get ID value
                    SPI_SendData(pSPIx, &dummy_write, 1);

                    // Receiving ID value
                    SPI_ReceiveData(pSPIx, &id[i], 1);

                }

                // Terminate end of string!
                id[10] = '\0';
                break;
            }
            default: {
                return 0;
            }

        }
    }

    return 1;
}

void main(void)
{
    SPI2_Init();
    Button_Init();

    uint8_t command_code;

    while(1) {

        // Wait until button is pressed
        while (!GPIO_ReadFromInputPin(GPIOB, 8));

        //Button Debouncing
        delay();

        // Enables SPI Peripheral
        SPI_PeripheralControl(SPI2, ENABLE);

        // Commands start
        SPI_Command_Arduino(SPI2, ARDUINO_COMMAND_LED_CTRL);

        // Wait until button is pressed
        while (!GPIO_ReadFromInputPin(GPIOB, 8));

        //Button Debouncing
        delay();

        SPI_Command_Arduino(SPI2, ARDUINO_COMMAND_SENSOR_READ);

        // Wait until button is pressed
        while (!GPIO_ReadFromInputPin(GPIOB, 8));

        //Button Debouncing
        delay();

        SPI_Command_Arduino(SPI2, ARDUINO_COMMAND_LED_READ);

        // Wait until button is pressed
        while (!GPIO_ReadFromInputPin(GPIOB, 8));

        //Button Debouncing
        delay();

        SPI_Command_Arduino(SPI2, ARDUINO_COMMAND_PRINT);

        // Wait until button is pressed
        while (!GPIO_ReadFromInputPin(GPIOB, 8));

        //Button Debouncing
        delay();

        SPI_Command_Arduino(SPI2, ARDUINO_COMMAND_ID_READ);


        //Only close SPI when BSY flag is cleared
        while (SPI_Get_Flag_Status(SPI2, SPI_SR_BSY));

        SPI_PeripheralControl(SPI2, DISABLE);

    }
}