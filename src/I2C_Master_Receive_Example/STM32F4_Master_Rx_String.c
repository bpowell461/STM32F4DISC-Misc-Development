//
// Created by Brad on 12/11/2022.
//

#include <stdio.h>
#include <string.h>
#include "stm32fxx_drivers.h"

I2C_Handle_t I2C1Handle;

const uint8_t I2C_Master_Address = 0x61;
const uint8_t I2C_Arduino_Address = 0x68;

static void delay()
{
    //Busy Loop (200ms)
    int i;
    for (i = 0; i < 500000/2; i++);
}

static void I2C1_Init_GPIO_Pins(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_ALTFUNC;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode   = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;

    //Init SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber       = 6;

    //Init SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber       = 7;

    GPIO_Init(&I2CPins);

}

static void I2C1_Init(void)
{

    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_ACKControl = ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = I2C_Master_Address;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handle);

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

void main(void)
{
    char receive_buffer[32];
    uint8_t command;
    uint8_t length;

    I2C1_Init_GPIO_Pins();

    Button_Init();

    I2C1_Init();

    I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

    I2C_Control_ACK(I2C1Handle.pI2Cx, ENABLE);

    while(1)
    {
        while(!GPIO_ReadFromInputPin(GPIOB, 8));

        delay();

        // Length request/response to slave
        command = 0x51;
        I2C_MasterSendData(&I2C1Handle, (uint8_t *) &command, sizeof(uint8_t), I2C_Arduino_Address, I2C_ENABLE_SR);
        I2C_MasterReceiveData(&I2C1Handle, &length, sizeof(uint8_t), I2C_Arduino_Address, I2C_ENABLE_SR);

        // Data transmission request/response
        command = 0x52;
        I2C_MasterSendData(&I2C1Handle, (uint8_t *) &command, sizeof(uint8_t), I2C_Arduino_Address, I2C_ENABLE_SR );
        I2C_MasterReceiveData(&I2C1Handle, (uint8_t *)receive_buffer, length, I2C_Arduino_Address, I2C_DISABLE_SR);
    }
}