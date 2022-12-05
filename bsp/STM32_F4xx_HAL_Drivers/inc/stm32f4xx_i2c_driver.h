//
// Created by Brad on 12/4/2022.
//
#include "stm32fxx_drivers.h"
#include <stdint.h>

#ifndef MAIN_C_STM32F4XX_I2C_DRIVER_H
#define MAIN_C_STM32F4XX_I2C_DRIVER_H

typedef struct
{
    uint32_t I2C_SCLSpeed;
    uint8_t  I2C_DeviceAddress;
    uint8_t  I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
}I2C_Handle_t;

/*
 * Serial Clock Speed Definitions
 */

#define I2C_SCL_SPEED_SM    100000
#define I2C_SCL_SPEED_FM4k  400000
#define I2C_SCL_SPEED_FM2k  200000

/*
 * Duty Cycle Definitions
 */

#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

/*
 * I2C API Definitions
 */

void I2C_PClKControl(I2C_RegDef_t *pI2Cx, uint8_t state);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t state);

uint32_t I2C_SendData(I2C_RegDef_t *pI2Cx, uint8_t *pTXBuffer, uint32_t payload_length);
uint32_t I2C_ReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRXBuffer, uint32_t payload_length);

uint32_t I2C_SendData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length);
uint32_t I2C_ReceiveData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t state);

void I2C_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state);
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority);

uint32_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t flag);

#endif //MAIN_C_STM32F4XX_I2C_DRIVER_H
