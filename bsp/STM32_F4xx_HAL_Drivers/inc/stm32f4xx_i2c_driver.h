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
    uint8_t         *pTXBuffer;
    uint8_t         *pRXBuffer;
    uint32_t        TX_Length;
    uint32_t        RX_Length;
    uint8_t         TX_RX_State;
    uint8_t         Device_Addr;
    uint32_t        RX_Size;
    uint8_t         Repeated_Start;
}I2C_Peer_t;

typedef struct
{
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
    I2C_Peer_t   I2C_Peer;
}I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

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
 * Status Flag Definitions
 */

#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

/*
 * Event Definitions
 */

#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

/*
 * I2C API Definitions
 */

void I2C_PClKControl(I2C_RegDef_t *pI2Cx, uint8_t state);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t state);

uint32_t I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start);
uint32_t I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

uint8_t I2C_MasterSendData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start);
uint8_t I2C_MasterReceiveData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t state);
void I2C_Slave_CallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t state);

void I2C_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority);

void I2C_Close_SendData(I2C_Handle_t *pI2CHandle);
void I2C_Close_ReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_Generate_Start_Condition(I2C_RegDef_t *pI2Cx);
void I2C_Generate_Stop_Condition(I2C_RegDef_t *pI2Cx);

uint32_t I2C_Get_Flag_Status(uint32_t I2C_Reg, uint32_t flag);
void I2C_Control_ACK(I2C_RegDef_t *pI2Cx, uint8_t state);

#endif //MAIN_C_STM32F4XX_I2C_DRIVER_H
