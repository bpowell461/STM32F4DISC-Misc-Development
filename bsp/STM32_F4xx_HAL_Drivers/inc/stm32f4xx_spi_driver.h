//
// Created by Brad on 11/10/2022.
//
#include "stm32fxx_drivers.h"
#include <stdint.h>
#ifndef MAIN_C_STM32F4XX_SPI_DRIVER_H
#define MAIN_C_STM32F4XX_SPI_DRIVER_H

typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct
{
    uint8_t         *pTXBuffer;
    uint8_t         *pRXBuffer;
    uint32_t        TX_Length;
    uint32_t        RX_Length;
    uint8_t         TX_State;
    uint8_t         RX_State;
}SPI_Peer_t;
typedef struct
{
    SPI_RegDef_t    *pSPIx;
    SPI_Config_t    SPIConfig;
    SPI_Peer_t      SPI_Peer;
}SPI_Handle_t;

#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE   0

#define SPI_BUS_CONFIG_FD               0
#define SPI_BUS_CONFIG_HD               1
#define SPI_BUS_CONFIG_SIMPLEX_RX       2

#define SPI_SCLK_PRESCALE_2     0
#define SPI_SCLK_PRESCALE_4     1
#define SPI_SCLK_PRESCALE_8     2
#define SPI_SCLK_PRESCALE_16    3
#define SPI_SCLK_PRESCALE_32    4
#define SPI_SCLK_PRESCALE_64    5
#define SPI_SCLK_PRESCALE_128   6
#define SPI_SCLK_PRESCALE_256   7

#define SPI_DFF_8BIT    0
#define SPI_DFF_16BIT   1

#define SPI_SSM_EN  1
#define SPI_SSM_DIS  0

#define SPI_READY       0
#define SPI_BUSY_RX     1
#define SPI_BUSY_TX     2

#define SPI_EVENT_TX_COMPLETE   1
#define SPI_EVENT_RX_COMPLETE   2
#define SPI_EVENT_OVR_ERROR     3

#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)


void SPI_PClKControl(SPI_RegDef_t *pSPIx, uint8_t state);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t state);

uint32_t SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t payload_length);
uint32_t SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t payload_length);

uint32_t SPI_SendData_NonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t payload_length);
uint32_t SPI_ReceiveData_NonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t payload_length);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t state);

void SPI_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority);

uint32_t SPI_Get_Flag_Status(SPI_RegDef_t *pSPIx, uint32_t flag);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t state);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t state);


#endif //MAIN_C_STM32F4XX_SPI_DRIVER_H
