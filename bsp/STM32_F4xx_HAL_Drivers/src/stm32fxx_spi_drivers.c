//
// Created by Brad on 11/10/2022.
//
#include "../inc/stm32f4xx_spi_drivers.h"

uint32_t SPI_Get_Flag_Status(SPI_RegDef_t *pSPIx, uint32_t flag)
{
    return ((pSPIx->SPI_SR & (1 << flag)));
}

void SPI_PClKControl(SPI_RegDef_t *pSPIx, uint8_t state)
{
    if (state == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DIS();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DIS();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_DIS();
        }
    }
}

uint32_t SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t tempReg = 0;

    //Init SPI Peripheral Clock
    SPI_PClKControl(pSPIHandle->pSPIx, ENABLE);

    // Configure device mode (2nd bit position)
    tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    switch (pSPIHandle->SPIConfig.SPI_BusConfig)
    {

        case SPI_BUS_CONFIG_FD:
            tempReg &= ~(1 << 15);
            break;

        case SPI_BUS_CONFIG_HD:
            tempReg |= 1 << 15;
            break;

        case SPI_BUS_CONFIG_SIMPLEX_RX:
            tempReg &= ~(1 << 15);
            tempReg |= 1 << 10;
            break;

    }

    tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    pSPIHandle->pSPIx->SPI_CR1 = tempReg;


}

uint32_t SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET;
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET;
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET;
    }
}

//Blocking SPI Send Data (Polling based)
uint32_t SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t payload_length)
{
    while(payload_length > 0)
    {
        // Wait for TX Empty Bit Set (Blocking)
        while( !(SPI_Get_Flag_Status(pSPIx, SPI_SR_TXE) ));

        if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
        {
            //16 Bit DFF
            //Type-casting TX Buffer to 16 bit pointer then de-referencing to load value
            pSPIx->SPI_DR = *((uint16_t*)pTXBuffer);

            //Decrementing payload
            payload_length -= 2;

            //Incrementing TX Buffer address (twice for 16 bits)
            (uint16_t*)pTXBuffer++;
            (uint16_t*)pTXBuffer++;
        }
        else
        {
            //8 Bit DFF
            pSPIx->SPI_DR = *(pTXBuffer);

            //Decrementing payload
            payload_length--;

            //Incrementing TX Buffer address
            pTXBuffer++;
        }
    }

    // If TX Buffer has data then send was successful!
    return SPI_Get_Flag_Status(pSPIx, SPI_SR_TXE);
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t state)
{
    if(state == ENABLE)
    {
        pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->SPI_CR1 &= ~ (1 << SPI_CR1_SPE);
    }
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t state)
{
    if(state == ENABLE)
    {
        pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->SPI_CR1 &= ~ (1 << SPI_CR1_SSI);
    }
}
uint32_t SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t payload_length)
{

}

void SPI_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state)
{

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}
void SPI_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{

}