//
// Created by Brad on 11/10/2022.
//
#include "../inc/stm32f4xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t state);


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

            //Incrementing TX Buffer address
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

uint32_t SPI_SendData_NonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t payload_length)
{
    if(pSPIHandle->TX_State != SPI_BUSY_TX) {

        // Save TX Buffer Address and Payload Length in Global Variables
        pSPIHandle->pTXBuffer = pTXBuffer;
        pSPIHandle->TX_Length = payload_length;

        // Mark SPI State as 'Busy' for thread safety
        pSPIHandle->TX_State = SPI_BUSY_TX;

        // Enable TXEIE Control Bit to get TXE Flag Interrupt from Status Register
        pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

        // Handle Data Transmission in ISR
    }

    return (pSPIHandle->TX_State);
}

uint32_t SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t payload_length)
{
    while(payload_length > 0)
    {
        // Wait for RX Not Empty Bit Set (Blocking)
        while( !(SPI_Get_Flag_Status(pSPIx, SPI_SR_RXNE) ));

        if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
        {
            //16 Bit DFF
            //Type-casting RX Buffer to 16 bit pointer then de-referencing to load value
            *((uint16_t*)pRXBuffer) = pSPIx->SPI_DR;

            //Decrementing payload
            payload_length -= 2;

            //Incrementing RX Buffer address
            (uint16_t*)pRXBuffer++;
        }
        else
        {
            //8 Bit DFF
            *(pRXBuffer) = pSPIx->SPI_DR;

            //Decrementing payload
            payload_length--;

            //Incrementing RX Buffer address
            pRXBuffer++;
        }
    }

    // If RX Buffer has data then send was successful!
    return SPI_Get_Flag_Status(pSPIx, SPI_SR_RXNE);
}

uint32_t SPI_ReceiveData_NonBlocking(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t payload_length)
{
    if (pSPIHandle->RX_State != SPI_BUSY_RX) {

        // Save RX Buffer Address and Payload Length in Global Variables
        pSPIHandle->pRXBuffer = pRXBuffer;
        pSPIHandle->RX_Length = payload_length;

        // Mark SPI State as 'Busy' for thread safety
        pSPIHandle->RX_State = SPI_BUSY_TX;

        // Enable RXNEIE Control Bit to get TXE Flag Interrupt from Status Register
        pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

        // Handle Data Transmission in ISR
    }

    return (pSPIHandle->RX_State);
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t state)
{
    if(state == ENABLE)
    {
        pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->SPI_CR2 &= ~ (1 << SPI_CR2_SSOE);
    }
}

void SPI_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state)
{
    if(state == ENABLE)
    {
        if(IRQ_Number <= 31)
        {
            // ISER0 Register
            *NVIC_ISER0 |= (1 << IRQ_Number);
        }
        else if (IRQ_Number > 31 && IRQ_Number < 64)
        {
            // ISER1 Register
            *NVIC_ISER1 |= (1 << IRQ_Number % 32);
        }
        else if (IRQ_Number >= 64 && IRQ_Number < 96)
        {
            // ISER3 Register
            *NVIC_ISER3 |= (1 << IRQ_Number % 64);
        }
    }
    else
    {
        if(IRQ_Number <= 31)
        {
            // ICER0 Register
            *NVIC_ICER0 |= (1 << IRQ_Number);
        }
        else if (IRQ_Number > 31 && IRQ_Number < 64)
        {
            // ICER1 Register
            *NVIC_ICER1 |= (1 << IRQ_Number % 32);
        }
        else if (IRQ_Number >= 64 && IRQ_Number < 96)
        {
            // ICER3 Register
            *NVIC_ICER3 |= (1 << IRQ_Number % 64);
        }
    }
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
    uint8_t Flag_Check1, Flag_Check2;

    // Decode Interrupt

    // Checking for TXE
    Flag_Check1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);
    Flag_Check2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

    // Check and Handle TXE Interrupt
    if (Flag_Check1 && Flag_Check2)
    {
        spi_txe_interrupt_handle(pSPIHandle);

    }

    // Checking for RXNE
    Flag_Check1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
    Flag_Check2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

    // Check and Handle RXNE Interrupt
    if (Flag_Check1 && Flag_Check2)
    {
        spi_rxne_interrupt_handle(pSPIHandle);

    }

    // Checking for Overrun Error
    Flag_Check1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
    Flag_Check2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);

    // Check and Handle Overrun Error
    if (Flag_Check1 && Flag_Check2)
    {
        spi_ovr_err_interrupt_handle(pSPIHandle);

    }

}

void SPI_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
    uint8_t iprx            = IRQ_Number / 4;
    uint8_t iprx_section    = IRQ_Number % 4;

    // Only upper half of 8 bits (4 bits) are usable
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= IRQ_Priority << shift_amount;
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
    {
        //16 Bit DFF
        //Type-casting TX Buffer to 16 bit pointer then de-referencing to load value
        pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTXBuffer);

        //Decrementing payload
        pSPIHandle->TX_Length -= 2;

        //Incrementing TX Buffer address
        (uint16_t*)pSPIHandle->pTXBuffer++;
    }
    else
    {
        //8 Bit DFF
        pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTXBuffer);

        //Decrementing payload
        pSPIHandle->TX_Length--;

        //Incrementing TX Buffer address
        pSPIHandle->pTXBuffer++;
    }

    // TX_Length is 0 then close SPI Communication
    if(! pSPIHandle->TX_Length)
    {
        // Clear TXEIE Bit to prevent interrupt
        pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

        // Resetting TX Buffer to NULL
        pSPIHandle->pTXBuffer = (void*)0;

        // Resetting Length to 0
        pSPIHandle->TX_Length = 0;

        // Set TX State to Ready
        pSPIHandle->TX_State = SPI_READY;

        // Call-back to application to inform Ready
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);

    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
    {
        //16 Bit DFF
        //Type-casting RX Buffer to 16 bit pointer then de-referencing to load value
        *((uint16_t*)pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->SPI_DR;

        //Decrementing payload
        pSPIHandle->RX_Length -= 2;

        //Incrementing RX Buffer address
        (uint16_t*)pSPIHandle->pRXBuffer++;
    }
    else
    {
        //8 Bit DFF
        *(pSPIHandle->pRXBuffer) = pSPIHandle->pSPIx->SPI_DR;

        //Decrementing payload
        pSPIHandle->RX_Length--;

        //Incrementing RX Buffer address
        pSPIHandle->pRXBuffer++;
    }

    // RX_Length is 0 then close SPI Communication
    if(! pSPIHandle->RX_Length)
    {
        // Clear RXNEIE Bit to prevent interrupt
        pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

        // Resetting RX Buffer to NULL
        pSPIHandle->pRXBuffer = (void*)0;

        // Resetting Length to 0
        pSPIHandle->RX_Length = 0;

        // Set RX State to Ready
        pSPIHandle->RX_State = SPI_READY;

        // Call-back to application to inform Ready
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);

    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t OVR_Clear_Read;
    // Clearing OVR Flag
    if(pSPIHandle->TX_State != SPI_BUSY_TX)
    {
        //OVR Bit is cleared by reading DR and SR registers
        OVR_Clear_Read = pSPIHandle->pSPIx->SPI_DR;
        OVR_Clear_Read = pSPIHandle->pSPIx->SPI_SR;
    }

    (void)OVR_Clear_Read;

    // Callback to application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t state)
{

}