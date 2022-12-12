//
// Created by Brad on 12/4/2022.
//
#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_rcc_driver.h"

static void I2C_Generate_Start_Condition(I2C_RegDef_t *pI2Cx);
static void I2C_Generate_Stop_Condition(I2C_RegDef_t *pI2Cx);

static void I2C_Execute_Address_Phase_Write(I2C_RegDef_t *pI2Cx, uint8_t Slave_Addr);
static void I2C_Execute_Address_Phase_Read(I2C_RegDef_t *pI2Cx, uint8_t Slave_Addr);

static void I2C_Clear_Addr_Flag(I2C_Handle_t *pI2CHandle);

void I2C_PClKControl(I2C_RegDef_t *pI2Cx, uint8_t state)
{
    if (state == ENABLE)
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_DIS();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_DIS();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_DIS();
        }
    }
}
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;
    uint8_t t_rise;

    // Enable Clock
    I2C_PClKControl(pI2CHandle->pI2Cx, ENABLE);

    // Configure ACK Field of CR1
    I2C_Control_ACK(pI2CHandle->pI2Cx, pI2CHandle->I2C_Config.I2C_ACKControl);

    // Configure FREQ Field of CR2
    tempreg = 0;
    tempreg = RCC_Get_PCLK1_Value() / 1000000U ;
    pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

    // Store Device Own Address
    tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

    // Must be kept at 1
    tempreg |= 1 << 14;
    pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

    // CCR Calculations
    uint16_t  ccr_value = 0;
    tempreg             = 0;

    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // Configure Standard Mode
        ccr_value = RCC_Get_PCLK1_Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
    }
    else
    {
        // Configure 2k Fast Mode
        tempreg |= 1 << 15;
        tempreg |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14;
        if ( pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = RCC_Get_PCLK1_Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        }
        else
        {
            ccr_value = RCC_Get_PCLK1_Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
        }
    }

    // Masking to get 12 bits
    tempreg |= (ccr_value & 0xFFF);
    pI2CHandle->pI2Cx->I2C_CCR = tempreg;

    // Configure T Rise
    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        tempreg = (RCC_Get_PCLK1_Value() / 1000000U) + 1;
    }
    else
    {
        tempreg = ( (RCC_Get_PCLK1_Value() * 300) / 1000000000U) + 1;
    }

    pI2CHandle->pI2Cx->I2C_TRISE = tempreg & 0x3F;
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if (pI2Cx == I2C1)
    {
        I2C1_REG_RESET;
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_REG_RESET;
    }
    else if (pI2Cx == I2C3)
    {
        I2C3_REG_RESET;
    }
}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t state)
{

}

static void I2C_Execute_Address_Phase_Write(I2C_RegDef_t *pI2Cx, uint8_t Slave_Addr)
{
    // Slave Addr is 7 bits with LSB being the R/W bit
    Slave_Addr      = Slave_Addr << 1;
    Slave_Addr      &= ~(1);
    pI2Cx->I2C_DR   = Slave_Addr;
}

static void I2C_Execute_Address_Phase_Read(I2C_RegDef_t *pI2Cx, uint8_t Slave_Addr)
{
    // Slave Addr is 7 bits with LSB being the R/W bit
    Slave_Addr      = Slave_Addr << 1;
    Slave_Addr      |= 1;
    pI2Cx->I2C_DR   = Slave_Addr;
}

static void I2C_Clear_Addr_Flag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_read;
    //check for device mode
    if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
    {
        //device is in master mode
        if(pI2CHandle->I2C_Peer.TX_RX_State == I2C_BUSY_IN_RX)
        {
            if(pI2CHandle->I2C_Peer.RX_Size == 1)
            {
                //first disable the ack
                I2C_Control_ACK(pI2CHandle->pI2Cx,DISABLE);

                //clear the ADDR flag ( read SR1 , read SR2)
                dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
                dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
                (void)dummy_read;
            }

        }
        else
        {
            //clear the ADDR flag ( read SR1 , read SR2)
            dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
            dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
            (void)dummy_read;

        }

    }
    else
    {
        // Device is in slave mode
        // Clear the ADDR flag ( read SR1 , read SR2)
        dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
        dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
        (void)dummy_read;
    }
}

uint32_t I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length, uint8_t Slave_Addr)
{
    // Generate Start Condition
    I2C_Generate_Start_Condition(pI2CHandle->pI2Cx);

    // Check the SB Flag in SR1 to confirm start
    while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_SB));

    // Send address of slave with r/w bit set to w(0)
    I2C_Execute_Address_Phase_Write(pI2CHandle->pI2Cx, Slave_Addr);

    // Check ADDR Flag to confirm address phase
    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_ADDR ));

    // Clear the ADDR Flag
    I2C_Clear_Addr_Flag(pI2CHandle);

    // Send until length is 0
    while(payload_length > 0)
    {
        // This blocking loop ensures that the TX Register is Empty
        while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_TXE));

        pI2CHandle->pI2Cx->I2C_DR = *pTXBuffer;
        pTXBuffer++;
        payload_length--;
    }

    // Wait for TXE and BTF to be set high
    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_TXE) );

    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_BTF) );

    // Generate Stop Condition
    I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx);

    return 1;
}

uint32_t I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start)
{
    // Generate Start Condition
    I2C_Generate_Start_Condition(pI2CHandle->pI2Cx);

    // Check the SB Flag in SR1 to confirm start
    while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_SB));

    // Send address of slave with r/w bit set to w(1)
    I2C_Execute_Address_Phase_Read(pI2CHandle->pI2Cx, Slave_Addr);

    // Check ADDR Flag to confirm address phase
    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_ADDR ));

    // Clear the ADDR Flag
    I2C_Clear_Addr_Flag(pI2CHandle);

    // Procedure for 1 Byte of Data
    if(payload_length == 1)
    {
        // Disable ACKing
        I2C_Control_ACK(pI2CHandle->pI2Cx, DISABLE);

        // Clear ADDR Flag
        I2C_Clear_Addr_Flag(pI2CHandle);

        // Wait for RXNE set
        while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_RXNE));

        // Generate Stop Condition
        if( Repeated_Start == I2C_DISABLE_SR)
        {
            I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx);
        }

        // Read data into buffer
        *pRXBuffer = pI2CHandle->pI2Cx->I2C_DR;
    }
    else if (payload_length > 1)
    {
        // Clear ADDR Flag
        I2C_Clear_Addr_Flag(pI2CHandle);

        for(uint32_t i = payload_length; i > 0; i--)
        {
            // Wait for RXNE set
            while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx, I2C_SR1_RXNE));

            // Last 2 bytes
            if(i == 2)
            {
                // Disable ACKing
                I2C_Control_ACK(pI2CHandle->pI2Cx, DISABLE);

                // Generate Stop Condition
                if( Repeated_Start == I2C_DISABLE_SR)
                {
                    I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx);
                }
            }

            // Read data into buffer
            *pRXBuffer = pI2CHandle->pI2Cx->I2C_DR;

            // Increment buffer address
            pRXBuffer++;
        }
    }

    // Re-enable ACKing
    if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
    {
        I2C_Control_ACK(pI2CHandle->pI2Cx,ENABLE);
    }
    
    return 1;
}

static void I2C_Generate_Start_Condition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_Generate_Stop_Condition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}
uint32_t I2C_ReceiveData(I2C_RegDef_t *pI2Cx, uint8_t *pRXBuffer, uint32_t payload_length)
{

}

uint32_t I2C_SendData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length);
uint32_t I2C_ReceiveData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t state);

void I2C_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state);
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority);

uint32_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t flag)
{
    return ((pI2Cx->I2C_SR1 & (1 << flag)));
}

void I2C_Control_ACK(I2C_RegDef_t *pI2Cx, uint8_t state)
{
    if(state == ENABLE)
    {
        pI2Cx->I2C_CR1 |= 1 << I2C_CR1_ACK;
    }
    else
    {
        // Disabling ACK
        pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
    }
}