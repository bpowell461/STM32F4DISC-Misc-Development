//
// Created by Brad on 12/4/2022.
//
#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_rcc_driver.h"

static void I2C_Execute_Address_Phase_Write(I2C_RegDef_t *pI2Cx, uint8_t Slave_Addr);
static void I2C_Execute_Address_Phase_Read(I2C_RegDef_t *pI2Cx, uint8_t Slave_Addr);

static void I2C_Clear_Addr_Flag(I2C_Handle_t *pI2CHandle);

static void I2C_Master_Handle_RXNE_Interrupt(I2C_Handle_t *pI2CHandle);
static void I2C_Master_Handle_TXE_Interrupt(I2C_Handle_t *pI2CHandle);

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

uint32_t I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start)
{
    // Generate Start Condition
    I2C_Generate_Start_Condition(pI2CHandle->pI2Cx);

    // Check the SB Flag in SR1 to confirm start
    while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_SB));

    // Send address of slave with r/w bit set to w(0)
    I2C_Execute_Address_Phase_Write(pI2CHandle->pI2Cx, Slave_Addr);

    // Check ADDR Flag to confirm address phase
    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_ADDR ));

    // Clear the ADDR Flag
    I2C_Clear_Addr_Flag(pI2CHandle);

    // Send until length is 0
    while(payload_length > 0)
    {
        // This blocking loop ensures that the TX Register is Empty
        while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_TXE));

        pI2CHandle->pI2Cx->I2C_DR = *pTXBuffer;
        pTXBuffer++;
        payload_length--;
    }

    // Wait for TXE and BTF to be set high
    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_TXE) );

    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_BTF) );

    // Generate Stop Condition
    if( Repeated_Start == I2C_DISABLE_SR)
    {
        I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx);
    }

    return 1;
}

uint32_t I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start)
{
    // Generate Start Condition
    I2C_Generate_Start_Condition(pI2CHandle->pI2Cx);

    // Check the SB Flag in SR1 to confirm start
    while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_SB));

    // Send address of slave with r/w bit set to w(1)
    I2C_Execute_Address_Phase_Read(pI2CHandle->pI2Cx, Slave_Addr);

    // Check ADDR Flag to confirm address phase
    while(!I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_ADDR ));

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
        while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_RXNE));

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
            while(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_RXNE));

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

uint8_t I2C_MasterSendData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start)
{
    uint8_t I2C_State = pI2CHandle->I2C_Peer.TX_RX_State;

    if( (I2C_State != I2C_BUSY_IN_TX) && (I2C_State != I2C_BUSY_IN_RX))
    {
        pI2CHandle->I2C_Peer.pTXBuffer = pTXBuffer;
        pI2CHandle->I2C_Peer.TX_Length = payload_length;
        pI2CHandle->I2C_Peer.TX_RX_State = I2C_BUSY_IN_TX;
        pI2CHandle->I2C_Peer.Device_Addr = Slave_Addr;
        pI2CHandle->I2C_Peer.Repeated_Start = Repeated_Start;

        // Generate START Condition
        I2C_Generate_Start_Condition(pI2CHandle->pI2Cx);

        // Enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

        // Enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

        // Enable ITERREN Control Bit
        pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

    }

    return I2C_State;
}
uint8_t I2C_MasterReceiveData_NonBlocking(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t payload_length, uint8_t Slave_Addr, uint8_t Repeated_Start)
{
    uint8_t I2C_State = pI2CHandle->I2C_Peer.TX_RX_State;

    if( (I2C_State != I2C_BUSY_IN_TX) && (I2C_State != I2C_BUSY_IN_RX))
    {
        pI2CHandle->I2C_Peer.pRXBuffer = pRXBuffer;
        pI2CHandle->I2C_Peer.RX_Length = payload_length;
        pI2CHandle->I2C_Peer.RX_Size = payload_length;
        pI2CHandle->I2C_Peer.TX_RX_State = I2C_BUSY_IN_RX;
        pI2CHandle->I2C_Peer.Device_Addr = Slave_Addr;
        pI2CHandle->I2C_Peer.Repeated_Start = Repeated_Start;

        // Generate START Condition
        I2C_Generate_Start_Condition(pI2CHandle->pI2Cx);

        // Enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

        // Enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

        // Enable ITERREN Control Bit
        pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

    }

    return I2C_State;
}
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->I2C_DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t) pI2Cx->I2C_DR;
}
void I2C_Generate_Start_Condition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

void I2C_Generate_Stop_Condition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state)
{
    if(state == ENABLE)
    {
        if(IRQ_Number <= 31)
        {
            //program ISER0 register
            *NVIC_ISER0 |= ( 1 << IRQ_Number );

        }else if(IRQ_Number > 31 && IRQ_Number < 64 ) //32 to 63
        {
            //program ISER1 register
            *NVIC_ISER1 |= ( 1 << (IRQ_Number % 32) );
        }
        else if(IRQ_Number >= 64 && IRQ_Number < 96 )
        {
            //program ISER2 register //64 to 95
            *NVIC_ISER3 |= ( 1 << (IRQ_Number % 64) );
        }
    }
    else
    {
        if(IRQ_Number <= 31)
        {
            //program ICER0 register
            *NVIC_ICER0 |= ( 1 << IRQ_Number );
        }
        else if(IRQ_Number > 31 && IRQ_Number < 64 )
        {
            //program ICER1 register
            *NVIC_ICER1 |= ( 1 << (IRQ_Number % 32) );
        }
        else if(IRQ_Number >= 6 && IRQ_Number < 96 )
        {
            //program ICER2 register
            *NVIC_ICER3 |= ( 1 << (IRQ_Number % 64) );
        }
    }
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);

    // Handle SB Event Interrupt
    temp3 = I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_SB);
    if(temp1 && temp3)
    {
        if(pI2CHandle->I2C_Peer.TX_RX_State == I2C_BUSY_IN_TX)
        {
            I2C_Execute_Address_Phase_Write(pI2CHandle->pI2Cx, pI2CHandle->I2C_Peer.Device_Addr);
        }
        else if(pI2CHandle->I2C_Peer.TX_RX_State == I2C_BUSY_IN_RX)
        {
            I2C_Execute_Address_Phase_Read(pI2CHandle->pI2Cx, pI2CHandle->I2C_Peer.Device_Addr);
        }
    }

    // Handle ADDR Event Interrupt
    temp3 = I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_ADDR);
    if(temp1 && temp3)
    {
        I2C_Clear_Addr_Flag(pI2CHandle);
    }

    // Handle BTF Event Interrupt
    temp3 = I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_BTF);
    if(temp1 && temp3)
    {
        if(pI2CHandle->I2C_Peer.TX_RX_State == I2C_BUSY_IN_TX)
        {
            if(I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_TXE))
            {
                if(pI2CHandle->I2C_Peer.TX_Length == 0)
                {
                    // Generating Stop Condition
                    if(pI2CHandle->I2C_Peer.Repeated_Start == I2C_DISABLE_SR)
                    {
                        I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx);
                    }

                    // Resetting Handle Structure Elements
                    I2C_Close_SendData(pI2CHandle);

                    // Callback to notify transmission complete
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
                }
            }
        }
        else if(pI2CHandle->I2C_Peer.TX_RX_State == I2C_BUSY_IN_RX)
        {
            if(I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_RXNE))
            {
                // NOP
                ;
            }
        }
    }

    // Handle STOPF Event Interrupt
    temp3 = I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_STOPF);
    if(temp1 && temp3)
    {
        // Clear CR1 through write
        pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

        // Application Callback
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    // Handle TXE Event Interrupt
    temp3 = I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_TXE);
    if(temp1 && temp2 && temp3)
    {
        if(I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR2, I2C_SR2_MSL))
        {
            if (pI2CHandle->I2C_Peer.TX_RX_State == I2C_BUSY_IN_TX)
            {
                // Master Mode
                I2C_Master_Handle_TXE_Interrupt(pI2CHandle);
            }
        }
        else
        {
            if(I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR2, I2C_SR2_TRA ))
            {
                // Slave Mode
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }

    // Handle RXNE Event Interrupt
    temp3 = I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR1, I2C_SR1_RXNE);
    if(temp1 && temp2 && temp3)
    {
        if(I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR2, I2C_SR2_MSL))
        {
            if (pI2CHandle->I2C_Peer.TX_RX_State == I2C_BUSY_IN_RX)
            {
                I2C_Master_Handle_RXNE_Interrupt(pI2CHandle);
            }
        }
        else
        {
            if(! I2C_Get_Flag_Status(pI2CHandle->pI2Cx->I2C_SR2, I2C_SR2_TRA ))
            {
                // Slave Mode
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
    temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


    /***********************Check for Bus error************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
    if(temp1  && temp2 )
    {
        pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
    }

    /***********************Check for arbitration lost error************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
    if(temp1  && temp2)
    {
        pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
    }

    /***********************Check for ACK failure  error************************************/

    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
    if(temp1  && temp2)
    {
        pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
    }

    /***********************Check for Overrun/underrun error************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
    if(temp1  && temp2)
    {
        pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
    }

    /***********************Check for Time out error************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
    if(temp1  && temp2)
    {
        pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
    }
}
void I2C_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
    uint8_t iprx = IRQ_Number / 4;
    uint8_t iprx_section  = IRQ_Number % 4 ;

    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

    *(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQ_Number << shift_amount );
}
uint32_t I2C_Get_Flag_Status(uint32_t I2C_Reg, uint32_t flag)
{
    return ((I2C_Reg & (1 << flag)));
}
void I2C_Master_Handle_RXNE_Interrupt(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle->I2C_Peer.RX_Size == 1)
    {
        *(pI2CHandle->I2C_Peer.pRXBuffer) = pI2CHandle->pI2Cx->I2C_DR;
        pI2CHandle->I2C_Peer.RX_Length--;
    }
    if (pI2CHandle->I2C_Peer.RX_Size > 1)
    {
        if (pI2CHandle->I2C_Peer.RX_Length == 2)
        {
            I2C_Control_ACK(pI2CHandle->pI2Cx, DISABLE);
        }

        *(pI2CHandle->I2C_Peer.pRXBuffer) = pI2CHandle->pI2Cx->I2C_DR;
        pI2CHandle->I2C_Peer.RX_Length--;
        pI2CHandle->I2C_Peer.pRXBuffer++;
    }
    if (pI2CHandle->I2C_Peer.RX_Size == 0)
    {
        if (pI2CHandle->I2C_Peer.Repeated_Start == I2C_DISABLE_SR)
        {
            I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx);
        }
        I2C_Close_ReceiveData(pI2CHandle);

        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}
void I2C_Master_Handle_TXE_Interrupt(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle->I2C_Peer.TX_Length > 0)
    {
        pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->I2C_Peer.pTXBuffer);
        pI2CHandle->I2C_Peer.pTXBuffer++;
        pI2CHandle->I2C_Peer.TX_Length--;
    }
}
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t state)
{
    if(state == ENABLE)
    {
        pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
        pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
        pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
    }else
    {
        pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
        pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
        pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);
    }

}
void I2C_Close_ReceiveData(I2C_Handle_t *pI2CHandle)
{
    //Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    //Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

    pI2CHandle->I2C_Peer.TX_RX_State = I2C_READY;
    pI2CHandle->I2C_Peer.pRXBuffer = (void*)0;
    pI2CHandle->I2C_Peer.RX_Length = 0;
    pI2CHandle->I2C_Peer.RX_Size = 0;

    if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
    {
        I2C_Control_ACK(pI2CHandle->pI2Cx,ENABLE);
    }

}

void I2C_Close_SendData(I2C_Handle_t *pI2CHandle)
{
    //Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    //Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


    pI2CHandle->I2C_Peer.TX_RX_State = I2C_READY;
    pI2CHandle->I2C_Peer.pTXBuffer = (void*)0;
    pI2CHandle->I2C_Peer.TX_Length = 0;
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