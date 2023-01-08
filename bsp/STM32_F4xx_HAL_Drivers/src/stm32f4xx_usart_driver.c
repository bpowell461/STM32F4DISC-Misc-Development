//
// Created by Brad on 12/26/2022.
//

#include "stm32f4xx_usart_driver.h"
#include "stm32f4xx_rcc_driver.h"

static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

void USART_PClKControl(USART_RegDef_t *pUSARTx, uint8_t state)
{
    if (state == ENABLE)
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if(pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }
        else if(pUSARTx == USART3)
        {
            USART3_PCLK_EN();
        }
        else if(pUSARTx == USART6)
        {
            USART6_PCLK_EN();
        }
    }
    else
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_DIS();
        }
        else if(pUSARTx == USART2)
        {
            USART2_PCLK_DIS();
        }
        else if(pUSARTx == USART3)
        {
            USART3_PCLK_DIS();
        }
        else if(pUSARTx == USART6)
        {
            USART6_PCLK_DIS();
        }
    }
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
    //Temporary variable
    uint32_t tempreg = 0;

/******************************** Configuration of CR1******************************************/

    //Implement the code to enable the Clock for given USART peripheral
    USART_PClKControl(pUSARTHandle->pUSARTx, ENABLE);

    //Enable USART Tx and Rx engines according to the USART_Mode configuration item
    if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        //Implement the code to enable the Receiver bit field
        tempreg |= (1 << USART_CR1_RE);
    }else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        //Implement the code to enable the Transmitter bit field
        tempreg |= ( 1 << USART_CR1_TE );

    }else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        //Implement the code to enable the both Transmitter and Receiver bit fields
        tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
    }

    //Implement the code to configure the Word length configuration item
    tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);


    //Configuration of parity control bit fields
    if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        //Implement the code to enable the parity control
        tempreg |= ( 1 << USART_CR1_PCE);

        //Implement the code to enable EVEN parity
        //Not required because by default EVEN parity will be selected once you enable the parity control

    }else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
    {
        //Implement the code to enable the parity control
        tempreg |= ( 1 << USART_CR1_PCE);

        //Implement the code to enable ODD parity
        tempreg |= ( 1 << USART_CR1_PS);

    }

    //Program the CR1 register
    pUSARTHandle->pUSARTx->USART_CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

    tempreg=0;

    //Implement the code to configure the number of stop bits inserted during USART frame transmission
    tempreg |= (pUSARTHandle->USART_Config.USART_StopBits << USART_CR2_STOP);

    //Program the CR2 register
    pUSARTHandle->pUSARTx->USART_CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

    tempreg=0;

    //Configuration of USART hardware flow control
    if ( pUSARTHandle->USART_Config.USART_HWFLowControl == USART_HW_FLOW_CTRL_CTS)
    {
        //Implement the code to enable CTS flow control
        tempreg |= ( 1 << USART_CR3_CTSE);


    }else if (pUSARTHandle->USART_Config.USART_HWFLowControl == USART_HW_FLOW_CTRL_RTS)
    {
        //Implement the code to enable RTS flow control
        tempreg |= ( 1 << USART_CR3_RTSE);

    }else if (pUSARTHandle->USART_Config.USART_HWFLowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        //Implement the code to enable both CTS and RTS Flow control
        tempreg |= ( 1 << USART_CR3_RTSE) | ( 1 << USART_CR3_CTSE);
    }


    pUSARTHandle->pUSARTx->USART_CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    if (pUSARTx == USART1)
    {
        USART1_REG_RESET;
    }
    else if (pUSARTx == USART2)
    {
        USART2_REG_RESET;
    }
    else if (pUSARTx == USART3)
    {
        USART3_REG_RESET;
    }
    else if (pUSARTx == UART4)
    {
        UART4_REG_RESET;
    }
    else if (pUSARTx == UART5)
    {
        UART5_REG_RESET;
    }
    else if (pUSARTx == USART6)
    {
        USART6_REG_RESET;
    }
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t state)
{
    if(state == ENABLE)
    {
        pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);
        //pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
    }
    else
    {
        pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
    }
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTXBuffer, uint32_t Length)
{

    uint16_t *pData;
    //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < Length; i++)
    {
        //Implement the code to wait until TXE flag is set in the SR
        while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx->USART_SR,USART_SR_TXE));

        //Check the USART_WordLength item for 9BIT or 8BIT in a frame
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            //if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
            pData = (uint16_t*) pTXBuffer;
            pUSARTHandle->pUSARTx->USART_DR = (*pData & (uint16_t)0x01FF);

            //check for USART_ParityControl
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used in this transfer. so, 9bits of user data will be sent
                //Implement the code to increment pTxBuffer twice
                pTXBuffer++;
                pTXBuffer++;
            }
            else
            {
                //Parity bit is used in this transfer . so , 8bits of user data will be sent
                //The 9th bit will be replaced by parity bit by the hardware
                pTXBuffer++;
            }
        }
        else
        {
            //This is 8bit data transfer
            pUSARTHandle->pUSARTx->USART_DR = (*pTXBuffer  & (uint8_t)0xFF);

            //Implement the code to increment the buffer address
            pTXBuffer++;
        }
    }

    //Implement the code to wait till TC flag is set in the SR
    while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx->USART_SR,USART_SR_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Length)
{
    //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < Length; i++)
    {
        //Wait until RXNE flag is set in the SR
        while( ! (USART_GetFlagStatus(pUSARTHandle->pUSARTx->USART_SR, USART_SR_RXNE)));

        //Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
        if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            //We are going to receive 9bit data in a frame

            //check are we using USART_ParityControl control or not
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used. so, all 9bits will be of user data

                //read only first 9 bits. so, mask the DR with 0x01FF
                *((uint16_t*) pRXBuffer) = (pUSARTHandle->pUSARTx->USART_DR  & (uint16_t)0x01FF);

                //Now increment the pRxBuffer two times
                pRXBuffer++;
                pRXBuffer++;
            }
            else
            {
                //Parity is used, so, 8bits will be of user data and 1 bit is parity
                *pRXBuffer = (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0xFF);

                //Increment the pRxBuffer
                pRXBuffer++;
            }
        }
        else
        {
            //We are going to receive 8bit data in a frame

            //check are we using USART_ParityControl control or not
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                //No parity is used , so all 8bits will be of user data

                //read 8 bits from DR
                *pRXBuffer = pUSARTHandle->pUSARTx->USART_DR;
            }

            else
            {
                //Parity is used, so , 7 bits will be of user data and 1 bit is parity

                //read only 7 bits , hence mask the DR with 0X7F
                *pRXBuffer = ((uint8_t) 0x7F & pUSARTHandle->pUSARTx->USART_DR);

            }

            //increment the pRxBuffer
            pRXBuffer++;
        }
    }

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendData_NonBlocking(USART_Handle_t *pUSARTHandle,uint8_t *pTXBuffer, uint32_t Length)
{
    uint8_t TX_State = pUSARTHandle->USART_Peer.TX_State;

    if(TX_State != USART_BUSY_IN_TX)
    {
        pUSARTHandle->USART_Peer.TX_Length = Length;
        pUSARTHandle->USART_Peer.pTXBuffer = pTXBuffer;
        pUSARTHandle->USART_Peer.TX_State = USART_BUSY_IN_TX;

        //Implement the code to enable interrupt for TXE
        pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TXEIE);

        //Implement the code to enable interrupt for TC
        pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TCIE);

    }

    return TX_State;
}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveData_NonBlocking(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Length)
{
    uint8_t RX_State = pUSARTHandle->USART_Peer.RX_State;

    if(RX_State != USART_BUSY_IN_RX)
    {
        pUSARTHandle->USART_Peer.RX_Length = Length;
        pUSARTHandle->USART_Peer.pRXBuffer = pRXBuffer;
        pUSARTHandle->USART_Peer.RX_State = USART_BUSY_IN_RX;

        //Implement the code to enable interrupt for RXNE
        pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RXNEIE);

    }

    return RX_State;

}

void USART_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state)
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
void USART_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
    uint8_t iprx            = IRQ_Number / 4;
    uint8_t iprx_section    = IRQ_Number % 4;

    // Only upper half of 8 bits (4 bits) are usable
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= IRQ_Priority << shift_amount;
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

    uint32_t temp1 , temp2, temp3;

    uint16_t *pData;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
    temp1 = pUSARTHandle->pUSARTx->USART_SR & ( 1 << USART_SR_TC);

    //Implement the code to check the state of TCEIE bit
    temp2 = pUSARTHandle->pUSARTx->USART_CR1 & ( 1 << USART_CR1_TCIE);

    if(temp1 && temp2 )
    {
        //this interrupt is because of TC

        //close transmission and call application callback if TxLen is zero
        if ( pUSARTHandle->USART_Peer.TX_State == USART_BUSY_IN_TX)
        {
            //Check the TxLen . If it is zero then close the data transmission
            if(! pUSARTHandle->USART_Peer.TX_Length )
            {
                //Implement the code to clear the TC flag
                pUSARTHandle->pUSARTx->USART_SR &= ~( 1 << USART_SR_TC);

                //Implement the code to clear the TCIE control bit
                pUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_TCIE);

                //Reset the application state
                pUSARTHandle->USART_Peer.TX_State = USART_READY;

                //Reset Buffer address to NULL
                pUSARTHandle->USART_Peer.pTXBuffer = (void*) 0;

                //Reset the length to zero
                pUSARTHandle->USART_Peer.TX_Length = 0;

                //Call the applicaton call back with event USART_EVENT_TX_CMPLT
                USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
            }
        }
    }

/*************************Check for TXE flag ********************************************/

    //Implement the code to check the state of TXE bit in the SR
    temp1 = pUSARTHandle->pUSARTx->USART_SR & ( 1 << USART_SR_TXE);

    //Implement the code to check the state of TXEIE bit in CR1
    temp2 = pUSARTHandle->pUSARTx->USART_CR1 & ( 1 << USART_CR1_TXEIE);


    if(temp1 && temp2 )
    {
        //this interrupt is because of TXE

        if(pUSARTHandle->USART_Peer.TX_State == USART_BUSY_IN_TX)
        {
            //Keep sending data until Txlen reaches to zero
            if(pUSARTHandle->USART_Peer.TX_Length > 0)
            {
                //Check the USART_WordLength item for 9BIT or 8BIT in a frame
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
                {
                    //if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
                    pData = (uint16_t*) pUSARTHandle->USART_Peer.pTXBuffer;

                    //loading only first 9 bits , so we have to mask with the value 0x01FF
                    pUSARTHandle->pUSARTx->USART_DR = (*pData & (uint16_t)0x01FF);

                    //check for USART_ParityControl
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        //No parity is used in this transfer , so, 9bits of user data will be sent
                        //Implement the code to increment pTxBuffer twice
                        pUSARTHandle->USART_Peer.pTXBuffer++;
                        pUSARTHandle->USART_Peer.pTXBuffer++;

                        //Implement the code to decrement the length
                        pUSARTHandle->USART_Peer.TX_Length-=2;
                    }
                    else
                    {
                        //Parity bit is used in this transfer . so , 8bits of user data will be sent
                        //The 9th bit will be replaced by parity bit by the hardware
                        pUSARTHandle->USART_Peer.pTXBuffer++;

                        //Implement the code to decrement the length
                        pUSARTHandle->USART_Peer.TX_Length--;
                    }
                }
                else
                {
                    //This is 8bit data transfer
                    pUSARTHandle->pUSARTx->USART_DR = (*pUSARTHandle->USART_Peer.pTXBuffer  & (uint8_t)0xFF);

                    //Implement the code to increment the buffer address
                    pUSARTHandle->USART_Peer.pTXBuffer++;

                    //Implement the code to decrement the length
                    pUSARTHandle->USART_Peer.TX_Length--;
                }

            }
            if (pUSARTHandle->USART_Peer.TX_Length == 0 )
            {
                //TxLen is zero
                //Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
                pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TXEIE);
            }
        }
    }

/*************************Check for RXNE flag ********************************************/

    temp1 = pUSARTHandle->pUSARTx->USART_SR & ( 1 << USART_SR_RXNE);
    temp2 = pUSARTHandle->pUSARTx->USART_CR1 & ( 1 << USART_CR1_RXNEIE);


    if(temp1 && temp2 )
    {
        //this interrupt is because of rxne
        //this interrupt is because of txe
        if(pUSARTHandle->USART_Peer.RX_State == USART_BUSY_IN_RX)
        {
            //TXE is set so send data
            if(pUSARTHandle->USART_Peer.RX_Length > 0)
            {
                //Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
                if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
                {
                    //We are going to receive 9bit data in a frame

                    //Now, check are we using USART_ParityControl control or not
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        //No parity is used. so, all 9bits will be of user data

                        //read only first 9 bits so mask the DR with 0x01FF
                        *((uint16_t*) pUSARTHandle->USART_Peer.pRXBuffer) = (pUSARTHandle->pUSARTx->USART_DR  & (uint16_t)0x01FF);

                        //Now increment the pRxBuffer two times
                        pUSARTHandle->USART_Peer.pRXBuffer++;
                        pUSARTHandle->USART_Peer.pRXBuffer++;

                        //Implement the code to decrement the length
                        pUSARTHandle->USART_Peer.RX_Length-=2;
                    }
                    else
                    {
                        //Parity is used. so, 8bits will be of user data and 1 bit is parity
                        *pUSARTHandle->USART_Peer.pRXBuffer = (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0xFF);

                        //Now increment the pRxBuffer
                        pUSARTHandle->USART_Peer.pRXBuffer++;

                        //Implement the code to decrement the length
                        pUSARTHandle->USART_Peer.RX_Length--;
                    }
                }
                else
                {
                    //We are going to receive 8bit data in a frame

                    //Now, check are we using USART_ParityControl control or not
                    if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
                    {
                        //No parity is used , so all 8bits will be of user data

                        //read 8 bits from DR
                        *pUSARTHandle->USART_Peer.pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0xFF);
                    }

                    else
                    {
                        //Parity is used, so , 7 bits will be of user data and 1 bit is parity

                        //read only 7 bits , hence mask the DR with 0X7F
                        *pUSARTHandle->USART_Peer.pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0x7F);

                    }

                    //Now , increment the pRxBuffer
                    pUSARTHandle->USART_Peer.pRXBuffer++;

                    //Implement the code to decrement the length
                    pUSARTHandle->USART_Peer.RX_Length--;
                }


            }//if of >0

            if(! pUSARTHandle->USART_Peer.RX_Length)
            {
                //disable the rxne
                pUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_RXNEIE );
                pUSARTHandle->USART_Peer.RX_State = USART_READY;
                USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
            }
        }
    }


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

    //Implement the code to check the status of CTS bit in the SR
    temp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_CTS);

    //Implement the code to check the state of CTSE bit in CR1
    temp2 = pUSARTHandle->pUSARTx->USART_CR3 & ( 1 << USART_CR3_CTSE);

    //Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
    temp3 = pUSARTHandle->pUSARTx->USART_CR3 & ( 1 << USART_CR3_CTSIE);


    if(temp1  && temp2 )
    {
        //Implement the code to clear the CTS flag in SR
        pUSARTHandle->pUSARTx->USART_SR &= ~ ( 1 << USART_SR_CTS);

        //this interrupt is because of cts
        USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
    }

/*************************Check for IDLE detection flag ********************************************/

    //Implement the code to check the status of IDLE flag bit in the SR

    //Implement the code to check the state of IDLEIE bit in CR1
    temp2 = pUSARTHandle->pUSARTx->USART_CR3 & ( 1 << USART_CR3_CTSE);


    if(temp1 && temp2)
    {
        //Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
        uint32_t dummyRead;

        dummyRead = pUSARTHandle->pUSARTx->USART_SR;
        dummyRead = pUSARTHandle->pUSARTx->USART_DR;
        (void)dummyRead;

        //this interrupt is because of idle
        USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
    }

/*************************Check for Overrun detection flag ********************************************/

    //Implement the code to check the status of ORE flag  in the SR
    temp1 = pUSARTHandle->pUSARTx->USART_SR & USART_SR_ORE;

    //Implement the code to check the status of RXNEIE  bit in the CR1
    temp2 = pUSARTHandle->pUSARTx->USART_CR1 & USART_CR1_RXNEIE;


    if(temp1  && temp2 )
    {
        //Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
        uint32_t dummyRead;

        dummyRead = pUSARTHandle->pUSARTx->USART_SR;
        dummyRead = pUSARTHandle->pUSARTx->USART_DR;
        (void)dummyRead;

        //this interrupt is because of Overrun error
        USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
    }



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//The below code will get executed in only if multibuffer mode is used.

    temp2 =  pUSARTHandle->pUSARTx->USART_CR3 & ( 1 << USART_CR3_EIE) ;

    if(temp2 )
    {
        temp1 = pUSARTHandle->pUSARTx->USART_SR;
        if(temp1 & ( 1 << USART_SR_FE))
        {
            /*
                This bit is set by hardware when a de-synchronization, excessive noise or a break character
                is detected. It is cleared by a software sequence (an read to the USART_SR register
                followed by a read to the USART_DR register).
            */
            uint32_t dummyRead;

            dummyRead = pUSARTHandle->pUSARTx->USART_SR;
            dummyRead = pUSARTHandle->pUSARTx->USART_DR;
            (void)dummyRead;

            USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
        }

        if(temp1 & ( 1 << USART_SR_NE) )
        {
            /*
                This bit is set by hardware when noise is detected on a received frame. It is cleared by a
                software sequence (an read to the USART_SR register followed by a read to the
                USART_DR register).
            */
            uint32_t dummyRead;

            dummyRead = pUSARTHandle->pUSARTx->USART_SR;
            dummyRead = pUSARTHandle->pUSARTx->USART_DR;
            (void)dummyRead;
            USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
        }

        if(temp1 & ( 1 << USART_SR_ORE) )
        {
            USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
        }
    }

}

uint8_t USART_GetFlagStatus(uint32_t USART_Reg, uint8_t flag)
{
    if (USART_Reg & (1 << flag))
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag)
{
    pUSARTx->USART_SR &= ~(1 << flag);
}

static void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    uint32_t peripheralClockValue = 0;

    uint32_t usartDiv;

    uint32_t mantissaValue, fractionalValue;

    uint32_t tempReg = 0;

    if (pUSARTx == USART1 || pUSARTx == USART6)
    {
        peripheralClockValue = RCC_Get_PCLK2_Value();
    }
    else
    {
        peripheralClockValue = RCC_Get_PCLK1_Value();
    }

    if (USART_GetFlagStatus(pUSARTx->USART_CR1, USART_CR1_OVER8))
    {
        // Oversampling 8
        usartDiv = ((25 * peripheralClockValue) / (2 * BaudRate));
    }
    else
    {
        // Oversampling 16
        usartDiv = ((25 * peripheralClockValue) / (4 * BaudRate));
    }

    mantissaValue = usartDiv / 100;

    tempReg |= mantissaValue << 4;

    fractionalValue = (usartDiv - (mantissaValue * 100));

    if(USART_GetFlagStatus(pUSARTx->USART_CR1, USART_CR1_OVER8))
    {
        fractionalValue = (((fractionalValue * 8) + 50) / 100) & ((uint8_t) 0x07);
    }
    else
    {
        fractionalValue = (((fractionalValue * 16) + 50) / 100) & ((uint8_t) 0x0F);
    }
    tempReg |= fractionalValue;

    pUSARTx->USART_BRR = tempReg;
}