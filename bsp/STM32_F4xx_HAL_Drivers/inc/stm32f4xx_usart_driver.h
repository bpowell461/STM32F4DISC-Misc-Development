//
// Created by Brad on 12/26/2022.
//

#ifndef MAIN_C_STM32F4XX_USART_DRIVER_H
#define MAIN_C_STM32F4XX_USART_DRIVER_H

#include "stm32fxx_drivers.h"

typedef struct
{
    uint8_t     USART_Mode;
    uint32_t    USART_Baud;
    uint8_t     USART_StopBits;
    uint8_t     USART_WordLength;
    uint8_t     USART_ParityControl;
    uint8_t     USART_HWFLowControl;
}USART_Config_t;

typedef struct
{
    uint8_t     *pTXBuffer;
    uint8_t     *pRXBuffer;
    uint32_t    TX_Length;
    uint32_t    RX_Length;
    uint8_t     TX_State;
    uint8_t     RX_State;
}USART_Peer_t;

typedef struct
{
    USART_RegDef_t *pUSARTx;
    USART_Config_t USART_Config;
    USART_Peer_t   USART_Peer;
}USART_Handle_t;

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * Application states
 */
#define USART_BUSY_IN_RX    1
#define USART_BUSY_IN_TX    2
#define USART_READY         0


#define USART_EVENT_TX_CMPLT    0
#define	USART_EVENT_RX_CMPLT    1
#define	USART_EVENT_IDLE        2
#define	USART_EVENT_CTS         3
#define	USART_EVENT_PE          4
#define	USART_ERR_FE     	    5
#define	USART_ERR_NE    	    6
#define	USART_ERR_ORE    	    7

void USART_PCLKControl(USART_RegDef_t *pUSARTx, uint8_t state);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTXBuffer, uint32_t Length);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Length);
uint8_t USART_SendData_NonBlocking(USART_Handle_t *pUSARTHandle,uint8_t *pTXBuffer, uint32_t Length);
uint8_t USART_ReceiveData_NonBlocking(USART_Handle_t *pUSARTHandle, uint8_t *pRXBuffer, uint32_t Length);

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t state);

uint8_t USART_GetFlagStatus(uint32_t USART_Reg, uint8_t flag);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag);

void USART_IRQ_Interrupt_Config(uint8_t IRQ_Number, uint8_t state);
void USART_IRQ_Priority_Config(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv) __attribute__((weak));

#endif //MAIN_C_STM32F4XX_USART_DRIVER_H
