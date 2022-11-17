#ifndef STM32FXX_DRIVERS_H
#define STM32FXX_DRIVERS_H

#include <stdint.h>

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1          ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2          ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3          ((volatile uint32_t*)0xE000E10c)


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  		((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((volatile uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/* IRQ Numbers */
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40


/* Memory Storage Addresses */
#define FLASH_BASEADDR          0x08000000U
#define SRAM1_BASEADDR          0x20000000U
#define SRAM2_BASEADDR          0x20001C00U
#define SYSMEM_BASEADDR         0x1FFF0000U
#define ROM_BASEADDR            SYSMEM_BASEADDR
#define SRAM_BASEADDR           SRAM1_BASEADDR

/* Bus Matrix Addresses*/

#define PERIPH_BASEADDR         0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     (PERIPH_BASEADDR + 0x00010000U)
#define AHB1PERIPH_BASEADDR     (PERIPH_BASEADDR + 0x00020000U)
#define AHB2PERIPH_BASEADDR     (PERIPH_BASEADDR + 0x01000000U)

/* AHB1 Bus Addresses */

#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Ptr Types
 */
#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

/* APB1 Bus Addresses */

#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR         (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR          (APB1PERIPH_BASEADDR + 0x5000)

/* APB2 Bus Addresses */

#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR + 0x1400)

/*
 * Clock Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	    (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		    (RCC->AHB1ENR |= (1 << 8))

#define GPIOA_PCLK_DIS()    	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS()		(RCC->AHB1ENR &= ~(1 << 8))


/*
 * Clock Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1 << 23))

#define I2C1_PCLK_DIS()         (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()         (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()         (RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1 << 13))

#define SPI1_PCLK_DIS()         (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()         (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()         (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DIS()         (RCC->APB2ENR &= ~(1 << 13))


/*
 * Clock Macros for USARTx peripherals
 */
#define USART1_PCCK_EN()        (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN()        (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()         (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()         (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN()        (RCC->APB1ENR |= (1 << 5))

#define USART1_PCCK_DIS()       (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCCK_DIS()       (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCCK_DIS()       (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCCK_DIS()        (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCCK_DIS()        (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCCK_DIS()       (RCC->APB1ENR &= ~(1 << 5))

/*
 * Clock Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))

#define SYSCFG_PCLK_DIS()       (RCC->APB2ENR &= ~(1 << 14))

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

#define LOW     0
#define HIGH    1

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

/*
 * RCC Macros
 */

#define GPIOA_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET     do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

#define SPI1_REG_RESET      do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET      do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET      do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/*
 * Register Structs and Unions
 */
typedef struct
{
    volatile uint32_t   ModeReg;
    volatile uint32_t   OTypeReg;
    volatile uint32_t   OSpeedReg;
    volatile uint32_t   PuPdReg;
    volatile uint32_t   IDReg;
    volatile uint32_t   ODReg;
    volatile uint32_t   BSRReg;
    volatile uint32_t   CFLReg;
    volatile uint32_t   AFReg[2];
} GPIO_RegDef_t;

typedef struct
{
    volatile uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
    volatile uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
    volatile uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
    volatile uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
    volatile uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
    volatile uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
    volatile uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
    uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
    volatile uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
    volatile uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
    uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
    volatile uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
    volatile uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
    volatile uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
    uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
    volatile uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
    volatile uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
    uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
    volatile uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
    volatile uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
    volatile uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
    uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
    volatile uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
    volatile uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
    uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
    volatile uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
    volatile uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
    uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
    volatile uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
    volatile uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
    volatile uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
    volatile uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */
} RCC_RegDef_t;

typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    volatile uint32_t RESERVED1[2];
    volatile uint32_t CMPCR;
    volatile uint32_t RESERVED2[2];
    volatile uint32_t CFGR;
} SYSCFG_RegDef_t;

typedef struct
{
    volatile uint32_t SPI_CR1;
    volatile uint32_t SPI_CR2;
    volatile uint32_t SPI_SR;
    volatile uint32_t SPI_DR;
    volatile uint32_t SPI_CRCPR;
    volatile uint32_t SPI_RXCRCR;
    volatile uint32_t SPI_TXCRCR;
    volatile uint32_t SPI_I2SCFGR;
    volatile uint32_t SPI_I2SPR;
}SPI_RegDef_t;

/*
 * Function
 */

uint32_t GPIO_BaseAddr_To_Code(GPIO_RegDef_t *pGPIOx_Handle);

#include "stm324xx_gpio_driver.h"
#include "stm32f4xx_spi_drivers.h"

#endif //STM32FXX_DRIVERS_H
