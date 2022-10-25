#ifndef STM32FXX_DRIVERS_H
#define STM32FXX_DRIVERS_H

#include <stdint.h>

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

/*
 * Register Structs and Unions
 */

typedef union {
    uint32_t value;
    struct {
        uint32_t ModeR0    : 2;
        uint32_t ModeR1    : 2;
        uint32_t ModeR2    : 2;
        uint32_t ModeR3    : 2;
        uint32_t ModeR4    : 2;
        uint32_t ModeR5    : 2;
        uint32_t ModeR6    : 2;
        uint32_t ModeR7    : 2;
        uint32_t ModeR8    : 2;
        uint32_t ModeR9    : 2;
        uint32_t ModeR10   : 2;
        uint32_t ModeR11   : 2;
        uint32_t ModeR12   : 2;
        uint32_t ModeR13   : 2;
        uint32_t ModeR14   : 2;
        uint32_t ModeR15   : 2;
    };
} GPIO_ModeReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t OT0        : 1;
        uint32_t OT1        : 1;
        uint32_t OT2        : 1;
        uint32_t OT3        : 1;
        uint32_t OT4        : 1;
        uint32_t OT5        : 1;
        uint32_t OT6        : 1;
        uint32_t OT7        : 1;
        uint32_t OT8        : 1;
        uint32_t OT9        : 1;
        uint32_t OT10       : 1;
        uint32_t OT11       : 1;
        uint32_t OT12       : 1;
        uint32_t OT13       : 1;
        uint32_t OT14       : 1;
        uint32_t OT15       : 1;
        uint32_t Reserved   : 16;
    };
}GPIO_OTypeReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t OSpeedR0    : 2;
        uint32_t OSpeedR1    : 2;
        uint32_t OSpeedR2    : 2;
        uint32_t OSpeedR3    : 2;
        uint32_t OSpeedR4    : 2;
        uint32_t OSpeedR5    : 2;
        uint32_t OSpeedR6    : 2;
        uint32_t OSpeedR7    : 2;
        uint32_t OSpeedR8    : 2;
        uint32_t OSpeedR9    : 2;
        uint32_t OSpeedR10   : 2;
        uint32_t OSpeedR11   : 2;
        uint32_t OSpeedR12   : 2;
        uint32_t OSpeedR13   : 2;
        uint32_t OSpeedR14   : 2;
        uint32_t OSpeedR15   : 2;
    };
}GPIO_OSpeedReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t PuPdR0    : 2;
        uint32_t PuPdR1    : 2;
        uint32_t PuPdR2    : 2;
        uint32_t PuPdR3    : 2;
        uint32_t PuPdR4    : 2;
        uint32_t PuPdR5    : 2;
        uint32_t PuPdR6    : 2;
        uint32_t PuPdR7    : 2;
        uint32_t PuPdR8    : 2;
        uint32_t PuPdR9    : 2;
        uint32_t PuPdR10   : 2;
        uint32_t PuPdR11   : 2;
        uint32_t PuPdR12   : 2;
        uint32_t PuPdR13   : 2;
        uint32_t PuPdR14   : 2;
        uint32_t PuPdR15   : 2;
    };
}GPIO_PuPdReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t IDR0        : 1;
        uint32_t IDR1        : 1;
        uint32_t IDR2        : 1;
        uint32_t IDR3        : 1;
        uint32_t IDR4        : 1;
        uint32_t IDR5        : 1;
        uint32_t IDR6        : 1;
        uint32_t IDR7        : 1;
        uint32_t IDR8        : 1;
        uint32_t IDR9        : 1;
        uint32_t IDR10       : 1;
        uint32_t IDR11       : 1;
        uint32_t IDR12       : 1;
        uint32_t IDR13       : 1;
        uint32_t IDR14       : 1;
        uint32_t IDR15       : 1;
        uint32_t Reserved   : 16;
    };
}GPIO_IDReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t ODR0        : 1;
        uint32_t ODR1        : 1;
        uint32_t ODR2        : 1;
        uint32_t ODR3        : 1;
        uint32_t ODR4        : 1;
        uint32_t ODR5        : 1;
        uint32_t ODR6        : 1;
        uint32_t ODR7        : 1;
        uint32_t ODR8        : 1;
        uint32_t ODR9        : 1;
        uint32_t ODR10       : 1;
        uint32_t ODR11       : 1;
        uint32_t ODR12       : 1;
        uint32_t ODR13       : 1;
        uint32_t ODR14       : 1;
        uint32_t ODR15       : 1;
        uint32_t Reserved   : 16;
    };
}GPIO_ODReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t BS0        : 1;
        uint32_t BS1        : 1;
        uint32_t BS2        : 1;
        uint32_t BS3        : 1;
        uint32_t BS4        : 1;
        uint32_t BS5        : 1;
        uint32_t BS6        : 1;
        uint32_t BS7        : 1;
        uint32_t BS8        : 1;
        uint32_t BS9        : 1;
        uint32_t BS10       : 1;
        uint32_t BS11       : 1;
        uint32_t BS12       : 1;
        uint32_t BS13       : 1;
        uint32_t BS14       : 1;
        uint32_t BS15       : 1;
        uint32_t BR0        : 1;
        uint32_t BR1        : 1;
        uint32_t BR2        : 1;
        uint32_t BR3        : 1;
        uint32_t BR4        : 1;
        uint32_t BR5        : 1;
        uint32_t BR6        : 1;
        uint32_t BR7        : 1;
        uint32_t BR8        : 1;
        uint32_t BR9        : 1;
        uint32_t BR10       : 1;
        uint32_t BR11       : 1;
        uint32_t BR12       : 1;
        uint32_t BR13       : 1;
        uint32_t BR14       : 1;
        uint32_t BR15       : 1;
    };
}GPIO_BSRReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t LCK0        : 1;
        uint32_t LCK1        : 1;
        uint32_t LCK2        : 1;
        uint32_t LCK3        : 1;
        uint32_t LCK4        : 1;
        uint32_t LCK5        : 1;
        uint32_t LCK6        : 1;
        uint32_t LCK7        : 1;
        uint32_t LCK8        : 1;
        uint32_t LCK9        : 1;
        uint32_t LCK10       : 1;
        uint32_t LCK11       : 1;
        uint32_t LCK12       : 1;
        uint32_t LCK13       : 1;
        uint32_t LCK14       : 1;
        uint32_t LCK15       : 1;
        uint32_t LCK16       : 1;
        uint32_t Reserved   : 15;
    };
}GPIO_CFLReg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t AFR0        : 4;
        uint32_t AFR1        : 4;
        uint32_t AFR2        : 4;
        uint32_t AFR3        : 4;
        uint32_t AFR4        : 4;
        uint32_t AFR5        : 4;
        uint32_t AFR6        : 4;
        uint32_t AFR7        : 4;
    };
}GPIO_AFReg_t;

typedef struct
{
    volatile GPIO_ModeReg_t     ModeReg;
    volatile GPIO_OTypeReg_t    OTypeReg;
    volatile GPIO_OSpeedReg_t   OSpeedReg;
    volatile GPIO_PuPdReg_t     PuPdReg;
    volatile GPIO_IDReg_t       IDReg;
    volatile GPIO_ODReg_t       ODReg;
    volatile GPIO_BSRReg_t      BSRReg;
    volatile GPIO_CFLReg_t      CFLReg;
    volatile GPIO_AFReg_t       AFReg[2];
} GPIO_RegDef_t;

typedef union {
    struct {
        uint32_t HSION          : 1;
        uint32_t HSIRDY         : 1;
        uint32_t HSITRIM0       : 1;
        uint32_t HSITRIM1       : 1;
        uint32_t HSITRIM2       : 1;
        uint32_t HSITRIM3       : 1;
        uint32_t HSITRIM4       : 1;
        uint32_t HSICAL0        : 1;
        uint32_t HSICAL1        : 1;
        uint32_t HSICAL2        : 1;
        uint32_t HSICAL3        : 1;
        uint32_t HSICAL4        : 1;
        uint32_t HSICAL5        : 1;
        uint32_t HSICAL6        : 1;
        uint32_t HSICAL7        : 1;
        uint32_t HSEON          : 1;
        uint32_t HSERDY         : 1;
        uint32_t HSEBYP         : 1;
        uint32_t CSSON          : 1;
        uint32_t Reserved1      : 4;
        uint32_t PLL_ON         : 1;
        uint32_t PLL_RDY        : 1;
        uint32_t PLL_I2SON      : 1;
        uint32_t PLL_I2SRDY     : 1;
        uint32_t Reserved2      : 4;
    };
}RCC_CReg_t;

typedef union {
	uint32_t value;
	struct {
		uint32_t GPIOAEN 		: 1;
		uint32_t GPIOBEN 		: 1;
		uint32_t GPIOCEN 		: 1;
		uint32_t GPIODEN 		: 1;
		uint32_t GPIOEEN 		: 1;
		uint32_t GPIOFEN 		: 1;
		uint32_t GPIOGEN 		: 1;
		uint32_t GPIOHEN 		: 1;
		uint32_t GPIOIEN 		: 1;
		uint32_t GPIOJEN 		: 1;
		uint32_t GPIOKEN 		: 1;
		uint32_t Reserved1 		: 3;
		uint32_t CRCEN			: 1;
		uint32_t Reserved2		: 5;
		uint32_t BKPSRAMEN		: 1;
		uint32_t Reserved3  	: 1;
		uint32_t CCMDATARAMEN	: 1;
		uint32_t DMA1EN			: 1;
		uint32_t DMA2EN			: 1;
		uint32_t DMA2DEN		: 1;
		uint32_t Reserved4		: 2;
		uint32_t ETHMACEN		: 1;
		uint32_t ETHMACTXEN		: 1;
		uint32_t ETHMACRXEN		: 1;
		uint32_t ETHMACPTPEN	: 1;
		uint32_t OTGHSEN		: 1;
		uint32_t OTGHSULPIEN	: 1;
		uint32_t Reserved5		: 1;
	};
}RCC_AHB1_PClock_Reg_t;

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

#include "stm324xx_gpio_driver.h"

#endif //STM32FXX_DRIVERS_H
