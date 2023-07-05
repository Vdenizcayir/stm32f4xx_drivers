
#include <stdint.h>

#define __vo volatile



/*Processor Specific Details*/
/*ARM Cortex Mx Processor NVIC ISERx register Addresses*/

#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/*ARM Cortex Mx Processor NVIC ICERx register Addresses*/
#define NVIC_ICER0				((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0XE000E180C)




#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U //SRAM1 112KB. 1KB=1024BYTE 112*1024=114688 HEX KARŞILIĞI=1C000
#define SRAM2_BASEADDR			0x2001C000U
#define ROM						0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

/*
 * AHBx ve APBx Bus Peripheral base addres
 */

#define PERIPH_BASE 			0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U


/*
 * AHB1 veriyolunda asılı olan çevre birimlerinin base adresleri
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASE+0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE+0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE+0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE+0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE+0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE+0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE+0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE+0x2000)

#define RCC_BASEADDR			(AHB1PERIPH_BASE+0x3800)

/*
 * AP1 veriyoluna bağl olan çevre birimleri base adresleri
 */

#define SPI2_BASEADDR			(APB1PERIPH_BASE+0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE+0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE+0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE+0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE+0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE+0x5000)
#define I2C1_BASEADDR			(APB1PERIPH_BASE+0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE+0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE+0x5400)

/*
 * AP2 veriyoluna bağl olan çevre birimleri base adresleri
 */

#define EXTI_BASEADDR			(APB2PERIPH_BASE+0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASE+0x3000)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE+0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASE+0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE+0x1400)


/********** peripheral register definition structures **********/

typedef struct
{
	__vo uint32_t MODER;				//GPIO port mode register
	__vo uint32_t OTYPER;				//GPIO port output type register
	__vo uint32_t OSPEEDR;				//GPIO port output speed register
	__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;					//GPIO port input data register
	__vo uint32_t ODR;					//GPIO port output data register
	__vo uint32_t BSRR;					//GPIO port bit set/reset register
	__vo uint32_t LCKR;					//GPIO port configuration lock register
	__vo uint32_t AFRL[2];				//AFRL[0]=GPIO alternate function low register	AFRL[1]=GPIO alternate function high register
}GPIO_RegDef_t;


#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)

typedef struct
{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_AHB1RSTR;
	__vo uint32_t RCC_AHB2RSTR;
	__vo uint32_t RCC_AHB3RSTR;
		 uint32_t RESERVED0;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_APB2RSTR;
		 uint32_t RESERVED1[2];
	__vo uint32_t RCC_AHB1ENR;
	__vo uint32_t RCC_AHB2ENR;
	__vo uint32_t RCC_AHB3ENR;
		 uint32_t RESERVED2;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
		 uint32_t RESERVED3[2];
	__vo uint32_t RCC_AHB1LPENR;
	__vo uint32_t RCC_AHB2LPENR;
	__vo uint32_t RCC_AHB3LPENR;
		 uint32_t RESERVED4;
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t RCC_APB2LPENR;
		 uint32_t RESERVED5[2];
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
		 uint32_t RESERVED6[2];
	__vo uint32_t RCC_SSCGR;
	__vo uint32_t RCC_PLLI2SCFGR;
}RCC_RegDef_t;



typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	__vo uint32_t RESERVED[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;




/******************** Clock Enable Macros for GPIOx Peripherals ********************/

#define GPIOA_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1<<10))


/******************** Clock Enable Macros for I2Cx Peripherals ********************/

#define I2C_PCLK_EN()			(RCC->RCC_APB1ENR |= (1<<21))


/******************** Clock Enable Macros for SPIx Peripherals ********************/

#define SPI1_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()			(RCC->RCC_APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->RCC_APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<13))


/******************** Clock Enable Macros for USARTx Peripherals ********************/

#define USART1_PCLK_EN()		(RCC->RCC_APB2ENR |= (1<<4))


/******************** Clock Enable Macros for SYSCFG Peripherals ********************/

#define SYSCFG_PCLK_EN()		(RCC->RCC_APB2ENR |= (1<<14))


/*Disable Clock*/
/******************** Clock Disable Macros for GPIOx Peripherals ********************/
#define GPIOA_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1<<10))

/******************** Clock Disable Macros for I2Cx Peripherals ********************/

#define I2C_PCLK_DI()			(RCC->APB1PERIPH_BASE &= ~(1<<21))


/******************** Clock Disable Macros for SPIx Peripherals ********************/

#define SPI1_PCLK_DI()			(RCC->APB2PERIPH_BASE &= ~(1<<12))


/******************** Clock Disable Macros for USARTx Peripherals ********************/

#define USART1_PCLK_DI()		(RCC->APB2PERIPH_BASE &= ~(1<<4))


/******************** Clock Disable Macros for SYSCFG Peripherals ********************/

#define SYSCFG_PCLK_DI()		(RCC->APB2PERIPH_BASE &= ~(1<<14))


/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)  //tek bir makroda iki adet ifade eklemek için do while döngüsü kullanılır
#define GPIOB_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<3)); (RCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<4)); (RCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<5)); (RCC->RCC_AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<6)); (RCC->RCC_AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<7)); (RCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET() 				do{ (RCC->RCC_AHB1RSTR |= (1<<8)); (RCC->RCC_AHB1RSTR &= ~(1<<8));}while(0)



#define GPIO_BASEADDR_TO_CODE(x)	((x==GPIOA)?0:\
									(x==GPIOB)?1:\
									(x==GPIOC)?2:\
									(x==GPIOD)?3:\
									(x==GPIOE)?4:\
									(x==GPIOF)?5:\
									(x==GPIOG)?6:\
									(x==GPIOH)?7:0)



/*
 * IEQ Numbers of STM32F407x MCU
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

//Some Generic Macros

#define ENABLE					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
