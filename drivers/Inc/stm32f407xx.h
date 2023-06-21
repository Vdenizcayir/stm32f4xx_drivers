/*
 * stm32f407xx.h
 *
 *  Created on: Jun 21, 2023
 *      Author: DENIZ
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

#define __vo volatile

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
	__vo uint32_t MODER;					//GPIO port mode register
	__vo uint32_t OTYPER;				//GPIO port output type register
	__vo uint32_t OSPEEDR;				//GPIO port output speed register
	__vo uint32_t PUPDR;					//GPIO port pull-up/pull-down register
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






