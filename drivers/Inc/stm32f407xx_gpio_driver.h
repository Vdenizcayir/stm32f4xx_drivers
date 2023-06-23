/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 22 Haz 2023
 *      Author: dcayir
 */



#include "stm32f407xx.h"

typedef struct
{
	uint8_t 				GPIO_PinNumber;						// possible values from @GPIO_PIN_NUM
	uint8_t 				GPIO_PinMode;						// possible values from @GPIO_PIN_MODES
	uint8_t 				GPIO_PinSpeed;						// possible values from @GPIO_PIN_SPEED
	uint8_t 				GPIO_PinPuPdControl;				//
	uint8_t 				GPIO_PinOPType;
	uint8_t 				GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t 			*pGPIOx; 		 			//This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t 		GPIO_PinConfig;				//This holds GPIO pin configuration settings
}GPIO_Handle_t;




/* @GPIO_PIN_NUM
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15




/* @GPIO_PIN_MODES
 * GPIO pin possible modes                      //***********GPIO registerları için kullanılabilecek makrolar. bitleri değiştirmek için.
 */
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*@GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

							                     //***********GPIO registerları için kullanılabilecek makrolar. bitleri değiştirmek için.



/* APIs supported by this driver. For more information about the APIs check the function definitions*/
/*Peripheral Clock Setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableOrDisable);

/*Init and De-init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutpuPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Configuration and ISR handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void GPIO_IRQHandling(uint8_t PinNumber);


