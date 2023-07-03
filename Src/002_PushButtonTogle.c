/*
 * 002_PushButtonTogle.c
 *
 *  Created on: 3 Tem 2023
 *      Author: DENIZ
 */

#include "stm32f407xx_gpio_driver.h"

#define HIGH ENABLE
#define BTN_PRESSED HIGH

int main(void)
{

	GPIO_Handle_t GPIOButton, GPIOLed;
	GPIOButton.pGPIOx=GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_INPUT;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	//GPIOButton.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP; // bu satır mode output olduğu zaman geçerlidir.
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD; // schematic de button için pullup-down ayarlı olduğu için burada gerek yok.
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOButton);


	GPIOLed.pGPIOx=GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOLed);
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)== BTN_PRESSED)
			{
				GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			}
	}

	return 0;
}
