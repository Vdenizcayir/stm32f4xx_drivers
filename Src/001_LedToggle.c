/*
 * 001_LedToggle.c
 *
 *  Created on: 3 Tem 2023
 *      Author: dcayir
 */

#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0; i<5000000;i++);
}
int main(void)
{

	GPIO_Handle_t GPIOLed;
	GPIOLed.pGPIOx=GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP; 	//pull-up-pull-down -> led lights up
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;		//open drain and no-pullup-pulldown led off

	//GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;		//pull-up -> led lights up but very little

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOLed);


	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
