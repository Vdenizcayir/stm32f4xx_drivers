/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 22 Haz 2023
 *      Author: dcayir
 */

#include "stm32f407xx_gpio_driver.h"


/*Peripheral Clock Setup*/
//Bu fonksiyon verilen GPIO portu için çevresel saati etkinleştirir veya devre dışı bırakır
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableOrDisable)
{
	if(EnableOrDisable==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx==GPIOA)
			{
			GPIOA_PCLK_DI();
			}
			else if(pGPIOx==GPIOB)
			{
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx==GPIOC)
			{
				GPIOC_PCLK_DI();
			}
			else if(pGPIOx==GPIOD)
			{
				GPIOD_PCLK_DI();
			}
			else if(pGPIOx==GPIOE)
			{
				GPIOE_PCLK_DI();
			}
			else if(pGPIOx==GPIOF)
			{
				GPIOF_PCLK_DI();
			}
			else if(pGPIOx==GPIOG)
			{
				GPIOG_PCLK_DI();
			}
			else if(pGPIOx==GPIOH)
			{
				GPIOH_PCLK_DI();
			}
			else if(pGPIOx==GPIOI)
			{
				GPIOI_PCLK_DI();
			}
	}
}

/*Init and De-init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//1.Configure the mode of GPIO PIN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)		//Interrupt olmayan modlar üzerinde çalışır
	{
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER|=temp; //setting  //sadece ilgili registerların ilgili bitleri ile çalışma yaptığımız için direkt olarak eşittir kullanılması yanlış olur. |= sadece ilgil biti yapar

	}
	else
	{
		//Interrupt olan modlar üzerinde çalışır
	}
	temp=0;

	//2.Configure the speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 							//clearing
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;


	temp=0;
	//3.Configure the pull up pull down settings
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 							//clearing
	pGPIOHandle->pGPIOx->PUPDR|=temp;

	temp=0;
	//4.Configure the optype
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 								//clearing
	pGPIOHandle->pGPIOx->OTYPER|=temp;

	temp=0;
	//5.Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
	{
		//configure the alt functionality re gister
		uint8_t temp1;
		uint8_t temp2;

		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFRL[temp1] &= ~(15 << (4*temp2));															//clearing
		pGPIOHandle->pGPIOx->AFRL[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2);

	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)  //bir çevre birimini sıfırlamak için RCC registerına bak
{



				if(pGPIOx==GPIOA)
				{
					GPIOA_REG_RESET();
				}
				else if(pGPIOx==GPIOB)
				{
					GPIOB_REG_RESET();
				}
				else if(pGPIOx==GPIOC)
				{
					GPIOC_REG_RESET();
				}
				else if(pGPIOx==GPIOD)
				{
					GPIOD_REG_RESET();
				}
				else if(pGPIOx==GPIOE)
				{
					GPIOE_REG_RESET();
				}
				else if(pGPIOx==GPIOF)
				{
					GPIOF_REG_RESET();
				}
				else if(pGPIOx==GPIOG)
				{
					GPIOG_REG_RESET();
				}
				else if(pGPIOx==GPIOH)
				{
					GPIOH_REG_RESET();
				}
				else if(pGPIOx==GPIOI)
				{
					GPIOI_REG_RESET();
				}
}

/*Data read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR ;
	return value;
}
void GPIO_WriteToOutpuPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &=  ~(1 << PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR=value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR =  pGPIOx->ODR ^ (1<<PinNumber);    //XOR işlemi uygulandı
}

/* IRQ Configuration and ISR handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis)
{

}
void GPIO_IRQHandling(uint8_t PinNumber){}
