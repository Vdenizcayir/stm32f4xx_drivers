/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 5 Tem 2023
 *      Author: dcayir
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

typedef struct
{
	 uint8_t SPI_DeviceMode;
	 uint8_t SPI_BusConfig;
	 uint8_t SPI_SclkSpeed;
	 uint8_t SPI_DFF;
	 uint8_t SPI_CPOL;
	 uint8_t SPI_CPHA;
	 uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	SPIConfig;
}SPI_Handle_t;


/*Peripheral Clock Setup*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnableOrDisable);

/*Init and De-init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data Send And Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer,uint32_t Len);





