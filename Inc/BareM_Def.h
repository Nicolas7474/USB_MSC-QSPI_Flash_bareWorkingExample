/********************************************************************************************************
 	 	BARE-METAL COMMON PARAMETERS
 	  #INCLUDE THIS FILE INTO THE HEADER FILES WHERE BARE METAL FUNCTIONS ARE USED (I2, SPI, UART...)
// ******************************************************************************************************/

#pragma once

// This enumeration is defined to standardize return values of the Bare Metal functions
// It is based on HAL_StatusTypeDef. Each file/protocol has its own TypeDef variable.
/* Besides this, the main functions in the drivers files (I2C, SPI, Uart...) possess often their own internal state variables,
	which are defined locally in the headers. */

typedef enum
{
	Bare_OK       = 0x00U,
	Bare_ERROR    = 0x01U,
	Bare_BUSY     = 0x02U,
	Bare_TIMEOUT  = 0x03U

} BareM_StatusTypeDef;







/*
*****	HAL	  *****
*****
Here is a breakdown of the HAL_StatusTypeDef and its usage:
1. The HAL_StatusTypeDef Enum
This enumeration is defined in the stm32f4xx_hal_def.h (or equivalent for your series) file to standardize return values across all peripherals.



2. Usage in Code
When calling HAL functions (e.g., UART, I2C, SPI, GPIO), the return type is usually HAL_StatusTypeDef. You check this against HAL_OK to ensure the operation worked.

// Example: Checking UART transmission status
HAL_StatusTypeDef status;
status = HAL_UART_Transmit(&huart1, (uint8_t*)data, 10, 100);

if (status == HAL_OK) {
    // Transmission was successful
} else {
    // Handle error (HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT)
}

*****

SPI_InitTypeDef Struct Reference:
	uint32_t 	Mode
	uint32_t 	Direction
	uint32_t 	DataSize
	uint32_t 	CLKPolarity
	uint32_t 	CLKPhase
	uint32_t 	NSS
	uint32_t 	BaudRatePrescaler
	uint32_t 	FirstBit
	uint32_t 	TIMode
	uint32_t 	CRCCalculation
	uint32_t 	CRCPolynomial

 * Declare a SPI_HandleTypeDef handle structure, for example: SPI_HandleTypeDef  hspi;
_SPI_HandleTypeDef Struct Reference:
	SPI_TypeDef * 	Instance
	SPI_InitTypeDef 	Init
	uint8_t * 	pTxBuffPtr
	uint16_t 	TxXferSize
	__IO uint16_t 	TxXferCount
	uint8_t * 	pRxBuffPtr
	uint16_t 	RxXferSize
	__IO uint16_t 	RxXferCount
	void(* 	RxISR )(struct __SPI_HandleTypeDef *hspi)
	void(* 	TxISR )(struct __SPI_HandleTypeDef *hspi)
	DMA_HandleTypeDef * 	hdmatx
	DMA_HandleTypeDef * 	hdmarx
	HAL_LockTypeDef 	Lock
	__IO HAL_SPI_StateTypeDef 	State
	__IO uint32_t 	ErrorCode


	Example:

	HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;

  if (hi2c->State == HAL_I2C_STATE_READY)
  ............
  ............
      SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


*/
