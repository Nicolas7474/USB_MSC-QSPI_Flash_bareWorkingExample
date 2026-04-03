/*********************************************************/
/***  	 Bare-Metal USB MSC - Working example 	    *****/
/* 													   */
/******************************************************/

#include "stm32f469xx.h"
#include "main.h"
#include "usb_msc_fs.h"
#include "qspi.h"
#include "myConfig.h"
#include "timers.h"

uint32_t first_byte;
uint32_t uid;
uint32_t sector[1024];
uint32_t readsect[1024];

int main (void)
{
	/** Initialization functions **/
	SysClockConfig();
	GPIO_Config();
	InterruptGPIO_Config();
	GPIOD->ODR^=GPIO_ODR_OD4;
	GPIOD->ODR^=GPIO_ODR_OD5;
	if(USB_OTG_FS_Init() != EP_OK) return -1;	// Initialize RCC, GPIO, Clocks, Registers configuration
	QSPI_Hardware_Init();     // 2. Now that the "heartbeat" is stable at 180MHz...
	QSPI_Flash_Reset();
	QSPI_EnableQuadMode();
	QSPI_DMA_Global_Init();
	QSPI_Enable_MemoryMapped();

//	uid = readID();
//	if(uid != 0x18BA20) return -1; // don't start if the correct Flash ID isn't read

//
//
//	uint8_t *flash_ptr = (uint8_t *)0x90000000;
//	first_byte = flash_ptr[0];



	//  Ensure QSPI_Enable_MemoryMapped() has been called before the first USB Read request arrives.
	while (1) {

	}
}


