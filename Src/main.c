/*********************************************************/
/*   Bare-Metal USB MSC + QSPI Flash MT25QL128
 *
 * Working example 			***/
/*Main Functions: 				*/
/* readID, MT25Q_SubsectorErase, MT25Q_SubsectorRead, MT25Q_PageProgram, MT25Q_SubsectorWrite */
/**  DMA for Indirect sub-sector Read (4KB) 	***/
/*	 Enable Memory-Mapped mode (read only) with QSPI_Enable_MemoryMapped()		*/
/* 	 Nicolas Prata 2026								   ***/
/*********************************************************/

#include "stm32f469xx.h"
#include "main.h"
#include "usb_msc_fs.h"
#include "qspi.h"
#include "myConfig.h"
#include "timers.h"

uint32_t first_byte;
uint32_t uid;

uint8_t sector[4096] __attribute__((aligned(4))); // remove volatile
uint8_t readsect[4096];


int main (void)
{
	/** Initialization functions **/
	SysClockConfig();
	GPIO_Config();
	InterruptGPIO_Config();
	GPIOD->ODR^=GPIO_ODR_OD4;
	GPIOD->ODR^=GPIO_ODR_OD5;
	Delay_us_TIM7(1000);
	if(USB_OTG_FS_Init() != EP_OK) return -1;	// Initialize RCC, GPIO, Clocks, Registers configuration
	QSPI_Hardware_Init();     // 2. Now that the "heartbeat" is stable at 180MHz...
	QSPI_Flash_Reset();

	uid = readID();
	if(uid != 0x18BA20) return -1; // don't start if the correct Flash ID isn't read

	QSPI_EnableQuadMode(); // comment to stay on SPI extended (1-1-1)
	//	uint8_t *flash_ptr = (uint8_t *)0x90000000;	//	first_byte = flash_ptr[0];

	//for (uint16_t i = 0; i < 4096; i++) { sector[i] = (uint8_t)((i % 94) + 32); }

	//MT25Q_SubsectorErase(0x0); // instruction 0x20 Erase at address 0x0 of the MT25QL128
//	Delay_us_TIM7(20); // juste pour separer visuellement//
//	MT25Q_SubsectorWrite(0x0, sector);//
//	Delay_us_TIM7(10);
//	MT25Q_SubsectorRead(0x0, readsect);

	//  Ensure QSPI_Enable_MemoryMapped() has been called before the first USB Read request arrives.
	QSPI_Enable_MemoryMapped();

	while (1) {

	}
}


