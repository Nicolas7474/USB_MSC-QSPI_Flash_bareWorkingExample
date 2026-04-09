/*********************************************************/
/* Bare-Metal USB MSC + QSPI Flash MT25QL128
 *
 * Working example 				*/
/* Main Functions: 				*/
/* readID, MT25Q_SubsectorErase, MT25Q_SubsectorRead, MT25Q_PageProgram, MT25Q_SubsectorWrite */
/* DMA for Indirect sub-sector Read (4KB) 	***/
/* Enable Memory-Mapped mode (read only) with QSPI_Enable_MemoryMapped()		*/
/* Nicolas Prata 2026								   ***/
/*********************************************************/

#include <stdio.h>
#include "stm32f469xx.h"
#include "main.h"
#include "usb_msc_fs.h"
#include "qspi.h"
#include "myConfig.h"
#include "timers.h"
#include "diskio.h" 	// FatFs driver wrappers
#include "uart3.h"
#include "ff_func.h" 	// examples of FATfs functions

uint32_t first_byte;
uint32_t uid;
uint8_t flag = 0;
//uint8_t sector[4096] __attribute__((aligned(4)));

/***** FATfs *****/
FATFS fs;        	  // Pointer to the filesystem object (The "Drive" instance)
FRESULT res;         // Check if operations succeed
//FIL fil;           // File object (The "Open File" instance)
//UINT br;           // Store how many bytes were actually read


int main (void)
{
	/** Initialization functions **/
	activateFPU();
	SysClockConfig();
	GPIO_Config();
	InterruptGPIO_Config();
	GPIOD->ODR^=GPIO_ODR_OD4;
	GPIOD->ODR^=GPIO_ODR_OD5;
	GPIOG->ODR^=GPIO_ODR_OD6; // green
	if(USB_OTG_FS_Init() != EP_OK) return -1;	// Initialize RCC, GPIO, Clocks, Registers configuration
	QSPI_Hardware_Init();     // 2. Now that the "heartbeat" is stable at 180MHz...
	QSPI_Flash_Reset();

	BareM_StatusTypeDef Ut_s = Uart3_Init(&huart3, 115200);
	while(Ut_s != Bare_OK);

	uid = readID();
	if(uid != 0x18BA20) return -1; // don't start if the correct Flash ID isn't read
	QSPI_EnableQuadMode(); // comment to stay on SPI extended (1-1-1)

	//for (uint16_t i = 0; i < 4096; i++) { sector[i] = (uint8_t)((i % 94) + 32); }
	// 0x90000000


	QSPI_Enable_MemoryMapped(); //  Ensure QSPI_Enable_MemoryMapped() has been called before the first USB Read request arrives
	USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;  // disable USB
	NBdelay_ms(2000);


	res = f_mount(&fs, "0:", 1); //  "": Defaut Drive (number 0) ; 1: Forced mount (checks for FAT structure immediately)
	if (res != FR_OK) {	GPIOG->ODR^=GPIO_ODR_OD6; /* If res is FR_NO_FILESYSTEM*/ }
	NBdelay_ms(100);
	append_to_file((char *)"example.txt", ((char *)"\r\nI wanna dance with somebody.\r\n"));
	disp_lines((char *)"example.txt");
	list_dir((char *)"number1");


	//	res = f_mkdir("number1");
	//	if (res == FR_OK) {	GPIOD->ODR^=GPIO_ODR_OD4; }

	while (1) {

		if(flag) { 	// "USB Maintenance" Button pressed
			NBdelay_ms(50);  // avoid debouncing
			maintenance_switch();
			GPIOG->ODR^=GPIO_ODR_OD6; // green
			flag = 0;
		}

	}

}


// PAO button input interrupt handler
void EXTI0_IRQHandler() {

	if (EXTI->PR & (1<<0)) {  // button pushed : if the PA0 triggered the interrupt
		EXTI->PR |= (1<<0);  // Clear the interrupt flag by writing a 1
		flag = 1;
	}
}

