/*********************************************************/
/* Bare-Metal USB MSC + QSPI Flash MT25QL128
 *
 * Working example 				*/
/* Main Functions: 				*/
/* readID, MT25Q_SubsectorErase, MT25Q_SubsectorRead, MT25Q_PageProgram, MT25Q_SubsectorWrite */
/* DMA for Indirect sub-sector Read (4KB) 	***/
/* Enable Memory-Mapped mode (read only) with QSPI_Enable_MemoryMapped()		*/
/* Nicolas Prata 2026	- // 0x90000000							   ***/
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
#include "ff_func.h" 	// homemage FATfs functions

uint32_t first_byte;
uint32_t uid;
uint8_t flag = 0;
FATFS fs; // Pointer to the filesystem object (The "Drive" instance)

volatile uint32_t TicksMs = 0;    // Stores the timestamp of the initial push
volatile uint8_t  was_long_press = 0;
volatile uint8_t pushed = 0;      // Tracks the physical state (1=Down, 0=Up)

int main (void)
{
	/** Initialization functions **/

	activateFPU();
	SysClockConfig();
	GPIO_Config();
	InterruptGPIO_Config();
	GPIOD->ODR^=GPIO_ODR_OD4; //orange
	GPIOD->ODR^=GPIO_ODR_OD5; // red
	GPIOG->ODR^=GPIO_ODR_OD6; // green
	if(USB_OTG_FS_Init() != EP_OK) return -1;	// Initialize RCC, GPIO, Clocks, Registers configuration
	QSPI_Hardware_Init();     // 2. Now that the "heartbeat" is stable at 180MHz...
	QSPI_Flash_Reset();

	BareM_StatusTypeDef Ut_s = Uart3_Init(&huart3, 115200);
	while(Ut_s != Bare_OK);

	uid = readID();
	if(uid != 0x18BA20) return -1; // don't start if the correct Flash ID isn't read
	QSPI_EnableQuadMode(); // comment to stay on SPI extended (1-1-1)
	QSPI_Enable_MemoryMapped(); //  Ensure QSPI_Enable_MemoryMapped() has been called before the first USB Read request arrives

	USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;  // disable USB at startup - to enable press button PA0

	NBdelay_ms(1500);

	FRESULT res;  // Check if operations succeed
	res = f_mount(&fs, "0:", 1); //  "": Defaut Drive (number 0) ; 1: Forced mount (checks for FAT structure immediately)
	if (res != FR_OK) {	GPIOG->ODR^=GPIO_ODR_OD6; /* If res is FR_NO_FILESYSTEM*/ }
	NBdelay_ms(100);
//	append_to_file((char *)"example.txt", ((char *)"\r\nI wanna dance with somebody.\r\n"));
//	disp_lines((char *)"example.txt");
//	list_dir((char *)"number1");

	 // MT25Q_BulkErase();
	//	res = f_mkdir("number1");
	//	if (res == FR_OK) {	GPIOD->ODR^=GPIO_ODR_OD4; }

	while (1) {
		if (was_long_press) {
			NBdelay_ms(50);  // avoid debouncing
		    maintenance_switch();
		    was_long_press = 0; // Reset for next time
		}
	}
}


// ISR PAO button input - both edges detection activated
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {
        EXTI->PR = (1 << 0); // Clear pending bit

        // 1. Button Pushed (Rising Edge for PA0 Discovery)
        if ((GPIOA->IDR & (1 << 0)) && pushed == 0) {

            pushed = 1;
            TicksMs = GetSysTick();
        }
        // 2. Button Released (Falling Edge)
        else if (!(GPIOA->IDR & (1 << 0)) && pushed == 1) {
            pushed = 0;
            GPIOG->ODR^=GPIO_ODR_OD6; // green
            uint32_t duration = GetSysTick() - TicksMs;

            // Only trigger if it was a deliberate press (e.g., > 100ms)
            if (duration > 150) {
                was_long_press = 1;
            }
        }
    }
}
