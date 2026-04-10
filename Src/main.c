/**********************************************************************************
* Bare-Metal USB OTG FS device Mass Storage (MSC) implementation
* ----------------------------------------------------------------
* Working example --- Nicolas Prata - 04/2026
*
* Lightweight, only two MSC files to integrate: usb_msc_fs.c and usb_msc_fs.h
* Notes:
  - You can edit the descriptors on usb_MSC_fs.h file
  - USB Slave-mode only (DMA not available on USB Full Speed with PA11 - PA12)
* - Added support for on-board QSPI Flash MT25QL128 with the FATfs library
* - Format to exFAT (32KB alloc size is faster for large transfers but for small files keep 4096KB)
* - DMA for Indirect sub-sector Read (4KB)
* - Enable Memory-Mapped mode (read only) with QSPI_Enable_MemoryMapped()
* - PA0 "Maintenance Button" enable/disable the USB to avoid conflicts (exclusive ownership)
* - Declare FATFS fs; to have a FATfs working area
*
*--------------------------------------------------------------------------------------
* For now all FATfs functions must be used along with USB status management (flags)
* see examples on ff_funct.c
*--------------------------------------------------------------------------------------
*
**********************************************************************************/
/*	WORK IN PROGRESS.
    IMPROVE TRANSFER SPEED (SLOW, ERASE TIME IS NOT REDUCIBLE, CHECK OTHER CAUSES)
    AUTOPOLLING OF ERASE = BLOCKING -> ADD INTERRUPTS
	CODE TO CLEAN AND ORGANIZE. IMPROVE ROBUSTNESS (REQUEST SENSE, "HOT-UNPLUG", etc)
	CHECK AND CREATE MORE FATfs USER FUNCTIONS ON ff_funct.c
*/

#include <stdio.h>
#include "main.h"
#include "timers.h"
#include "uart3.h"
#include "usb_msc_fs.h"
#include "qspi.h"
#include "diskio.h" 	// FatFs driver wrappers
#include "ff_func.h" 	// a few homemade FATfs functions


uint32_t uid;
uint8_t flag = 0;

/* If you want to use FATfs, you need to register the work area of the volume */
FATFS fs; // Pointer to the filesystem object (The "Drive" instance)

volatile uint32_t TicksMs = 0;    // Stores the timestamp of the initial push
volatile uint8_t  was_long_press = 0;
volatile uint8_t pushed = 0;      // Tracks the physical state (1=Down, 0=Up)
// Memory-Mapped Start address: 0x90000000

int main (void)
{
	/** Initialization functions **/
	activateFPU();
	SysClockConfig();
	GPIO_Config();
	InterruptGPIO_Config();
	ITM_Init();
	GPIOD->ODR^=GPIO_ODR_OD4; //orange
	GPIOD->ODR^=GPIO_ODR_OD5; // red
	GPIOG->ODR^=GPIO_ODR_OD6; // green
	if(USB_OTG_FS_Init() != EP_OK) return -1;	// Initialize RCC, GPIO, Clocks, Registers configuration
	QSPI_Hardware_Init();     // 2. Now that the "heartbeat" is stable at 180MHz...
	QSPI_Flash_Reset();

	BareM_StatusTypeDef Ut_s = Uart3_Init(&huart3, 115200); // Init UART3
	while(Ut_s != Bare_OK);

	uid = MT25Q_readID();
	if(uid != DEVICE_ID) return -1; // don't start if the correct Flash ID isn't read
	QSPI_EnableQuadMode(); // comment to stay on SPI extended (1-1-1)

	USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;  // disable USB at startup - to enable press button PA0

	NBdelay_ms(50); // safety delay
	QSPI_Enable_MemoryMapped(); //  Ensure QSPI_Enable_MemoryMapped() has been called before the first USB Read request arrives
	FRESULT res;  // Check if operations succeed
	res = f_mount(&fs, "0:", 1); //  "": Defaut Drive (number 0) ; 1: Forced mount (checks for FAT structure immediately)
	if (res != FR_OK) {	GPIOG->ODR^=GPIO_ODR_OD6; /* If res is FR_NO_FILESYSTEM*/ }
	NBdelay_ms(1000);

	//	append_to_file((char *)"example.txt", ((char *)"\r\nI wanna dance with somebody.\r\n"));
	//	disp_lines((char *)"example.txt");
	//	list_dir((char *)"number1");

	 // MT25Q_BulkErase(); // format the Flash with 0xFF (Windows formats with 0x00)

	while (1) {

		// switch USB on/off
		if (was_long_press) {
			NBdelay_ms(50);  // avoid debouncing
		    maintenance_switch();
		    // turn on / off the led according to the real state of the USB
		    if(USB_OTG_DEVICE->DCTL & USB_OTG_DCTL_SDIS) GPIOG->BSRR = GPIO_BSRR_BS6;
		    else GPIOG->BSRR = GPIO_BSRR_BR6;
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
            uint32_t duration = GetSysTick() - TicksMs;
            // Only trigger if it was a deliberate press
            if (duration > 150) {
                was_long_press = 1;
            }
        }
    }
}
