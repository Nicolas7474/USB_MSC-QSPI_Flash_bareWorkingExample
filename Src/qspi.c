/*********************************************************/
/*   Bare-Metal USB MSC + QSPI Flash MT25QL128
 *
 * Working example -  Main Functions: 			***/
/*			*/
/* readID, MT25Q_SubsectorErase, MT25Q_SubsectorRead, MT25Q_PageProgram, MT25Q_SubsectorWrite */
/** 	***/
/*	 Enable Memory-Mapped mode (read only) with QSPI_Enable_MemoryMapped()		*/
/* 	 Nicolas Prata 2026								   ***/
/*********************************************************/

#include "qspi.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f469xx.h"
#include "timers.h"


static void QSPI_DMA_Global_Init(void);
static void QSPI_AutoPollingMode(void);

void QSPI_Hardware_Init(void) {
	/* Uses the pins PF6-PF10 and PB6 - respects the 180MHz clock tree from the USB drivers */

	// 1. Enable GPIO Clocks (Port B and Port F)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOFEN;

	// 2. Enable QUADSPI Clock (AHB3 Bus)
	RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;

	// --- GPIO Port B Configuration (NCS) ---
	// PB6: Alternate Function (AF10), Very High Speed
	GPIOB->MODER   &= ~(GPIO_MODER_MODER6);
	GPIOB->MODER   |=  (2U << GPIO_MODER_MODER6_Pos);  // AF Mode
	GPIOB->OSPEEDR |=  (3U << GPIO_MODER_MODER6_Pos);  // Very High Speed
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPDR6);            // No pull (R55 10K is external)

	GPIOB->AFR[0]  &= ~(0xFU << (6 * 4));
	GPIOB->AFR[0]  |=  (10U << (6 * 4));               // PB6 -> AF10

	// --- GPIO Port F Configuration (CLK, IO0, IO1, IO2, IO3) ---
	// PF6, PF7, PF8, PF9, PF10: Alternate Function, Very High Speed
	GPIOF->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7 |
			GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
	GPIOF->MODER |=  (2U << GPIO_MODER_MODER6_Pos)  | (2U << GPIO_MODER_MODER7_Pos) |
			(2U << GPIO_MODER_MODER8_Pos)  | (2U << GPIO_MODER_MODER9_Pos) |
			(2U << GPIO_MODER_MODER10_Pos);

	GPIOF->OSPEEDR |= (3U << GPIO_MODER_MODER6_Pos)  | (3U << GPIO_MODER_MODER7_Pos) |
			(3U << GPIO_MODER_MODER8_Pos)  | (3U << GPIO_MODER_MODER9_Pos) |
			(3U << GPIO_MODER_MODER10_Pos);

	// --- Alternate Function Mapping for Port F ---
	// PF6, PF7, PF10 -> AF9  |  PF8, PF9 -> AF10

	// AFR[0] (Pins 0-7)
	GPIOF->AFR[0] &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
	GPIOF->AFR[0] |=  ((9U << (6 * 4)) | (9U << (7 * 4)));

	// AFR[1] (Pins 8-15) -  PS: The STM32 handles the NCS pin (PB6) automatically !
	GPIOF->AFR[1] &= ~((0xFU << (0 * 4)) | (0xFU << (1 * 4)) | (0xFU << (2 * 4)));
	GPIOF->AFR[1] |=  ((10U << (0 * 4)) | (10U << (1 * 4)) | (9U << (2 * 4)));

	// 3. Reset the QUADSPI memory controller to ensure a clean state
	RCC->AHB3RSTR |=  RCC_AHB3RSTR_QSPIRST;
	RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST;

	// 4. Peripheral Configuration (Device Configuration Register)
	// FSIZE: Flash size is 128Mbits = 16MBytes -  Value in register is N where 2^(N+1) = bytes -> 2^(23+1) = 16MB
	// CSHT: NCS (chip select) stays high for at least 2 cycles between flash memory commands
	QUADSPI->DCR = (23U << QUADSPI_DCR_FSIZE_Pos) | (1U << QUADSPI_DCR_CSHT_Pos); // with N = (log(Flash Size in Bytes)/log(2))-1

	// PRESCALER = 1 -> Clock = 180MHz / (1+2) = 60MHz / Clock frequency 133 MHz (MAX) for all protocols in STR
	// SSHIFT = 1    -> Sample on the half-cycle (Critical for 90MHz) - allows the data to be sampled later
	QUADSPI->CR = (2U << QUADSPI_CR_PRESCALER_Pos) | QUADSPI_CR_SSHIFT | QUADSPI_CR_EN; // CR Configuration

	QSPI_DMA_Global_Init(); // Call to configure DMA
}


static void QSPI_DMA_Global_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; 			 // Enable DMA2 Clock
	DMA2_Stream7->CR = (3U << DMA_SxCR_CHSEL_Pos)  | // QUADSPI is mapped to DMA2, Stream 7, Channel 3.
			(0U << DMA_SxCR_MBURST_Pos) | // Incremental burst: single transfer (Memory)
			(0U << DMA_SxCR_PBURST_Pos) | // Incremental burst: single transfer (Peripheral)
			(1U << DMA_SxCR_PL_Pos)     | // Mid Priority
			(0U << DMA_SxCR_MSIZE_Pos)  | // Memory size 8-bit (a byte)
			(0U << DMA_SxCR_PSIZE_Pos)  | // Peripheral size 8-bit
			DMA_SxCR_MINC               | // Increment RAM address
			DMA_SxCR_TCIE;                // Transfer Complete Interrupt Enable

	DMA2_Stream7->FCR = DMA_SxFCR_DMDIS | (3U << DMA_SxFCR_FTH_Pos); // FIFO Direct mode disabled / FIFO threshold: Full
	DMA2_Stream7->PAR = (uint32_t)&QUADSPI->DR;  // Set Peripheral Address (QUADSPI DR)
}


void QSPI_Flash_Reset(void) {
	// --- Step 1: Try resetting in Quad Mode (just in case it's stuck there) ---
	QUADSPI->CCR = (3U << QUADSPI_CCR_IMODE_Pos) | (0x66U << QUADSPI_CCR_INSTRUCTION_Pos);
	for (volatile uint32_t i = 0; i < 2000; i++) { __NOP(); }
	QUADSPI->CCR = (3U << QUADSPI_CCR_IMODE_Pos) | (0x99U << QUADSPI_CCR_INSTRUCTION_Pos);
	for (volatile uint32_t i = 0; i < 18000; i++) { __NOP(); }
	// --- Step 2: Try resetting in 1-line Mode (Standard) ---
	QUADSPI->CCR = (1U << QUADSPI_CCR_IMODE_Pos) | (0x66U << QUADSPI_CCR_INSTRUCTION_Pos);
	while (QUADSPI->SR & QUADSPI_SR_BUSY);
	QUADSPI->CCR = (1U << QUADSPI_CCR_IMODE_Pos) | (0x99U << QUADSPI_CCR_INSTRUCTION_Pos);
	// Wait for the Flash to re-initialize internally
	for (volatile uint32_t i = 0; i < 18000; i++) { __NOP(); } // at least 1ms
	QUADSPI->FCR = 0x1F; // Clear all flags
}

void QSPI_Enable_MemoryMapped(void) {

	if ((QUADSPI->CCR & QUADSPI_CCR_FMODE_Msk) == (3U << QUADSPI_CCR_FMODE_Pos)) return; // early exit if already in MMM (otherwise it hangs)

	QSPI_WaitUntilReady(); // Poll the Flash until status is no longer "write in progress"

	// 2. Configure the communication parameters
	QUADSPI->CCR = (3U  << QUADSPI_CCR_FMODE_Pos)  | // MEMORY MAPPED MODE
			(10U << QUADSPI_CCR_DCYC_Pos)   | // 10 Dummy Cycles
			(2U  << QUADSPI_CCR_ADSIZE_Pos) | // 24-bit address
			(3U  << QUADSPI_CCR_DMODE_Pos)  | // Data 4 lines
			(3U  << QUADSPI_CCR_ADMODE_Pos) | // Address 4 lines
			(3U  << QUADSPI_CCR_IMODE_Pos)  | // Instruction 4 lines
			(0x6BU << QUADSPI_CCR_INSTRUCTION_Pos); // Opcode 0x6B (QUAD OUTPUT FAST READ)
	/*   Once the QSPI is in Memory-Mapped mode, the QUADSPI->DR register is no longer used,
		and you cannot send manual commands like readID() without first switching FMODE back to 0 or 1 */
}

void QSPI_Prepare_Indirect(void) {
	// If in Memory-Mapped Mode, we must force an Abort
	if ((QUADSPI->CCR & QUADSPI_CCR_FMODE_Msk) == (3U << QUADSPI_CCR_FMODE_Pos)) {
		QUADSPI->CR |= QUADSPI_CR_ABORT;
		while (QUADSPI->SR & QUADSPI_SR_BUSY); // Must wait after an Abort TODO add a tim
	}
	// Clear all status flags (Transfer Complete, FIFO Threshold, etc.)
	QUADSPI->FCR = QUADSPI_FCR_CTCF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTEF;
}


void QSPI_WriteEnable(void) {

	// IMODE = 3 (Quad), Instruction = 0x06 (MT25QL128 write enable)
	QUADSPI->CCR = (3U << QUADSPI_CCR_IMODE_Pos) | (0x06U << QUADSPI_CCR_INSTRUCTION_Pos); // All other phases (Address, Data, etc.) are 0

	// Wait for the instruction to be sent (Transfer complete flag)
	while (!(QUADSPI->SR & QUADSPI_SR_TCF)); // the TCF bit is set in indirect mode when the programmed number of data is transferred

	QUADSPI->FCR = QUADSPI_FCR_CTCF;  // Clear transfer complete flag (Flag Clear Register)
}


void QSPI_EnableQuadMode(void) {

	/*First, write enable in 1-Wire mode */
	while (QUADSPI->SR & QUADSPI_SR_BUSY);
	QUADSPI->CCR = (1U << QUADSPI_CCR_IMODE_Pos) | (0x06U << QUADSPI_CCR_INSTRUCTION_Pos);
	while (!(QUADSPI->SR & QUADSPI_SR_TCF)); // Wait for the instruction to be sent
	QUADSPI->FCR = QUADSPI_FCR_CTCF;  // Clear Transfer Complete Flag
	while (QUADSPI->SR & QUADSPI_SR_BUSY);

	QUADSPI->DLR = 1 - 1; 	// Sending 1 byte of data (data length register)

	// Configure CCR
	QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos) |	 // FMODE = 0 (Indirect Write)
			(1U << QUADSPI_CCR_DMODE_Pos) |  		 // DMODE = 1 (Data on 1 line)
			(1U << QUADSPI_CCR_IMODE_Pos) | 		 // IMODE = 1 (Instruction on 1 line)
			(0x61U << QUADSPI_CCR_INSTRUCTION_Pos);  // 0x61 = Enhanced Volatile Configuration Register

	// write 0x7F (Quad I/O Protocol Enable) to the MT25QL128 "Enhanced Volatile Configuration Register"
	QUADSPI->DR = 0x7F;  // This starts the transmission

	while (!(QUADSPI->SR & QUADSPI_SR_TCF)); // Wait for the PERIPHERAL to finish the SPI transaction
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	QSPI_WaitUntilReady(); // Wait for the FLASH CHIP to finish its internal write cycle
	/* Once the QSPI is configured in Quad mode, use value (3) in QUADSPI_CCR for DMODE, ADMODE and IMODE */
}


uint8_t QSPI_GetStatus(void) {

	uint32_t timeout = 0;
	while (QUADSPI->SR & QUADSPI_SR_BUSY)  {
		timeout++;
		if (timeout > 500000) return -1;
	}

	QUADSPI->DLR = 1 - 1; // Expecting 1 byte
	QUADSPI->CCR = (1U << QUADSPI_CCR_FMODE_Pos) | // Functional mode: Indirect-read mode
			(3U << QUADSPI_CCR_DMODE_Pos) | 	   // We are in Quad mode
			(3U << QUADSPI_CCR_IMODE_Pos) |
			(0x05U << QUADSPI_CCR_INSTRUCTION_Pos); // Read MT25QL128 Status Register (can be read continuously and at any time)

	while (!(QUADSPI->SR & QUADSPI_SR_TCF));
	uint8_t status = (uint8_t)(QUADSPI->DR);
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	return status;
}

void QSPI_WaitUntilReady(void) {
	// for MT25Q_SubsectorErase_4KB() env. 30ms of polling (0x05)
	// Bit 0 of Status Register is WIP (Write In Progress)
	while (QSPI_GetStatus() & 0x01); // bit 1 = "Write in progress" of MT25QL128 "Read Status Register"
}


uint32_t readID(void) {
	// Manufacturer ID=0x20 JEDEC, Memory type=0xBA (3V), Memory capacity=18h (128Mb)
	QSPI_Prepare_Indirect();
	uint32_t id = 0;
	QUADSPI->DLR = 3 - 1; // Expecting 3 bytes
	// Important: READ ID does not support Full Quand SPI Mode
	QUADSPI->CCR = (1U << QUADSPI_CCR_FMODE_Pos) |	// Functional mode: Indirect-READ mode
			(1U << QUADSPI_CCR_DMODE_Pos) | // Data 1 line
			(1U << QUADSPI_CCR_IMODE_Pos) | // Inst 1 line: send the Command on 1 wire
			(0x9E << QUADSPI_CCR_INSTRUCTION_Pos); // Read MT25QL128 Status Register (can be read continuously and at any time)

	while (!(QUADSPI->SR & QUADSPI_SR_TCF));
	id = QUADSPI->DR;
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	return id & 0x00FFFFFF; // Mask to 24 bits
}


void MT25Q_SubsectorRead(uint32_t address, uint8_t *rData) {
	// READ 4KB
	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"

	// Set up DMA for Memory-to-Peripheral
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;            // Disable DMA
	while(DMA2_Stream7->CR & DMA_SxCR_EN);       // Wait for hardware to release
	DMA2->HIFCR = 0x0F800000; // Clear all flags
	DMA2_Stream7->M0AR = (uint32_t)rData;        // Set the Rx buffer
	DMA2_Stream7->NDTR = 4096;                   // (4096 bytes / 4) transfers
	// DIR = 00 (Peripheral-to-Memory), MINC = 1, CHSEL = 3
	DMA2_Stream7->CR = (3U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC;
	DMA2_Stream7->FCR = 0; // Direct Mode
	DMA2_Stream7->CR &= ~DMA_SxCR_DIR_Msk;       // Set DIR to 00 (Peripheral-to-Memory)
	DMA2_Stream7->CR |= DMA_SxCR_EN;             // Enable DMA

	QUADSPI->DLR = 4096 - 1; // data length register (read 4KB)
	QUADSPI->FCR = 0x1F;

	QUADSPI->CCR = (1U << QUADSPI_CCR_FMODE_Pos) |	// Configure QSPI for Indirect Read // works with 1 line
			(10U << QUADSPI_CCR_DCYC_Pos)  |  // Dummy bytes - must comply with the Datasheet
			(2U << QUADSPI_CCR_ADSIZE_Pos) |
			(3U << QUADSPI_CCR_DMODE_Pos)  | // must 3
			(3U << QUADSPI_CCR_ADMODE_Pos) | // must 3
			(3U << QUADSPI_CCR_IMODE_Pos)  | // must 3
			(0x6BU << QUADSPI_CCR_INSTRUCTION_Pos); // FAST READ

	QUADSPI->CR |= QUADSPI_CR_DMAEN;   // Enable DMA mode in QSPI
	QUADSPI->AR = address;  // Trigger start

	while (!(DMA2->HISR & DMA_HISR_TCIF7)); // Wait for DMA to finish (or use an IRQ)

	// Clean up
	DMA2->HIFCR = DMA_HIFCR_CTCIF7; // Stream 7: clear transfer complete interrupt flag
	while (QUADSPI->SR & QUADSPI_SR_BUSY);
	QUADSPI->CR &= ~QUADSPI_CR_DMAEN;
	QUADSPI->FCR = QUADSPI_FCR_CTCF; // CTCF: Clear transfer complete flag
}


void MT25Q_SubsectorErase_4KB(uint32_t address) {

	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"
	QSPI_WriteEnable();  // Write Enable (Required before every Erase/Program)

	// 1. Clear everything to prevent leftover state
	QUADSPI->DLR = 0;   // No data phase
	QUADSPI->ABR = 0;   // Explicitly disable Alternate Bytes
	QUADSPI->FCR = 0x1F; // Clear all status flags

	QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos)  | // Indirect Write
			(2U << QUADSPI_CCR_ADSIZE_Pos) |   // 4-4-4 mode: IMODE=3, ADMODE=3, ADSIZE=2
			(0U << QUADSPI_CCR_DMODE_Pos)  | // NO DATA
			(3U << QUADSPI_CCR_ADMODE_Pos) |
			(3U << QUADSPI_CCR_IMODE_Pos)  |
			(0x20U << QUADSPI_CCR_INSTRUCTION_Pos);  // Send Subsector Erase (0x20)

	QUADSPI->AR = address; // Trigger start

	while (QUADSPI->SR & QUADSPI_SR_BUSY);

	QSPI_AutoPollingMode(); // BLOCKING 30ms !! TO IMPROVE (addinterrupt/ or a queue logic ??)
}

void MT25Q_SubsectorErase_32KB (uint32_t address) {

	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"
	QSPI_WriteEnable();  // Write Enable (Required before every Erase/Program)

	// 1. Clear everything to prevent leftover state
	QUADSPI->DLR = 0;   // No data phase
	QUADSPI->ABR = 0;   // Explicitly disable Alternate Bytes
	QUADSPI->FCR = 0x1F; // Clear all status flags

	QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos)  | // Indirect Write
			(2U << QUADSPI_CCR_ADSIZE_Pos) |   // 4-4-4 mode
			(0U << QUADSPI_CCR_DMODE_Pos)  | // NO DATA
			(3U << QUADSPI_CCR_ADMODE_Pos) |
			(3U << QUADSPI_CCR_IMODE_Pos)  |
			(0x52U << QUADSPI_CCR_INSTRUCTION_Pos);  // Send 32KB Subsector Erase

	QUADSPI->AR = address; // Trigger start

	while (QUADSPI->SR & QUADSPI_SR_BUSY);

	QSPI_AutoPollingMode(); // Long BLOCKING !! TO IMPROVE (addinterrupt/ or a queue logic ??)
}

void MT25Q_SectorErase_64KB (uint32_t address) {

	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"
	QSPI_WriteEnable();  // Write Enable (Required before every Erase/Program)

	// 1. Clear everything to prevent leftover state
	QUADSPI->DLR = 0;   // No data phase
	QUADSPI->ABR = 0;   // Explicitly disable Alternate Bytes
	QUADSPI->FCR = 0x1F; // Clear all status flags

	QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos)  | // Indirect Write
			(2U << QUADSPI_CCR_ADSIZE_Pos) |   // 4-4-4 mode: IMODE=3, ADMODE=3, ADSIZE=2
			(0U << QUADSPI_CCR_DMODE_Pos)  | // NO DATA
			(3U << QUADSPI_CCR_ADMODE_Pos) |
			(3U << QUADSPI_CCR_IMODE_Pos)  |
			(0xD8U << QUADSPI_CCR_INSTRUCTION_Pos);  // Send 64KB Subsector Erase

	QUADSPI->AR = address; // Trigger start

	while (QUADSPI->SR & QUADSPI_SR_BUSY);
	QSPI_AutoPollingMode(); // BLOCKING 120ms !! TO IMPROVE (addinterrupt/ or a queue logic ??)
}


void MT25Q_BulkErase(void) {
	// BULK ERASE - Sets the device bits to FFh.
	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"
	QSPI_WriteEnable();  // Write Enable (Required before every Erase/Program)

	// 1. Clear everything to prevent leftover state
	QUADSPI->DLR = 0;   // No data phase
	QUADSPI->ABR = 0;   // Explicitly disable Alternate Bytes
	QUADSPI->FCR = 0x1F; // Clear all status flags

	QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos)  | // Indirect Write
			(0U << QUADSPI_CCR_ADSIZE_Pos) |   // 4-4-4 mode
			(0U << QUADSPI_CCR_DMODE_Pos)  | // NO DATA
			(0U << QUADSPI_CCR_ADMODE_Pos) |
			(3U << QUADSPI_CCR_IMODE_Pos)  |
			(0xC7U << QUADSPI_CCR_INSTRUCTION_Pos);  // Bulk Erase

			while (QUADSPI->SR & QUADSPI_SR_BUSY);
			//QSPI_AutoPollingMode(); // 38s < BLOCKING < 114s !! TO IMPROVE (addinterrupt/ or a queue logic ??)
}


void MT25Q_PageProgram(uint32_t address, uint8_t *pData) {

	/* DMA cannot be used with Quad-SPI page programming.Even if you try single-line DMA with Quad mode enabled,
	the QSPI controller won’t generate the proper signals for the MT25QL flash — hence the reads of 0x0/0x1. */
	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady();
	QSPI_WriteEnable();

	// 1. Configure the Data Length (256 bytes)
	QUADSPI->DLR = 256 - 1;

	// 2. Configure CCR for Indirect Write (FMODE = 00)
	// Using 1-1-1 (Instruction-Address-Data all on 1 line)
	QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos)  |
			(2U << QUADSPI_CCR_ADSIZE_Pos) | // 24-bit address
			(3U << QUADSPI_CCR_IMODE_Pos)  | // Instruction on 1 line
			(3U << QUADSPI_CCR_ADMODE_Pos) | // Address on 1 line
			(3U << QUADSPI_CCR_DMODE_Pos)  | // Data on 1 line
			(0x32U << QUADSPI_CCR_INSTRUCTION_Pos);

	// 3. Set the Address (This triggers the start of the communication)
	QUADSPI->AR = address;

	uint32_t i = 0;
	while (i < 256)
	{
		// Wait until FIFO has space
		while (!(QUADSPI->SR & QUADSPI_SR_FTF));
		//Use FIFO threshold flag (FTF) instead of TCF - fill FIFO continuously, wait only at the end for TCF
		// Faster: → Write 4 bytes at once instead of 1
		// Buffer is aligned and has a fixed-size multiple of 4 = no extra bytes -> all ok
		*(volatile uint32_t*)&QUADSPI->DR = *(uint32_t*)&pData[i];
		i += 4;
	}

	// 5. Wait for the Transfer to be physically complete on the pins
	while (!(QUADSPI->SR & QUADSPI_SR_TCF));

	// Clear Transfer Complete Flag
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	// 6. Final Busy check
	//  while (QUADSPI->SR & QUADSPI_SR_BUSY);

	// Crucial: Wait for the Flash to actually finish burning the bits
	QSPI_WaitUntilReady();
}



void MT25Q_SubsectorWrite_4KB(uint32_t address, uint8_t *data) {

	for(int i=0; i < 16; i++) {
		MT25Q_PageProgram(address, &data[i*256]); //  256 bytes per page
		address += 256;
	}
}




static void QSPI_AutoPollingMode(void) {
	// Use Automatic Status-polling Mode only inside Erase functions (big process time)

	// 1. Set the match value and mask
	QUADSPI->PSMAR = 0x00; // We want the bit to be 0
	QUADSPI->PSMKR = 0x01; // We only care about Bit 0 (WIP)

	// 2. Set polling interval (how often the hardware checks)
	// This value defines how many CLK cycles to wait between polls
	QUADSPI->PIR = 0x4000; // T = 1/(QSPI clock /  QUADSPI->PIR)

	// 3. Configure the Control Register (CR)
	// PMM = 0 (Bit 23): Match is found if ANY polling read matches
	// APMS = 1 (Bit 22): STOP polling as soon as a match is found (saves power/bus)
	// SMIE = 1 (Bit 19): Interrupt Enable - trigger the IRQ when SMF is set
	QUADSPI->CR |= QUADSPI_CR_APMS | QUADSPI_CR_SMIE;

	// 3. Configure CCR for Auto-Polling
	QUADSPI->CCR = (2U << QUADSPI_CCR_FMODE_Pos) | // 10: Automatic polling mode
			(3U << QUADSPI_CCR_DMODE_Pos) | // Quad Data
			(3U << QUADSPI_CCR_IMODE_Pos) | // Quad Instruction
			(0x05U << QUADSPI_CCR_INSTRUCTION_Pos);

	// 4. Wait for SMF (Status Match Flag) or enable the interrupt
	while (!(QUADSPI->SR & QUADSPI_SR_SMF));

	// 5. Clear flag and move on
	QUADSPI->FCR = QUADSPI_FCR_CSMF;
}


