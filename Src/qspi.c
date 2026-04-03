

#include "qspi.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f469xx.h"

static inline void QSPI_Prepare_Indirect(void);

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

	// AFR[1] (Pins 8-15)
	GPIOF->AFR[1] &= ~((0xFU << (0 * 4)) | (0xFU << (1 * 4)) | (0xFU << (2 * 4)));
	GPIOF->AFR[1] |=  ((10U << (0 * 4)) | (10U << (1 * 4)) | (9U << (2 * 4)));
	// PS: The STM32 handles the NCS pin (PB6) automatically !

	// 3. Reset the QUADSPI memory controller to ensure a clean state
	RCC->AHB3RSTR |=  RCC_AHB3RSTR_QSPIRST;
	RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST;

	// 4. Peripheral Configuration (Device Configuration Register)
	// FSIZE: Flash size is 128Mbits = 16MBytes -  Value in register is N where 2^(N+1) = bytes -> 2^(23+1) = 16MB
	// with (N=log2​(Flash Size in Bytes)−1) = (log(Flash Size in Bytes)/log(2))-1
	// CSHT: NCS (chip select) stays high for at least 2 cycles between flash memory commands
	QUADSPI->DCR = (23U << QUADSPI_DCR_FSIZE_Pos) | (1U << QUADSPI_DCR_CSHT_Pos);

	// PRESCALER = 1 -> Clock = 180MHz / (3+1) = 45MHz
	// SSHIFT = 1    -> Sample on the half-cycle (Critical for 90MHz) - allows the data to be sampled later
	// EN = 1        -> Enable peripheral
	QUADSPI->CR = (3U << QUADSPI_CR_PRESCALER_Pos) | QUADSPI_CR_SSHIFT | QUADSPI_CR_EN; // CR Configuration
}


void QSPI_DMA_Global_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable DMA2 Clock

    DMA2_Stream7->CR = (3U << DMA_SxCR_CHSEL_Pos)  | // QUADSPI is mapped to DMA2, Stream 7, Channel 3.
                       (1U << DMA_SxCR_MBURST_Pos) | // Incremental burst of 4 beats (Memory)
                       (1U << DMA_SxCR_PBURST_Pos) | // Incremental burst of 4 beats (Peripheral)
                       (2U << DMA_SxCR_PL_Pos)     | // High Priority
                       (2U << DMA_SxCR_MSIZE_Pos)  | // Memory size 32-bit (a complete word)
                       (2U << DMA_SxCR_PSIZE_Pos)  | // Peripheral size 32-bit
                       DMA_SxCR_MINC               | // Increment RAM address
    				   DMA_SxCR_TCIE;                // Transfer Complete Interrupt Enable

    DMA2_Stream7->FCR = DMA_SxFCR_DMDIS | (3U << DMA_SxFCR_FTH_Pos); // FIFO Full
    DMA2_Stream7->PAR = (uint32_t)&QUADSPI->DR;  // Set Peripheral Address (QUADSPI DR)
}


void QSPI_Flash_Reset(void) {
	// 1. Reset Enable Command (0x66)
	while (QUADSPI->SR & QUADSPI_SR_BUSY);
	// IMODE=1, Instruction=0x66, FMODE=0 (Write)
	QUADSPI->CCR = (1U << QUADSPI_CCR_IMODE_Pos) | (0x66U << QUADSPI_CCR_INSTRUCTION_Pos);
	while (!(QUADSPI->SR & QUADSPI_SR_TCF));
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	// 2. Reset Memory Command (0x99)
	while (QUADSPI->SR & QUADSPI_SR_BUSY);
	// IMODE=1, Instruction=0x99, FMODE=0 (Write)
	QUADSPI->CCR = (1U << QUADSPI_CCR_IMODE_Pos) | (0x99U << QUADSPI_CCR_INSTRUCTION_Pos);
	while (!(QUADSPI->SR & QUADSPI_SR_TCF));
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	// 3. Mandatory Delay
	// The MT25QL needs time to perform the internal reset (tRST)
	// A simple volatile loop for roughly 1-5ms
	for(volatile int i = 0; i < 0x7FFF; i++);
}

void QSPI_Enable_MemoryMapped(void) {
	// 1. Ensure any previous command is finished
	while (QUADSPI->SR & QUADSPI_SR_BUSY);

	// 2. Configure the communication parameters
	// We use Opcode 0x0B (Fast Read) for 4-4-4 mode
	// IMODE=3, ADMODE=3, DMODE=3, ADSIZE=2 (24-bit), DCYC=10, FMODE=3
	QUADSPI->CCR = (3U  << QUADSPI_CCR_FMODE_Pos)  | // MEMORY MAPPED MODE
			(10U << QUADSPI_CCR_DCYC_Pos)   | // 10 Dummy Cycles
			(2U  << QUADSPI_CCR_ADSIZE_Pos) | // 24-bit address
			(3U  << QUADSPI_CCR_DMODE_Pos)  | // Data 4 lines
			(3U  << QUADSPI_CCR_ADMODE_Pos) | // Address 4 lines
			(3U  << QUADSPI_CCR_IMODE_Pos)  | // Instruction 4 lines
			(0x0BU << QUADSPI_CCR_INSTRUCTION_Pos);

	// No need to wait for TCF here. The peripheral is now "transparent".
}

static inline void QSPI_Prepare_Indirect(void) {
    // If in Memory-Mapped Mode, we must force an Abort
    if ((QUADSPI->CCR & QUADSPI_CCR_FMODE_Msk) == (3U << QUADSPI_CCR_FMODE_Pos)) {
        QUADSPI->CR |= QUADSPI_CR_ABORT;
        while (QUADSPI->SR & QUADSPI_SR_BUSY); // Must wait after an Abort
    }
    // Clear all status flags (Transfer Complete, FIFO Threshold, etc.)
    QUADSPI->FCR = QUADSPI_FCR_CTCF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTEF;
}


void QSPI_WriteEnable(void) {
	while (QUADSPI->SR & QUADSPI_SR_BUSY); // Wait if busy

	// Instruction Only: 0x06
	// IMODE = 3 (Instruction on all lines), Instruction = 0x06 (MT25QL128 write enable)
	QUADSPI->CCR = (3U << QUADSPI_CCR_IMODE_Pos) | (0x06U << QUADSPI_CCR_INSTRUCTION_Pos); // All other phases (Address, Data, etc.) are 0

	// Wait for the instruction to be sent (Transfer complete flag)
	while (!(QUADSPI->SR & QUADSPI_SR_TCF)); // the TCF bit is set in indirect mode when the programmed number of data is transferred

	QUADSPI->FCR = QUADSPI_FCR_CTCF;  // Clear transfer complete flag (Flag Clear Register)
}


void QSPI_EnableQuadMode(void) {

	QSPI_WriteEnable(); // send WREN first

	while (QUADSPI->SR & QUADSPI_SR_BUSY);

	// We are sending 1 byte of data (data length register)
	QUADSPI->DLR = 1 - 1;

	// Configure CCR for Indirect Write
	QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos) |			// FMODE = 0 (Indirect Write)
			(1U << QUADSPI_CCR_DMODE_Pos) |  		// DMODE = 1 (Data on 1 line)
			(1U << QUADSPI_CCR_IMODE_Pos) | 			// IMODE = 1 (Instruction on 1 line)
			(0x61U << QUADSPI_CCR_INSTRUCTION_Pos);  // Instruction to be sent = 0x61 (Enhanced Volatile Configuration Register)

	// write 0x7F (Quad I/O protocol enabled) to the MT25QL128 "Enhanced Volatile Configuration Register"
	QUADSPI->DR = 0x7F;  // Send the value to the FIFO for "Full Quad Spi" (4-4-4) mode. This starts the transmission

	// Wait for completion
	while (!(QUADSPI->SR & QUADSPI_SR_TCF)); // Wait for the PERIPHERAL to finish the SPI transaction
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	QSPI_WaitUntilReady(); // Wait for the FLASH CHIP to finish its internal write cycle

	/*   Once the QSPI is in Memory-Mapped mode, the QUADSPI->DR register is no longer used,
	and you cannot send manual commands like readID() without first switching FMODE back to 0 or 1.*/
}


uint8_t QSPI_GetStatus(void) {
	while (QUADSPI->SR & QUADSPI_SR_BUSY);

	QUADSPI->DLR = 1 - 1; // Expecting 1 byte
	QUADSPI->CCR = (1U << QUADSPI_CCR_FMODE_Pos) | // Functional mode: Indirect-READ mode
			(3U << QUADSPI_CCR_DMODE_Pos) | // Data 4 line
			(3U << QUADSPI_CCR_IMODE_Pos) | // Inst 4 line: send the Command on all 4 wires
			(0x05U << QUADSPI_CCR_INSTRUCTION_Pos); // Read MT25QL128 Status Register (can be read continuously and at any time)

	while (!(QUADSPI->SR & QUADSPI_SR_TCF));
	uint8_t status = (uint8_t)(QUADSPI->DR);
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	return status;
}

void QSPI_WaitUntilReady(void) {
	// Bit 0 of Status Register is WIP (Write In Progress)
	while (QSPI_GetStatus() & 0x01); // bit 1 = "Write in progress" of MT25QL128 "Read Status Register"
}


uint32_t readID(void) {
	// Manufacturer ID=0x20 JEDEC, Memory type=0xBA (3V), Memory capacity=18h (128Mb)
	QSPI_Prepare_Indirect();
	uint32_t id = 0;
	QUADSPI->DLR = 3 - 1; // Expecting 3 bytes
	QUADSPI->CCR = (1U << QUADSPI_CCR_FMODE_Pos) |	// Functional mode: Indirect-READ mode
			(3U << QUADSPI_CCR_DMODE_Pos) | // Data 3 line
			(3U << QUADSPI_CCR_IMODE_Pos) | // Inst 3 line: send the Command on all 4 wires
			(0x9E << QUADSPI_CCR_INSTRUCTION_Pos); // Read MT25QL128 Status Register (can be read continuously and at any time)

	while (!(QUADSPI->SR & QUADSPI_SR_TCF));
	id = QUADSPI->DR;
	QUADSPI->FCR = QUADSPI_FCR_CTCF;

	return id & 0x00FFFFFF; // Mask to 24 bits
}


void MT25Q_SubsectorRead_DMA(uint32_t address, uint32_t *rData) {

	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"

	// Set up DMA for Memory-to-Peripheral
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;            // Disable DMA
	while(DMA2_Stream7->CR & DMA_SxCR_EN);       // Wait for hardware to release
	DMA2_Stream7->M0AR = (uint32_t)rData;        // Set the Rx buffer
	DMA2_Stream7->NDTR = 1024;                   // (4096 bytes / 4) transfers
	DMA2_Stream7->CR &= ~DMA_SxCR_DIR_Msk;       // Set DIR to 00 (Peripheral-to-Memory)
	DMA2_Stream7->CR |= DMA_SxCR_EN;             // Enable DMA

	QUADSPI->DLR = 4096 - 1; // data length register (read 4KB)

	QUADSPI->CR |= QUADSPI_CR_DMAEN;   // Enable DMA mode in QSPI

	QUADSPI->CCR = (1U << QUADSPI_CCR_FMODE_Pos) |	// Configure QSPI for Indirect Read
			(10U << QUADSPI_CCR_DCYC_Pos)  |  	 	// Dummy bytes
			(2U << QUADSPI_CCR_ADSIZE_Pos) |
			(3U << QUADSPI_CCR_DMODE_Pos)  |
			(3U << QUADSPI_CCR_ADMODE_Pos) |
			(3U << QUADSPI_CCR_IMODE_Pos)  |
			(0x0BU << QUADSPI_CCR_INSTRUCTION_Pos); // Fast read

	QUADSPI->AR = address;  // Trigger start

    while (!(DMA2->HISR & DMA_HISR_TCIF7)); // Wait for DMA to finish (or use an IRQ)

    // Clean up
    DMA2->HIFCR = DMA_HIFCR_CTCIF7; // Stream 7: clear transfer complete interrupt flag
    while (QUADSPI->SR & QUADSPI_SR_BUSY);
    QUADSPI->CR &= ~QUADSPI_CR_DMAEN;
    QUADSPI->FCR = QUADSPI_FCR_CTCF; // CTCF: Clear transfer complete flag
}


void MT25Q_SubsectorErase(uint32_t address) {

	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"

	QSPI_WriteEnable();  // Write Enable (Required before every Erase/Program)

    QUADSPI->CCR = (2U << QUADSPI_CCR_ADSIZE_Pos) |   // 4-4-4 mode: IMODE=3, ADMODE=3, ADSIZE=2
                   (3U << QUADSPI_CCR_ADMODE_Pos) |
                   (3U << QUADSPI_CCR_IMODE_Pos)  |
                   (0x20U << QUADSPI_CCR_INSTRUCTION_Pos);  // Send Subsector Erase (0x20)

    QUADSPI->AR = address; // Trigger start

    QSPI_WaitUntilReady();   // Wait for the physical erase to finish (WIP bit in Status Register)
}

void MT25Q_PageProgram(uint32_t address, uint32_t *data) {

	QSPI_Prepare_Indirect(); // If necessary, abort from Memory-Mapped mode and clear the FCR Flags
	QSPI_WaitUntilReady(); // Polling the Flash until status is no longer "write in progress"

    QSPI_WriteEnable(); // Write Enable is MANDATORY before every program command

    // Set up DMA for Memory-to-Peripheral
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;            // Disable DMA
    while(DMA2_Stream7->CR & DMA_SxCR_EN);       // Wait for hardware to release
    DMA2_Stream7->M0AR = (uint32_t)data;         // Set the Tx buffer
    DMA2_Stream7->NDTR = 64;                     // (256 bytes / 4) transfers
    DMA2_Stream7->CR |= DMA_SxCR_DIR_0;          // Set DIR to 01 (Memory-to-Peripheral)
    DMA2_Stream7->CR |= DMA_SxCR_EN;             // Enable DMA

    // Configure QSPI
    QUADSPI->DLR = 256 - 1; // length
    QUADSPI->CR |= QUADSPI_CR_DMAEN; // Enable DMA
    QUADSPI->CCR = (0U << QUADSPI_CCR_FMODE_Pos)  | // Indirect Write
                   (2U << QUADSPI_CCR_ADSIZE_Pos) | // 24-bit
                   (3U << QUADSPI_CCR_DMODE_Pos)  |
                   (3U << QUADSPI_CCR_ADMODE_Pos) |
                   (3U << QUADSPI_CCR_IMODE_Pos)  |
                   (0x02U << QUADSPI_CCR_INSTRUCTION_Pos);// Command 0x02 (Page Program)

    // Essentially, a command starts as soon as firmware gives the last information that is necessary for this command
    QUADSPI->AR = address;

    // 4. Wait for DMA and Flash Busy
    while (!(DMA2->HISR & DMA_HISR_TCIF7)); // DMA finishes 64 words (Page Program) - Stream 7 Transfer Complete Interrupt Flag
    DMA2->HIFCR = DMA_HIFCR_CTCIF7; // Clear flag

    while (QUADSPI->SR & QUADSPI_SR_BUSY);
    QUADSPI->CR &= ~QUADSPI_CR_DMAEN; // Disable DMA
    QUADSPI->FCR = QUADSPI_FCR_CTCF; // CTCF: Clear transfer complete flag

    QSPI_WaitUntilReady(); // The Flash needs time to physically program the bits (up to 0.4ms)
}

void MT25Q_SubsectorWrite(uint32_t address, uint32_t *data) {

	for(int i=0; i < 16; i++) {
		MT25Q_PageProgram(address, &data[i*64]); // 64 * 4 bytes = 256 bytes per page
		address += 256;
	}
}

/* If DMAEN = 1, a DMA transfer is initiated
	when FTF is set. FTF is cleared by hardware as soon as the threshold condition is no longer
	true (after enough data is transferred by the CPU or DMA).

*/


