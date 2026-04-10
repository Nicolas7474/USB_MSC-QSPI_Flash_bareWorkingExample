/****************************************************************************************************
* STM32F469 - USB MSC Driver
* USB OTG FS device Mass Storage (MSC) implementation  - Nicolas Prata - 2026
* Lightweight, only two MSC files to integrate to your project: usb_msc_fs.c and usb_msc_fs.h
* Usage:
    - You can edit the descriptors on usb_MSC_fs.h file
    - USB Slave-mode only (DMA not available on USB Full Speed with PA11 - PA12)
* Added support for on-board QSPI Flash MT25QL128 and the FATfs library
* Format to exFAT (32KB alloc size is faster for large transfers but for small files keep 4096KB)
******************************************************************************************************/

#include "main.h"
#include "usb_msc_fs.h"
#include "qspi.h"
#include "timers.h"

static volatile uint32_t device_state = DEVICE_STATE_DEFAULT; /* Device state */
volatile MSC_State_t msc_state = MSC_STATE_IDLE; // msc_state is tracking where things are in the MSC "Command-Data-Status" loop

volatile static USB_setup_req_data setup_pkt_data; /* Setup Packet var */
EndPointStruct EndPoint[EP_COUNT];	/* All the Enpoints are included in this array */

// Global variables to track the USB enable
volatile uint8_t flag_fatfs_busy = 0;
volatile uint8_t flag_usb_connected;

// Global variables to track the write progress
uint32_t write_lba;
uint16_t write_block_count;
uint32_t msc_tag = 0; // Save the tag from the CBW here
static uint32_t msc_bytes_remaining = 0;
static uint8_t is_sector_pre_erased = 0; // Track if the current multi-block transfer already performed a 64KB erase
static uint32_t current_32k_block = 0xFFFFFFFF;

// Global or static variables to hold current error state
uint8_t msc_sense_key = 0;
uint8_t msc_asc = 0;
uint8_t msc_ascq = 0;

uint8_t write_buf[4096];
static uint8_t rxBufferEp0[RX_BUFFER_EP0_SIZE]; /* Received data is stored here after application reads DFIFO. RX FIFO is shared */
__attribute__ ((aligned (4))) uint8_t rxBufferEp1[RX_BUFFER_EP1_SIZE]; // Word-aligned: The OTG engine performs better when the DMA/FIFO can access data on word boundaries

/*
 Standard Inquiry Data (always 36 bytes). This data is crucial for the USB Host to identify the device type,
 vendor, product, and revision, enabling it to load the appropriate driver and map the device as a removable disk. */
const uint8_t MSC_InquiryData[] = {
	0x00,          // Direct Access Block Device (Flash Drive / Hard Drive)
    0x80,          // 0x80: RMB = 1 (Removable Medium). Embedded MSC devices → almost always 0x80. Use RMB = 0x00 only if the device behaves like a fixed disk
    0x02,          // Version (ANSI SPC-3)
    0x02,          // Response Data Format
    31,            // Additional Length (36 - 5 bytes)
    0x00, 0x00, 0x00, // SCCS, etc.
    'N', 'i', 'c', 'o', 'l', 'a', 's', ' ', // Vendor ID (8 chars)
    'S', 'T', 'M', '3', '2', 'F', '4', ' ', // Product ID (16 chars)
    'D', 'i', 's', 'c', 'o', 'v', 'e', 'r',
    'y', ' ', '1', '.', '0'                 // Revision (4 chars)
};

/****************************************************************
 * 		static functions' declarations
*****************************************************************/

/* Device state */
static inline void set_device_status(eDeviceState state);

/* Init EP */
static void init_EndPoints(void);
static void activate_Endpoints(void);

/* FIFO handler */
static inline void set_FIFOs_sz(void);

static void SysTick_init(void);
static void Get_ID_To_String(uint8_t *dest); // Get STM32 serial number (UID)

// MSC static functions
static MSC_Status_t MSC_Parse_SCSI_Command(USB_MSC_CBW_t *cbw);
static void MSC_Handle_Inquiry(USB_MSC_CBW_t *cbw);
static void MSC_Send_CSW(uint32_t tag, uint32_t residue, uint8_t status);
static void MSC_Handle_TestUnitReady(USB_MSC_CBW_t *cbw);
static void MSC_Handle_ReadCapacity10(USB_MSC_CBW_t *cbw);
static MSC_Status_t MSC_Handle_Read10(USB_MSC_CBW_t *cbw);
static MSC_Status_t MT25Q_Read_Indirect(uint32_t address, uint8_t *pData, uint32_t len);
static MSC_Status_t MSC_Handle_Write10(USB_MSC_CBW_t *cbw);
static MSC_Status_t MSC_WriteComplete_Callback(void);
static uint8_t MSC_is_blank(uint32_t flash_addr, uint32_t size);
static void MSC_Handle_ModeSense6(USB_MSC_CBW_t *cbw);
static void MSC_Stall_Endpoints(void);
static uint32_t MSC_transferTXCallback_EP0(void);
static uint32_t MSC_transferTXCallback_EP1(void);
static void MSC_Handle_RequestSense(USB_MSC_CBW_t *cbw);
static void MSC_Handle_GetEventStatusNotification(USB_MSC_CBW_t *cbw);
static void MSC_Handle_GetConfiguration(USB_MSC_CBW_t *cbw);
static void MSC_Handle_ReadTOC(USB_MSC_CBW_t *cbw);
static void MSC_Handle_ReadDiscInfo(USB_MSC_CBW_t *cbw);
static void MSC_Update_Sense_Data(uint8_t key, uint8_t asc, uint8_t ascq);
static void MSC_Handle_ReadFormatCapacity(USB_MSC_CBW_t *cbw);
static void MSC_ForceResetState(void);

/**********************************************************************
 * GetSysTick() is required for Timeout detection in several functions
 * you can add msTicks++ in your SysTick_Handler if you use it already
 * or you can provide your own Timeout function (eg with a timer)
 **********************************************************************/

uint32_t SystemCoreClock = 180000000;
volatile uint32_t msTicks = 0; // Volatile ensures the compiler doesn't optimize out reads of this value

static void SysTick_init(void) {
	// SysTick generates an interrupt every 1ms. ticks = SystemCoreClock / 1000
	/* SysTick_Config() returns a value (typically 0 for success and 1 for failure)
     1 = Force the whole chip to reboot and try again (Capture error, should not happen with valid clock) */
	if (SysTick_Config(SystemCoreClock / 1000)) { NVIC_SystemReset(); }
}

void SysTick_Handler(void) { msTicks++; } // The SysTick handler is predefined in the vector table

uint32_t GetSysTick(void) { return msTicks; } // GetTick: this is a direct replacement for HAL_GetTick()


/***************************************************
*
* 	Initialization functions
*
***************************************************/

uint32_t USB_OTG_FS_Init(void) {
	/* configuration RCC CLOCKS GPIO ; Model-dependant (example for STM32F469) */

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12); //  Configure PA11 (DM) and PA12 (DP) as Alternate Function (AF10)
	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);   // Mode: 10 (Alternate Function)
	GPIOA->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL11_Pos);
	GPIOA->AFR[1] |=  (10U << GPIO_AFRH_AFSEL11_Pos);
	GPIOA->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->AFR[1] |=  (10U << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->PUPDR &= ~(1<<22 | 1<<24); // No pull-up, no pull-down
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12); 	// Speed: 11 (Very High Speed)
	/* PA9 (VBUS) Configuration: the VBUS sensing on PA9 bypasses the standard AF multiplexer
	 and connects directly to the sensing block when enabled in the USB core (not needed in USB Device mode) */
	GPIOA->MODER &= ~(GPIO_MODER_MODER9); 	// Set to Input mode (00) - This is the "Default State"
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9); 	//Ensure No Pull-up/Pull-down to allow the 200 uA sensing circuit to work

	RCC->CR |= RCC_CR_HSEON; // Enable HSE (8MHz External Crystal)
	while(!(RCC->CR & RCC_CR_HSERDY));

	// Disable the PLL to modify the registers
	RCC->CR |= RCC_CR_HSION; 	// Enable HSI (Internal High Speed oscillator)
	while (!(RCC->CR & RCC_CR_HSIRDY));     //  Wait until HSI is ready
	RCC->CFGR &= ~RCC_CFGR_SW;   // Switch System Clock (SYSCLK) to HSI
	RCC->CFGR |= RCC_CFGR_SW_HSI;   // Clear SW bits and set them to 00 (HSI select)
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // 4. Wait until HSI is actually being used as the system clock,  SWS bits (System Clock Switch Status) should report 00
	RCC->CR &= ~RCC_CR_PLLON; 	// The PLL is now "free" and can be safely disabled, Disable the main PLL
	while (RCC->CR & RCC_CR_PLLRDY);     // Wait until PLL is fully stopped
	WRITE_REG(RCC->PLLCFGR, (0)); // !! NEEDED !! Why !?

	// Configure Main PLL (M=4, N=180, P=2), f_VCO = 8MHz*(180/4) = 360MHz,  f_SYS = 360MHz/2 = 180MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE 		//  main PLL clock source = HSE
			     |  RCC_PLLCFGR_PLLM_2
				 |  180 << RCC_PLLCFGR_PLLN_Pos
				 |  (6 << RCC_PLLCFGR_PLLQ_Pos)
				 |  (RCC_PLLCFGR_PLLR_1);
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_Msk);

	// Re-enable the PLL
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)); //  Wait for the PLL to lock

	//The Over-drive Sequence is strict. If you skip a step, the MCU will likely hang during the clock switch.
	RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable Power Control clock,
	PWR->CR |= PWR_CR_VOS; //Set Regulator Voltage Scaling to Scale 1
	PWR->CR |= PWR_CR_ODEN; // Enable Over-Drive Mode
	while(!(PWR->CSR & PWR_CSR_ODRDY));
	PWR->CR |= PWR_CR_ODSWEN; // Switch to Over-Drive
	while(!(PWR->CSR & PWR_CSR_ODSWRDY));

	// Configure Flash Latency and ART Accelerator, it's critical to do this before switching the clock to 180MHz
	FLASH->ACR = FLASH_ACR_ICEN           // Instruction Cache Enable
			   | FLASH_ACR_DCEN          // Data Cache Enable
			   | FLASH_ACR_PRFTEN       // Prefetch Enable
			   | FLASH_ACR_LATENCY_5WS;    // Implement the 5-cycle latency & the power scaling required for 180MHz

	while (!(RCC->CR & RCC_CR_PLLRDY)); //  Wait for the PLL to lock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;   // Switch System Clock to PLL, Flash Latency is set correctly for new speed
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	//Configure Prescalers (AHB=1, APB1=4, APB2=2), work without these instructions though
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1
			  |  RCC_CFGR_PPRE1_DIV4
			  |  RCC_CFGR_PPRE2_DIV2;

	// Configure PLLSAI for USB 48MHz
	RCC->PLLSAICFGR = (144 << RCC_PLLSAICFGR_PLLSAIN_Pos) | (RCC_PLLSAICFGR_PLLSAIP_1); // f_VCO_SAI = (f_HSE/M)*N = (8/4)*144 = 288MHz
	RCC->CR |= RCC_CR_PLLSAION; 	// PLLSAI enable
	while(!(RCC->CR & RCC_CR_PLLSAIRDY));
	RCC->DCKCFGR |= RCC_DCKCFGR_CK48MSEL; // select PLLSAI as the 48MHz source for USB

	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; // USB OTG FS clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN; // OTGHSEN: USB OTG HS clock enable

	USB_OTG_FS_init_registers();

	return EP_OK;
}


void USB_OTG_FS_init_registers(){

	device_state = DEVICE_STATE_DEFAULT;

	/* OTG general core configuration register */
	while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)); // Wait for AHB master IDLE state before resetting (not mandatory)
	USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;  // Apply Core Soft Reset (not mandatory)
	while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST);
	USB_OTG_FS->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_VBDEN; // activate the FS PHY in transmission/reception & USB VBUS detection enable
	USB_OTG_FS->GAHBCFG = USB_OTG_GAHBCFG_GINT; // Enable Global Interrupt (GINTMSK: Global interrupt mask)

	/* OTG device control register */
	USB_OTG_DEVICE->DCTL = USB_OTG_DCTL_SDIS; // signal the USB OTG core to perform a soft disconnect (device disconnect event)
	/*OTG power and clock gating control register*/
	USB_OTG_PCGCCTL->PCGCCTL = 0;

	/* OTG USB configuration register */
	USB_OTG_FS->GUSBCFG =  USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL; // forces the core to device mode & Full Speed serial transceiver mode select
	//for(volatile int i = 0; i < 200000; i++); // Wait for PHY to stabilize
	USB_OTG_FS->GUSBCFG &= ~(uint32_t)(0x0FUL << 10UL) ;
	USB_OTG_FS->GUSBCFG |= (0x6 << 10); // USB (PHY clocks) turnaround time (according to AHB and Reference Manual)

	set_FIFOs_sz();

	// Initialize the structure of all EP (EP1, EP2 are hardware-enabled later in Activate_Composite_Endpoints()) and
	init_EndPoints(); 					   // the Hardware for EP0: the STM32 can thus receive the first SETUP packet

	// Set TXFE level to 'Completely Empty' in the Global AHB Config Register (check space before loading bytes into Fifo)
	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_TXFELVL; // (1U << 7)

	/* Enable Global Interrupt for Reset, IN, OUT, RX not empty */
	USB_OTG_FS->GINTMSK = USB_OTG_GINTMSK_USBRST // unmask USBRST: USB reset mask
	//		| USB_OTG_GINTMSK_USBSUSPM  // to activate for enabling Suspend interrupt
	//		| USB_OTG_GINTMSK_WUIM //   // to activate for enabling Wakeup interrupt
			| USB_OTG_GINTMSK_OEPINT    // unmask OUT endpoints interrupt
			| USB_OTG_GINTMSK_IEPINT    // unmask IN endpoints
			| USB_OTG_GINTMSK_RXFLVLM   // unmask Receive FIFO non-empty mask
			| USB_OTG_GINTMSK_ENUMDNEM; // Enumeration done mask // ?

	NVIC_SetPriority(OTG_FS_IRQn, 10);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	SysTick_init(); // may be optional if you provide your own SysTick_Handler() / GetSysTick() functions
}


static void init_EndPoints(){
	/*	Fill endpoint structures with initial data */

	for (uint32_t i = 0; i < EP_COUNT; i++) {
			/* Global defaults for all Endpoints */
	        EndPoint[i].statusRx     = EP_READY;
	        EndPoint[i].statusTx     = EP_READY;
	        EndPoint[i].rxCounter    = 0;
	        EndPoint[i].txCounter    = 0;
	        // Point to your generic or new MSC-specific handlers
	        EndPoint[i].setTxBuffer  = &MSC_setTxBuffer;
	    }

	    /* --- EP0: Control --- */
	    EndPoint[0].rxBuffer_ptr = rxBufferEp0;
	    EndPoint[0].txCallBack   = &MSC_transferTXCallback_EP0;

	    /* --- EP1: MSC Bulk Data --- */
	    EndPoint[1].rxBuffer_ptr = rxBufferEp1; // Ensure this is 64-byte aligned
	    EndPoint[1].rxCallBack   = &MSC_transferRXCallback_EP1; // Your BOT State Machine starts here
	    EndPoint[1].txCallBack   = &MSC_transferTXCallback_EP1;

	    /* Hardware: Setup EP0 to receive the first SETUP packet */
	    // 3 Packets allowed (STUPCNT=3), 1 Packet count for data, 64 bytes size
	    USB_EP_OUT(0)->DOEPTSIZ = (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << 19)) // This field is decremented to zero after a packet is written into the RxFIFO
	    						| USB_MSC_MAX_PACKET_SIZE    	 // 64, set in descriptor
	    						| USB_OTG_DOEPTSIZ_STUPCNT;  	 // STUPCNT = 0x11 means, EP can receive 3 packets. RM says to set STUPCNT = 3
	    USB_EP_OUT(0)->DOEPCTL  |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);    // Enable EP0 and Clear NAK

	    USB_OTG_DEVICE->DCFG |= USB_OTG_DCFG_DSPD_Msk;  // DSPD: Device speedDevice speed - FS
	    USB_OTG_FS->GINTSTS = 0xFFFFFFFF; 	 			// Reset Global Interrupt status (core interrupt register OTG_GINTSTS)
	    USB_OTG_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;     // Soft connect
}


static void activate_Endpoints(void) {
    // --- Configure Bulk IN (Endpoint 1) ---
    USB_EP_IN(1)->DIEPCTL &= ~(USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP | USB_OTG_DIEPCTL_TXFNUM);
    USB_EP_IN(1)->DIEPCTL |= (64 << 0)                // Max Packet Size
                          |  (2 << 18)                // Endpoint Type: Bulk
                          |  (1 << 22)                // TxFIFO Number
                          |  USB_OTG_DIEPCTL_USBAEP;  // Endpoint Active

    // --- Configure Bulk OUT (Endpoint 1) ---
    USB_EP_OUT(1)->DOEPCTL &= ~(USB_OTG_DOEPCTL_MPSIZ | USB_OTG_DOEPCTL_EPTYP);
    USB_EP_OUT(1)->DOEPCTL |= (USB_MSC_MAX_PACKET_SIZE << 0)                // Max Packet Size
                           |  (2 << 18)                 // Endpoint Type: Bulk
                           |  USB_OTG_DOEPCTL_USBAEP;   // Endpoint Active

    // Force Data0 PID for both IN and OUT
    USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
    USB_EP_OUT(1)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;

    // Tell the hardware we are ready to receive the first 31-byte CBW
    USB_EP_OUT(1)->DOEPTSIZ = (1 << 19) | (USB_MSC_MAX_PACKET_SIZE << 0); // 1 Packet, 64 bytes
    USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}


static inline void set_FIFOs_sz(){
	/*Set RX and TX FIFO size and offset for each EP*/

	USB_OTG_FS->GRXFSIZ = RX_FIFO_SIZE; // all EPs RX FIFO RAM size (GRXFSIZ) OTG receive FIFO size register)

	/* OTG Host Non-Periodic Transmit FIFO Size register - EP0 TX FIFO RAM size (Start: 128, Size: 64) */
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ((TX_EP0_FIFO_SIZE) << 16) | RX_FIFO_SIZE; // EP0 TX FIFO RAM size

	/* OTG device IN endpoint transmit FIFO x size register - EP1 TX FIFO (MSC Data) (Start: 192, Size: 64)*/
	USB_OTG_FS->DIEPTXF[0] = ((TX_EP1_FIFO_SIZE) << 16) | (RX_FIFO_SIZE + TX_EP0_FIFO_SIZE); // EP1 TX FIFO RAM size

	/* Clear remaining FIFOs (Start loop at index 1 to protect EP1) */
	for(uint32_t i = 1; i < 0x10 ; i++){
		USB_OTG_FS->DIEPTXF[i] = 0;
	}
}


/****************************************************
* 	Miscellaneous service functions*
*****************************************************/

void maintenance_switch(void) {
	/*switch between USB and FATfs control over the Flash*/

	// Check if USB is currently DISCONNECTED (SDIS bit is set)
	if (USB_OTG_DEVICE->DCTL & USB_OTG_DCTL_SDIS) {

		f_mount(NULL, "0:", 0); // Effectively "closes" the filesystem in the CPU RAM
		flag_usb_connected = 1; // Block the STM32 from starting any new writes
		while (flag_fatfs_busy); // The "Atomic" Handover Logic: Wait for any existing write to finish ("Busy" flag check)

		// --- GOING TO USB MODE ---
		QSPI_Prepare_Indirect(); // mandatory before checking (QUADSPI->SR & QUADSPI_SR_BUSY=
		QSPI_WaitUntilReady();  // Wait for physical Flash to finish any background Erase/Write
		QSPI_Enable_MemoryMapped(); // re-enable MMM for USB (cf. MSC_Handle_Read10 function)
		USB_OTG_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;  // Finally enable USB (Clear SDIS)
	}
	else {
		// --- GOING TO FATfs MODE ---
		FRESULT res;
		USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;   // Eject USB (Set SDIS)
        QSPI_WaitUntilReady(); // Wait for Flash Ready (the PC might have been writing right when the button was pressed)
        QSPI_Enable_MemoryMapped();
        flag_usb_connected = 0; // Tell the STM32 logic it's safe to work again
        NBdelay_ms(50);   // Safety Delay: Windows takes a moment to realize the device is gone
    	// Mount the drive - This doesn't "touch" the flash much; it just tells FatFs to initialize the fs structure and prepare for communication.
    	res = f_mount(&fs, "0:", 1); //  "": Defaut Drive (number 0) ; 1: Forced mount (checks for FAT structure immediately)
    	if (res != FR_OK) {	/* If res is FR_NO_FILESYSTEM*/ }
    }
}

static void Get_ID_To_String(uint8_t *dest) {
	// Pointer to the UID start address (Adjust for your specific STM32)
	uint32_t *id_ptr = (uint32_t *)0x1FFF7A10;
	const char hex_table[] = "0123456789ABCDEF";

	dest[0] = 50;   // Total length = 2 bytes header + (12 bytes * 2 chars/byte * 2 bytes/char) = 50
	dest[1] = 0x03; //  Descriptor Type: 3 always means "String"

	uint8_t *pStr = &dest[2];  // Start writing at the 3rd byte (after the header)

	// Convert 12 bytes of raw ID to 24 Unicode characters - process 3 words (12 bytes total)
	for (int i = 0; i < 3; i++) {
		uint32_t word = id_ptr[i];
		// Process each word byte-by-byte (4 bytes per word)
		for (int j = 0; j < 4; j++) {
			uint8_t byte = (word >> (j * 8)) & 0xFF; // Extract byte

			// High nibble
			*pStr++ = hex_table[byte >> 4];
			*pStr++ = 0x00;
			// Low nibble
			*pStr++ = hex_table[byte & 0x0F];
			*pStr++ = 0x00;
		}
	}
}


/***************************************************
** 					MSC functions				  **
****************************************************/
/*
The flow is a strict loop: Data Phase: The Host asks for X bytes (e.g., 4096).  Status Phase: You send exactly 13 bytes (the CSW).
Receive 31 bytes (CBW) on OUT, Send/Receive Data (Optional) on IN or OUT, Send 13 bytes (CSW) on IN.
If you skip any of these or send them out of order, the PC will show the dreaded "USB Device Not Recognized."
If a FIFO error occurs, the hardware sets a bit in DIEPINT, and we usually just STALL the endpoint. A software state bit isn't enough to recover.
 */

static MSC_Status_t MSC_Parse_SCSI_Command(USB_MSC_CBW_t *cbw) {
	uint8_t opcode = cbw->CB[0];

	switch (opcode) {
	case 0x00: // TEST_UNIT_READY
		MSC_Handle_TestUnitReady(cbw);
		break;

	case 0x12: // INQUIRY
		MSC_Handle_Inquiry(cbw); // Reply essential device informations (type, removable, vendor, etc)
		break;

	case 0x43: // READ TOC
		MSC_Handle_ReadTOC(cbw);
		break;

	case 0x46: // SCSI_GET_CONFIGURATION
		MSC_Handle_GetConfiguration(cbw);
		break;

	case 0x4A: // GET_EVENT_STATUS_NOTIFICATION 	- without it, the default case triggers, sending a failure status (0x01)
		// The host checks if the "media" (SD card...) has been swapped. By providing a "No Event" 8-byte response, the host knows the drive is stable and ready.
		MSC_Handle_GetEventStatusNotification(cbw);
		break;

	case 0x51: // READ DISC INFORMATION
	    MSC_Handle_ReadDiscInfo(cbw);
	    break;

	case 0x1A: // MODE_SENSE_6
		MSC_Handle_ModeSense6(cbw);
		break;

	case 0x23: // READ FORMAT CAPACITIES
	    MSC_Handle_ReadFormatCapacity(cbw);
	    break;

	case 0x25: // READ_CAPACITY_10
		MSC_Handle_ReadCapacity10(cbw);
		break;

	case 0x28: // READ_10
		MSC_Handle_Read10(cbw);
		break;

	case 0x2A: // WRITE_10
		MSC_Handle_Write10(cbw);
		break;

	case 0x03: // REQUEST_SENSE
		MSC_Handle_RequestSense(cbw);
		break;

	case 0x1E: // PREVENT_ALLOW_MEDIUM_REMOVAL (0x1E)
		// Just ACK it: Some OS's won't mount if this stalls
		MSC_Send_CSW(cbw->dTag, 0, 0x00);
		break;

	default:
		// Set SCSI Sense Data
		// This is vital so the host knows the command was "Illegal/Unsupported"
		// Sense Key: 0x05 (Illegal Request), ASC: 0x20 (Invalid Command OpCode)
		MSC_Update_Sense_Data(0x05, 0x20, 0x00);

		// Handle the Data Phase Residue
		// If the host expected data, we tell it we sent 0 bytes.
		// The residue is the full length the host originally requested.
		uint32_t residue = cbw->dDataTransferLength;

		// Finalize the BOT protocol by sending the CSW
		// Status 0x01 = Command Failed
		MSC_Send_CSW(cbw->dTag, residue, 0x01);
		break;
	}
	return MSC_OK;
	/* Handling REQUEST_SENSE (0x03):
	If a command fails (status 0x01), the Host will immediately send a REQUEST_SENSE command to ask why it failed.
	If you don't implement this, the OS might try to reset the entire USB bus.
	For a basic "it works" driver, you can usually get away with returning 18 bytes of zeros, but if you start seeing "I/O Errors" in Windows,
	you'll need to implement a small buffer for "Sense Keys" (like MEDIUM_ERROR or NOT_READY). 	*/
}


static void MSC_Handle_Inquiry(USB_MSC_CBW_t *cbw) {
	/*Handling the Inquiry (The "Identity" Phase)
	The Host expects a specific 36-byte response. You must send this data back via Endpoint 1 IN.*/

	// 1. Send the 36 bytes of Inquiry Data
	uint32_t len = cbw->dDataTransferLength;
	if (len > 36) len = 36; // Don't send more than host asked or we have

	// 1. Set state FIRST so the callback sees it
	EndPoint[1].setTxBuffer(1, (uint8_t*)MSC_InquiryData, len);
	// Result: The first synchronous call to the callback does the FIFO fill.
	// The second call (from the ISR when hardware is done) will see DATA_IN
}


static void MSC_Send_CSW(uint32_t tag, uint32_t residue, uint8_t status) {
	/* The CSW (Status) Packet: Every SCSI command must end with a CSW (Command Status Wrapper).
	If you don't send this, the host will think the device has hung and will reset the USB bus.
	The dDataResidue field is the difference between what the host asked for (in cbw->dDataTransferLength) and what you actually sent. */

	// Set state first so the ISR knows what to do on completion
	msc_state = MSC_STATE_SEND_CSW;

	// __attribute__((aligned(4))) ensures write_Fifo doesn't crash on unaligned access
	// Stores the 13-byte status packet sent after every command.
	static __attribute__((aligned(4))) USB_MSC_CSW_t csw; // Static to persist for DMA/FIFO

	// If the Host asks to write 512 bytes and you successfully receive and write 512 bytes to your SD card, the residue is 0
	csw.dSignature = 0x53425355; // "USBS"
	csw.dTag = tag;
	csw.dDataResidue = residue;
    csw.bStatus = status;

    EndPoint[1].setTxBuffer(1, (uint8_t*)&csw, 13);  // Send the 13 bytes
}


static void MSC_Handle_TestUnitReady(USB_MSC_CBW_t *cbw) {
   /* This is the easiest one. The host is just asking "Are you ready?".
	We are always ready! Send no data, just the Status (CSW)
	The host will spam TEST_UNIT_READY (0x00) to see if the "disk" is inserted.
	Since there is no data phase for this, you send the CSW immediately.*/
	msc_state = MSC_STATE_IDLE;
    MSC_Send_CSW(cbw->dTag, 0, 0x00); // 0x00 = Good Status
}

static void MSC_Handle_ReadCapacity10(USB_MSC_CBW_t *cbw) {
	/* READ_CAPACITY_10 (0x25): Called once or twice at startup.
	   The PC needs to know the "Last LBA" (Total blocks - 1) and the "Block Size" (usually 512).
	   Since STM32 is Little Endian and USB/SCSI is Big Endian, you must manually swap the bytes. */

	static uint8_t capacity[8];
	// For 128Mbit (16MB) Flash: 16,777,216 bytes / 4096 bytes per block = 4096 blocks
	uint32_t block_size = MSC_BLOCK_SIZE;
	uint32_t last_lba = MSC_TOTAL_BLOCKS - 1; // 4095 (0x0FFF)

	// Last LBA (Big Endian): 0x00 00 0F FF
	capacity[0] = (uint8_t)(last_lba >> 24);
	capacity[1] = (uint8_t)(last_lba >> 16);
	capacity[2] = (uint8_t)(last_lba >> 8);
	capacity[3] = (uint8_t)(last_lba);

	// Block Size (Big Endian): 0x00 00 10 00
	capacity[4] = (uint8_t)(block_size >> 24);
	capacity[5] = (uint8_t)(block_size >> 16);
	capacity[6] = (uint8_t)(block_size >> 8);
	capacity[7] = (uint8_t)(block_size);

	msc_state = MSC_STATE_DATA_IN;

	EndPoint[1].setTxBuffer(1, capacity, 8);
	// After Read Capacity, the callback triggers, you send the CSW, and the Host says "Cool, a 128MB drive."
}


static MSC_Status_t MSC_Handle_Read10(USB_MSC_CBW_t *cbw) {
	// READ_10 (0x28): sends the actual content of the disk - MEMORY-MAPPED MODE, no DMA here !

    // Extract LBA and Block Count from SCSI Command: you must parse the CBW to know which sector the PC wants
    uint32_t lba = (cbw->CB[2] << 24) | (cbw->CB[3] << 16) | (cbw->CB[4] << 8) | cbw->CB[5];
    uint16_t block_count = (cbw->CB[7] << 8) | cbw->CB[8];

    if ((lba + block_count) > MSC_TOTAL_BLOCKS) {
        MSC_Update_Sense_Data(0x05, 0x21, 0x00); // Logical Block Address Out of Range
        return MSC_FAIL;     // Out of bounds!
    }

    // Calculate the source address and total length
    uint32_t total_bytes = (uint32_t)block_count * MSC_BLOCK_SIZE;
    uint32_t flash_addr = 0x90000000 + (lba * MSC_BLOCK_SIZE);  // Each LBA represents 4096 bytes

    msc_tag = cbw->dTag; 	// Set the Global State
    msc_state = MSC_STATE_DATA_IN;

    // Kick the transfer - pass the Flash address as if it were a normal pointer
    if (MSC_setTxBuffer(1, (uint8_t*)flash_addr, total_bytes) != EP_OK) {
        MSC_Stall_Endpoints();
        return MSC_FAIL; // If the endpoint was busy, we stall or handle error
    }
    return MSC_OK;
    	/* The MSC_setTxBuffer will now:
    	 - Set the pointer to 0x90000000 + offset
     	 - Set the counter to 4096 (or more)
     	 - Call txCallBack() which fills the first packets from Flash
     	 - Enable DIEPEMPMSK to continue filling the FIFO as it empties */
}

static MSC_Status_t MSC_Handle_Write10(USB_MSC_CBW_t *cbw) {
    /* When you receive the WRITE_10 command in your parser, you must first "prime" the OUT EP to catch the
       data the Host is about to dump on you. Flow of control:
    1) CBW Arrives: MSC_Handle_Write10 is called. It enables the OUT EP to receive 4096 bytes.
    2) Data Arrives: MSC_transferRXCallback_EP1 fires.
    3) You must wait until you have processed the data and sent the CSW (Bulk IN) before you re-enable the Bulk OUT EP for the next CBW. */

    // 1. Calculate LBA and Number of Blocks from CDB (Big Endian)
    write_lba = (cbw->CB[2] << 24) | (cbw->CB[3] << 16) | (cbw->CB[4] << 8) | cbw->CB[5]; // LBA
    write_block_count = (cbw->CB[7] << 8) | cbw->CB[8]; // length

    if ((write_lba + write_block_count) > MSC_TOTAL_BLOCKS) {
    	MSC_Update_Sense_Data(0x05, 0x21, 0x00); // Logical Block Address Out of Range
    	return MSC_FAIL;     // Out of bounds!
    }

    // 2. Calculate TOTAL bytes the Host is going to send
    // Since your ReadCapacity reported 4096, total = write_block_count * 4096
    // We cannot write directly to 0x90000000; we must catch the data in our SRAM buffer (write_buf) first.
    msc_bytes_remaining = (uint32_t)write_block_count * 4096;
    msc_tag = cbw->dTag;
    msc_state = MSC_STATE_DATA_OUT;
    is_sector_pre_erased = 0; // Reset the optimization flag at the start of every new command

    // 3. Prepare the first chunk - only trigger 4096 at a time to fit the 'write_buf' buffer
    uint32_t chunk_size = (msc_bytes_remaining > 4096) ? 4096 : msc_bytes_remaining;

    // Point USB hardware to your workspace RAM buffer
    EndPoint[1].rxBuffer_ptr = (uint8_t*)write_buf;
    EndPoint[1].rxCounter = chunk_size;

    // Trigger Hardware Receive for the first 4096 bytes, then MSC_WriteComplete_Callback() will be called upon data reception
    // (64 packets of 64 bytes)
    USB_EP_OUT(1)->DOEPTSIZ = (64 << 19) | chunk_size;
    USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

    return MSC_OK;
}


MSC_Status_t MSC_WriteComplete_Callback(void) {

	uint32_t flash_addr = write_lba * 4096;
	MSC_Status_t write_status = MSC_OK;

	/** ERASE LOGIC **/
	if (write_lba <= 64) {
	    // Sector-by-sector safety (slower) for FAT/Headers
	    MT25Q_SubsectorErase_4KB(flash_addr);
	}
	else if (write_lba % 8 == 0) {
	    // Start of a new 32KB chunk: Erase the whole thing once
	    MT25Q_SubsectorErase_32KB(flash_addr); // 32KB block erase is faster than 4KB
	    is_sector_pre_erased = 1;
	    //SWV_SendString("32KB_Erased_");
	}
	else if (is_sector_pre_erased == 0) {
	    // We are in the data zone, but the transfer didn't start on a 32KB boundary
	    MT25Q_SubsectorErase_4KB(flash_addr); // We must erase 4KB to be safe.
	    //SWV_SendString("4KB_Erased_");
	}
	// If none of the above are true, it means is_sector_pre_erased == 1
	// and we are mid-way through a 32KB block. We skip Erase and just Write.

	MT25Q_SubsectorWrite_4KB(flash_addr, (uint8_t*)write_buf); // Always write the data received in 'write_buf'

	//  Update tracking
	msc_bytes_remaining -= 4096;
	write_lba++; // Move to the next 4096-byte LBA

	// Check if the Host is still pushing more blocks (e.g., 16 blocks total)
	if (msc_bytes_remaining > 0) {
		// The Host has more data! We must re-arm the OUT EP to catch the next 4096 bytes.
		// This prevents the NAK loop on the bus that causes the hang.
		uint32_t next_chunk = (msc_bytes_remaining > 4096) ? 4096 : msc_bytes_remaining;

		EndPoint[1].rxBuffer_ptr = (uint8_t*)write_buf;
		EndPoint[1].rxCounter = next_chunk;

		USB_EP_OUT(1)->DOEPTSIZ = (64 << 19) | next_chunk;
		USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);

		 return MSC_OK; // Exit and wait for the next chunk to arrive
	}
	// CHECK FOR HARDWARE FAILURE (Last possible moment)
	if (QSPI_GetStatus(READ_FLAG_STATUS_REGISTER) & 0x30) {			 // bits 4 and 5 of Flag status register
		MSC_Update_Sense_Data(0x03, 0x03, 0x00); // MEDIUM ERROR, SCSI_AS_WRITE_FAULT
		write_status = MSC_FAIL;
		MT25Q_SendCommand(CLEAR_FLAG_STATUS_REGISTER); // Clear the flags from status register
	}

	// DATA PHASE FINISHED (msc_bytes_remaining == 0)
	is_sector_pre_erased = 0;
    current_32k_block = 0xFFFFFFFF;
    // Re-enable Memory Mapping so the next 'Read' works!
    QSPI_Enable_MemoryMapped();

    // Now send the CSW. The Host has finished sending all data and will finally send an IN token for the status.
    uint8_t csw_result = (write_status == MSC_FAIL) ? 0x01 : 0x00; // Pass 0x01 if hardware failure was detected
    MSC_Send_CSW(msc_tag, 0, csw_result);

    return MSC_OK;
}


__attribute__((unused)) static uint8_t MSC_is_blank(uint32_t flash_addr, uint32_t size) {
    uint8_t check_buf[PAGE_SIZE];
    uint32_t bytes_checked = 0;
    uint32_t chunk;

    while (bytes_checked < size) {
        chunk = (size - bytes_checked > PAGE_SIZE) ? PAGE_SIZE : (size - bytes_checked);

        // This replaces the pointer access that was crashing the system
        MT25Q_Read_Indirect(flash_addr + bytes_checked, check_buf, chunk);

        for (uint16_t i = 0; i < chunk; i++) {
            if (check_buf[i] != 0xFF) {
                return 0; // Found data, must erase!
            }
        }
        bytes_checked += chunk;
    }
    return 1; // It's all 0xFF
}

static void MSC_Handle_ModeSense6(USB_MSC_CBW_t *cbw) {
	/*	Most Operating Systems (Windows/Linux) will send a MODE_SENSE_6 (0x1A) command before trying to write.
	They are asking: "Is this disk write-protected?". If you don't answer MODE_SENSE_6 correctly, the OS will mount the drive as Read-Only. */

	// Add alignment to ensure write_Fifo doesn't perform an unaligned 32-bit read
	static __attribute__((aligned(4))) uint8_t mode_sense_data[4] = {
        0x03, // Mode Data Length (Length of the following 3 bytes)
        0x00, // Medium Type (0x00 = SBC / Direct Access)
        0x00, // Device-Specific Parameter (Bit 7 is Write Protect: 0 = Writable)
        0x00  // Block Descriptor Length (0x00 = No descriptors)
    };

    msc_state = MSC_STATE_DATA_IN;
    msc_tag = cbw->dTag;

    // The host might ask for more than 4 bytes (check cbw->dDataTransferLength), but we only have 4 to give
    uint32_t len = cbw->dDataTransferLength;
    if (len > 4) len = 4;

    EndPoint[1].setTxBuffer(1, mode_sense_data, len);
}

static void MSC_Handle_RequestSense(USB_MSC_CBW_t *cbw) {
	/*	Host sends some command (e.g. READ(10)), something fails → the STM32 returns a failed CSW.
		Host immediately issues REQUEST SENSE and the MSC_Handle_RequestSense must return stored error info */

	/*	Set the Sense: msc_sense_key = 0x05; msc_asc = 0x24; (Illegal Request / Invalid Field).
	    Send Failed CSW: MSC_Send_CSW(tag, residue, 0x01); (Status = 1).
	    The Host asks: The PC sees the 0x01 and immediately calls REQUEST SENSE.
	    You respond: Your dynamic handler sends the 0x05/0x24, and Windows understands perfectly why the command failed.
	*/
    /* Fixed 18-byte response for "No Error / Everything is fine" */
	static __attribute__((aligned(4))) uint8_t sense_data[18];

	    // Clear the buffer
	    memset(sense_data, 0, 18);

	    sense_data[0] = 0x70; // Response Code (Current Sense)
	    sense_data[2] = msc_sense_key; // Key (e.g., 0x05 for Illegal Request)
	    sense_data[7] = 0x0A;          // Additional Length
	    sense_data[12] = msc_asc;      // Additional Sense Code (e.g., 0x24)
	    sense_data[13] = msc_ascq;     // ASC Qualifier

	    // After reporting the error once, we reset the sense to "No Sense"
	    msc_sense_key = 0;
	    msc_asc = 0;
	    msc_ascq = 0;

	    uint32_t len = cbw->dDataTransferLength;
	    if (len > 18) len = 18;

	    msc_state = MSC_STATE_DATA_IN;
	    msc_tag = cbw->dTag;

	    EndPoint[1].setTxBuffer(1, sense_data, len);
}

static void MSC_Handle_ReadTOC(USB_MSC_CBW_t *cbw) {
    // Standard TOC Response (format 0)
    // 12-byte response: 4-byte Header + 8-byte Track Descriptor
    static uint8_t toc_data[12] = {
        0x00, 0x0A,             // TOC Data Length (10 bytes follow this field)
        0x01,                   // First Track
        0x01,                   // Last Track
        // Track Descriptor for Track 1
        0x00,                   // Reserved
        0x14,                   // ADR/Control (0x14 = Data track, error correction)
        0x01,                   // Track Number
        0x00,                   // Reserved
        0x00, 0x00, 0x00, 0x00  // Track Start Address (LBA 0 in Big-Endian)
    };

    // Extract Allocation Length from the SCSI CDB (bytes 7 and 8)
    uint32_t allocation_length = (cbw->CB[7] << 8) | cbw->CB[8]; // These are Big-Endian: [7] is MSB, [8] is LSB

    // Cap the transfer at 12 bytes or what the host requested
    uint16_t send_len = (allocation_length < 12) ? (uint16_t)allocation_length : 12;

    // 1. Prepare State for XFRC interrupt
    msc_state = MSC_STATE_DATA_IN;

    // 2. Load the buffer and kick the hardware
    EndPoint[1].setTxBuffer(1, toc_data, send_len);     // This will trigger your txCallBack and program DIEPTSIZ
}

static void MSC_Handle_ReadDiscInfo(USB_MSC_CBW_t *cbw) {
    // 32-byte Disc Information Response
    static uint8_t disc_info[32] = {
        0x00, 0x1E,             // Data Length (30 bytes follow)
        0x0E,                   // Disc Status: Complete, Erasable, Finalized
        0x01,                   // First Track Number
        0x01,                   // Number of Sessions
        0x01,                   // First Track in Last Session
        0x01,                   // Last Track in Last Session
        0x20,                   // Disc Type (0x20 = CD-RW/DVD-RAM type)
        0x00,                   // Number of OPC Entries
        0x00, 0x00, 0x00, 0x00, // Reserved
        0x00, 0x00, 0x00, 0x00, // Disc Identification
        0x00, 0x00, 0x00, 0x00, // Last Session Lead-in Start Address
        0x00, 0x00, 0x00, 0x00, // Last Possible Lead-out Start Address
        0x00, 0x00, 0x00, 0x00  // Disc Bar Code / Reserved
    };

    // Extract Allocation Length from the SCSI CDB (bytes 7 and 8)
    // Big-Endian: [7] is MSB, [8] is LSB
    uint32_t allocation_length = (cbw->CB[7] << 8) | cbw->CB[8];

    // Cap the transfer at 32 bytes or what the host requested
    uint16_t send_len = (allocation_length < 32) ? (uint16_t)allocation_length : 32;

    // 1. Prepare State for the XFRC interrupt
    msc_state = MSC_STATE_DATA_IN;

    // 2. Load the buffer and kick the hardware
    EndPoint[1].setTxBuffer(1, disc_info, send_len);
}


static void MSC_Handle_ReadFormatCapacity(USB_MSC_CBW_t *cbw) {
    // 12-byte response
    static uint8_t capacity_data[12] = {
        0x00, 0x00, 0x00, 0x08, // Header
        0x00, 0x00, 0x10, 0x00, // Number of Blocks: 4096 (0x00001000)
        0x02,                   // Descriptor Code: Formatted Media
        0x00, 0x10, 0x00        // Block Length: 4096 bytes (0x001000)
    };

    // Note: 16MB / 4096 = 4096 blocks exactly.
    // In ReadFormatCapacity (0x23), the host expects the total number of blocks,
    // unlike ReadCapacity10 (0x25) which expects Max LBA (Total - 1).

    uint16_t allocation_length = (cbw->CB[7] << 8) | cbw->CB[8];  // Extract Allocation Length from the SCSI CB (bytes 7 and 8)
    uint16_t send_len = (allocation_length < 12) ? allocation_length : 12;  // Cap the transfer at 12 bytes or the requested allocation length

    msc_state = MSC_STATE_DATA_IN;

    // 2. Load the buffer and kick the hardware
    EndPoint[1].setTxBuffer(1, capacity_data, send_len);
}

static void MSC_Handle_GetConfiguration(USB_MSC_CBW_t *cbw) {
    // Basic 8-byte Feature Header - SCSI multi-byte fields are Big-Endian
    static uint8_t config_header[8] = {
        0x00, 0x00, 0x00, 0x04, // Data Length: 0x00000004 means 4 bytes follow the length field.
        0x00, 0x00,             // Reserved
        0x00, 0x08              // Current Profile: 0x0008 is the profile for a "Removable Disk"
    };

    // Cap the transfer at 8 bytes or what the host requested (allocation_length)
    uint32_t len = cbw->dDataTransferLength;
    uint16_t send_len = (len < 8) ? (uint16_t)len : 8;

    msc_state = MSC_STATE_DATA_IN;     // Update state so XFRC ISR knows to send CSW next

    // 2. Load the buffer and kick the hardware
    MSC_setTxBuffer(1, config_header, send_len);  // Your function handles DIEPTSIZ and the initial FIFO fill internally.
}

static MSC_Status_t MT25Q_Read_Indirect(uint32_t address, uint8_t *pData, uint32_t len) {
    if (len == 0)  return MSC_FAIL;

    QSPI_Prepare_Indirect(); // Abort Mem-Mapped and clear flags

    // 1. Configure Data Length
    QUADSPI->DLR = len - 1;

    // 2. Configure CCR for Indirect Read
    // Using 4-4-4 mode (IMODE=3, ADMODE=3, DMODE=3) to match your Erase setup
    QUADSPI->CCR = (1U << QUADSPI_CCR_FMODE_Pos)  | // 01: Indirect Read mode
                   (2U << QUADSPI_CCR_ADSIZE_Pos) | // 2: 24-bit address
                   (3U << QUADSPI_CCR_DMODE_Pos)  | // 3: Data on 4 lines
                   (3U << QUADSPI_CCR_ADMODE_Pos) | // 3: Address on 4 lines
                   (3U << QUADSPI_CCR_IMODE_Pos)  | // 3: Instruction on 4 lines
                   (READ << QUADSPI_CCR_INSTRUCTION_Pos); // 0x03: Normal Read

    // 3. Set Address and Trigger
    QUADSPI->AR = address;

    // 4. Manual FIFO Read Loop
    uint32_t count = 0;
    while (count < len) {
        // Wait until there is at least 1 byte in the FIFO
        if (QUADSPI->SR & QUADSPI_SR_FLEVEL_Msk) {
            pData[count++] = *(volatile uint8_t *)&QUADSPI->DR;
        }
    }
    // 5. Wait for transaction to finish
    while (QUADSPI->SR & QUADSPI_SR_BUSY);

    return MSC_OK;
}


static void MSC_Stall_Endpoints(void) {
    // Halt the Bulk pipes
    USB_EP_IN(1)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    USB_EP_OUT(1)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    msc_state = MSC_STATE_IDLE;
}

static void MSC_Update_Sense_Data(uint8_t key, uint8_t asc, uint8_t ascq) {
    msc_sense_key = key;
    msc_asc = asc;
    msc_ascq = ascq;
}

static void MSC_Handle_GetEventStatusNotification(USB_MSC_CBW_t *cbw) {
    // Standard 8-byte "No Event" response
    static const uint8_t event_data[8] = {
        0x00, 0x06, // Event Header Length (Total bytes - 2)
        0x00,       // No Event available
        0x00,       // Operational Change Class
        0x00, 0x00, 0x00, 0x00 // Reserved
    };

    uint32_t len = cbw->dDataTransferLength;
    if (len > 8) len = 8;

    // Transition to Data Phase
    msc_state = MSC_STATE_DATA_IN;
    msc_tag = cbw->dTag;

    // Prime the endpoint with the 8 bytes.
    EndPoint[1].setTxBuffer(1, (uint8_t*)event_data, len); // Your ISR will call MSC_Send_CSW once these 8 bytes are ACKed.
}

static void MSC_ForceResetState(void) {
	/*	A "Force Reset" means manually clearing the hardware's status registers, flushing the FIFO buffers,
		and re-enabling the "listening" state for the next incoming packet. */

    // 1. Atomic lock, protect against the USB Interrupt firing while we are resetting its world.
    __disable_irq();

    // 3. Harware FIFO flush : Flush TX FIFO 0 (or whichever FIFO your IN endpoint uses)
    USB_OTG_FS->GRSTCTL = (1 << 5) | (16 << 6); // 0x20: TxFIFO Flush + all Tx FIFO
    while (USB_OTG_FS->GRSTCTL & (1 << 5));    // Wait for hardware to finish flushing

    // Flush ALL RX FIFOs
    USB_OTG_FS->GRSTCTL = (1 << 4);            // 0x10: RxFIFO Flush
    while (USB_OTG_FS->GRSTCTL & (1 << 4));    // Wait for hardware to finish flushing

    // 4. Clear stuck interrupts: Clear any pending transfer complete or error flags for EP1 (your MSC data EP)
    // Writing 1 to these bits usually clears them in CMSIS/Bare-metal
    USB_EP_OUT(1)->DOEPINT = 0xFF;
    USB_EP_IN(1)->DIEPINT  = 0xFF;

    // 5. RE-PRIME THE RECEIVE ENDPOINT (The "Unstick" Step)
    // If a big packet caused a NAK, we manually tell the hardware to start listening again for a fresh packet.

    // Set expected transfer size to 1 Max Packet Size (e.g., 128 bytes)
    USB_EP_OUT(1)->DOEPTSIZ = (1 << 19) | (MAX_MSC_EP1_TX_SIZ << 0); // Bit 19-30: Packet Count (set to 1), Bit 0-18: Transfer Size

    // Enable the endpoint and Clear NAK (CNAK)
	USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA); //Bit 26: CNAK (Clear NAK), Bit 31: EPENA (Enable)

    // 6. Synchronize our software flag with the fresh hardware state and atomic unlock
    EndPoint[1].statusTx = EP_READY;

    __enable_irq();
}


/****************************************************
* 				DFIFO
***************************************************/

uint32_t USB_FlushTxFifo(uint32_t EPnum, uint32_t timeout){
	uint32_t count = 0;
	USB_OTG_FS->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (EPnum << 6));
	do{
		if (++count > timeout){
			return EP_FAILED;
		}
	}
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	 return EP_OK;
}

uint32_t USB_FlushRxFifo(uint32_t timeout){
	uint32_t count = 0;
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	do{
		if (++count > timeout){
			return EP_FAILED;
		}
	}
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

	return EP_OK;
}

void read_Setup_Fifo(){
	/*  Read Setup Packet EP0. Always 8 bytes (2 words) from the FIFO */
	uint32_t first_word = USB_OTG_DFIFO(0);
	uint32_t second_word = USB_OTG_DFIFO(0);

	setup_pkt_data.raw_data[0] = first_word; //
	setup_pkt_data.raw_data[1] = second_word;
}

void read_Fifo(uint8_t dfifo, uint16_t len) {
	// Handle immediately received data on OEPINT event

    uint32_t word_count = (len + 3) >> 2;
    // Use the pointer EXACTLY as it is currently set in the EndPoint struct
    uint8_t *dest = EndPoint[dfifo].rxBuffer_ptr;

    for (uint32_t i = 0; i < word_count; i++) {
        // Use the correct FIFO for the endpoint
        uint32_t temp_word = USB_OTG_DFIFO(dfifo);

        if (i < (word_count - 1)) {
            *((uint32_t*)dest) = temp_word;
            dest += 4;
        } else {
            uint8_t bytes_left = len - (i * 4);
            memcpy(dest, &temp_word, bytes_left);
            dest += bytes_left;
        }
    }

    EndPoint[dfifo].rxBuffer_ptr = dest;   // Update the pointer so the NEXT packet in this transfer
    EndPoint[dfifo].rxCounter += len;     // (e.g. the 2nd packet of a 4096-byte block) lands in the right place
}

uint32_t write_Fifo(uint8_t dfifo, uint8_t *src, uint16_t len) {
    uint32_t word_count = (len + 3) >> 2;

    // Because src is aligned (checked via attribute), we can blast data
    uint32_t *src_32 = (uint32_t *)src;

    for (uint32_t i = 0; i < word_count; i++) {
        // The USB OTG core internally handles the BCNT for the last word
        USB_OTG_DFIFO(dfifo) = *src_32++;
    }

    EndPoint[dfifo].txBuffer_ptr += len;
    EndPoint[dfifo].txCounter -= len;
    return EP_OK; // EP_OK
}


/****************************************************
* 			EndPoints' Callbacks
***************************************************/

static uint32_t MSC_transferTXCallback_EP1(void) {

	uint32_t start_tick = GetSysTick();

	// Ensure the hardware is physically ready for the next packet, preventing the "Incomplete" errors
    while(USB_EP_IN(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
    {
    	// Do not use 'if' instead, it doesn't work. Measured (with DWT) 70ns delay at most among all phases
		if (GetSysTick() - start_tick > 1) {
			return EP_FAILED; // small 1ms timeout
		}
	}

    // Ensure that the coming XFRC interrupt cannot possibly trigger the CSW logic until the FIFO was filled
    if(msc_state == MSC_STATE_IDLE) msc_state = MSC_STATE_DATA_IN;

    /* ZLP ? Enquiry (36 bytes): Not a multiple of 64 ; CSW (13 bytes): Not a multiple of 64 ; Read/Write (4096 bytes): this is a multiple of 64 !
       However, the SCSI protocol dictates that the host knows to stop after 4096 bytes based on the CBW. -> no case where a ZLP is actually needed here */

    // 1. If there is data to send, configure the Hardware registers
    // During Transfer: If the file is large, the TXFE (FIFO Empty) interrupt in your ISR calls write_Fifo repeatedly until txCounter hits zero.
    uint32_t len = EndPoint[1].txCounter; // EndPoint[EPnum].txCounter value was set inside MSC_setTxBuffer()
    if (len > 0) {
    	// Limit this specific hardware "burst" to 4096 bytes (64 packets)
    	uint32_t chunk_size = (len > 4096) ? 4096 : len;
    	uint32_t pktcnt = (chunk_size + 63) / 64; // TODO

    	USB_EP_IN(1)->DIEPTSIZ = (pktcnt << 19) | chunk_size; 	// Program DIEPTSIZ with the CHUNK size, not the total size
    	USB_EP_IN(1)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

    	uint16_t first_fill = (chunk_size > 128) ? 128 : chunk_size; // Fill the FIFO with the first part of THIS chunk
        write_Fifo(1, EndPoint[1].txBuffer_ptr, first_fill);  // write_Fifo() will decrement EndPoint[1].txCounter (the ISR call won't think there is still data to send)

        // If we have more than 128 bytes, enable the Empty FIFO interrupt
        if (chunk_size > 128) USB_OTG_DEVICE->DIEPEMPMSK |= (1 << 1);
    }



    return EP_OK;
}


uint32_t MSC_transferTXCallback_EP0(void) {
    uint16_t len = EndPoint[0].txCounter;
    while(USB_EP_IN(0)->DIEPCTL & USB_OTG_DIEPCTL_EPENA); // not mandatory in case

    if (len == 0) {
            // Prepare the hardware for a ZLP (Status Phase)
            USB_EP_IN(0)->DIEPTSIZ = (1 << 19) | 0; // 1 packet, 0 bytes
            USB_EP_IN(0)->DIEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

            // No need to call write_Fifo, the hardware handles the empty packet
            return EP_OK;
        }

    // Set hardware for the remaining total transfer
    uint32_t pktcnt = (len + 63) / 64;
    USB_EP_IN(0)->DIEPTSIZ = (pktcnt << 19) | len;
    USB_EP_IN(0)->DIEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

    // Initial FIFO fill (Control is always 64 bytes max per packet)
    uint16_t fill = (len > 64) ? 64 : len;
    write_Fifo(0, EndPoint[0].txBuffer_ptr, fill);

    // If there's more, the TXFE interrupt in the ISR will finish it
    if (len > 64) {
        USB_OTG_DEVICE->DIEPEMPMSK |= (1 << 0);
    }
    return EP_OK;
}

/*
 * brief  Set and start TX transaction
 * param  EP number, TX Buffer, length
 * retval OK/FAILED
 */
uint32_t MSC_setTxBuffer(uint8_t EPnum, uint8_t *txBuff, uint32_t len){

	/*	When you want to start a transfer (like sending 4096 bytes of a file),
		you call this. It prepares the variables and then "kicks" the callback to start the hardware.
		Initial Call: When you call setTxBuffer, the top part runs, sets DIEPTSIZ, fills the first 128 bytes, and exits.*/

    /************** Safety check ****************/
    if((EndPoint[EPnum].txCounter != 0) || (check_USB_device_status(DEVICE_STATE_TX_PR) == EP_OK)) {
        return EP_FAILED;
    }

    EndPoint[EPnum].txBuffer_ptr = txBuff;
    EndPoint[EPnum].txCounter = len; // number of total bytes to send
    //EndPoint[EPnum].totXferLen = len; // Track total for ZLP logic if needed

    set_device_status(DEVICE_STATE_TX_PR);
    // The hardware DIEPTSIZ register can handle up to 512KB.

    /* 1. Kick the transfer manually first to program DIEPTSIZ and fill the first packets */
    EndPoint[EPnum].txCallBack();

    /* 2. ONLY NOW enable the Empty FIFO interrupt if there is still data to send */
    /* This prevents the ISR from pre-empting the Callback setup */
    if (EPnum == 1 && EndPoint[1].txCounter > 0) {
        USB_OTG_DEVICE->DIEPEMPMSK |= (1 << 1);
    }
    else if (EPnum == 0 && EndPoint[0].txCounter > 64) {
        USB_OTG_DEVICE->DIEPEMPMSK |= (1 << 0);
    }

    return EP_OK;
}


uint32_t MSC_transferRXCallback_EP1(uint32_t param) {
	// 1. Get the actual amount of data the hardware just wrote to memory
	// (In STM32 OTG, you usually calculate this by looking at how much DOEPTSIZ decremented)

	// DO NOT reset EndPoint[1].rxBuffer_ptr here!
	switch (msc_state) {
	case MSC_STATE_IDLE:
		// We are looking for a CBW (31 bytes) in rxBufferEp1
		if (EndPoint[1].rxCounter >= 31) {
			USB_MSC_CBW_t *cbw = (USB_MSC_CBW_t *)rxBufferEp1;

			if (cbw->dSignature == 0x43425355) {
				msc_tag = cbw->dTag;
				MSC_Parse_SCSI_Command(cbw);
			}
		}
		break;

	case MSC_STATE_DATA_OUT:
		// Data has landed in ram_disk, we'll write to the Flash
		MSC_WriteComplete_Callback();
		break;

	case MSC_STATE_ERROR:
		// Handle error state if necessary, or just reset
		MSC_ForceResetState();
		break;

	default:
		// This covers any other enum values (like MSC_STATE_SEND_CSW)
		// If we get an RX packet while sending, it's usually out of sync.
		break;
	}
	return param;
}


/***************************************************
*			USB enumeration
***************************************************/

void enumerate_Reset(){

	/************************************************************/
	/* 1. CLEAN THE PIPES FIRST                                 */
	/************************************************************/
/*
	The Issue: Your FLUSH_FIFO_TIMEOUT is currently a simple loop counter. Because the STM32F469 is very fast (180MHz),
	that loop might finish before the hardware has actually cleared the RAM.
	Correction: Use a proper millisecond timer or a much larger constant (like 0xFFFF).
	If the FIFOs aren't flushed properly during a reset, the first SCSI command the PC sends will be mixed with "garbage" data from the previous session.
*/
	USB_FlushRxFifo(2000);       // Clear the Global Receive FIFO
	USB_FlushTxFifo(0, 2048);    // Clear Control EP0 TX FIFO
	USB_FlushTxFifo(1, 2048);    // Clear MSC Data EP1 TX FIFO

	/************************************************************/
	/* 2. RESET SOFTWARE STATE                                  */
	/************************************************************/
	set_device_status(DEVICE_STATE_RESET);
	USB_OTG_FS->GINTSTS = 0xFFFFFFFF; // Clear interrupts

	/************************************************************/
	/* 3. RECONFIGURE ENDPOINTS 							    */
	/************************************************************/
	init_EndPoints(); // Hardware-enable EP0 and reassert EP1/EP2 structure

	USB_OTG_FS->GINTSTS &= ~0xFFFFFFFF; // reset OTG core interrupt register

	/* OTG all endpoints interrupt mask register */
	USB_OTG_DEVICE->DAINTMSK = 0x30003; // IEPINT-> IN EP0 & IN EP1 interrupts unmasked, OEPINT: OUT endpoint 0 & 1 interrupts unmasked
	USB_OTG_DEVICE->DOEPMSK  = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM; /* Unmask SETUP Phase done Mask,  TransfeR Completed interrupt for OUT */
	USB_OTG_DEVICE->DIEPMSK  =  /* USB_OTG_DIEPMSK_ITTXFEMSK | */ USB_OTG_DIEPMSK_XFRCM; /* TransfeR Completed interrupt for IN */

	USB_OTG_DEVICE->DCFG  &= ~USB_OTG_DCFG_DAD_Msk;  /* before Enumeration set address 0 */

	/* Endpoint 1 */
	USB_EP_IN(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
			USB_OTG_DIEPCTL_TXFNUM_0 |  /* TX Number 1 */
			USB_OTG_DIEPCTL_EPTYP_1 |  /* Eptype 10 means Bulk */
			USB_OTG_DIEPCTL_USBAEP |  /* Set Endpoint active */
			USB_MSC_MAX_PACKET_SIZE;  /* Max Packet size (bytes) */

	USB_EP_OUT(1)->DOEPTSIZ = 0;
	USB_EP_OUT(1)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << USB_OTG_DOEPTSIZ_PKTCNT_Pos)); /* RM quote: Indicates the total number of USB packets that constitute the Transfer Size amount of data for this endpoint. This field is decremented every time a packet (maximum size or short packet) is written to the RxFIFO */
	USB_EP_OUT(1)->DOEPTSIZ |= DOEPT_TRANSFER_SIZE; /* Transfer size. If you set transfer size = max. packet, the core will interrupt the application at the end of each packet */

	USB_EP_OUT(1)->DOEPCTL = USB_OTG_DOEPCTL_EPENA | 	/* Enable Endpoint after DOEPTSIZ programmed  */
			USB_OTG_DOEPCTL_CNAK |  /* Clear NAK */
			USB_OTG_DOEPCTL_EPTYP_1 |  /* Eptype 10 means Bulk */
			USB_OTG_DOEPCTL_USBAEP | /* Set Endpoint active */
			USB_MSC_MAX_PACKET_SIZE; /* CHK MPSIZ The application must program this field with the maximum packet size for the current logical endpoint. This value is in bytes */
}


/**
 * brief  Handle all host requests, send all descriptors data
 * param
 * param
 * retval
 */
void enumerate_Setup(){
	// Combined request for your existing logic
	    uint16_t request = (setup_pkt_data.setup_pkt.bRequest << 8) | setup_pkt_data.setup_pkt.bmRequestType;
	    uint16_t len = setup_pkt_data.setup_pkt.wLength;
	    // Point the USB Tx function directly to the descriptor constants to save RAM compared to a copy to an intermediate buffer
	    uint8_t *data_to_send = NULL;

	switch(request){

	case REQ_TYPE_HOST_TO_DEVICE_GET_DEVICE_DECRIPTOR:
		switch(setup_pkt_data.setup_pkt.wValue){
		case DESCRIPTOR_TYPE_DEVICE: 				/* Request 0x0680  Value 0x0100 */
			if(DEVICE_DESCRIPTOR_LENGTH < len) len = DEVICE_DESCRIPTOR_LENGTH;
			data_to_send = (uint8_t*)&deviceDescriptor;
			break;
		case DESCRIPTOR_TYPE_CONFIGURATION: 			/* Request 0x0680  Value 0x0200 */
			if(CONFIGURATION_DESCRIPTOR_LENGTH < len) len = CONFIGURATION_DESCRIPTOR_LENGTH;
			data_to_send = (uint8_t*)&configurationDescriptor;
			break;
		case DESCRIPTOR_TYPE_DEVICE_QUALIFIER: 			/* Request 0x0680  Value 0x0600 */
			if(DEVICE_QUALIFIER_LENGTH < len) len = DEVICE_QUALIFIER_LENGTH;
			data_to_send = (uint8_t*)&deviceQualifierDescriptor;
			break;
		case DESCRIPTOR_TYPE_LANG_STRING: 			/* Request 0x0680  Value 0x0300 */
			if(LANG_DESCRIPTOR_LENGTH < len) len = LANG_DESCRIPTOR_LENGTH;
			data_to_send = (uint8_t*)&languageStringDescriptor;
			break;
		case DESCRIPTOR_TYPE_MFC_STRING: 			/* Request 0x0680  Value 0x0301 */
			if(MFC_DESCRIPTOR_LENGTH < len) len = MFC_DESCRIPTOR_LENGTH;
			data_to_send = (uint8_t*)&manufactorStringDescriptor;
			break;
		case DESCRIPTOR_TYPE_PROD_STRING: 			/* Request 0x0680  Value 0x0302 */
			if(PRODUCT_DESCRIPTOR_LENGTH < len) len = PRODUCT_DESCRIPTOR_LENGTH;
			data_to_send = (uint8_t*)&productStringDescriptor;
			break;
		case DESCRIPTOR_TYPE_SERIAL_STRING: 			/* Request 0x0680  Value 0x0303 */
			static uint8_t serial_buf[50];
			Get_ID_To_String(serial_buf); // fetch the UID
			// The Host's requested length (len) vs our actual length (50)
			if (len > 50) len = 50;
			data_to_send = serial_buf;
			break;
		case DESCRIPTOR_TYPE_CONFIGURATION_STRING: 		/* Request 0x0680  Value 0x0304 */
			if(CONFIG_STRING_LENGTH < len) len = CONFIG_STRING_LENGTH;
			data_to_send = (uint8_t*)&configurationStringDescriptor;
			break;
		case DESCRIPTOR_TYPE_INTERFACE_STRING: 			/* Request 0x0680  Value 0x0305 */
			if(INTERFACE_STRING_LENGTH < len) len = INTERFACE_STRING_LENGTH;
			data_to_send = (uint8_t*)&stringInterface;
			break;
		default:
			return;
		}
		break;

	case MSC_GET_MAX_LUN:                /* Request 0xFEA1 */
		//data_to_send[0] = 0; // udf ! crash ! dereferencing a NULL pointer is Undefined Behavior (UB)
		static uint8_t max_lun = 0; // Create a tiny persistent buffer - 0 means 1 Logical Unit (LUN 0)
		data_to_send = &max_lun;    // Point to it
		len = 1;                     // We must send exactly 1 byte
		break;

	case MSC_BOT_RESET:                  /* Request 0xFF21 */
		len = 0;                         // ZLP (Zero Length Packet)
		// Logic: Prepare your BOT state machine to receive a fresh CBW
		// and clear HALT conditions on Bulk endpoints if necessary.
		//MSC_ForceResetState();
		break;

	case REQ_TYPE_DEVICE_TO_HOST_SET_ADDRESS: 				/* Request 0x0500  */
		len=0; // ZLP
		USB_OTG_DEVICE->DCFG &= ~((uint32_t)0x7F << 4); // Clear the 7-bit DAD field (bits 4 to 10) first!
		USB_OTG_DEVICE->DCFG |= (uint32_t)(setup_pkt_data.setup_pkt.wValue << 4);
		device_state = DEVICE_STATE_ADDRESSED;
		break;
	case REQ_TYPE_DEVICE_TO_HOST_SET_CONFIGURATION: 			/* Request 0x0900  */
		len=0; // ZLP
		activate_Endpoints(); // Activate the actual Bulk pipes now that the host has selected this config
		device_state = DEVICE_STATE_CONFIGURED;
		break;
	case CLEAR_FEATURE_ENDP: 						/* Request 0x0201  */
		/* The CLEAR_FEATURE request is the "handshake" that allows the Host to reset an endpoint after you have stalled it.
		   Without it, once the device stalls (due to an error), it will stay "stuck" forever, the Host thinking the pipe is still broken. */
		if (setup_pkt_data.setup_pkt.wValue == 0) {
		        uint8_t ep_num = setup_pkt_data.setup_pkt.wIndex & 0x7F;

		        if (setup_pkt_data.setup_pkt.wIndex & 0x80) {
		            // --- Existing Hardware Logic ---
		            USB_EP_IN(ep_num)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
		            USB_EP_IN(ep_num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;

		            // --- MSC Software Reset ---
		            if (ep_num == 1) {
		                msc_state = MSC_STATE_IDLE;
		                EndPoint[1].txCounter = 0;
		                EndPoint[1].statusTx = EP_READY;
		                clear_USB_device_status(DEVICE_STATE_TX_PR);
		            }
		        } else {
		            // --- Existing Hardware Logic ---
		            USB_EP_OUT(ep_num)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
		            USB_EP_OUT(ep_num)->DOEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;

		            // --- MSC Software Reset ---
		            if (ep_num == 1) {
		                // Re-prime the OUT endpoint so it can receive the next CBW
		                USB_EP_OUT(1)->DOEPTSIZ = (1 << 19) | 64;
		                USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
		            }
		        }
		    }
		// Send the Status Phase ACK (Zero Length Packet)
		USB_EP_IN(0)->DIEPTSIZ = 0; // 0 bytes
		USB_EP_IN(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
		return;
	default:
		break;
	}

	EndPoint[0].setTxBuffer(0, data_to_send, len); // must be sent even if len == 0
}



/************************************************************/
/*************************** inline *************************/
/************************************************************/

/* Device status functions. Set/clear/check*/

static inline void set_device_status(eDeviceState state){
	device_state |= state;
}

void clear_USB_device_status(eDeviceState state){
	device_state &= ~state;
}

uint32_t check_USB_device_status(eDeviceState state){
	if(device_state & state){
		return EP_OK;
	}
	else return EP_FAILED;
}

/*************************** End of inline functions *************************/



/*********************************************************************************/
/**************************** OTG FS ISR *****************************************/
/*********************************************************************************/

extern void OTG_FS_IRQHandler(void);

void OTG_FS_IRQHandler(){

	// Identify only the interrupts that are both PENDING and ENABLED
	uint32_t active_irq = USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;

	/* SUSPEND DETECTED - to use uncomment in USB_OTG_FS_init_registers()
	 If USB bus has gone quiet (which happens about 3ms after you unplug the cable or the PC goes to sleep) */
	if (active_irq & USB_OTG_GINTSTS_USBSUSP){
		USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBSUSP; // clear flag
		MSC_ForceResetState(); // Perform the robust cleanup
	}

	/* Wakeup DETECTED - to use uncomment in USB_OTG_FS_init_registers() */
	if (active_irq & USB_OTG_GINTSTS_WKUINT){
		USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_WKUINT; // clear flag
		MSC_ForceResetState();
	}


	/****************** USBRST Reset event ************************/
	// The core sets this bit to indicate that a reset is detected on the USB.
	if(active_irq & USB_OTG_GINTSTS_USBRST){
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_USBRST);
		enumerate_Reset();
		return;
	}

	/****************** ENUMDNEM event ****************************/
	/* The core sets this bit to indicate that speed enumeration is complete: the hardware-level connection and speed negotiation (reset/chirp) are complete,
	   allowing software to begin handling USB control transfers (such as address assignment and descriptor setup).
	   The application must read the OTG_DSTS register to obtain the enumerated speed.
	   */
	if(active_irq & USB_OTG_GINTSTS_ENUMDNE){
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_ENUMDNE);
		set_device_status(DEVICE_STATE_DEFAULT);
	}


	/***************** RXFLVL EVENT (Receive FIFO Level) ***********
	This is the very first interrupt to fire when receiving data. As soon as a packet is successfully received and validated, the hardware
	pushes it into the Global Receive FIF. The RXFLVL interrupt signals that there is at least one packet sitting in the FIFO waiting to be read.
	The CPU must read the status from the USB_OTG_GRXSTSP register, which tells it the byte count and the endpoint number.
	Then, it reads the data out of the FIFO. */

	// (Receive FIFO Level): This triggers every time a packet (or a status update) lands in the RX FIFO
	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {

		// 1. Loop until the FIFO is actually empty
		while(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {

			uint32_t temp = USB_OTG_FS->GRXSTSP;

			// Correct the shift: PKTSTS is bits [20:17] - 			      Indicates the status of the received packet:
			uint8_t pktsts = (temp & USB_OTG_GRXSTSP_PKTSTS) >> 17; /*    0001: Global OUT NAK (triggers an interrupt)
																		  0010: OUT data packet received
																		  0011: OUT transfer completed (triggers an interrupt)
																		  0100: SETUP transaction completed (triggers an interrupt)
																		  0110: SETUP data packet received  */
			uint8_t EpNum  = (temp & USB_OTG_GRXSTSP_EPNUM); // EP number
			uint16_t bcnt  = (temp & USB_OTG_GRXSTSP_BCNT) >> 4; // Indicates the byte count of the received data packet.

			if (pktsts == 0x02) { // DATA packet
				if (bcnt > 0) {
					read_Fifo(EpNum, bcnt);
				}
			}
			else if (pktsts == 0x06) { // SETUP packet
				read_Setup_Fifo();
			}
			// 3. IMPORTANT: If pktsts is 0x03 (Transfer Complete),
			// reading GRXSTSP has already "popped" it. We don't need to do anything else, just let the loop continue.
		}
	}


	/********************** IN ENDPOINT EVENT *****************************/

	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT){

	    uint32_t epnums = USB_OTG_DEVICE->DAINT;

	    // --- Check EP0 IN ---
	    if(epnums & 0x0001){
	        uint32_t IN_interrupt = USB_EP_IN(0)->DIEPINT;

	        // Tx FIFO is empty
	        if (IN_interrupt & USB_OTG_DIEPINT_TXFE) {
	            // Only fill if we actually have data pending - // write_Fifo internally updates txBuffer_ptr and txCounter
	            if (EndPoint[0].txCounter > 0) write_Fifo(0, EndPoint[0].txBuffer_ptr, EndPoint[0].txCounter);
	            // Always mask EP0 TXFE after one fill or if empty to prevent interrupt storm
	            USB_OTG_DEVICE->DIEPEMPMSK &= ~(1 << 0);
	        }

	        if (IN_interrupt & USB_OTG_DIEPINT_XFRC) {
	            // CRITICAL: Handshake for Control Status Phase - Prepare for the next Setup Packet
	            USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	            EndPoint[0].statusTx = EP_READY;
	            clear_USB_device_status(DEVICE_STATE_TX_PR);
	            USB_EP_IN(0)->DIEPINT = USB_OTG_DIEPINT_XFRC;
	        }
	        CLEAR_IN_EP_INTERRUPT(0, IN_interrupt);
	    }

	    // --- Check EP1 IN ---
	    if (epnums & 0x0002) {
	    	uint32_t IN_interrupt = USB_EP_IN(1)->DIEPINT;
	    	uint32_t fifo_empty_mask = USB_OTG_DEVICE->DIEPEMPMSK;

	    	// Handle FIFO Empty: This fills the FIFO 128 bytes at a time until txCounter is 0
	    	if ((IN_interrupt & USB_OTG_DIEPINT_TXFE) && (fifo_empty_mask & (1 << 1))) {

	    		/* When write_Fifo is called inside the TxCallback, data are pushed into the hardware. If the TXFE interrupt is enabled, the hardware might trigger
	    		the ISR while the CPU is still inside the write_Fifo function or immediately after it returns, but before the callback has finished its logic.
				The Result: the ISR sees txCounter > 0, thinks it's a standard "continue filling" situation, and pushes more data into the FIFO before the hardware
				has even started the 4096-byte burst you just programmed. This results in the "Extra Byte" or "Incomplete" error.
				To fix this, the ISR must stop relying on txCounter to decide how much to fill. It must rely on the Hardware's own remaining count.
	    	 	 */
	    		// The XFRSIZ field is the lower 19 bits of DIEPTSIZ - indicates the transfer size in bytes for EP1, it decrements when Tx Fifo is written
	    		uint32_t hardware_needs = USB_EP_IN(1)->DIEPTSIZ & 0x7FFFF;
	     		//  Ask the hardware: "How many bytes do you actually want for this burst ?"
	    		if (hardware_needs > 0) {
	    			uint16_t fifo_space_bytes = (USB_EP_IN(1)->DTXFSTS & 0xFFFF) * 4;
	    			// Only fill what the hardware is currently programmed to take
	    			uint16_t fill = (hardware_needs > fifo_space_bytes) ? fifo_space_bytes : hardware_needs;
	    			if (fill > 128) fill = 128; // Fills to 64 or 128 (max) for bus efficiency
	    			write_Fifo(1, EndPoint[1].txBuffer_ptr, fill);
	    		}
	    		// If the hardware has all the bytes it needs for this programmed burst
	    		if ((USB_EP_IN(1)->DIEPTSIZ & 0x7FFFF) == 0) {
	    			USB_OTG_DEVICE->DIEPEMPMSK &= ~(1 << 1); // turn off the interrupt mask so we don't loop here.
	    		}
	    	}

	    	// 2. Handle Transfer Complete (XFRC)
	    	if (IN_interrupt & USB_OTG_DIEPINT_XFRC) {

	    		USB_EP_IN(1)->DIEPINT = USB_OTG_DIEPINT_XFRC; // Clear flag
	    		EndPoint[1].statusTx = EP_READY;
	    		clear_USB_device_status(DEVICE_STATE_TX_PR);
	    		// EndPoint[1].txCounter represents the total number of bytes that remain to be sent
	    		if(msc_state == MSC_STATE_DATA_IN) {
	    			if(EndPoint[1].txCounter > 0) EndPoint[1].txCallBack(); // a chunk of 4096 has been sent and there is more to send
	    			else if(EndPoint[1].txCounter == 0)	MSC_Send_CSW(msc_tag, 0, 0x00); // trigger MSC_Send_CSW if we just finished a Data phase.
	    		}

	    		// We finished sending the Status. Back to IDLE.
	    		else if (msc_state == MSC_STATE_SEND_CSW) {
	    			msc_state = MSC_STATE_IDLE;
	    			clear_USB_device_status(DEVICE_STATE_TX_PR);

	    			EndPoint[1].rxBuffer_ptr = rxBufferEp1; // Re-prime the OUT endpoint to listen for the next Command (CBW)
	    			EndPoint[1].rxCounter = 0; 	// Reset the counter to zero so the next read starts fresh

	    			USB_EP_OUT(1)->DOEPTSIZ = (1 << 19) | 31; // Expect 31 bytes
	    			USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	    		}
	    	}
	    }
	}

	// outside the individual endpoint checks. This ensures that if EP1 also had an interrupt, you don't clear the "Global" flag before you've had a chance to see EP1's flag.
	USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_IEPINT);


	/********************* OUT ENDPOINT EVENT - OEPINT ************************/

	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT){

		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_OEPINT);
		uint32_t epnums  = USB_OTG_DEVICE->DAINT;    /* Read out EndPoint INTerrupt bits */

		/********************	 EP0 OUT	  **********************/
		if( epnums & 0x00010000){ /* EndPoint INTerrupt bits correspond to EP0 OUT */

			uint32_t epint = USB_EP_OUT(0)->DOEPINT; /* Read out Endpoint Interrupt register for EP0 */

			if(epint & USB_OTG_DOEPINT_STUP){		/*  SETUP phase done, Setup packet received */
				/*	Indicates that the SETUP phase for the control endpoint is complete and no more back-to-back
					SETUP packets were received for the	current control transfer. On this interrupt, the application can decode the received SETUP data packet. */

				/* Set the "Global IN NAK": only essential to ensure compatibility when RMB is set at 0x00 :
				 * It tells the hardware: "Do not let ANY IN endpoint (EP1 or EP0) respond to the host until I say so."
				 * This is a bit in the DCTL register that most drivers ignore, but it's vital for high-speed "Fixed Disk" emulation.
				 * With RMB = 0x00, the host can send an EP0 packet immediately after a starting a transfer on EP1 (eg CBW packet), and
				 * the following EP1 packets prevents further communication with EP0. In short, without SGINAK, EP1 data packets overlap with EP0 setup packets.	 */
				USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_SGINAK;

				enumerate_Setup();

				CLEAR_OUT_EP_INTERRUPT(0, USB_OTG_DOEPINT_STUP); 	// Clear the Setup flag
				USB_OTG_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;		// Clear the Global IN NAK
			}
			if(epint & USB_OTG_DOEPINT_XFRC){
				// CNAK and EPENA must be set again after every interrupt to let this EP receive upcoming data
				USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
			}
			CLEAR_OUT_EP_INTERRUPT(0, epint);
		}
		/********************	 EP1 OUT	 *********************/
		if( epnums & 0x00020000){ /* EndPoint INTerrupt bits correspond to EP1 OUT */

			// OTG_DOEPINTx : this register indicates the status of an endpoint with respect to USB- and AHB-related events.
			uint32_t epint = USB_EP_OUT(1)->DOEPINT; /* Read out Endpoint Interrupt register for EP1 */

			// XFRC: Transfer completed interrupt. Indicates that the programmed transfer is complete on the AHB as well as on the USB, for this endpoint.
			if(epint & USB_OTG_DOEPINT_XFRC){
				USB_EP_OUT(1)->DOEPINT = USB_OTG_DOEPINT_XFRC; // Clear flag
				MSC_transferRXCallback_EP1(0); // Transfer is finished, the callback can parse the CBW (if IDLE) or handle the Data (if DATA_OUT).
			}
			CLEAR_OUT_EP_INTERRUPT(1, epint);
		}
		return;
	}
}



/* The BOT (Bulk-Only Transport) Flow :
    - Host sends CBW (31 bytes) -> RXFLVL ISR -> read_Fifo -> rxBufferEp1.
    - OUT XFRC ISR -> MSC_transferRXCallback_EP1 calls MSC_Parse_SCSI_Command.
    - Parser calls MSC_Handle_Read10.
    - Read10 calls setTxBuffer -> TXCallback_EP1 primes hardware + fills first 128 bytes.
    - ISR (TXFE) refills the FIFO until all data is gone or until 8192 bytes are pushed.
    - ISR (TXFE) sees XFRSIZ == 0, disables itself.
    - IN XFRC ISR -> TXCallback_EP1 sees msc_state == MSC_STATE_DATA_IN and automatically calls MSC_Send_CSW.
    - CSW Sent -> IN XFRC ISR -> TXCallback_EP1 sees msc_state == MSC_STATE_SEND_CSW and goes back to IDLE, re-priming the OUT endpoint for the next CBW.
*/

/* You can use the __REV macros below to replace the bit shifts to align the word
#include "cmsis_compiler.h"
// Converts Big-Endian from USB/SCSI to Little-Endian for STM32
#define BE32_TO_LE32(x) __REV(x)
#define BE16_TO_LE16(x) __REV16(x) */

/* In the USB Mass Storage world, you are dealing with two different "languages" speaking at the same time, and they have different rules for numbers:
    - USB BOT Layer (The Wrapper): The dCBWDataTransferLength (4 bytes at offset 0x08) is Little-Endian. This is defined by the USB Bulk-Only Transport spec.
    - SCSI Layer (The Command): The bytes inside the CBWCB array (the Command Descriptor Block) follow SCSI rules,
      which almost always use Big-Endian for multi-byte fields like LBA (Logical Block Address) or Allocation Length.
    Why didn't we do this for the other cases?
      - Standard Inquiry (0x12): In a standard Inquiry, the host usually asks for 36 bytes. The length is stored in only one byte (CBWCB[4]).
        Since it's a single byte, there is no "endianness" to worry about—the value is just the value.
      - Test Unit Ready (0x00): This command has no length or LBA fields; it’s just the OpCode and padding.
      - Read Capacity (0x25): The host doesn't send a length here; the device just returns a fixed 8-byte structure.
   Why it's different for 0x43, 0x46, and 0x4A:
   These are "newer" or more complex SCSI commands. To allow for large amounts of data, the SCSI spec designers allocated two bytes (16 bits) for the length:
   Byte 7: MSB (Most Significant Byte) / Byte 8: LSB (Least Significant Byte) /  Length=(MSB×256)+LSB ->  (CBWCB[7] << 8) | CBWCB[8]
 */
